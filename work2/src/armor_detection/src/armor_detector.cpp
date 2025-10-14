#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;

class ArmorDetector : public rclcpp::Node {
public:
    ArmorDetector() : Node("armor_detector") {
        string package_path;
        try {
            package_path = ament_index_cpp::get_package_share_directory("armor_detection");
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "找不到包 'armor_detection'!");
            return;
        }

        string image_path = package_path + "/images/work2.png";
        RCLCPP_INFO(this->get_logger(), "图片路径: %s", image_path.c_str());
        detectArmor(image_path);
    }

private:
    struct LightBar {
        Rect rect;
        Point center;
        double area;
        float ratio;
        int y_center;
    };

    void stepShow(const string& title, const Mat& img, int delay = 1000) {
        namedWindow(title, WINDOW_AUTOSIZE);
        imshow(title, img);
        RCLCPP_INFO(this->get_logger(), "显示窗口: %s（%dms后关闭）", title.c_str(), delay);
        waitKey(delay);
        destroyWindow(title);
    }

    // 1. 读取图片
    void detectArmor(const string& image_path) {
        Mat img = imread(image_path, IMREAD_COLOR);
        if (img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "图片读取失败！请检查路径");
            Mat err = Mat::zeros(480, 640, CV_8UC3);
            err.setTo(Scalar(0,0,255));
            putText(err, "Image Not Found", Point(50,240), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255,255,255), 2);
            stepShow("错误", err, 3000);
            return;
        }
        stepShow("1. 原始图像", img);

         // 2. 提取蓝色
        Mat hsv, mask;
        cvtColor(img, hsv, COLOR_BGR2HSV);
        Scalar lower_blue = Scalar(80, 30, 30);
        Scalar upper_blue = Scalar(160, 255, 255);
        inRange(hsv, lower_blue, upper_blue, mask);
        stepShow("2. 蓝色区域（放宽阈值）", mask);

         // 3. 轻度去噪
        Mat kernel = getStructuringElement(MORPH_RECT, Size(2,2));
        morphologyEx(mask, mask, MORPH_CLOSE, kernel, Point(-1,-1), 1);
        stepShow("3. 轻度去噪后", mask);

         // 4. 识别灯条
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        
        vector<LightBar> light_bars;
        // 5. 框选所有非倒影的装甲板
        Mat light_img = img.clone();
        for (const auto& cnt : contours) {
            double area = contourArea(cnt);
            if (area < 20) {
                RCLCPP_DEBUG(this->get_logger(), "过滤小轮廓（面积=%.1f）", area);
                continue;
            }

            Rect rect = boundingRect(cnt);
            float ratio = (float)rect.width / rect.height;
            if (ratio > 0.8) {
                RCLCPP_DEBUG(this->get_logger(), "过滤宽高比过大的轮廓（ratio=%.2f）", ratio);
                continue;
            }

            int y_center = rect.y + rect.height / 2;
            LightBar lb;
            lb.rect = rect;
            lb.center = Point(rect.x + rect.width/2, y_center);
            lb.area = area;
            lb.ratio = ratio;
            lb.y_center = y_center;
            light_bars.push_back(lb);

            rectangle(light_img, rect, Scalar(0,0,255), 2);
            putText(light_img, to_string(light_bars.size()), lb.center, 
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
        }

        RCLCPP_INFO(this->get_logger(), "识别到灯条数量: %zu", light_bars.size());
        stepShow("4. 识别到的灯条", light_img, 2000);

        Mat final_img = light_img.clone();
        vector<Rect> valid_armors; // 存储有效装甲板，避免重复框选

        int left_area_end = img.cols * 2 / 3;  // 左区域
        int right_area_start = img.cols * 1 / 3; // 右区域

        // 配对左装甲板
        for (size_t i = 0; i < light_bars.size(); ++i) {
            for (size_t j = i + 1; j < light_bars.size(); ++j) {
                LightBar& l = light_bars[i];
                LightBar& r = light_bars[j];
                if (l.center.x > r.center.x) swap(l, r);

                // 左装甲板的灯条都在左2/3区域
                if (r.center.x > left_area_end) continue;
                // 过滤倒影
                if (l.y_center > img.rows * 0.65 || r.y_center > img.rows * 0.65) continue;
                // 间距合理
                double dist = r.center.x - l.center.x;
                double avg_h = (l.rect.height + r.rect.height) / 2;
                if (dist < avg_h * 1.5 || dist > avg_h * 4) continue;
                // 高度/垂直偏差正常
                if (abs(l.rect.height - r.rect.height) > avg_h * 0.4) continue;
                if (abs(l.center.y - r.center.y) > avg_h * 0.3) continue;

                // 左装甲板有效，加入列表
                Rect armor(
                    min(l.rect.x, r.rect.x), min(l.rect.y, r.rect.y),
                    max(l.rect.x + l.rect.width, r.rect.x + r.rect.width) - min(l.rect.x, r.rect.x),
                    max(l.rect.y + l.rect.height, r.rect.y + r.rect.height) - min(l.rect.y, r.rect.y)
                );
                valid_armors.push_back(armor);
            }
        }

        // 配对右装甲板
        for (size_t i = 0; i < light_bars.size(); ++i) {
            for (size_t j = i + 1; j < light_bars.size(); ++j) {
                LightBar& l = light_bars[i];
                LightBar& r = light_bars[j];
                if (l.center.x > r.center.x) swap(l, r);

                if (l.center.x < right_area_start) continue;
                if (l.y_center > img.rows * 0.65 || r.y_center > img.rows * 0.65) continue;
                double dist = r.center.x - l.center.x;
                double avg_h = (l.rect.height + r.rect.height) / 2;
                if (dist < avg_h * 1.5 || dist > avg_h * 4) continue;

                if (abs(l.rect.height - r.rect.height) > avg_h * 0.4) continue;
                if (abs(l.center.y - r.center.y) > avg_h * 0.3) continue;

                // 右装甲板有效，加入列表
                Rect armor(
                    min(l.rect.x, r.rect.x), min(l.rect.y, r.rect.y),
                    max(l.rect.x + l.rect.width, r.rect.x + r.rect.width) - min(l.rect.x, r.rect.x),
                    max(l.rect.y + l.rect.height, r.rect.y + r.rect.height) - min(l.rect.y, r.rect.y)
                );
                valid_armors.push_back(armor);
            }
        }

        // 绘制有效装甲板
        for (size_t k = 0; k < valid_armors.size(); ++k) {
            bool is_dup = false;
            for (size_t m = 0; m < k; ++m) {
                Rect overlap = valid_armors[k] & valid_armors[m];
                double overlap_ratio = (double)overlap.area() / valid_armors[k].area();
                if (overlap_ratio > 0.5) {
                    is_dup = true;
                    break;
                }
            }
            if (!is_dup) {
                rectangle(final_img, valid_armors[k], Scalar(0,255,0), 2);
                putText(final_img, "Armor " + to_string(k+1), Point(valid_armors[k].x, valid_armors[k].y-10),
                        FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0), 2);
            }
        }

        // 提示信息
        if (valid_armors.empty() && !light_bars.empty()) {
            putText(final_img, "No Valid Armor (Filtered Reflection)", Point(50,50),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,255), 2);
        } else if (light_bars.empty()) {
            putText(final_img, "No Enough Light Bars", Point(50,50),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,255), 2);
        }

        // 显示最终结果
        namedWindow("5. 最终结果", WINDOW_NORMAL);
        resizeWindow("5. 最终结果", 1024, 768);
        imshow("5. 最终结果", final_img);
        waitKey(0);
        destroyAllWindows();
    }
};

int main(int argc, char**argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<ArmorDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}