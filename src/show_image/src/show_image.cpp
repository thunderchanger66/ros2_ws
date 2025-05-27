#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include "ai_msgs/msg/perception_targets.hpp"

class CompressedImageViewer : public rclcpp::Node {
public:
  CompressedImageViewer()
  : Node("compressed_image_viewer") {
    sub_image_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/image", 10,
      std::bind(&CompressedImageViewer::image_callback, this, std::placeholders::_1));

    sub_target_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      "/hobot_face_landmarks_detection", 10,
      std::bind(&CompressedImageViewer::target_callback, this, std::placeholders::_1));

  }

private:
  cv::Mat current_image_;
  std::mutex image_mutex_;

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) 
  {
    std::vector<uint8_t> image_data(msg->data.begin(), msg->data.end());
    cv::Mat image = cv::imdecode(image_data, cv::IMREAD_COLOR);
    if (image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Failed to decode image.");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      current_image_ = image;
    }

    //draw_and_show();
  }

  void target_callback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg) 
  {
    std::lock_guard<std::mutex> lock(image_mutex_);

    if (current_image_.empty()) {
      return;
    }

    cv::Mat image = current_image_.clone();

    for (const auto& target : msg->targets) {
      // 遍历每个 target 的关键点组
      for (const auto& kps : target.points) {
        // 可以根据需要只绘制特定类型，比如 "face_kps"
        if (kps.type != "face_kps") continue;

        for (const auto& p : kps.point) {
          int x = static_cast<int>(p.x);
          int y = static_cast<int>(p.y);
          if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
            cv::circle(image, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);
          }
        }
      }
    }

    cv::imshow("with Keypoints", image);
    cv::waitKey(1);
  }

  void draw_and_show() {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!current_image_.empty()) {
      cv::imshow("viewer with Keypoints", current_image_);
      cv::waitKey(1);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr sub_target_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompressedImageViewer>());
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}
