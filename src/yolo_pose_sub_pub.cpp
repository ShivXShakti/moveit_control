#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <dualarm_custom_msgs/msg/obj_pose_array.hpp>
#include <dualarm_custom_msgs/msg/obj_pose.hpp>
#include <depthai_ros_msgs/msg/track_detection2_d_array.hpp>
#include <depthai_ros_msgs/msg/track_detection2_d.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ObjectPoseTransformer : public rclcpp::Node
{
public:
    ObjectPoseTransformer()
    : Node("object_pose_transformer"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ObjectPoseTransformer YOLO node...");

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image", 10,
            std::bind(&ObjectPoseTransformer::imageCallback, this, _1));

        detection_sub_ = this->create_subscription<depthai_ros_msgs::msg::TrackDetection2DArray>(
            "/color/yolo_Spatial_tracklets", 10,
            std::bind(&ObjectPoseTransformer::detectionCallback, this, _1));

        publisher_ = this->create_publisher<dualarm_custom_msgs::msg::ObjPoseArray>(
            "object_pose_torso", 10);

        object_classes_ = {
            "person","bicycle","car","motorbike","aeroplane","bus","train","truck","boat",
            "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
            "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
            "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
            "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
            "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
            "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake",
            "chair","sofa","pottedplant","bed","diningtable","toilet","tvmonitor","laptop",
            "mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink",
            "refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"
        };
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            current_img_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void detectionCallback(const depthai_ros_msgs::msg::TrackDetection2DArray::SharedPtr msg)
    {
        if (current_img_.empty())
            return;

        cv::Mat img = current_img_.clone();
        dualarm_custom_msgs::msg::ObjPoseArray obj_array;

        for (auto &detection : msg->detections) {
            if (detection.results.empty())
                continue;

            auto result = detection.results[0];
            int class_id = std::stoi(result.hypothesis.class_id);
            float score = result.hypothesis.score;

            int cx = static_cast<int>(detection.bbox.center.position.x);
            int cy = static_cast<int>(detection.bbox.center.position.y);
            int w  = static_cast<int>(detection.bbox.size_x);
            int h  = static_cast<int>(detection.bbox.size_y);
            int x1 = cx - w / 2;
            int y1 = cy - h / 2;
            int x2 = cx + w / 2;
            int y2 = cy + h / 2;

            cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
            cv::putText(img, 
                        "ID:" + object_classes_[class_id] + " (" + std::to_string(score).substr(0,4) + ")",
                        cv::Point(x1, y1 - 10), cv::FONT_HERSHEY_SIMPLEX,
                        0.5, cv::Scalar(0, 255, 0), 2);

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->get_clock()->now();
            pose_stamped.header.frame_id = msg->header.frame_id;
            pose_stamped.pose = result.pose.pose;

            try {
                geometry_msgs::msg::PoseStamped transformed_pose;
                tf_buffer_.transform(pose_stamped, transformed_pose, "torso", 1s);

                dualarm_custom_msgs::msg::ObjPose obj;
                obj.object_name = object_classes_[class_id];
                obj.pose_stamped = transformed_pose;
                obj_array.data.push_back(obj);
            }
            catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform pose: %s", ex.what());
            }
        }

        cv::imshow("YOLO Detections", img);
        cv::waitKey(1);

        publisher_->publish(obj_array);
    }

    // Members
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<depthai_ros_msgs::msg::TrackDetection2DArray>::SharedPtr detection_sub_;
    rclcpp::Publisher<dualarm_custom_msgs::msg::ObjPoseArray>::SharedPtr publisher_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    cv::Mat current_img_;
    std::vector<std::string> object_classes_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectPoseTransformer>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
