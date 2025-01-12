#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class CameraHandler : public rclcpp::Node
{
public:
    CameraHandler()
            : Node("camera_handler_cpp")
            , image_transport_(static_cast<SharedPtr>(this))
    {
        publisher_ = image_transport_.advertise("/bpc_prp_robot/camera", 1);
        timer_ = create_wall_timer(
                std::chrono::milliseconds(33), std::bind(&CameraHandler::timer_callback, this));
        open();

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(get_logger(), "Could not open camera");
        } else {
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        }
    }

    void open() {
        cap_.open(0);
    }


private:
    void timer_callback()
    {
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(get_logger(), "Camera is not opened");
            open();
            return;
        }

        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_ERROR(get_logger(), "Captured empty frame");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_.publish(*msg);
    }

    image_transport::ImageTransport image_transport_;
    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};
