#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/subscriber.h"
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"
#include "cv_bridge/cv_bridge.h"

#include <Eigen/Geometry>

class Bag2SeqNode : public rclcpp::Node
{
public:
    Bag2SeqNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    void setup();

private:
    void frameCallback(sensor_msgs::msg::Image::ConstSharedPtr sensor1, sensor_msgs::msg::Image::ConstSharedPtr sensor2, sensor_msgs::msg::Image::ConstSharedPtr sensor3);
    
private:
    
    using SyncPolicy = typename message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    std::unique_ptr<Synchronizer> sync_;

    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::SubscriberFilter sensor_sub_[3] ;

    rclcpp::Time latest_msg_ns_ ;
        
    cv::Mat sensor_img_[3] ;

    std::mutex frame_mutex_;
    std::atomic<bool> frame_ready_{false};
    uint count = 0 ;
};
