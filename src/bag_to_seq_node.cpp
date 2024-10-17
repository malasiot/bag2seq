#include "bag_to_seq_node.hpp"

#include <chrono>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

using namespace std::literals::chrono_literals;
using namespace std ;

Bag2SeqNode::Bag2SeqNode(const rclcpp::NodeOptions &options) : rclcpp::Node("bag2seq", options)
{
    declare_parameter("sensor1_topic", "");
    declare_parameter("sensor2_topic", "");
    declare_parameter("sensor3_topic", "");
}


void Bag2SeqNode::setup()
{
	string sensor_topic[3] ;
    sensor_topic[0] = get_parameter("sensor1_topic").as_string();
    sensor_topic[1] = get_parameter("sensor2_topic").as_string();
    sensor_topic[2] = get_parameter("sensor3_topic").as_string();
   
    sync_.reset(new Synchronizer(SyncPolicy(10), sensor_sub_[0], sensor_sub_[1], sensor_sub_[2]));
    sync_->registerCallback(std::bind(&Bag2SeqNode::frameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    rclcpp::SubscriptionOptions sub_opts;
    // Update the subscription options to allow reconfigurable qos settings.
    sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions{
        {
            // Here all policies that are desired to be reconfigurable are listed.
            rclcpp::QosPolicyKind::Depth,
            rclcpp::QosPolicyKind::Durability,
            rclcpp::QosPolicyKind::History,
            rclcpp::QosPolicyKind::Reliability,
        }};
   
    image_transport::TransportHints hints(this, "raw", "rgb_image_transport");

	for( uint i=0 ; i<3 ; i++ ) {
	    sensor_sub_[i].subscribe(
        this, sensor_topic[i],
        hints.getTransport(), rmw_qos_profile_default, sub_opts);
        }

}

void Bag2SeqNode::frameCallback(sensor_msgs::msg::Image::ConstSharedPtr sensor1Msg,
                                     sensor_msgs::msg::Image::ConstSharedPtr sensor2Msg,
                                     sensor_msgs::msg::Image::ConstSharedPtr sensor3Msg
                                     )
{
    std::lock_guard<std::mutex> frame_lock_(frame_mutex_);


    rclcpp::Time now = this->get_clock()->now();

    double dt = (now - latest_msg_ns_).seconds();

    if ( count > 0 && dt > 1 ) {
        RCLCPP_INFO_ONCE(get_logger(), "Recording finished. Captured %d frames", count) ;
        return ;
    }

    const sensor_msgs::msg::Image::ConstSharedPtr msgs[3] = { sensor1Msg, sensor2Msg, sensor3Msg } ;
   
    latest_msg_ns_ = sensor1Msg->header.stamp ;

    RCLCPP_INFO(get_logger(), "Frame %d", count) ;

    cout << sensor1Msg->header.stamp.sec << ' ' << sensor1Msg->header.stamp.nanosec << endl ;
    for( int i=0 ; i<3 ; i++ ) {
        rclcpp::Time msg_time(msgs[i]->header.stamp);
        latest_msg_ns_ = std::max(msg_time, latest_msg_ns_) ;
   
        ostringstream s_suffix ;
        s_suffix << i << "_" << std::setw(3) << std::setfill('0') << count << ".png";
        string suffix = s_suffix.str() ;

        try
        {
            auto grayPtr = cv_bridge::toCvCopy(msgs[i], sensor_msgs::image_encodings::TYPE_8UC1);
            sensor_img_[i] = grayPtr->image;

            cv::imwrite("/tmp/img_" + suffix, grayPtr->image) ;
        }
        catch (cv_bridge::Exception &e)
        {
        // display the error at most once per 10 seconds
          RCLCPP_ERROR_THROTTLE(get_logger(), *this->get_clock(), 10, "cv_bridge exception %s at line number %d on function %s in file %s", e.what(), __LINE__,
                              __FUNCTION__, __FILE__);
         return;
        }

        
    }
    ++count ;
    

}


