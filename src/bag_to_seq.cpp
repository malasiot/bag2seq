#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/subscriber.h"
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"
#include "cv_bridge/cv_bridge.h"

#include <rosbag2_cpp/reader.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <fstream>

using namespace std ;

class ToFData
{
public:
    sensor_msgs::msg::Image::ConstPtr image_[3];

    ToFData(const sensor_msgs::msg::Image::ConstPtr &s1,
            const sensor_msgs::msg::Image::ConstPtr &s2,
            const sensor_msgs::msg::Image::ConstPtr &s3)
    {
        image_[0] = s1;
        image_[1] = s2;
        image_[2] = s3;
    }
};

std::vector<ToFData> dataset;

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
    void newMessage(const std::shared_ptr<const M> &msg)
    {
        this->signalMessage(msg);
    }
};

// Callback for synchronized messages
void callback(const sensor_msgs::msg::Image::ConstPtr &s1,
              const sensor_msgs::msg::Image::ConstPtr &s2,
              const sensor_msgs::msg::Image::ConstPtr &s3)
{
    ToFData sd(s1, s2, s3);

    // Stereo dataset is class variable to store data
    dataset.push_back(sd);
}

void loadBag(const std::string &filename, const std::vector<std::string> &topics)
{
    rosbag2_cpp::Reader reader;
    reader.open(filename);

    // Set up fake subscribers to capture images
    BagSubscriber<sensor_msgs::msg::Image> sub[3];

    // Use time synchronizer to make sure we get properly synchronized images
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image,
                                      sensor_msgs::msg::Image, sensor_msgs::msg::Image>
        sync(sub[0], sub[1], sub[2], 25);
    sync.registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;

    // Load all messages into our stereo dataset
    while (reader.has_next())
    {
        auto message = reader.read_next();

        for (uint i = 0; i < 3; i++)
        {
            if (message->topic_name == topics[i])
            {
                sensor_msgs::msg::Image extracted_msg;
                rclcpp::SerializedMessage extracted_serialized_msg(*message->serialized_data);

                serialization.deserialize_message(&extracted_serialized_msg, &extracted_msg);

                sub[i].newMessage(std::make_shared<sensor_msgs::msg::Image>(extracted_msg));
            }
        }
    }

    reader.close();
}

void writeData(const std::string &prefix) {
    for ( uint channel = 0 ; channel < 3 ; channel ++ ) {
        std::ostringstream f_strm ;
        f_strm << prefix << channel << ".csv" ;
        auto filename = f_strm.str() ;
        ofstream ostrm(filename) ;

        for( uint frame = 0 ; frame < dataset.size() ; frame++ ) {
            const auto &data = dataset[frame] ;
           for( uint count = 0 ; count < 64 ; count ++ ) {
                const auto &msg = data.image_[channel] ;
                if ( count > 0 ) ostrm << ',' ;
                ostrm << (uint)(msg->data[count]) ;
           }
           ostrm << endl ;
        }
    }


}

int main(int argc, char *argv[])
{
    std::vector<std::string> topics = {argv[2], argv[3], argv[4]};
    loadBag(argv[1], topics);

    writeData(argv[5]) ;

    std::cout << dataset.size() << std::endl ;
}