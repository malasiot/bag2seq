#include <rclcpp/rclcpp.hpp>
#include "bag_to_seq_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Bag2SeqNode>() ;
    node->setup() ;
    rclcpp::spin(node);
    rclcpp::shutdown();
}