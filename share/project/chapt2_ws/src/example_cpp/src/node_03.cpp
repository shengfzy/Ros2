#include "rclcpp/rclcpp.hpp"
#include <memory>

class Node03 : public rclcpp::Node
{
public:
    Node03(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "hello, I'm %s", name.c_str());
    }
private:

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Node03>("node_03");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
