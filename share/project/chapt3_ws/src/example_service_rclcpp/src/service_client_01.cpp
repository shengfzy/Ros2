#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <memory>

class ServiceClient01 : public rclcpp::Node
{
public:
    ServiceClient01(std::string name) : Node(name){
        RCLCPP_INFO(this->get_logger(), "client %s has been started.", name.c_str());

        add_ints_client_ = this->create_client<example_interfaces::srv::AddTwoInts>(
            "add_two_ints_srv"
        );
    }

    void send_request(int a, int b){
        RCLCPP_INFO(this->get_logger(), "calculate for %d + %d", a, b);

        while(!add_ints_client_->wait_for_service(std::chrono::seconds(1))){
            if(!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(), "Interrupted during wait service...");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "waiting for service...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        add_ints_client_->async_send_request(
            request,
            std::bind(&ServiceClient01::result_callback_, 
                      this,
                      std::placeholders::_1)
        );
    }
    
private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr add_ints_client_;

    void result_callback_(
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_feature
    ){
        auto response = result_feature.get();
        RCLCPP_INFO(this->get_logger(), "cal result = %ld", response->sum);
    }
};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);

    auto node = std::make_shared<ServiceClient01>("service_client_01");

    node->send_request(5, 6);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
