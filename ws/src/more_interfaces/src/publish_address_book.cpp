#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <more_interfaces/msg/address_book.hpp>

using namespace std::chrono_literals;

class AddressBookPublisher : public rclcpp::Node
{
public:
    AddressBookPublisher() 
    : Node("address_book_publisher")
    {
        address_book_publisher_p = this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);

        auto publish_message = [this]() -> void {
            auto message = more_interfaces::msg::AddressBook();
            message.first_name = "Rendell";
            message.last_name = "Cale";
            message.age = 25;
            message.gender = message.MALE;
            message.address = "Vegamot 1,i-204";

            RCLCPP_INFO(this->get_logger(), "Publishing " + message.first_name + " " + message.last_name);
            address_book_publisher_p->publish(message);
        };

        timer_p = this->create_wall_timer(1s, publish_message);
    }
private:
    rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_p;
    rclcpp::TimerBase::SharedPtr timer_p;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddressBookPublisher>());
    rclcpp::shutdown();
    return 0;
}