#include <chrono>
#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std;



class Master : public rclcpp::Node
{
public:
  Master()
  : Node("Master")
  {
    cmd_pub = this->create_publisher<std_msgs::msg::String>("m2s", 10);
    status_sub=this->create_subscription<std_msgs::msg::String>("s2m", 10, bind(&Master::on_status, this, placeholders::_1));
    
    running_.store(true);
    kbthread = std::thread([this]() { this->keyboard_loop(); });
    RCLCPP_INFO(this->get_logger(), "Master is established.Waiting for INIT_OK...");
  }
  ~Master() override
  {
    running_.store(false);
    if (kbthread.joinable()) kbthread.join();
  }

private:
  void on_status(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I received: '%s'", msg->data.c_str());
  }
  void keyboard_loop()
  {
    // 1) waiting for subs
    while (running_.load()) {
      auto subs = cmd_pub->get_subscription_count();
      if (subs > 0) {
        cout << subs << " subscriber(s) detected" << endl;
        break;
      }
      cout << "No subscribers detected, waiting..." << endl;
      this_thread::sleep_for(1s);
    }

    // 2) waiting for cmd
    while (running_.load()) {
      std::cout << "Enter clock: ";
      std::string input;
      if (!std::getline(std::cin, input)) {
        break;  // 
      }

      try {
        int clock = std::stoi(input);
        publish_clock(clock);
      } catch (...) {
        std::cout << "Invalid input, please try again" << std::endl;
      }
    }
  }

  void publish_clock(int clock)
  {
    std_msgs::msg::String msg;
    msg.data = std::to_string(clock);
    cmd_pub->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
  }

  //rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub;
  
  std::atomic<bool> running_{false};
  std::thread kbthread; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Master>());
  rclcpp::shutdown();
  return 0;
}