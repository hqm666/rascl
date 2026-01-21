#include <memory>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std;
using namespace std::chrono_literals;

class Slave : public rclcpp::Node
{
public:
  Slave()
  : Node("Slave")
  {
    status_pub = this->create_publisher<std_msgs::msg::String>("s2m", 10);

    auto topic_callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void {   
        if (!ready_.load()) {
            RCLCPP_WARN(this->get_logger(), "Command received but device not ready. Ignored: '%s'",
                        msg->data.c_str());
            return;
        }
        handle_command(msg->data);
      };
    cmd_sub =this->create_subscription<std_msgs::msg::String>("m2s", 10, topic_callback);

    start_timer_ = this->create_wall_timer(5ms, std::bind(&Slave::start_init, this));

    RCLCPP_INFO(this->get_logger(), "Sub node started. Initializing motor communication...");
  }
  ~Slave() override
  {
    running_.store(false);
    if (init_thread_.joinable()) init_thread_.join();
  }

private:
  void start_init()
  {
    start_timer_->cancel();

    running_.store(true);
    init_thread_ = std::thread([this]() { this->init_flow(); });
  }

  void init_flow()
  {
    
    bool ok = motor_comm_init();

    
    std_msgs::msg::String st;
    st.data = ok ? "INIT_OK" : "INIT_FAIL";
    status_pub->publish(st);

    if (ok) {
      ready_.store(true);
      RCLCPP_INFO(this->get_logger(), "Init OK. Waiting for commands...");
    } else {
      ready_.store(false);
      RCLCPP_ERROR(this->get_logger(), "Init FAILED. Still listening, but commands will be ignored.");
    }
  }

  bool motor_comm_init()
  {
    return true;
  }

  void handle_command(const std::string & cmd)
  {
    cout<<"handle command() done."<<endl;
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;
  rclcpp::TimerBase::SharedPtr start_timer_;
  std::atomic<bool> running_{false};
  std::atomic<bool> ready_{false};
  std::thread init_thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slave>());
  rclcpp::shutdown();
  return 0;
}