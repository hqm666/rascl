#include <memory>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <soem/soem.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std;
using namespace std::chrono_literals;


static constexpr uint16_t MOTOR_ENABLE_VOLTAGE = 6;
static constexpr uint16_t MOTOR_SWITCH_ON = 7;
static constexpr uint16_t MOTOR_ENABLE_OPERATION = 15;
static constexpr uint16_t MOTOR_START_HOMING = 31;
static constexpr uint16_t MOTOR_START_MOTION = 63;

// Operation modes
static constexpr uint8_t MODE_HOMING = 6;
static constexpr uint8_t MODE_CYCLIC_SYNC_POSITION = 8;
static constexpr uint8_t HOMING_MODE_METHOD_20 = 20;
static constexpr uint8_t HOMING_MODE_METHOD_26 = 26;

// Homing configuration
static constexpr uint32_t HOMING_SPEED_DEFAULT = 100;
static constexpr uint32_t HOMING_SPEED_GRIPPER = 25;
static constexpr uint16_t HOMING_STATUS_MASK = 0x1400;

// Motor joint slave IDs
static constexpr uint8_t SHOULDER_SLAVE_ID = 1;
static constexpr uint8_t UPPERARM_SLAVE_ID = 2;
static constexpr uint8_t LOWERARM_SLAVE_ID = 3;
static constexpr uint8_t GRIPPER_SLAVE_ID = 4;

// Motion configuration
static constexpr uint32_t MAX_MOTOR_SPEED = 2000;
static constexpr int32_t POSITION_TOLERANCE = 5;
static constexpr int DELAY_MULTIPLIER = 10;

// Fake hardware simulation
static constexpr double FAKE_MOVEMENT_RATE = 0.005;
static constexpr double FAKE_POSITION_THRESHOLD = 0.1;

// CANopen 402 Status word bit masks for validation
static constexpr uint16_t STATUS_OPERATION_ENABLED_MASK = 0x6F;  // bits 0,1,2,3,5,6
static constexpr uint16_t STATUS_OPERATION_ENABLED_VALUE = 0x27; // Operation Enable state
static constexpr uint16_t STATUS_SWITCHED_ON_MASK = 0x6F;
static constexpr uint16_t STATUS_SWITCHED_ON_VALUE = 0x23;       // Switched On state
static constexpr uint16_t STATUS_READY_TO_SWITCH_ON_MASK = 0x6F;
static constexpr uint16_t STATUS_READY_TO_SWITCH_ON_VALUE = 0x21; // Ready to Switch On state
static constexpr uint16_t STATUS_FAULT_MASK = 0x08;              // bit 3
static constexpr uint16_t STATUS_FAULT_VALUE = 0x08;             // Fault bit set








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
            std_msgs::msg::String st;
            st.data = "device not ready.";
            status_pub->publish(st);

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
    
    bool ok = false;
    ok = motor_comm_init();

    
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
    // this_thread::sleep_for(10s);
    // ecx_contextt ecx_context;
    // if(ecx_init(&ecx_context, "eno1"))
    // {
    //   int slaves=ecx_config_init(&ecx_context);
    //   cout<<"Slaves found: "<< slaves<<endl;
    //   ecx_configdc(&ecx_context);
    // }

    ecx_contextt ctx;
    // xxx1328 for station 1
    //enx00e04c054280 for station 2
    //eno1 is not suitable, slaves=-1
    int ok1 = ecx_init(&ctx, "enx00e04c1c1328");
    int ok2 = ecx_init(&ctx, "enx00e04c054280");
    uint16_t status_word;
    int size= sizeof(status_word);
    uint16_t delay=100;

    if((ok1||ok2)==1)
    {
      std::cout << "ecx_init() succeed. " << "\n";
    }
    int slaves = ecx_config_init(&ctx);
    if (slaves<0)
    {
      RCLCPP_INFO(this->get_logger(), "Wrong ecx_config_int()");
    }
    else if(slaves==0)
    {
      RCLCPP_INFO(this->get_logger(), "No devices are detected.");
    }
    else
    {
      //slaves=ctx.slavecount
      std::cout << "ecx_config_init() succeed, " << slaves << " are detected. \n";
      //std::cout << "ctx.slavecount: " << ctx.slavecount << "\n";
      ecx_config_map_group(&ctx, IOmap, 0);

      ecx_configdc(&ctx);

      /* wait for all slaves to reach SAFE_OP state */
      ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

      // sets actual slave state (if slave number = 0, then write to all slaves)
      ecx_writestate(&ctx, 1);

      RCLCPP_INFO(this->get_logger(), "(on_configure) Init done.");


      for(uint8_t slave = 1; slave <= ctx.slavecount; slave++){
        //enable voltage
        ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(MOTOR_ENABLE_VOLTAGE), &MOTOR_ENABLE_VOLTAGE, EC_TIMEOUTRXM);
        ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
        std::cout<<"status after enable voltage: "<<hex<<status_word<<std::endl;

        //switch on
        ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(MOTOR_SWITCH_ON), &MOTOR_SWITCH_ON, EC_TIMEOUTRXM);
        ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
        std::cout<<"status after switch on: "<<hex<<status_word<<std::endl;
        
        //enable operation
        ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(MOTOR_ENABLE_OPERATION), &MOTOR_ENABLE_OPERATION, EC_TIMEOUTRXM);
        ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
        std::cout<<"status after enable operation: "<<status_word<<std::endl;

        //Homing Mode
        ecx_SDOwrite(&ctx, slave, 0x6060, 0x00, false, sizeof(MODE_HOMING), &MODE_HOMING, EC_TIMEOUTRXM);
        ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
        std::cout<<"status after Homing Mode: "<<hex<<status_word<<std::endl;

        //Homing Method
        ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(HOMING_MODE_METHOD_26), &HOMING_MODE_METHOD_26, EC_TIMEOUTRXM);
        //Homing Speed
        ecx_SDOwrite(&ctx, slave, 0x6099, 0x02, false, sizeof(HOMING_SPEED_DEFAULT), &HOMING_SPEED_DEFAULT, EC_TIMEOUTRXM);
        //Start Homing
        ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(MOTOR_START_HOMING), &MOTOR_START_HOMING, EC_TIMEOUTRXM);
        
        // Wait for Home to be reached
        // ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
        // while(status_word != HOMING_STATUS_MASK)
        // {
        //     ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
        //     std::cout<<"waiting for Home to be reached."
        // }
        // Wait for Home to be reached
        uint16_t mask = HOMING_STATUS_MASK;
        ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
        status_word = status_word & mask;
        cout<<"status_word & mask ="<< hex<< status_word;
        // while(status_word != mask)
        // {
        //     ecx_SDOread(&ctx, slave, 0x6041, 0x00, false, &size, &status_word, EC_TIMEOUTRXM);
        //     status_word = status_word & mask;
        //     cout << "Inside while-loop (Home). Waiting for: 0x" << hex << uppercase << mask << "\tReceiving: 0x" << status_word << endl;
        //     this_thread::sleep_for(chrono::milliseconds(delay));
        // }

        RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(Home) Home reached");

        // Get actual position after homing
        int32_t act_pos = 0;
        int size_act_pos = sizeof(act_pos);
        ecx_SDOread(&ctx, slave, 0x6064, 0x00, false, &size_act_pos, &act_pos, EC_TIMEOUTRXM);
        RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "(Home) Home position for slave %d: %d", slave, act_pos);

        ecx_SDOread(&ctx, slave, 0x606C, 0x00, false, &size_act_pos, &act_pos, EC_TIMEOUTRXM);
        RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "Act n for slave %d: %d", slave, act_pos);
        
        ecx_SDOread(&ctx, slave, 0x6077, 0x00, false, &size_act_pos, &act_pos, EC_TIMEOUTRXM);
        RCLCPP_INFO(rclcpp::get_logger("RASCL_HI"), "P121...Act Torque for slave %d: %d", slave, act_pos);

        double target_angle= 180.0; // degree
        int32_t target_steps=int(target_angle/360*1325000);
   
        ecx_SDOread(&ctx, slave, 0x6064, 0x00, false, &size_act_pos, &act_pos, EC_TIMEOUTRXM);
        cout<<"target pos in steps: %d"<<target_steps<<std::endl<<"act pos in steps: "<<act_pos<<"/n";

        //Mode of operation = Cyclic Synchronous Position mode -> 8 (prefered over profile pos. mode -> 1)
        uint8_t mode = MODE_CYCLIC_SYNC_POSITION;
        ecx_SDOwrite(&ctx, slave, 0x6060, 0x00, false, sizeof(mode), &mode, EC_TIMEOUTRXM);
        this_thread::sleep_for(chrono::milliseconds(delay));

        //Maximum Motor Speed
        uint32_t max_speed = MAX_MOTOR_SPEED;
        ecx_SDOwrite(&ctx, slave, 0x6080, 0x00, false, sizeof(max_speed), &max_speed, EC_TIMEOUTRXM);
        this_thread::sleep_for(chrono::milliseconds(delay));

        //Write Target Position
        ecx_SDOwrite(&ctx, slave, 0x607A, 0x00, false, sizeof(target_steps), &target_steps, EC_TIMEOUTRXM);
        this_thread::sleep_for(chrono::milliseconds(delay));

        //Start Motion
        uint16_t start_motion = MOTOR_START_MOTION;
        ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, false, sizeof(start_motion), &start_motion, EC_TIMEOUTRXM);
        this_thread::sleep_for(chrono::milliseconds(delay));

        // Wait for the target to be reached
        
        while(std::abs(act_pos - target_steps) > POSITION_TOLERANCE)
        {
            ecx_SDOread(&ctx, slave, 0x6064, 0x00, false, &size_act_pos, &act_pos, EC_TIMEOUTRXM);
            RCLCPP_DEBUG(rclcpp::get_logger("RASCL_HI"), "(SetPosition) slave: %s Inside while-loop (SetPosition). Waiting for: %d Receiving: %d", to_string(slave).c_str(), target_steps, act_pos);
            this_thread::sleep_for(chrono::milliseconds(DELAY_MULTIPLIER * delay));
        }
        RCLCPP_DEBUG(rclcpp::get_logger("RASCL_HI"), "(SetPosition) slave: %s exiting while-loop", to_string(slave).c_str());


      }


    

      



    } 
    



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

  uint8_t IOmap[128];
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slave>());
  rclcpp::shutdown();
  return 0;
}