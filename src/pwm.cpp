#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <navio2_ros/pwm.hpp>

namespace blurr
{

using sensor_msgs::msg::JointState;
using std_msgs::msg::Bool;
using std_msgs::msg::Float32;
using navio2_ros::PWM_Base;
using namespace std::chrono_literals;

// force -> pwm map
constexpr auto forces{std::array{-40., -35.5, -28.5, -22.2, -16.1, -10., -5.1, -1.1, 0., 0., 1.8, 6.2, 12.2, 19.2, 26.4, 33.8, 41.1, 49.9}};
constexpr auto forces_pwm{std::array{1100.f, 1150.f, 1200.f, 1250.f, 1300.f, 1350.f, 1400.f, 1450.f, 1480.f, 1520.f, 1550.f, 1600.f, 1650.f, 1700.f, 1750.f, 1800.f, 1850.f, 1900.f}};
static_assert (forces.size() == forces_pwm.size(), "force/pwm map should have same sizes");

constexpr auto watchdog_period{1};

float interpPWM(double v)
{
  // no extrapolation
  if(v <= forces.front())
    return forces_pwm.front();
  else if(v >=forces.back())
    return forces_pwm.back();

  uint i = 0;
  while ( v > forces[i+1] ) i++;

  const auto xL = forces[i], xR = forces[i+1];
  const auto yL = forces_pwm[i], yR = forces_pwm[i+1];
  return yL + ( yR - yL ) / ( xR - xL ) * ( v - xL );
}

// special pins
constexpr auto thruster_pins{std::array{0,2,4,6,8,10}};
constexpr auto light_pin{13};
constexpr auto tilt_pin{12};

class BlurrPWM : public PWM_Base
{
public:
  BlurrPWM(rclcpp::NodeOptions options) : PWM_Base(options, {0, 2, 4, 6, 8, 10, tilt_pin, light_pin})
  {
    pwm_timer = create_wall_timer(20ms, [&](){toPWM();});
    thruster_watchdog = create_wall_timer(std::chrono::seconds(watchdog_period), [&](){watchDog();});

    run_sub = create_subscription<Bool>("run", 10, [&](Bool::UniquePtr msg){running = msg->data;});

    tilt_sub = create_subscription<JointState>("joint_setpoint", 10, [&](JointState::UniquePtr msg)
    {readTilt(*msg);});

    thruster_sub = create_subscription<JointState>("thruster_command", 10, [&](JointState::UniquePtr msg)
    {readThrusters(*msg);});
    light_sub = create_subscription<Float32>("light", 10, [&](Float32::UniquePtr msg) {light_intensity = msg->data;});

    tilt_pub = create_publisher<JointState>("joint_states", rclcpp::SensorDataQoS());
    tilt_msg.name = {"tilt"};
    tilt_msg.position = {0};
  }

private:    

  rclcpp::TimerBase::SharedPtr pwm_timer, thruster_watchdog;

  rclcpp::Subscription<Bool>::SharedPtr run_sub;
  bool running{true};

  rclcpp::Subscription<Float32>::SharedPtr light_sub;
  rclcpp::Subscription<JointState>::SharedPtr tilt_sub, thruster_sub;

  rclcpp::Publisher<JointState>::SharedPtr tilt_pub;
  JointState tilt_msg;

  std::array<double, 6> thruster_force{0};
  double thruster_time{};

  double tilt_angle{0};
  float light_intensity{0};

  // callbacks

  void readTilt(const JointState &msg)
  {
    if(msg.name.size() && msg.position.size())
      tilt_angle = std::min(0.785, std::max(-0.785, msg.position[0]));
  }

  void readThrusters(const JointState &msg)
  {
    thruster_time = now().seconds();
    const static std::vector<std::string> names{"thr1", "thr2", "thr3", "thr4", "thr5", "thr6"};
    for(uint i = 0; i<msg.name.size();++i)
    {
      for(uint j = 0; j<6; ++j)
      {
        if(msg.name[i] == names[j])
        {
          thruster_force[j] = msg.effort[i];
          break;
        }
      }
    }
  }

  void watchDog()
  {
    if(now().seconds() - thruster_time > watchdog_period)
    {
      for(auto pin: thruster_pins)
        toRest(pin);
    }
  }

  void toPWM()
  {
    // at least remind the tilt angle
    tilt_msg.position[0] = tilt_angle;
    tilt_msg.header.stamp = now();
    tilt_pub->publish(tilt_msg);

    if(!running)  return;

    // thrusters
    for(uint i = 0; i < 6; ++i)
      set_duty_cycle(pins[i], interpPWM(thruster_force[i]));

    // tilt: pwm + joint_states
    set_duty_cycle(tilt_pin, 1500 + 509.3*tilt_angle);

    // lumen light
    set_duty_cycle(light_pin, 1100 + 800*light_intensity);
  }
};

}

// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<blurr::BlurrPWM>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
