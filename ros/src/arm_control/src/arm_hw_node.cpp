// Minimal ros_control hardware loop for an arm.
//
// This node provides:
// - hardware_interface::JointStateInterface   (for joint_state_controller)
// - hardware_interface::PositionJointInterface (for position_controllers/JointTrajectoryController)
//
// By default it runs in "sim_mode": it will move the reported joint positions
// toward the commanded positions with a simple first-order response, so you can
// bring up ros_control + MoveIt without real hardware.
//
// To connect to real hardware (RoboRIO/NetworkTables), implement readFromHardware()
// and writeToHardware() and set ~sim_mode:=false.

#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/spinner.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>

namespace arm_control
{

class ArmHW : public hardware_interface::RobotHW
{
public:
  explicit ArmHW(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
  {
    // Joints list: must match controllers.yaml
    if (!pnh_.getParam("joints", joint_names_))
    {
      joint_names_ = {"joint_1", "joint_2", "joint_3"};
      ROS_WARN_STREAM("~joints not set. Using default: [joint_1, joint_2, joint_3]");
    }
    if (joint_names_.empty())
    {
      throw std::runtime_error("~joints is empty");
    }

    // Behavior
    pnh_.param("sim_mode", sim_mode_, true);
    pnh_.param("sim_max_vel", sim_max_vel_, 2.0);          // rad/s
    pnh_.param("sim_tau", sim_tau_, 0.10);                 // seconds (time constant)
    pnh_.param("warn_no_hw_every_s", warn_no_hw_every_s_, 2.0);
    pnh_.param<std::string>("state_topic", state_topic_, "/arm_hw/nt_state");
    pnh_.param<std::string>("command_topic", command_topic_, "/arm_hw/nt_command");

    const std::size_t n = joint_names_.size();
    pos_.assign(n, 0.0);
    vel_.assign(n, 0.0);
    eff_.assign(n, 0.0);
    cmd_pos_.assign(n, 0.0);

    // Register interfaces
    for (std::size_t i = 0; i < n; ++i)
    {
      const std::string& name = joint_names_[i];

      hardware_interface::JointStateHandle state_handle(name, &pos_[i], &vel_[i], &eff_[i]);
      jnt_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle pos_handle(jnt_state_interface_.getHandle(name), &cmd_pos_[i]);
      jnt_pos_interface_.registerHandle(pos_handle);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_pos_interface_);

    if (!sim_mode_)
    {
      cmd_pub_ = nh_.advertise<sensor_msgs::JointState>(command_topic_, 1);
      state_sub_ = nh_.subscribe(state_topic_, 1, &ArmHW::stateCb, this);
      ROS_INFO_STREAM("arm_hw_node: real IO via ROS topics enabled. state_topic=" << state_topic_
                      << " command_topic=" << command_topic_);
    }
  }

  void read(const ros::Time& /*time*/, const ros::Duration& period)
  {
    if (sim_mode_)
    {
      simStep(period);
      return;
    }
    readFromHardware(period);
  }

  void write(const ros::Time& /*time*/, const ros::Duration& period)
  {
    if (sim_mode_)
    {
      // no-op: controller commands already affect simulated read() dynamics
      (void)period;
      return;
    }
    writeToHardware(period);
  }

private:
  void stateCb(const sensor_msgs::JointStateConstPtr& msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    last_state_msg_ = *msg;
    last_state_stamp_ = ros::Time::now();
  }

  void simStep(const ros::Duration& period)
  {
    const double dt = std::max(0.0, period.toSec());
    if (dt <= 0.0)
      return;

    const double tau = std::max(1e-3, sim_tau_);
    const double alpha = 1.0 - std::exp(-dt / tau);  // first-order response

    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      const double prev = pos_[i];
      double next = prev + alpha * (cmd_pos_[i] - prev);

      // Limit velocity to something reasonable to avoid crazy jumps when the controller starts
      const double max_step = std::abs(sim_max_vel_) * dt;
      const double step = std::max(-max_step, std::min(max_step, next - prev));
      next = prev + step;

      pos_[i] = next;
      vel_[i] = step / dt;
      eff_[i] = 0.0;
    }
  }

  void readFromHardware(const ros::Duration& /*period*/)
  {
    // Real IO is provided via a separate NT bridge node that publishes sensor_msgs/JointState.
    sensor_msgs::JointState msg;
    ros::Time stamp;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      msg = last_state_msg_;
      stamp = last_state_stamp_;
    }

    if (stamp.isZero())
    {
      throttledWarnNoHardware();
      return;
    }

    // Map by joint name
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      const std::string& name = joint_names_[i];
      int idx = -1;
      for (std::size_t k = 0; k < msg.name.size(); ++k)
      {
        if (msg.name[k] == name)
        {
          idx = static_cast<int>(k);
          break;
        }
      }
      if (idx < 0)
        continue;

      if (idx < static_cast<int>(msg.position.size()))
        pos_[i] = msg.position[static_cast<std::size_t>(idx)];
      if (idx < static_cast<int>(msg.velocity.size()))
        vel_[i] = msg.velocity[static_cast<std::size_t>(idx)];
      else
        vel_[i] = 0.0;
      if (idx < static_cast<int>(msg.effort.size()))
        eff_[i] = msg.effort[static_cast<std::size_t>(idx)];
      else
        eff_[i] = 0.0;
    }
  }

  void writeToHardware(const ros::Duration& /*period*/)
  {
    if (!cmd_pub_)
    {
      throttledWarnNoHardware();
      return;
    }

    sensor_msgs::JointState cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.name = joint_names_;
    cmd.position = cmd_pos_;
    cmd_pub_.publish(cmd);
  }

  void throttledWarnNoHardware()
  {
    const ros::Time now = ros::Time::now();
    if ((now - last_no_hw_warn_).toSec() >= warn_no_hw_every_s_)
    {
      last_no_hw_warn_ = now;
      ROS_WARN("arm_hw_node: hardware IO not implemented (sim_mode=false).");
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::vector<std::string> joint_names_;
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> eff_;
  std::vector<double> cmd_pos_;

  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;

  bool sim_mode_{true};
  double sim_max_vel_{2.0};
  double sim_tau_{0.10};
  double warn_no_hw_every_s_{2.0};
  ros::Time last_no_hw_warn_{0.0};

  std::string state_topic_;
  std::string command_topic_;
  ros::Publisher cmd_pub_;
  ros::Subscriber state_sub_;
  std::mutex state_mtx_;
  sensor_msgs::JointState last_state_msg_;
  ros::Time last_state_stamp_{0.0};
};

}  // namespace arm_control

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_hw");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Use an AsyncSpinner so controller_manager services (load/switch/list) are always serviced.
  // This prevents the spawner from hanging if callbacks are starved by the control loop.
  ros::AsyncSpinner spinner(2);
  spinner.start();

  double control_rate_hz = 100.0;
  pnh.param("control_rate", control_rate_hz, 100.0);
  control_rate_hz = std::max(1.0, control_rate_hz);

  arm_control::ArmHW hw(nh, pnh);
  controller_manager::ControllerManager cm(&hw, nh);

  ros::Rate rate(control_rate_hz);
  ros::Time last_time = ros::Time::now();

  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();
    const ros::Duration period = now - last_time;
    last_time = now;

    hw.read(now, period);
    cm.update(now, period);
    hw.write(now, period);

    rate.sleep();
  }

  return 0;
}

