#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <xmlrpcpp/XmlRpcValue.h>

// WPILib NT4
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>

namespace {

struct JointNT {
  // Publishers
  nt::BooleanPublisher ros_enable_pub;
  nt::DoublePublisher ros_timeout_pub;
  nt::DoublePublisher ros_pos_pub;
  nt::DoublePublisher ros_vel_pub;
  nt::DoublePublisher ros_acc_pub;
  nt::DoublePublisher ros_seq_pub;

  // Subscribers
  nt::DoubleSubscriber angle_deg_sub;

  int64_t seq = 0;
  double last_angle_rad = 0.0;
};

}  // namespace

class ArmNT4Bridge {
 public:
  explicit ArmNT4Bridge(ros::NodeHandle& nh) : nh_(nh), pnh_("~") {
    pnh_.param<std::string>("server_ip", server_ip_, std::string("10.1.14.2"));
    pnh_.param<std::string>("table", table_, std::string("Arm"));

    pnh_.param<std::string>("servo_topic", servo_topic_, std::string("/servo/joint_position_cmd"));
    pnh_.param<std::string>("servo_msg_type", servo_msg_type_, std::string("array"));  // "array" or "trajectory"
    // cmd_source:
    // - "servo": read Servo output from ~servo_topic (array/trajectory)
    // - "jtc_state": read desired pos/vel/acc from JointTrajectoryControllerState
    // - "jtc_command": read the command JointTrajectory stream that we feed into JTC (INPUT; not interpolated output)
    // - "arm_hw_command": read the final command published by RobotHW (arm_hw_node) on ~arm_hw_command_topic
    //                     (this is closest to what JTC actually writes to hardware; vel/acc are estimated if missing)
    pnh_.param<std::string>("cmd_source", cmd_source_, std::string("servo"));
    pnh_.param<std::string>("jtc_state_topic", jtc_state_topic_, std::string("/arm_controller/state"));
    pnh_.param<std::string>("jtc_command_topic", jtc_command_topic_, std::string("/arm_controller/command"));
    pnh_.param<std::string>("arm_hw_command_topic", arm_hw_command_topic_, std::string("/arm_hw/nt_command"));
    pnh_.param<std::string>("joint_states_topic", joint_states_topic_, std::string("/joint_states"));
    pnh_.param<bool>("publish_joint_states", publish_joint_states_, true);

    pnh_.param<double>("loop_hz", loop_hz_, 50.0);
    pnh_.param<double>("nt_periodic_s", nt_periodic_s_, 0.01);
    pnh_.param<double>("vel_eps_rad", vel_eps_rad_, 1e-4);
    pnh_.param<double>("max_abs_acc_rad_s2", max_abs_acc_rad_s2_, 200.0);
    pnh_.param<double>("ros_timeout_s", ros_timeout_s_, 0.15);
    pnh_.param<double>("root_keepalive_s", root_keepalive_s_, 1.0);
    pnh_.param<bool>("monitor_only", monitor_only_, false);
    pnh_.param<std::string>("active_joint", active_joint_, std::string("Both"));

    // "b" smoothing (industrial-style): apply to Servo (trajectory mode) velocity/accel only.
    // Pipeline (order matters):
    //   vel clamp -> accel clamp -> 1st order LPF on vel -> derive acc_out
    //
    // Default is conservative; tune at runtime via rosparam.
    pnh_.param<bool>("b_enable", b_enable_, true);
    pnh_.param<double>("b_vel_max", b_vel_max_rad_s_, 0.6);
    pnh_.param<double>("b_acc_max", b_acc_max_rad_s2_, 1.5);
    pnh_.param<double>("b_jerk_max", b_jerk_max_rad_s3_, 10.0);  // S-curve: jerk limit (rad/s³)
    pnh_.param<double>("b_tau", b_tau_s_, 0.05);
    // If the (filtered) incoming velocity changes by less than this, hold the previous value.
    // Helps remove tiny Servo-side quantization/jitter that shows up as "pos waviness" after integration.
    pnh_.param<double>("b_vel_input_hold_eps", b_vel_input_hold_eps_rad_s_, 0.0);
    // Apply the input-velocity "hold" only at low speed. This avoids stair-stepping at higher speeds.
    // Typical: 0.2~0.4 rad/s. Set <=0 to always apply (legacy behavior).
    pnh_.param<double>("b_vel_input_hold_max_vel", b_vel_input_hold_max_vel_rad_s_, 0.0);
    // Deceleration jerk multiplier: when slowing down / stopping, allow faster jerk so we don't keep moving
    // long after the jog command stops ("tail distance"). Accel ramp still uses b_jerk_max.
    pnh_.param<double>("b_decel_jerk_mult", b_decel_jerk_mult_, 1.0);
    // Optional position output low-pass time constant (s). 0 disables.
    // Applies AFTER integrating vel_next, with monotonic constraint (uses vel sign).
    pnh_.param<double>("b_pos_tau", b_pos_tau_s_, 0.0);
    pnh_.param<bool>("b_use_pos_ref", b_use_pos_ref_, false);  // false=pass Servo pos, true=integrate filtered vel
    pnh_.param<double>("b_pos_realign_rad", b_pos_realign_rad_, 0.05);
    pnh_.param<double>("b_pos_realign_rate", b_pos_realign_rate_, 2.0);  // Soft realign rate (rad/s per rad error)
    pnh_.param<double>("b_reset_on_dt_s", b_reset_on_dt_s_, 0.2);
    // In jtc_state mode, optionally read vel/acc from Servo instead of JTC (smoother)
    pnh_.param<bool>("b_use_servo_vel_acc", b_use_servo_vel_acc_, false);

    // joint_order: order of Float64MultiArray positions
    if (!pnh_.getParam("joint_order", joint_order_)) {
      joint_order_ = {"joint1", "joint2", "joint3", "joint4"};
    }

    // joint_map: ros_joint -> "J1"/"J2"/"J3"
    if (!pnh_.getParam("joint_map", joint_map_)) {
      joint_map_ = {{"joint1", "J1"}, {"joint2", "J2"}, {"joint3", "J3"}};
    }

    // extra_joints: list of {name, position(rad)} to append to /joint_states
    XmlRpc::XmlRpcValue extra_joints_param;
    if (pnh_.getParam("extra_joints", extra_joints_param)) {
      if (extra_joints_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < extra_joints_param.size(); ++i) {
          try {
            auto& ej = extra_joints_param[i];
            if (ej.getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;
            if (!ej.hasMember("name")) continue;
            std::string name = static_cast<std::string>(ej["name"]);
            if (name.empty()) continue;
            double pos = 0.0;
            if (ej.hasMember("position")) {
              if (ej["position"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                pos = static_cast<int>(ej["position"]);
              } else if (ej["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                pos = static_cast<double>(ej["position"]);
              }
            }
            extra_joints_parsed_.push_back({name, pos});
          } catch (...) {
            continue;
          }
        }
      }
    }

    // Init NT4 client
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
    inst_ = inst;
    // Use port=0 so NTCore selects the correct default port for the chosen mode (NT4).
    // (Hard-coding 1735 would force the legacy NT3 port.)
    inst_.SetServer(server_ip_.c_str());
    inst_.StartClient4("jetson_arm_nt4_bridge");

    // IMPORTANT: WPILib default publish periodic is 0.1s (~10Hz).
    // We need 50Hz-ish updates for smooth control / watchdog.
    const double periodic_s = std::max(0.001, nt_periodic_s_);

    nt::PubSubOptions pub_opts;
    pub_opts.periodic = periodic_s;
    pub_opts.keepDuplicates = true;
    pub_opts.sendAll = true;  // Ensure every Set() is sent, not merged

    root_table_ = inst_.GetTable(table_);
    root_active_pub_ = root_table_->GetStringTopic("ActiveJoint").Publish(pub_opts);
    root_monitor_pub_ = root_table_->GetBooleanTopic("MonitorOnly").Publish(pub_opts);

    // Setup per-joint pubs/subs
    for (const auto& kv : joint_map_) {
      const std::string& ros_joint = kv.first;
      const std::string& nt_joint = kv.second;
      auto st = root_table_->GetSubTable(nt_joint);

      JointNT j;
      j.ros_enable_pub = st->GetBooleanTopic("RosEnable").Publish(pub_opts);
      j.ros_timeout_pub = st->GetDoubleTopic("RosTimeoutS").Publish(pub_opts);
      j.ros_pos_pub = st->GetDoubleTopic("RosPosRad").Publish(pub_opts);
      j.ros_vel_pub = st->GetDoubleTopic("RosVelRadPerS").Publish(pub_opts);
      j.ros_acc_pub = st->GetDoubleTopic("RosAccRadPerS2").Publish(pub_opts);
      j.ros_seq_pub = st->GetDoubleTopic("RosSeq").Publish(pub_opts);

      j.angle_deg_sub = st->GetDoubleTopic("AngleDeg").Subscribe(std::numeric_limits<double>::quiet_NaN(),
                                                                 pub_opts);

      joints_[ros_joint] = std::move(j);
    }

    if (cmd_source_ == "jtc_state") {
      jtc_state_sub_ = nh_.subscribe(jtc_state_topic_, 1, &ArmNT4Bridge::jtcStateCb, this);
      // Also subscribe to Servo trajectory for vel/acc if b_use_servo_vel_acc is enabled
      if (b_use_servo_vel_acc_) {
        servo_traj_sub_ = nh_.subscribe(servo_topic_, 1, &ArmNT4Bridge::servoTrajCb, this);
        ROS_INFO("jtc_state mode with b_use_servo_vel_acc=true: also subscribing to Servo for vel/acc");
      }
    } else if (cmd_source_ == "jtc_command") {
      jtc_command_sub_ = nh_.subscribe(jtc_command_topic_, 1, &ArmNT4Bridge::jtcCommandCb, this);
    } else if (cmd_source_ == "arm_hw_command") {
      arm_hw_command_sub_ =
          nh_.subscribe(arm_hw_command_topic_, 1, &ArmNT4Bridge::armHwCommandCb, this);
    } else {
    if (servo_msg_type_ == "trajectory") {
      servo_traj_sub_ = nh_.subscribe(servo_topic_, 1, &ArmNT4Bridge::servoTrajCb, this);
    } else {
      servo_array_sub_ = nh_.subscribe(servo_topic_, 1, &ArmNT4Bridge::servoArrayCb, this);
    }
    }
    if (publish_joint_states_) {
    js_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_states_topic_, 1);
    }

    ROS_INFO_STREAM("arm_nt4_bridge_node ready. server_ip=" << server_ip_ << " table=" << table_
                                                            << " cmd_source=" << cmd_source_
                                                            << " servo_topic=" << servo_topic_
                                                            << " jtc_state_topic=" << jtc_state_topic_
                                                            << " arm_hw_command_topic=" << arm_hw_command_topic_
                                                            << " joint_states_topic=" << joint_states_topic_
                                                            << " publish_joint_states=" << (publish_joint_states_ ? "true" : "false"));
  }

  void spin() {
    ros::Rate rate(std::max(1.0, loop_hz_));
    while (ros::ok()) {
      ros::spinOnce();
      writeNT();
      publishJointStates();
      rate.sleep();
    }
  }

 private:
  void servoArrayCb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    last_servo_positions_ = msg->data;
    last_servo_stamp_ = ros::Time::now();
  }

  void servoTrajCb(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
    last_traj_joint_names_ = msg->joint_names;
    if (!msg->points.empty()) {
      const auto& p = msg->points.front();
      last_traj_positions_ = p.positions;
      last_traj_velocities_ = p.velocities;
      last_traj_accelerations_ = p.accelerations;
    } else {
      last_traj_positions_.clear();
      last_traj_velocities_.clear();
      last_traj_accelerations_.clear();
    }
    last_servo_stamp_ = ros::Time::now();
  }

  void jtcStateCb(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg) {
    // Cache desired state as name->value maps (robust to ordering differences)
    last_jtc_des_pos_.clear();
    last_jtc_des_vel_.clear();
    last_jtc_des_acc_.clear();
    for (size_t i = 0; i < msg->joint_names.size(); ++i) {
      const std::string& name = msg->joint_names[i];
      if (i < msg->desired.positions.size()) last_jtc_des_pos_[name] = msg->desired.positions[i];
      if (i < msg->desired.velocities.size()) last_jtc_des_vel_[name] = msg->desired.velocities[i];
      if (i < msg->desired.accelerations.size()) last_jtc_des_acc_[name] = msg->desired.accelerations[i];
    }
    last_servo_stamp_ = ros::Time::now();  // reuse "have_cmd" time gate
  }

  void jtcCommandCb(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
    last_jtc_cmd_joint_names_ = msg->joint_names;
    if (!msg->points.empty()) {
      const auto& p = msg->points.front();
      last_jtc_cmd_positions_ = p.positions;
      last_jtc_cmd_velocities_ = p.velocities;
      last_jtc_cmd_accelerations_ = p.accelerations;
    } else {
      last_jtc_cmd_positions_.clear();
      last_jtc_cmd_velocities_.clear();
      last_jtc_cmd_accelerations_.clear();
    }
    last_servo_stamp_ = ros::Time::now();  // reuse "have_cmd" time gate
  }

  void armHwCommandCb(const sensor_msgs::JointState::ConstPtr& msg) {
    last_hw_cmd_joint_names_ = msg->name;
    last_hw_cmd_positions_ = msg->position;
    // RobotHW output typically does not include vel/acc; if present, we'll forward.
    last_hw_cmd_velocities_ = msg->velocity;
    last_hw_cmd_efforts_ = msg->effort;
    last_servo_stamp_ = ros::Time::now();  // reuse "have_cmd" time gate
  }

  void writeNT() {
    const double now_s = ros::Time::now().toSec();
    const double dt_nominal = 1.0 / std::max(1.0, loop_hz_);
    const double dt_meas =
        (last_write_time_s_ > 0.0) ? std::max(1e-4, now_s - last_write_time_s_) : dt_nominal;
    // Use a stable dt to avoid enormous acc spikes from timing jitter / very small dt.
    const double dt = std::max(dt_nominal, dt_meas);
    last_write_time_s_ = now_s;

    // Lightweight debug to verify callback + NT connectivity
    auto it_dbg = joints_.find("joint2");
    const int64_t j2_seq = (it_dbg != joints_.end()) ? it_dbg->second.seq : -1;
    const size_t cmd_len =
        (cmd_source_ == "jtc_state") ? last_jtc_des_pos_.size()
        : (cmd_source_ == "jtc_command") ? last_jtc_cmd_positions_.size()
        : (cmd_source_ == "arm_hw_command") ? last_hw_cmd_positions_.size()
        : (servo_msg_type_ == "trajectory" ? last_traj_positions_.size() : last_servo_positions_.size());
    ROS_INFO_THROTTLE(1.0, "arm_nt4_bridge: nt4_connected=%d cmd_source=%s servo_msg_type=%s cmd_size=%zu j2_seq=%ld",
                      static_cast<int>(inst_.IsConnected()), cmd_source_.c_str(), servo_msg_type_.c_str(),
                      cmd_len, static_cast<long>(j2_seq));

    // Root keepalive (low rate)
    if ((now_s - last_root_keepalive_s_) >= root_keepalive_s_) {
      last_root_keepalive_s_ = now_s;
      root_active_pub_.Set(active_joint_);
      root_monitor_pub_.Set(monitor_only_);
    }

    // Always keep RosEnable/RosTimeoutS alive and push setpoints at loop rate
    const bool have_cmd =
        (cmd_source_ == "jtc_state") ? (!last_jtc_des_pos_.empty())
        : (cmd_source_ == "jtc_command") ? (!last_jtc_cmd_joint_names_.empty() && !last_jtc_cmd_positions_.empty())
        : (cmd_source_ == "arm_hw_command") ? (!last_hw_cmd_joint_names_.empty() && !last_hw_cmd_positions_.empty())
        : ((servo_msg_type_ == "trajectory") ? (!last_traj_joint_names_.empty() && !last_traj_positions_.empty())
                                             : (!last_servo_positions_.empty()));
    if (!have_cmd) {
      // No commands received yet; still keep enable/timeout alive so RoboRIO shows state.
      for (auto& kv : joints_) {
        kv.second.ros_enable_pub.Set(true);
        kv.second.ros_timeout_pub.Set(ros_timeout_s_);
      }
      inst_.Flush();
      return;
    }

    // Build command maps
    std::map<std::string, double> cmd_pos;
    std::map<std::string, double> cmd_vel;
    std::map<std::string, double> cmd_acc;

    if (cmd_source_ == "jtc_state") {
      cmd_pos = last_jtc_des_pos_;
      cmd_vel = last_jtc_des_vel_;
      cmd_acc = last_jtc_des_acc_;
    } else if (cmd_source_ == "jtc_command") {
      for (size_t i = 0; i < last_jtc_cmd_joint_names_.size(); ++i) {
        const std::string& name = last_jtc_cmd_joint_names_[i];
        if (i < last_jtc_cmd_positions_.size()) cmd_pos[name] = last_jtc_cmd_positions_[i];
        if (i < last_jtc_cmd_velocities_.size()) cmd_vel[name] = last_jtc_cmd_velocities_[i];
        if (i < last_jtc_cmd_accelerations_.size()) cmd_acc[name] = last_jtc_cmd_accelerations_[i];
      }
    } else if (cmd_source_ == "arm_hw_command") {
      for (size_t i = 0; i < last_hw_cmd_joint_names_.size(); ++i) {
        const std::string& name = last_hw_cmd_joint_names_[i];
        if (i < last_hw_cmd_positions_.size()) cmd_pos[name] = last_hw_cmd_positions_[i];
        if (i < last_hw_cmd_velocities_.size()) cmd_vel[name] = last_hw_cmd_velocities_[i];
        // No acceleration field in JointState; if needed we'll estimate below.
      }
    } else {
    if (servo_msg_type_ == "trajectory") {
      // Prefer name-based mapping (robust to ordering differences)
      for (size_t i = 0; i < last_traj_joint_names_.size(); ++i) {
        const std::string& name = last_traj_joint_names_[i];
        if (i < last_traj_positions_.size()) cmd_pos[name] = last_traj_positions_[i];
        if (i < last_traj_velocities_.size()) cmd_vel[name] = last_traj_velocities_[i];
        if (i < last_traj_accelerations_.size()) cmd_acc[name] = last_traj_accelerations_[i];
      }
    } else {
      // Array mode: map by joint_order
      for (size_t i = 0; i < joint_order_.size() && i < last_servo_positions_.size(); ++i) {
        cmd_pos[joint_order_[i]] = last_servo_positions_[i];
        }
      }
    }

    for (auto& kv : joints_) {
      const std::string& ros_joint = kv.first;
      JointNT& j = kv.second;

      j.ros_enable_pub.Set(true);
      j.ros_timeout_pub.Set(ros_timeout_s_);

      auto it = cmd_pos.find(ros_joint);
      // Always publish full setpoint contract every frame:
      // RosPosRad + RosVelRadPerS + RosAccRadPerS2 + RosSeq
      //
      // If we didn't receive a new position for this joint, hold last commanded pos.
      const double prev_pos = j_last_cmd_pos_[ros_joint];
      const double prev_vel = j_last_cmd_vel_[ros_joint];
      double pos = prev_pos;
      if (it != cmd_pos.end()) {
        pos = it->second;
      }
      // Vel/acc:
      // - cmd_source=jtc_state: forward JTC desired vel/acc (no guessing)
      // - cmd_source=arm_hw_command: prefer provided vel (if any), otherwise estimate from pos
      // - servo trajectory mode: forward Servo-provided vel/acc (no guessing)
      // - array mode: estimate via finite differences (legacy)
      double vel = 0.0;
      double acc = 0.0;
      if (cmd_source_ == "jtc_state" || cmd_source_ == "jtc_command") {
        // Position comes from JTC (already smooth 100Hz interpolated).
        // Vel/Acc: choose source based on b_use_servo_vel_acc.
        double vel_raw = 0.0;
        double acc_raw = 0.0;
        if (b_use_servo_vel_acc_) {
          // Use Servo's vel/acc (more stable, 50Hz update from Servo)
          for (size_t k = 0; k < last_traj_joint_names_.size(); ++k) {
            if (last_traj_joint_names_[k] == ros_joint) {
              if (k < last_traj_velocities_.size()) vel_raw = last_traj_velocities_[k];
              if (k < last_traj_accelerations_.size()) acc_raw = last_traj_accelerations_[k];
              break;
            }
          }
        } else {
          // Use JTC's vel/acc (may have ripple from trajectory switching)
          vel_raw = (cmd_vel.count(ros_joint) ? cmd_vel[ros_joint] : 0.0);
          acc_raw = (cmd_acc.count(ros_joint) ? cmd_acc[ros_joint] : 0.0);
        }

        if (!b_enable_) {
          vel = vel_raw;
          acc = acc_raw;
        } else {
          // ===== S-CURVE RAMP GENERATOR =====
          // Instead of a trapezoidal velocity profile (constant acceleration = square wave),
          // we use an S-curve profile where acceleration ramps linearly (jerk-limited).
          // This makes acceleration continuous, eliminating the "jerk" at start/stop.

          // Lazy init per-joint state.
          if (b_last_vel_filt_.find(ros_joint) == b_last_vel_filt_.end()) {
            b_last_vel_filt_[ros_joint] = 0.0;
          }
          if (b_last_acc_.find(ros_joint) == b_last_acc_.end()) {
            b_last_acc_[ros_joint] = 0.0;
          }
          // Second-order low-pass filter state (for input velocity)
          if (b_lpf2_x1_.find(ros_joint) == b_lpf2_x1_.end()) {
            b_lpf2_x1_[ros_joint] = 0.0;
            b_lpf2_x2_[ros_joint] = 0.0;
            b_lpf2_y1_[ros_joint] = 0.0;
            b_lpf2_y2_[ros_joint] = 0.0;
          }
          // Reset if loop stalls.
          if (b_reset_on_dt_s_ > 0.0 && dt_meas > b_reset_on_dt_s_) {
            b_last_vel_filt_[ros_joint] = vel_raw;
            b_last_acc_[ros_joint] = 0.0;
            b_lpf2_x1_[ros_joint] = vel_raw;
            b_lpf2_x2_[ros_joint] = vel_raw;
            b_lpf2_y1_[ros_joint] = vel_raw;
            b_lpf2_y2_[ros_joint] = vel_raw;
          }

          const double dt_ctrl = dt_nominal;
          
          // ===== SECOND-ORDER BUTTERWORTH LOW-PASS FILTER on input velocity =====
          // This provides -40dB/decade attenuation (vs -20dB for first-order)
          // Much better at removing high-frequency noise from Servo velocity
          const double fs = 1.0 / dt_ctrl;  // Sample rate
          const double fc = 1.0 / (2.0 * M_PI * b_tau_s_);  // Cutoff frequency from tau
          const double omega = std::tan(M_PI * fc / fs);
          const double omega2 = omega * omega;
          const double c = 1.0 + std::sqrt(2.0) * omega + omega2;
          const double b0 = omega2 / c;
          const double b1 = 2.0 * b0;
          const double b2 = b0;
          const double a1 = 2.0 * (omega2 - 1.0) / c;
          const double a2 = (1.0 - std::sqrt(2.0) * omega + omega2) / c;
          
          // Apply filter: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
          const double x0 = vel_raw;
          const double x1 = b_lpf2_x1_[ros_joint];
          const double x2 = b_lpf2_x2_[ros_joint];
          const double y1 = b_lpf2_y1_[ros_joint];
          const double y2 = b_lpf2_y2_[ros_joint];
          const double y0 = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2;
          
          // Update filter state
          b_lpf2_x2_[ros_joint] = x1;
          b_lpf2_x1_[ros_joint] = x0;
          b_lpf2_y2_[ros_joint] = y1;
          b_lpf2_y1_[ros_joint] = y0;
          
          double vel_filtered = y0;  // Use filtered velocity as input to S-curve
          // Optional small-change hold to kill tiny jitter
          const bool hold_allowed =
              (b_vel_input_hold_max_vel_rad_s_ <= 0.0) ||
              (std::abs(vel_filtered) <= b_vel_input_hold_max_vel_rad_s_);
          if (hold_allowed && b_vel_input_hold_eps_rad_s_ > 0.0) {
            if (b_last_vel_in_.find(ros_joint) == b_last_vel_in_.end()) {
              b_last_vel_in_[ros_joint] = vel_filtered;
            } else {
              const double last = b_last_vel_in_[ros_joint];
              if (std::abs(vel_filtered - last) < b_vel_input_hold_eps_rad_s_) {
                vel_filtered = last;
              } else {
                b_last_vel_in_[ros_joint] = vel_filtered;
              }
            }
          }
          
          const double vel_current = b_last_vel_filt_[ros_joint];
          const double acc_current = b_last_acc_[ros_joint];

          // Target velocity (clamped to max speed)
          const double v_max = std::max(0.0, b_vel_max_rad_s_);
          double vel_target = std::clamp(vel_filtered, -v_max, v_max);
          const double vel_cmd_base = vel_target;  // IMPORTANT: command before any realign correction

          // ===== REALIGN as a *velocity bias* (NOT a direct position jump) =====
          // Key idea: keep position purely as integral of the smoothed velocity.
          // Realign produces a small correction velocity that is jerk-limited, then added into vel_target.
          // This avoids injecting ripple directly into pos_ref.
          if (b_use_pos_ref_ && b_pos_realign_rate_ > 0.0) {
            if (b_last_pos_ref_.find(ros_joint) == b_last_pos_ref_.end()) {
              b_last_pos_ref_[ros_joint] = pos;  // init to JTC position
            }
            if (b_last_correction_vel_.find(ros_joint) == b_last_correction_vel_.end()) {
              b_last_correction_vel_[ros_joint] = 0.0;
            }

            // Predict where integrated position would be if we keep current velocity.
            const double pos_ref_pred = b_last_pos_ref_[ros_joint] + vel_current * dt_ctrl;
            const double pos_error = pos - pos_ref_pred;  // JTC pos - predicted integrated pos
            // IMPORTANT:
            // Use the *base commanded* speed (vel_cmd_base) to gate/scale realign.
            // If we gate on vel_current, realign can keep pulling the setpoint backwards after
            // the jog command stops, which looks like "0.8 -> 0.73" drift.
            // If we gate on vel_target (after adding correction), the correction can become self-sustaining
            // and keep moving the setpoint even after the jog command stops.
            const double vel_cmd_abs = std::abs(vel_cmd_base);

            double correction_vel = b_last_correction_vel_[ros_joint];
            // Only realign while command is actively moving (prevents post-stop setpoint drift).
            if (vel_cmd_abs > 0.02) {
              // Cap realign velocity: allow up to 0.30 rad/s, scaled with motion speed.
              const double max_correction_vel =
                  std::min(0.30, std::max(0.01, vel_cmd_abs * b_pos_realign_rate_));
              const double correction_vel_target =
                  std::clamp(pos_error * 0.25, -max_correction_vel, max_correction_vel);

              // Jerk for realign: responsive but still smooth (slightly faster than before).
              const double correction_jerk = std::max(0.02, vel_cmd_abs * 0.04);
              const double max_dvel = correction_jerk * dt_ctrl;
              const double dvel =
                  std::clamp(correction_vel_target - correction_vel, -max_dvel, max_dvel);
              correction_vel += dvel;
            } else {
              // When nearly stopped, decay correction velocity to zero smoothly.
              const double decay_rate = 5.0;  // rad/s^2
              const double max_decay = decay_rate * dt_ctrl;
              if (std::abs(correction_vel) < max_decay) {
                correction_vel = 0.0;
              } else {
                correction_vel -= std::copysign(max_decay, correction_vel);
              }
            }
            b_last_correction_vel_[ros_joint] = correction_vel;
            // Apply correction to the *base* command, so correction cannot "keep itself alive".
            vel_target = std::clamp(vel_cmd_base + correction_vel, -v_max, v_max);

            // Hard realign only if error is very large (safety): snap integrated pos to JTC pos.
            if (std::abs(pos_error) > std::max(0.0, b_pos_realign_rad_)) {
              b_last_pos_ref_[ros_joint] = pos;
              b_last_correction_vel_[ros_joint] = 0.0;
            }
          }

          // S-curve: acceleration ramps toward target at rate ±jerk_max
          const double a_max = std::max(0.0, b_acc_max_rad_s2_);
          const double j_base = std::max(0.0, b_jerk_max_rad_s3_);
          const double vel_error = vel_target - vel_current;

          // Compute desired acceleration to reach target velocity
          // Use a simple proportional approach with clamping
          double acc_desired = vel_error / std::max(1e-4, dt_ctrl);
          acc_desired = std::clamp(acc_desired, -a_max, a_max);

          // If we're decelerating (including stopping), allow higher jerk so the tail distance is smaller.
          const bool decel_phase =
              (std::abs(vel_target) < std::abs(vel_current) - 1e-6) ||
              ((vel_target * vel_current) < 0.0);
          double j_max = j_base;
          if (decel_phase && b_decel_jerk_mult_ > 1.0) {
            j_max = j_base * b_decel_jerk_mult_;
          }

          // Ramp acceleration toward desired acceleration at rate ±jerk_max
          const double max_acc_step = j_max * dt_ctrl;
          const double acc_error = acc_desired - acc_current;

          double acc_next;
          if (acc_error > max_acc_step) {
            acc_next = acc_current + max_acc_step;
          } else if (acc_error < -max_acc_step) {
            acc_next = acc_current - max_acc_step;
          } else {
            acc_next = acc_desired;
          }

          // Update velocity using the new acceleration
          double vel_next = vel_current + acc_next * dt_ctrl;
          vel_next = std::clamp(vel_next, -v_max, v_max);

          // ===== STOP HARD-LOCK: when command & current are both near zero, clamp to zero =====
          const double stop_eps = 0.03;
          if (std::abs(vel_target) < stop_eps && std::abs(vel_next) < stop_eps) {
            vel_next = 0.0;
            acc_next = 0.0;
          }

          // If velocity overshoots target, snap to target and zero acceleration
          if ((vel_error > 0 && vel_next > vel_target) ||
              (vel_error < 0 && vel_next < vel_target) ||
              (std::abs(vel_error) < 1e-4 && std::abs(acc_next) < 0.01)) {
            vel_next = vel_target;
            acc_next = 0.0;
          }

          b_last_vel_filt_[ros_joint] = vel_next;
          b_last_acc_[ros_joint] = acc_next;
          vel = vel_next;
          acc = acc_next;

          // ===== POSITION: integrate velocity for monotonic output =====
          // JTC's position can have backward jumps due to trajectory switching.
          // Integrating the smoothed velocity ensures monotonic position.
          if (b_use_pos_ref_) {
            if (b_last_pos_ref_.find(ros_joint) == b_last_pos_ref_.end()) {
              b_last_pos_ref_[ros_joint] = pos;  // init to JTC position
            }
            if (b_last_pos_out_.find(ros_joint) == b_last_pos_out_.end()) {
              b_last_pos_out_[ros_joint] = b_last_pos_ref_[ros_joint];
            }
            // Reset if loop stalls
            if (b_reset_on_dt_s_ > 0.0 && dt_meas > b_reset_on_dt_s_) {
              b_last_pos_ref_[ros_joint] = pos;
              b_last_pos_out_[ros_joint] = pos;
            }

            // NOTE: realign is handled above as a velocity bias. Here we only integrate velocity.
            double pos_ref = b_last_pos_ref_[ros_joint] + vel_next * dt_ctrl;
            b_last_pos_ref_[ros_joint] = pos_ref;

            // Optional extra smoothing on position output (helps remove residual micro-waviness).
            // Keep it monotonic by clamping based on velocity sign.
            if (b_pos_tau_s_ > 0.0) {
              const double alpha = std::clamp(dt_ctrl / std::max(1e-6, b_pos_tau_s_), 0.0, 1.0);
              double pos_out = b_last_pos_out_[ros_joint] + alpha * (pos_ref - b_last_pos_out_[ros_joint]);
              if (vel_next >= 0.0) {
                // moving forward: never go backwards, never exceed setpoint
                pos_out = std::min(pos_ref, std::max(b_last_pos_out_[ros_joint], pos_out));
              } else {
                // moving backward: never go forwards, never go below setpoint
                pos_out = std::max(pos_ref, std::min(b_last_pos_out_[ros_joint], pos_out));
              }
              b_last_pos_out_[ros_joint] = pos_out;
              pos = pos_out;
            } else {
              // no extra smoothing
              b_last_pos_out_[ros_joint] = pos_ref;
              pos = pos_ref;
            }
          }

          if (ros_joint == "joint2") {
            ROS_INFO_THROTTLE(0.5,
                              "arm_nt4_bridge S-curve(jtc%s): j2 pos=%.4f vel=%.3f acc=%.3f",
                              b_use_servo_vel_acc_ ? "+servo" : "", pos, vel, acc);
          }
        }
      } else if (cmd_source_ == "arm_hw_command") {
        const double dpos = pos - prev_pos;
        if (cmd_vel.count(ros_joint)) {
          vel = cmd_vel[ros_joint];
          acc = (vel - prev_vel) / dt;
        } else {
          vel = dpos / dt;
          acc = (vel - prev_vel) / dt;
        }
        if (std::abs(dpos) < vel_eps_rad_) {
          vel = 0.0;
          acc = 0.0;
        }
        if (std::abs(acc) > max_abs_acc_rad_s2_) {
          acc = (acc > 0.0) ? max_abs_acc_rad_s2_ : -max_abs_acc_rad_s2_;
        }
      } else if (servo_msg_type_ == "trajectory") {
        const double vel_raw = (cmd_vel.count(ros_joint) ? cmd_vel[ros_joint] : 0.0);
        const double acc_raw = (cmd_acc.count(ros_joint) ? cmd_acc[ros_joint] : 0.0);

        // Lazy init per-joint smoothing states.
        if (b_last_vel_filt_.find(ros_joint) == b_last_vel_filt_.end()) {
          b_last_vel_filt_[ros_joint] = 0.0;
        }
        if (b_last_pos_ref_.find(ros_joint) == b_last_pos_ref_.end()) {
          b_last_pos_ref_[ros_joint] = pos;
        }

        // If loop stalls, reset smoothing state to avoid a big catch-up step.
        if (b_reset_on_dt_s_ > 0.0 && dt_meas > b_reset_on_dt_s_) {
          b_last_vel_filt_[ros_joint] = vel_raw;
          b_last_pos_ref_[ros_joint] = pos;
        }

        if (!b_enable_) {
          // b disabled: pass-through Servo-provided vel/acc.
          vel = vel_raw;
          acc = acc_raw;
        } else {
          // ===== S-CURVE RAMP GENERATOR (Servo mode) =====
          // Same as jtc_state: use jerk-limited S-curve for smooth acceleration.
          if (b_last_acc_.find(ros_joint) == b_last_acc_.end()) {
            b_last_acc_[ros_joint] = 0.0;
          }

          const double dt_ctrl = dt_nominal;
          const double vel_current = b_last_vel_filt_[ros_joint];
          const double acc_current = b_last_acc_[ros_joint];

          // Target velocity (clamped)
          const double v_max = std::max(0.0, b_vel_max_rad_s_);
          const double vel_target = std::clamp(vel_raw, -v_max, v_max);

          // S-curve: acceleration ramps toward target at rate ±jerk_max
          const double a_max = std::max(0.0, b_acc_max_rad_s2_);
          const double j_base = std::max(0.0, b_jerk_max_rad_s3_);
          const double vel_error = vel_target - vel_current;

          // Compute desired acceleration
          double acc_desired = vel_error / std::max(1e-4, dt_ctrl);
          acc_desired = std::clamp(acc_desired, -a_max, a_max);

          const bool decel_phase =
              (std::abs(vel_target) < std::abs(vel_current) - 1e-6) ||
              ((vel_target * vel_current) < 0.0);
          double j_max = j_base;
          if (decel_phase && b_decel_jerk_mult_ > 1.0) {
            j_max = j_base * b_decel_jerk_mult_;
          }

          // Ramp acceleration toward desired at rate ±jerk_max
          const double max_acc_step = j_max * dt_ctrl;
          const double acc_error = acc_desired - acc_current;

          double acc_next;
          if (acc_error > max_acc_step) {
            acc_next = acc_current + max_acc_step;
          } else if (acc_error < -max_acc_step) {
            acc_next = acc_current - max_acc_step;
          } else {
            acc_next = acc_desired;
          }

          // Update velocity
          double vel_next = vel_current + acc_next * dt_ctrl;
          vel_next = std::clamp(vel_next, -v_max, v_max);

          // Stop hard-lock (servo trajectory path too)
          const double stop_eps = 0.03;
          if (std::abs(vel_target) < stop_eps && std::abs(vel_next) < stop_eps) {
            vel_next = 0.0;
            acc_next = 0.0;
          }

          // Snap to target if close
          if ((vel_error > 0 && vel_next > vel_target) ||
              (vel_error < 0 && vel_next < vel_target) ||
              (std::abs(vel_error) < 1e-4 && std::abs(acc_next) < 0.01)) {
            vel_next = vel_target;
            acc_next = 0.0;
          }

          // Position: integrate velocity (always use pos_ref in servo mode for consistency)
          double pos_ref = b_last_pos_ref_[ros_joint] + vel_next * dt_ctrl;
          // Realign to Servo position if drift exceeds threshold
          if (std::abs(pos - pos_ref) > std::max(0.0, b_pos_realign_rad_)) {
            pos_ref = pos;
          }
          b_last_pos_ref_[ros_joint] = pos_ref;
          pos = pos_ref;

          b_last_vel_filt_[ros_joint] = vel_next;
          b_last_acc_[ros_joint] = acc_next;
          vel = vel_next;
          acc = acc_next;

          if (ros_joint == "joint2") {
            ROS_INFO_THROTTLE(0.5,
                              "arm_nt4_bridge S-curve(servo): j2 vel_target=%.3f vel_out=%.3f acc_out=%.3f",
                              vel_target, vel, acc);
          }
        }
      } else {
        const double dpos = pos - prev_pos;
        vel = dpos / dt;
        acc = (vel - prev_vel) / dt;

        // If position isn't changing (or changes are tiny), force vel/acc to 0
        if (std::abs(dpos) < vel_eps_rad_) {
          vel = 0.0;
          acc = 0.0;
        }
        // Clamp acceleration
        if (std::abs(acc) > max_abs_acc_rad_s2_) {
          acc = (acc > 0.0) ? max_abs_acc_rad_s2_ : -max_abs_acc_rad_s2_;
        }
      }

      j_last_cmd_pos_[ros_joint] = pos;
      j_last_cmd_vel_[ros_joint] = vel;

      j.ros_pos_pub.Set(pos);
      j.ros_vel_pub.Set(vel);
      j.ros_acc_pub.Set(acc);

      if (ros_joint == "joint2") {
        ROS_INFO_THROTTLE(0.5, "arm_nt4_bridge: joint2 setpoint pos=%.5f vel=%.3f acc=%.3f (cmd_source=%s)",
                          pos, vel, acc, cmd_source_.c_str());
      }

      j.seq += 1;
      j.ros_seq_pub.Set(static_cast<double>(j.seq));
    }

    // Flush each cycle to minimize latency/jitter
    inst_.Flush();
  }

  void publishJointStates() {
    if (!publish_joint_states_) {
      return;
    }
    sensor_msgs::JointState js;
    js.header.stamp = ros::Time::now();

    // read AngleDeg -> rad
    for (const auto& kv : joint_map_) {
      const std::string& ros_joint = kv.first;
      auto it = joints_.find(ros_joint);
      if (it == joints_.end()) continue;

      const double deg = it->second.angle_deg_sub.Get();
      double rad = it->second.last_angle_rad;
      if (!std::isnan(deg)) {
        rad = deg * M_PI / 180.0;
        it->second.last_angle_rad = rad;
      }

      // IMPORTANT: always publish full joint state for Servo (even if feedback is temporarily missing),
      // otherwise Servo may time out waiting for current state.
      js.name.push_back(ros_joint);
      js.position.push_back(rad);
    }

    // Append fixed extra joints if configured
    for (const auto& ej : extra_joints_parsed_) {
      const std::string& name = ej.first;
      const double pos = ej.second;
      bool exists = false;
      for (const auto& n : js.name) {
        if (n == name) {
          exists = true;
          break;
        }
      }
      if (!exists) {
        js.name.push_back(name);
        js.position.push_back(pos);
      }
    }

    if (!js.name.empty()) {
      js_pub_.publish(js);
    }
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string server_ip_;
  std::string table_;
  std::string servo_topic_;
  std::string servo_msg_type_;
  std::string cmd_source_;
  std::string jtc_state_topic_;
  std::string jtc_command_topic_;
  std::string arm_hw_command_topic_;
  std::string joint_states_topic_;
  bool publish_joint_states_{true};

  double loop_hz_{50.0};
  double nt_periodic_s_{0.01};
  double vel_eps_rad_{1e-4};
  double max_abs_acc_rad_s2_{200.0};
  double ros_timeout_s_{0.15};
  double root_keepalive_s_{1.0};
  bool monitor_only_{false};
  std::string active_joint_{"Both"};

  std::vector<std::string> joint_order_;
  std::map<std::string, std::string> joint_map_;
  std::vector<std::pair<std::string, double>> extra_joints_parsed_;

  nt::NetworkTableInstance inst_;
  std::shared_ptr<nt::NetworkTable> root_table_;
  nt::StringPublisher root_active_pub_;
  nt::BooleanPublisher root_monitor_pub_;

  std::map<std::string, JointNT> joints_;

  ros::Subscriber servo_array_sub_;
  ros::Subscriber servo_traj_sub_;
  ros::Subscriber jtc_state_sub_;
  ros::Subscriber jtc_command_sub_;
  ros::Subscriber arm_hw_command_sub_;
  ros::Publisher js_pub_;

  std::vector<double> last_servo_positions_;
  ros::Time last_servo_stamp_;
  double last_root_keepalive_s_{0.0};
  double last_write_time_s_{0.0};

  // Trajectory cache (for "trajectory" mode)
  std::vector<std::string> last_traj_joint_names_;
  std::vector<double> last_traj_positions_;
  std::vector<double> last_traj_velocities_;
  std::vector<double> last_traj_accelerations_;

  // JTC desired cache (for cmd_source=jtc)
  std::map<std::string, double> last_jtc_des_pos_;
  std::map<std::string, double> last_jtc_des_vel_;
  std::map<std::string, double> last_jtc_des_acc_;

  // JTC command cache (for cmd_source=jtc_command)
  std::vector<std::string> last_jtc_cmd_joint_names_;
  std::vector<double> last_jtc_cmd_positions_;
  std::vector<double> last_jtc_cmd_velocities_;
  std::vector<double> last_jtc_cmd_accelerations_;

  // RobotHW command cache (for cmd_source=arm_hw_command)
  std::vector<std::string> last_hw_cmd_joint_names_;
  std::vector<double> last_hw_cmd_positions_;
  std::vector<double> last_hw_cmd_velocities_;
  std::vector<double> last_hw_cmd_efforts_;

  // For publishing vel/acc when Servo output is positions-only
  std::map<std::string, double> j_last_cmd_pos_;
  std::map<std::string, double> j_last_cmd_vel_;

  // "b" smoothing state (per joint; used for cmd_source=servo && servo_msg_type=trajectory)
  bool b_enable_{true};
  double b_vel_max_rad_s_{0.6};
  double b_acc_max_rad_s2_{1.5};
  double b_jerk_max_rad_s3_{10.0};  // S-curve: jerk limit (rad/s³)
  double b_tau_s_{0.05};
  double b_vel_input_hold_eps_rad_s_{0.0};
  double b_vel_input_hold_max_vel_rad_s_{0.0};
  double b_decel_jerk_mult_{1.0};
  double b_pos_tau_s_{0.0};
  bool b_use_pos_ref_{false};
  double b_pos_realign_rad_{0.05};
  double b_pos_realign_rate_{2.0};  // Soft realign rate (rad/s per rad error)
  double b_reset_on_dt_s_{0.2};
  bool b_use_servo_vel_acc_{false};  // In jtc_state mode, use Servo's vel/acc instead of JTC's
  std::map<std::string, double> b_last_vel_filt_;
  std::map<std::string, double> b_last_acc_;  // S-curve: track current acceleration
  std::map<std::string, double> b_last_pos_ref_;
  std::map<std::string, double> b_last_pos_out_;  // Smoothed position output
  std::map<std::string, double> b_last_correction_vel_;  // S-curve smoothed realign velocity
  std::map<std::string, double> b_last_vel_in_;  // (filtered) input velocity after optional hold
  // Second-order Butterworth LPF state (for input velocity)
  std::map<std::string, double> b_lpf2_x1_;  // x[n-1]
  std::map<std::string, double> b_lpf2_x2_;  // x[n-2]
  std::map<std::string, double> b_lpf2_y1_;  // y[n-1]
  std::map<std::string, double> b_lpf2_y2_;  // y[n-2]
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "arm_nt4_bridge");
  ros::NodeHandle nh;
  ArmNT4Bridge node(nh);
  node.spin();
  return 0;
}


