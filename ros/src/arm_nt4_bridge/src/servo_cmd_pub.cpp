#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "servo_cmd_pub");
  ros::NodeHandle nh("~");

  std::string topic = "/servo/joint_position_cmd";
  nh.param<std::string>("topic", topic, topic);

  double rate_hz = 50.0;
  nh.param("rate_hz", rate_hz, rate_hz);

  double duration_s = 5.0;
  nh.param("duration_s", duration_s, duration_s);

  std::vector<double> data{0.18, 0.36, 0.54, 0.0};
  nh.getParam("data", data);

  ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>(topic, 1);
  ros::Time start = ros::Time::now();
  ros::Rate rate(std::max(1.0, rate_hz));

  while (ros::ok() && (ros::Time::now() - start).toSec() < duration_s) {
    std_msgs::Float64MultiArray msg;
    msg.data = data;
    pub.publish(msg);
    rate.sleep();
  }

  return 0;
}






