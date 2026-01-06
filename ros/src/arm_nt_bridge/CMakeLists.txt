cmake_minimum_required(VERSION 3.0.2)
project(arm_nt_bridge)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/xyz_to_arm_nt.py
    scripts/interactive_xyz_marker.py
    scripts/moveit_marker_feedback_to_arm_nt.py
    scripts/moveit_plan_to_arm_nt.py
    scripts/nt_angles_to_joint_states.py
    scripts/joint_jog_pub.py
    scripts/nt_arm_hw_bridge.py
    scripts/servo_traj_to_jtc.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



