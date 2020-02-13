#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/service/control_mode.hpp>
#include <iiwa_ros/conversions.hpp>

int main (int argc, char **argv) {

  // Initialize ROS
  ros::init(argc, argv, "ControlMode");
  ros::NodeHandle nh("~");

  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  iiwa_ros::state::CartesianPose iiwa_pose_state;
  iiwa_ros::service::ControlModeService iiwa_control_mode;

  iiwa_pose_state.init("iiwa");
  iiwa_control_mode.init("iiwa");

  // Dynamic parameter to choose the rate at wich this node should run
  double ros_rate = 0.1;
  nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

  bool job_done = true;

  while (job_done && ros::ok()) {
    if (iiwa_pose_state.isConnected()) {

      ROS_INFO_STREAM("The robot will be now set in Cartesian Impedance Mode");
      // Low stiffness only along Z.
      iiwa_control_mode.setCartesianImpedanceMode(iiwa_ros::conversions::CartesianQuantityFromFloat(1500,1500,350,300,300,300), iiwa_ros::conversions::CartesianQuantityFromFloat(0.7));
      ROS_INFO_STREAM("In 20 seconds the robot will go back to Position Control Mode");
      ros::Duration(20.0).sleep();

      iiwa_control_mode.setPositionControlMode();
      ROS_INFO_STREAM("In 20 seconds the robot will be set in Joint Impedance Mode");
      ros::Duration(20.0).sleep();

      // Low stiffness only for the 6th joint.
      iiwa_control_mode.setJointImpedanceMode(iiwa_ros::conversions::jointQuantityFromFloat(1500,1500,1500,1500,1500,0,1500), iiwa_ros::conversions::jointQuantityFromFloat(0.7));
      ROS_INFO_STREAM("In 20 seconds the robot will go back to Position Control Mode");
      ros::Duration(20.0).sleep();

      iiwa_control_mode.setPositionControlMode();

      ROS_INFO_STREAM("Job done! Shutting down...");
      job_done = false;
    }
    else {
      ROS_ERROR_STREAM("Robot is not connected...");
      ros::Duration(5.0).sleep(); // 5 seconds
    }
  }
};
