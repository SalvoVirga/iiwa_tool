#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/state/joint_position.hpp>
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/joint_position.hpp>
#include <iiwa_ros/service/time_to_destination.hpp>
#include <cmath>

// getTimeToDestination() can also return negative values and the info from the cabinet take some milliseconds to update once the motion is started.
// That means that if you call getTimeToDestination() right after you set a target pose, you might get the wrong info (e.g. a negative number).
// This function tried to call getTimeToDestination() until something meaningful is obtained or until a maximum amount of time passed.
void sleepForMotion(iiwa_ros::service::TimeToDestinationService& iiwa, const double maxSleepTime) {
  double ttd = iiwa.getTimeToDestination();
  ros::Time start_wait = ros::Time::now();
  while (ttd < 0.0 && (ros::Time::now() - start_wait) < ros::Duration(maxSleepTime)) {
    ros::Duration(0.5).sleep();
    ttd = iiwa.getTimeToDestination();
  }
  if (ttd > 0.0) {
    ROS_INFO_STREAM("Sleeping for " << ttd << " seconds.");
    ros::Duration(ttd).sleep();
  }
}

int main (int argc, char **argv) {

	// Initialize ROS
	ros::init(argc, argv, "CommandRobot");
	ros::NodeHandle nh("~");

  iiwa_ros::state::CartesianPose iiwa_pose_state;
  iiwa_ros::state::JointPosition iiwa_joint_state;
  iiwa_ros::command::CartesianPose iiwa_pose_command;
  iiwa_ros::command::JointPosition iiwa_joint_command;
  iiwa_ros::service::TimeToDestinationService iiwa_time_destination;

  iiwa_pose_state.init("iiwa");
  iiwa_pose_command.init("iiwa");
  iiwa_joint_state.init("iiwa");
  iiwa_joint_command.init("iiwa");
  iiwa_time_destination.init("iiwa");

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  bool use_cartesian_command;
	nh.param("use_cartesian_command", use_cartesian_command, true);

	// Dynamic parameter to choose the rate at wich this node should run
  double ros_rate;
	nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
	ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

  iiwa_msgs::CartesianPose command_cartesian_position;
  iiwa_msgs::JointPosition command_joint_position;
  bool new_pose = false, motion_done = false;

	int direction = 1;

	while (ros::ok()) {
    if (iiwa_pose_state.isConnected()) {

			if (use_cartesian_command) {
        while (!iiwa_pose_state.isConnected()) {}
        command_cartesian_position = iiwa_pose_state.getPose();
				// Here we set the new commanded cartesian position, we just move the tool TCP 10 centemeters down and back up, every 10 seconds.
				command_cartesian_position.poseStamped.pose.position.z -= direction * 0.10;
				iiwa_pose_command.setPose(command_cartesian_position.poseStamped);
			} else {
        while (!iiwa_joint_state.isConnected()) {}
        command_joint_position = iiwa_joint_state.getPosition();
        command_joint_position.position.a4 -= direction * 5 * M_PI / 180; // 0.0872665 // Adding/Subtracting 5 degrees (in radians) to the 4th joint
				iiwa_joint_command.setPosition(command_joint_position);
			}

			sleepForMotion(iiwa_time_destination, 2.0);

			direction *= -1; // In the next iteration the motion will be on the opposite direction

			loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
		}
		else {
			ROS_WARN_STREAM("Robot is not connected...");
			ros::Duration(5.0).sleep(); // 5 seconds
		}
	}
};
