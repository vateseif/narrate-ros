#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int8.h>

#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <thread>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <l2c/PIDConfig.h>  // Replace with the actual path to your generated header


static const std::string LOGNAME = "cpp_interface_example";


// Global pointer declaration
moveit_servo::PoseTracking* tracker_ptr = nullptr;

void parameterCallback(l2c::PIDConfig &config, uint32_t level) {
  ROS_INFO_STREAM_NAMED(LOGNAME, "Reconfigure Request: "
                        << " windup_limit=" << config.windup_limit
                        << " x_proportional_gain=" << config.x_proportional_gain
                        << " y_proportional_gain=" << config.y_proportional_gain
                        << " z_proportional_gain=" << config.z_proportional_gain
                        << " x_integral_gain=" << config.x_integral_gain
                        << " y_integral_gain=" << config.y_integral_gain
                        << " z_integral_gain=" << config.z_integral_gain
                        << " x_derivative_gain=" << config.x_derivative_gain
                        << " y_derivative_gain=" << config.y_derivative_gain
                        << " z_derivative_gain=" << config.z_derivative_gain
                        << " angular_proportional_gain=" << config.angular_proportional_gain
                        << " angular_integral_gain=" << config.angular_integral_gain
                        << " angular_derivative_gain=" << config.angular_derivative_gain);

  // Use the updatePIDConfig function to apply the new settings
  if(tracker_ptr) {  // Ensure the tracker pointer is not null
    tracker_ptr->updatePIDConfig(config.x_proportional_gain, config.x_integral_gain, config.x_derivative_gain,
                                 config.y_proportional_gain, config.y_integral_gain, config.y_derivative_gain,
                                 config.z_proportional_gain, config.z_integral_gain, config.z_derivative_gain,
                                 config.angular_proportional_gain, config.angular_integral_gain, config.angular_derivative_gain);
  }
}



// Class for monitoring status of moveit_servo
class StatusMonitor {
public:
  StatusMonitor(ros::NodeHandle &nh, const std::string &topic) {
    sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
  }

private:
  void statusCB(const std_msgs::Int8ConstPtr &msg) {
    moveit_servo::StatusCode latest_status =
        static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_) {
      status_ = latest_status;
      const auto &status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: " << status_str);
    }
  }
  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  ros::Subscriber sub_;
};

/**
 * Instantiate the pose tracking interface.
 * Send a pose slightly different from the starting pose
 * Then keep updating the target pose a little bit
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, LOGNAME);
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(8);
  spinner.start();

  // Define the dynamic reconfigure server type for your specific config
  dynamic_reconfigure::Server<l2c::PIDConfig> server;
  server.setCallback(boost::bind(&parameterCallback, _1, _2));  

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
          "robot_description");
  if (!planning_scene_monitor->getPlanningScene()) {
    ROS_ERROR_STREAM_NAMED(LOGNAME,
                           "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::
          DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::
          DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  // Create the pose tracker
  static moveit_servo::PoseTracking tracker(nh, planning_scene_monitor);

  // Assign the address of tracker to the global pointer
  tracker_ptr = &tracker;

  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(nh, "status");

  // TODO, reset when hand is closed etc.
  tracker.resetTargetPose();
  // Run the pose tracking in a new thread

  Eigen::Vector3d lin_tol{0.005, 0.005, 0.005};
  double rot_tol = 0.05;

  ros::Rate loop_rate(30);
  while (ros::ok()) {
    // Modify the pose target a little bit each cycle
    // This is a dynamic pose target
    tracker.moveToPose(lin_tol, rot_tol, 1/30.0 /* target pose timeout */);
  }

  // Make sure the tracker is stopped and clean up
  tracker.stopMotion();

  return EXIT_SUCCESS;
}