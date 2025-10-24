#include <iostream>
#include <vector>
#include <fmt/core.h>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Geometry>

// Kalman Filter for velocity estimation
#include <mocap_kalman/KalmanFilter.h>

// Motion Capture
#include <libmotioncapture/motioncapture.h>

// Rigid Body tracker
#include <librigidbodytracker/rigid_body_tracker.h>
#include <librigidbodytracker/cloudlog.hpp>

void logWarn(rclcpp::Logger logger, const std::string& msg)
{
  RCLCPP_WARN(logger, "%s", msg.c_str());
}

std::set<std::string> extract_names(
  const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
  const std::string& pattern)
{
  std::set<std::string> result;
  for (const auto &i : parameter_overrides)
  {
    if (i.first.find(pattern) == 0)
    {
      size_t start = pattern.size() + 1;
      size_t end = i.first.find(".", start);
      result.insert(i.first.substr(start, end - start));
    }
  }
  return result;
}

std::vector<double> get_vec(const rclcpp::ParameterValue& param_value)
{
  if (param_value.get_type() == rclcpp::PARAMETER_INTEGER_ARRAY) {
    const auto int_vec = param_value.get<std::vector<int64_t>>();
    std::vector<double> result;
    for (int v : int_vec) {
      result.push_back(v);
    }
    return result;
  }
  return param_value.get<std::vector<double>>();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("motion_capture_tracking_node");
  node->declare_parameter<std::string>("type", "vicon");
  node->declare_parameter<std::string>("hostname", "localhost");
  node->declare_parameter<std::string>("topics.frame_id", "world");
  node->declare_parameter<std::string>("topics.poses.qos.mode", "none");
  node->declare_parameter<double>("topics.poses.qos.deadline", 100.0);
  node->declare_parameter<std::string>("topics.tf.child_frame_id", "{}");
  node->declare_parameter<int>("frame_rate", 200);
  node->declare_parameter<double>("max_accel", 10.0);
  node->declare_parameter<bool>("use_kalman_filter", true);

  node->declare_parameter<std::string>("logfilepath", "");

  std::string motionCaptureType = node->get_parameter("type").as_string();
  std::string motionCaptureHostname = node->get_parameter("hostname").as_string();
  std::string frame_id = node->get_parameter("topics.frame_id").as_string();
  std::string poses_qos = node->get_parameter("topics.poses.qos.mode").as_string();
  double poses_deadline = node->get_parameter("topics.poses.qos.deadline").as_double();
  std::string tf_child_frame_id = node->get_parameter("topics.tf.child_frame_id").as_string();
  int frame_rate = node->get_parameter("frame_rate").as_int();
  double max_accel = node->get_parameter("max_accel").as_double();
  bool use_kalman_filter = node->get_parameter("use_kalman_filter").as_bool();
  std::string logFilePath = node->get_parameter("logfilepath").as_string();
  
  // Calculate time interval from frame rate (in seconds)
  double dt_expected = 1.0 / static_cast<double>(frame_rate);
  
  // Setup Kalman filter noise matrices
  mocap_kalman::KalmanFilter::Matrix12d process_noise;
  mocap_kalman::KalmanFilter::Matrix6d measurement_noise;
  
  process_noise.topLeftCorner<6, 6>() = 0.5 * Eigen::Matrix<double, 6, 6>::Identity() * dt_expected * dt_expected * max_accel;
  process_noise.bottomRightCorner<6, 6>() = Eigen::Matrix<double, 6, 6>::Identity() * dt_expected * max_accel;
  process_noise *= process_noise; // Make it a covariance
  
  measurement_noise = Eigen::Matrix<double, 6, 6>::Identity() * 1e-3;
  measurement_noise *= measurement_noise; // Make it a covariance

  auto node_parameters_iface = node->get_node_parameters_interface();
  const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
      node_parameters_iface->get_parameter_overrides();

  librigidbodytracker::PointCloudLogger pointCloudLogger(logFilePath);
  const bool logClouds = !logFilePath.empty();
  std::cout << "logClouds=" <<logClouds << std::endl;  // 1

  // Make a new client
  std::map<std::string, std::string> cfg;
  cfg["hostname"] = motionCaptureHostname;

  // if the mock type is selected, add the defined rigid bodies
  if (motionCaptureType == "mock") {
    auto rigid_body_names = extract_names(parameter_overrides, "rigid_bodies");
    for (const auto &name : rigid_body_names)
    {
      const auto pos = get_vec(parameter_overrides.at("rigid_bodies." + name + ".initial_position"));
      cfg["rigid_bodies"] += name + "(" + std::to_string(pos[0]) + "," + std::to_string(pos[1]) + "," + std::to_string(pos[2]) +",1,0,0,0);";
    }
  }

  libmotioncapture::MotionCapture *mocap = libmotioncapture::MotionCapture::connect(motionCaptureType, cfg);

  // prepare point cloud publisher
  auto pubPointCloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("pointCloud", 1);

  sensor_msgs::msg::PointCloud2 msgPointCloud;
  msgPointCloud.header.frame_id = frame_id;
  msgPointCloud.height = 1;

  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  msgPointCloud.fields.push_back(field);
  field.name = "y";
  field.offset = 4;
  msgPointCloud.fields.push_back(field);
  field.name = "z";
  field.offset = 8;
  msgPointCloud.fields.push_back(field);
  msgPointCloud.point_step = 12;
  msgPointCloud.is_bigendian = false;
  msgPointCloud.is_dense = true;

  // Prepare publishers for poses
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_publishers;
  
  // Prepare publishers for odometry
  std::unordered_map<std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_publishers;
  
  // Kalman filters for velocity estimation (one per rigid body)
  std::unordered_map<std::string, std::shared_ptr<mocap_kalman::KalmanFilter>> kalman_filters;
  
  // Store previous poses for simple differentiation (when Kalman filter is disabled)
  struct PreviousPose {
    rclcpp::Time timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    bool valid;
  };
  std::unordered_map<std::string, PreviousPose> previous_poses;


  auto dynamics_config_names = extract_names(parameter_overrides, "dynamics_configurations");
  std::vector<librigidbodytracker::DynamicsConfiguration> dynamicsConfigurations(dynamics_config_names.size());
  std::map<std::string, size_t> dynamics_name_to_index;
  size_t i = 0;
  for (const auto& name : dynamics_config_names) {
    const auto max_vel = get_vec(parameter_overrides.at("dynamics_configurations." + name + ".max_velocity"));
    dynamicsConfigurations[i].maxXVelocity = max_vel.at(0);
    dynamicsConfigurations[i].maxYVelocity = max_vel.at(1);
    dynamicsConfigurations[i].maxZVelocity = max_vel.at(2);
    const auto max_angular_velocity = get_vec(parameter_overrides.at("dynamics_configurations." + name + ".max_angular_velocity"));
    dynamicsConfigurations[i].maxRollRate = max_angular_velocity.at(0);
    dynamicsConfigurations[i].maxPitchRate = max_angular_velocity.at(1);
    dynamicsConfigurations[i].maxYawRate = max_angular_velocity.at(2);
    dynamicsConfigurations[i].maxRoll = parameter_overrides.at("dynamics_configurations." + name + ".max_roll").get<double>();
    dynamicsConfigurations[i].maxPitch = parameter_overrides.at("dynamics_configurations." + name + ".max_pitch").get<double>();
    dynamicsConfigurations[i].maxFitnessScore = parameter_overrides.at("dynamics_configurations." + name + ".max_fitness_score").get<double>();
    dynamics_name_to_index[name] = i;
    ++i;
  }

  auto marker_config_names = extract_names(parameter_overrides, "marker_configurations");
  std::vector<librigidbodytracker::MarkerConfiguration> markerConfigurations;
  std::map<std::string, size_t> marker_name_to_index;
  i = 0;
  for (const auto &name : marker_config_names)
  {
    markerConfigurations.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
    const auto offset = get_vec(parameter_overrides.at("marker_configurations." + name + ".offset"));
    for (const auto &param : parameter_overrides)
    {
      if (param.first.find("marker_configurations." + name + ".points") == 0)
      {
        const auto points = get_vec(param.second);
        markerConfigurations.back()->push_back(pcl::PointXYZ(points[0] + offset[0], points[1] + offset[1], points[2] + offset[2]));
      }
    }
    marker_name_to_index[name] = i;
    ++i;
  }

  std::vector<librigidbodytracker::RigidBody> rigidBodies;
  // only add the rigid bodies to the tracker if we are not using the "mock" mode
  if (motionCaptureType != "mock") {
    auto rigid_body_names = extract_names(parameter_overrides, "rigid_bodies");
    for (const auto &name : rigid_body_names)
    {
      const auto pos = get_vec(parameter_overrides.at("rigid_bodies." + name + ".initial_position"));
      Eigen::Affine3f m;
      m = Eigen::Translation3f(pos[0], pos[1], pos[2]);
      const auto marker = parameter_overrides.at("rigid_bodies." + name + ".marker").get<std::string>();
      const auto dynamics = parameter_overrides.at("rigid_bodies." + name + ".dynamics").get<std::string>();

      rigidBodies.push_back(librigidbodytracker::RigidBody(marker_name_to_index.at(marker), dynamics_name_to_index.at(dynamics), m, name));
    }
  }

  librigidbodytracker::RigidBodyTracker tracker(
      dynamicsConfigurations,
      markerConfigurations,
      rigidBodies);
  tracker.setLogWarningCallback(std::bind(logWarn, node->get_logger(), std::placeholders::_1));

  // prepare TF broadcaster
  tf2_ros::TransformBroadcaster tfbroadcaster(node);
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  pcl::PointCloud<pcl::PointXYZ>::Ptr markers(new pcl::PointCloud<pcl::PointXYZ>);

  for (size_t frameId = 0; rclcpp::ok(); ++frameId) {

    // Get a frame
    mocap->waitForNextFrame();
    auto chrono_now = std::chrono::high_resolution_clock::now();
    auto time = node->now();

    auto pointcloud = mocap->pointCloud();

    // publish as pointcloud
    msgPointCloud.header.stamp = time;
    msgPointCloud.width = pointcloud.rows();
    msgPointCloud.data.resize(pointcloud.rows() * 3 * 4); // width * height * pointstep
    memcpy(msgPointCloud.data.data(), pointcloud.data(), msgPointCloud.data.size());
    msgPointCloud.row_step = msgPointCloud.data.size();

    pubPointCloud->publish(msgPointCloud);
    if (logClouds) {
      // pointCloudLogger.log(timestamp/1000, markers);  // point cloud log format: infinite repetitions of:  timestamp (milliseconds) : uint32
      // std::cout << "0000000000000before log" << std::endl;
      pointCloudLogger.log(markers);
    }


    // run tracker
    markers->clear();
    for (long int i = 0; i < pointcloud.rows(); ++i)
    {
      const auto &point = pointcloud.row(i);
      markers->push_back(pcl::PointXYZ(point(0), point(1), point(2)));
    }
    tracker.update(markers);

    transforms.clear();
    transforms.reserve(mocap->rigidBodies().size());
    for (const auto &iter : mocap->rigidBodies())
    {
      const auto& rigidBody = iter.second;

      // const auto& transform = rigidBody.transformation();
      // transforms.emplace_back(eigenToTransform(transform));
      transforms.resize(transforms.size() + 1);
      transforms.back().header.stamp = time;
      transforms.back().header.frame_id = frame_id;
      transforms.back().child_frame_id = rigidBody.name();
      transforms.back().transform.translation.x = rigidBody.position().x();
      transforms.back().transform.translation.y = rigidBody.position().y();
      transforms.back().transform.translation.z = rigidBody.position().z();
      transforms.back().transform.rotation.x = rigidBody.rotation().x();
      transforms.back().transform.rotation.y = rigidBody.rotation().y();
      transforms.back().transform.rotation.z = rigidBody.rotation().z();
      transforms.back().transform.rotation.w = rigidBody.rotation().w();
    }

    for (const auto& rigidBody : tracker.rigidBodies())
    {
      if (rigidBody.lastTransformationValid())
      {
        const Eigen::Affine3f &transform = rigidBody.transformation();
        Eigen::Quaternionf q(transform.rotation());
        const auto &translation = transform.translation();

        transforms.resize(transforms.size() + 1);
        transforms.back().header.stamp = time;
        transforms.back().header.frame_id = frame_id;
        transforms.back().child_frame_id = rigidBody.name();
        transforms.back().transform.translation.x = translation.x();
        transforms.back().transform.translation.y = translation.y();
        transforms.back().transform.translation.z = translation.z();
        if (rigidBody.orientationAvailable()) {
          transforms.back().transform.rotation.x = q.x();
          transforms.back().transform.rotation.y = q.y();
          transforms.back().transform.rotation.z = q.z();
          transforms.back().transform.rotation.w = q.w();
        } else {
          transforms.back().transform.rotation.x = std::nan("");
          transforms.back().transform.rotation.y = std::nan("");
          transforms.back().transform.rotation.z = std::nan("");
          transforms.back().transform.rotation.w = std::nan("");
        }
      }
      else
      {
        std::chrono::duration<double> elapsedSeconds = chrono_now - rigidBody.lastValidTime();
        // RCLCPP_WARN(node->get_logger(), "No updated pose for %s for %f s.", rigidBody.name().c_str(), elapsedSeconds.count());
      }
    }
    
    if (!transforms.empty()) {
      // publish poses and odometry
      for (const auto& tf : transforms) {
        const std::string& name = tf.child_frame_id;

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = time;
        pose_msg.header.frame_id = tf.header.frame_id;

        pose_msg.pose.position.x = tf.transform.translation.x;
        pose_msg.pose.position.y = tf.transform.translation.y;
        pose_msg.pose.position.z = tf.transform.translation.z;
        pose_msg.pose.orientation = tf.transform.rotation;

        // Publish pose
        auto it = pose_publishers.find(name);
        if (it != pose_publishers.end()) {
          it->second->publish(pose_msg);
        } else {
          // create publisher if needed
          std::string topic_name = name + "/pose3d";
          
          rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
          if (poses_qos == "none") {
            pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 1);
          } else if (poses_qos == "sensor") {
            rclcpp::SensorDataQoS sensor_data_qos;
            sensor_data_qos.keep_last(1);
            sensor_data_qos.deadline(rclcpp::Duration(0/*s*/, static_cast<int>(1e9/poses_deadline) /*ns*/));
            pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, sensor_data_qos);
          } else {
            throw std::runtime_error("Unknown QoS mode! " + poses_qos);
          }
          RCLCPP_WARN(node->get_logger(), "Created Pose Publisher for '%s'", name.c_str());
          pose_publishers[name] = pub;
        }

        // Compute velocities and publish odometry
        Eigen::Vector3d current_position(tf.transform.translation.x,
                                          tf.transform.translation.y,
                                          tf.transform.translation.z);
        Eigen::Quaterniond current_orientation(tf.transform.rotation.w,
                                                tf.transform.rotation.x,
                                                tf.transform.rotation.y,
                                                tf.transform.rotation.z);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = time;
        odom_msg.header.frame_id = tf.header.frame_id;
        odom_msg.child_frame_id = name;
        odom_msg.pose.pose = pose_msg.pose;

        if (use_kalman_filter) {
          // Use Kalman filter for velocity estimation
          auto kf_it = kalman_filters.find(name);
          if (kf_it == kalman_filters.end()) {
            // Create new Kalman filter for this rigid body
            auto kf = std::make_shared<mocap_kalman::KalmanFilter>();
            kf->init(process_noise, measurement_noise, frame_rate);
            kalman_filters[name] = kf;
            kf_it = kalman_filters.find(name);
            RCLCPP_INFO(node->get_logger(), "Created Kalman Filter for '%s'", name.c_str());
          }

          auto& kf = kf_it->second;
          double current_time = time.seconds();

          if (!kf->isReady()) {
            // Initialize the filter
            kf->prepareInitialCondition(current_time, current_orientation, current_position);
            // Velocities are zero during initialization
            odom_msg.twist.twist.linear.x = 0.0;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.linear.z = 0.0;
            odom_msg.twist.twist.angular.x = 0.0;
            odom_msg.twist.twist.angular.y = 0.0;
            odom_msg.twist.twist.angular.z = 0.0;
          } else {
            // Perform Kalman filter prediction and update
            kf->prediction(current_time);
            kf->update(current_orientation, current_position);

            // Use filtered state for odometry
            odom_msg.pose.pose.position.x = kf->position.x();
            odom_msg.pose.pose.position.y = kf->position.y();
            odom_msg.pose.pose.position.z = kf->position.z();
            
            odom_msg.pose.pose.orientation.w = kf->attitude.w();
            odom_msg.pose.pose.orientation.x = kf->attitude.x();
            odom_msg.pose.pose.orientation.y = kf->attitude.y();
            odom_msg.pose.pose.orientation.z = kf->attitude.z();

            // Get velocities from Kalman filter
            odom_msg.twist.twist.linear.x = kf->linear_vel.x();
            odom_msg.twist.twist.linear.y = kf->linear_vel.y();
            odom_msg.twist.twist.linear.z = kf->linear_vel.z();
            
            odom_msg.twist.twist.angular.x = kf->angular_vel.x();
            odom_msg.twist.twist.angular.y = kf->angular_vel.y();
            odom_msg.twist.twist.angular.z = kf->angular_vel.z();

            // Populate covariance matrices
            // Pose covariance (position and orientation)
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> pose_cov(odom_msg.pose.covariance.begin());
            pose_cov.topLeftCorner<3, 3>() = kf->state_cov.block<3, 3>(3, 3);  // position cov
            pose_cov.topRightCorner<3, 3>() = kf->state_cov.block<3, 3>(3, 0); // position-orientation cross
            pose_cov.bottomLeftCorner<3, 3>() = kf->state_cov.block<3, 3>(0, 3); // orientation-position cross
            pose_cov.bottomRightCorner<3, 3>() = kf->state_cov.block<3, 3>(0, 0); // orientation cov
            
            // Velocity covariance (linear and angular)
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> vel_cov(odom_msg.twist.covariance.begin());
            vel_cov.topLeftCorner<3, 3>() = kf->state_cov.block<3, 3>(9, 9);  // linear vel cov
            vel_cov.topRightCorner<3, 3>() = kf->state_cov.block<3, 3>(9, 6); // linear-angular cross
            vel_cov.bottomLeftCorner<3, 3>() = kf->state_cov.block<3, 3>(6, 9); // angular-linear cross
            vel_cov.bottomRightCorner<3, 3>() = kf->state_cov.block<3, 3>(6, 6); // angular vel cov
          }
        } else {
          // Use simple finite difference for velocity calculation
          auto prev_it = previous_poses.find(name);
          if (prev_it != previous_poses.end() && prev_it->second.valid) {
            // Use fixed time interval based on frame rate
            double dt = dt_expected;
            
            // Linear velocity: differentiate position
            Eigen::Vector3d linear_vel = (current_position - prev_it->second.position) / dt;
            odom_msg.twist.twist.linear.x = linear_vel.x();
            odom_msg.twist.twist.linear.y = linear_vel.y();
            odom_msg.twist.twist.linear.z = linear_vel.z();

            // Angular velocity: differentiate orientation
            Eigen::Quaterniond q_diff = current_orientation * prev_it->second.orientation.inverse();
            q_diff.normalize();
            
            Eigen::AngleAxisd angle_axis(q_diff);
            Eigen::Vector3d angular_vel = (angle_axis.angle() * angle_axis.axis()) / dt;
            
            odom_msg.twist.twist.angular.x = angular_vel.x();
            odom_msg.twist.twist.angular.y = angular_vel.y();
            odom_msg.twist.twist.angular.z = angular_vel.z();
          }
          
          // Store current pose for next iteration
          previous_poses[name] = {time, current_position, current_orientation, true};
        }

        // Publish odometry
        auto odom_it = odom_publishers.find(name);
        if (odom_it != odom_publishers.end()) {
          odom_it->second->publish(odom_msg);
        } else {
          // Create odometry publisher if needed
          std::string odom_topic_name = name + "/odom";
          
          rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
          if (poses_qos == "none") {
            odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 1);
          } else if (poses_qos == "sensor") {
            rclcpp::SensorDataQoS sensor_data_qos;
            sensor_data_qos.keep_last(1);
            sensor_data_qos.deadline(rclcpp::Duration(0/*s*/, static_cast<int>(1e9/poses_deadline) /*ns*/));
            odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, sensor_data_qos);
          } else {
            throw std::runtime_error("Unknown QoS mode! " + poses_qos);
          }
          RCLCPP_WARN(node->get_logger(), "Created Odometry Publisher for '%s'", name.c_str());
          odom_publishers[name] = odom_pub;
          odom_pub->publish(odom_msg);
        }
      }

      // send TF
      
      // Since RViz and others can't handle nan's, report a fake orientation if needed
      for (auto& tf : transforms) {
        if (std::isnan(tf.transform.rotation.x)) {
          tf.transform.rotation.x = 0;
          tf.transform.rotation.y = 0;
          tf.transform.rotation.z = 0;
          tf.transform.rotation.w = 1;
        }
      }

      // allow custom child_frame_ids before sending
      for (auto& tf : transforms) {
        std::string name = tf.child_frame_id;
        tf.child_frame_id = fmt::format(tf_child_frame_id, name);
      }

      tfbroadcaster.sendTransform(transforms);
    }
    if (logClouds) {
      pointCloudLogger.flush();
    }
    rclcpp::spin_some(node);
  }

  if (logClouds) {
    pointCloudLogger.flush();
  }

  return 0;
  }
