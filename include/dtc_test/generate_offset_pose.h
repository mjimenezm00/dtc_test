#pragma once

// ROS
//#include <dynamic_task_planner/utils.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <job_msgs/msg/link_reference.hpp>
//#include <job_planner_msgs/msg/automation_solution.hpp>
#include <math.h>
//#include <model_msgs/msg/link.hpp>
#include <moveit/task_constructor/cost_queue.h>
#include <moveit/task_constructor/stage.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <string>
#include <vector>

using namespace moveit::task_constructor;

namespace dynamic_task_planner::stages
{

class GenerateOffsetPose : public MonitoringGenerator
{
  public:
    GenerateOffsetPose(const std::string& name = "generate_offset_pose");

    void reset() override;
    bool canCompute() const override;
    void compute() override;

    void generateUsingCompleteSearch(const planning_scene::PlanningScenePtr& scene,
                                     const std::vector<geometry_msgs::msg::PoseStamped>& object_poses);

    std::vector<double> generateSample(const double& default_val, const double& min_val, const double& max_val,
                                       const double& delta_val);

    bool spawnPose(const planning_scene::PlanningScenePtr& scene, const geometry_msgs::msg::PoseStamped& object_pose,
                   const Eigen::Isometry3d& tf_object_pose_to_offset_frame,
                   const Eigen::Isometry3d& tf_offset_frame_to_eef_frame, const double& radial_distance_m,
                   const double& polar_angle_rad, const double& azimuth_angle_rad, const double& eef_roll_rad,
                   const double& eef_pitch_rad, const double& eef_yaw_rad);

    void setPropertiesFromParameters(const std::vector<rclcpp::Parameter>& parameters);

    void setPropertiesFromNodeParameters(const rclcpp::Node::SharedPtr& node);

    //void setPropertiesFromAutomationSolution(
        //const std::shared_ptr<job_planner_msgs::msg::AutomationSolution>& automation_solution_ptr);

    void setObjectPoses(const std::vector<geometry_msgs::msg::PoseStamped>& object_poses)
    {
        setProperty("object_poses", object_poses);
    }
    //    TODO: add other setters

  protected:
    void onNewSolution(const SolutionBase& s) override;
    ordered<const SolutionBase*> upstream_solutions_;

    const int DEFAULT_MAX_SOLUTIONS_ = 100;

    const double DEFAULT_RADIAL_DISTANCE_DEFAULT_M_ = 0.25;
    const double DEFAULT_RADIAL_DISTANCE_MIN_M_ = 0.0;
    const double DEFAULT_RADIAL_DISTANCE_MAX_M_ = 0.0;
    const double DEFAULT_RADIAL_DISTANCE_DELTA_M_ = 0.0;

    const double DEFAULT_POLAR_ANGLE_DEFAULT_RAD_ = 0.0;
    const double DEFAULT_POLAR_ANGLE_MIN_RAD_ = 0.0;
    const double DEFAULT_POLAR_ANGLE_MAX_RAD_ = 0.0;
    const double DEFAULT_POLAR_ANGLE_DELTA_RAD_ = 0.0;

    const double DEFAULT_AZIMUTH_ANGLE_DEFAULT_RAD_ = 0.0;
    const double DEFAULT_AZIMUTH_ANGLE_MIN_RAD_ = 0.0;
    const double DEFAULT_AZIMUTH_ANGLE_MAX_RAD_ = 0.0;
    const double DEFAULT_AZIMUTH_ANGLE_DELTA_RAD_ = 0.0;

    const double DEFAULT_EEF_ROLL_DEFAULT_RAD_ = 0.0;
    const double DEFAULT_EEF_ROLL_MIN_RAD_ = 0.0;
    const double DEFAULT_EEF_ROLL_MAX_RAD_ = 0.0;
    const double DEFAULT_EEF_ROLL_DELTA_RAD_ = 0.0;

    const double DEFAULT_EEF_PITCH_DEFAULT_RAD_ = 0.0;
    const double DEFAULT_EEF_PITCH_MIN_RAD_ = 0.0;
    const double DEFAULT_EEF_PITCH_MAX_RAD_ = 0.0;
    const double DEFAULT_EEF_PITCH_DELTA_RAD_ = 0.0;

    const double DEFAULT_EEF_YAW_DEFAULT_RAD_ = 0.0;
    const double DEFAULT_EEF_YAW_MIN_RAD_ = 0.0;
    const double DEFAULT_EEF_YAW_MAX_RAD_ = 0.0;
    const double DEFAULT_EEF_YAW_DELTA_RAD_ = 0.0;

    const Eigen::Isometry3d DFT_TF_OBJECT_POSE_TO_OFFSET_FRAME_ = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d DFT_TF_OFFSET_FRAME_TO_EEF_FRAME_ = Eigen::Isometry3d::Identity();

    const std::string export_prefix_ = "export_";
    const std::string solution_prefix_ = "solution_";
};

}  // namespace dynamic_task_planner::stages
