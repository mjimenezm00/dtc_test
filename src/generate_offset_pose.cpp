#include <dtc_test/generate_offset_pose.h>
#include <math.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/marker_tools.h>
#include <moveit/task_constructor/storage.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit_task_constructor_msgs/msg/task_description.hpp>
#include <rviz_marker_tools/marker_creation.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <dtc_test/utils.h>
#include <string>
#include <vector>

using namespace moveit::task_constructor;

namespace dynamic_task_planner::stages
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("GenerateOffsetPose");

GenerateOffsetPose::GenerateOffsetPose(const std::string& name) : MonitoringGenerator(name)
{
    // Set stage cost
    setCostTerm(std::make_unique<cost::Constant>(0.0));

    // Get stage properties
    auto& p = properties();

    p.declare<int>("max_solutions", DEFAULT_MAX_SOLUTIONS_, "maximum number of solutions to generate");

    p.declare<std::vector<geometry_msgs::msg::PoseStamped>>("object_poses",
                                                            "objects for which to generate offset poses");

    p.declare<Eigen::Isometry3d>("tf_object_pose_to_offset_frame", DFT_TF_OBJECT_POSE_TO_OFFSET_FRAME_,
                                 "offset frame is an intermediate frame defining the spherical coordinate origin");
    p.declare<Eigen::Isometry3d>("tf_offset_frame_to_eef_frame", DFT_TF_OFFSET_FRAME_TO_EEF_FRAME_,
                                 "offset frame is an intermediate frame defining the spherical coordinate origin");

    p.declare<double>("radial_distance_default_m", DEFAULT_RADIAL_DISTANCE_DEFAULT_M_,
                      "default radial distance (spherical coordinates) (metres)");
    p.declare<double>("radial_distance_min_m", DEFAULT_RADIAL_DISTANCE_MIN_M_,
                      "minimum radial distance (spherical coordinates) (metres)");
    p.declare<double>("radial_distance_max_m", DEFAULT_RADIAL_DISTANCE_MAX_M_,
                      "maximum radial distance (spherical coordinates) (metres)");
    p.declare<double>("radial_distance_delta_m", DEFAULT_RADIAL_DISTANCE_DELTA_M_,
                      "radial distance (spherical coordinates) step size (metres)");

    p.declare<double>("polar_angle_default_rad", DEFAULT_POLAR_ANGLE_DEFAULT_RAD_,
                      "default polar angle (spherical coordinates) in target z-axis (radians)");
    p.declare<double>("polar_angle_min_rad", DEFAULT_POLAR_ANGLE_MIN_RAD_,
                      "minimum polar angle (spherical coordinates) in target z-axis (radians)");
    p.declare<double>("polar_angle_max_rad", DEFAULT_POLAR_ANGLE_MAX_RAD_,
                      "maximum polar angle (spherical coordinates) in target z-axis (radians)");
    p.declare<double>("polar_angle_delta_rad", DEFAULT_POLAR_ANGLE_DELTA_RAD_,
                      "delta polar angle (spherical coordinates) in target z-axis (radians)");

    p.declare<double>("azimuth_angle_default_rad", DEFAULT_AZIMUTH_ANGLE_DEFAULT_RAD_,
                      "default azimuth angle (spherical coordinates) in target x-axis (radians)");
    p.declare<double>("azimuth_angle_min_rad", DEFAULT_AZIMUTH_ANGLE_MIN_RAD_,
                      "minimum azimuth angle (spherical coordinates) in target x-axis (radians)");
    p.declare<double>("azimuth_angle_max_rad", DEFAULT_AZIMUTH_ANGLE_MAX_RAD_,
                      "maximum azimuth angle (spherical coordinates) in target x-axis (radians)");
    p.declare<double>("azimuth_angle_delta_rad", DEFAULT_AZIMUTH_ANGLE_DELTA_RAD_,
                      "delta azimuth angle (spherical coordinates) in target x-axis (radians)");

    p.declare<double>("eef_roll_default_rad", DEFAULT_EEF_ROLL_DEFAULT_RAD_,
                      "default extrinsic end effector roll (radians)");
    p.declare<double>("eef_roll_min_rad", DEFAULT_EEF_ROLL_MIN_RAD_, "minimum extrinsic end effector roll (radians)");
    p.declare<double>("eef_roll_max_rad", DEFAULT_EEF_ROLL_MAX_RAD_, "maximum extrinsic end effector roll (radians)");
    p.declare<double>("eef_roll_delta_rad", DEFAULT_EEF_ROLL_DELTA_RAD_,
                      "extrinsic end effector roll step size (radians)");

    p.declare<double>("eef_pitch_default_rad", DEFAULT_EEF_PITCH_DEFAULT_RAD_,
                      "default extrinsic end effector pitch (radians)");
    p.declare<double>("eef_pitch_min_rad", DEFAULT_EEF_PITCH_MIN_RAD_,
                      "minimum extrinsic end effector pitch (radians)");
    p.declare<double>("eef_pitch_max_rad", DEFAULT_EEF_PITCH_MAX_RAD_,
                      "maximum extrinsic end effector pitch (radians)");
    p.declare<double>("eef_pitch_delta_rad", DEFAULT_EEF_PITCH_DELTA_RAD_,
                      "extrinsic end effector pitch step size (radians)");

    p.declare<double>("eef_yaw_default_rad", DEFAULT_EEF_YAW_DEFAULT_RAD_,
                      "default extrinsic end effector yaw (radians)");
    p.declare<double>("eef_yaw_min_rad", DEFAULT_EEF_YAW_MIN_RAD_, "minimum extrinsic end effector yaw (radians)");
    p.declare<double>("eef_yaw_max_rad", DEFAULT_EEF_YAW_MAX_RAD_, "maximum extrinsic end effector yaw (radians)");
    p.declare<double>("eef_yaw_delta_rad", DEFAULT_EEF_YAW_DELTA_RAD_,
                      "extrinsic end effector yaw step size (radians)");

    p.declare<double>(solution_prefix_ + "radial_distance_m", "initial value seeded from AutomationSolution");
    p.declare<double>(solution_prefix_ + "polar_angle_rad", "initial value seeded from AutomationSolution");
    p.declare<double>(solution_prefix_ + "azimuth_angle_rad", "initial value seeded from AutomationSolution");
    p.declare<double>(solution_prefix_ + "eef_roll_rad", "initial value seeded from AutomationSolution");
    p.declare<double>(solution_prefix_ + "eef_pitch_rad", "initial value seeded from AutomationSolution");
    p.declare<double>(solution_prefix_ + "eef_yaw_rad", "initial value seeded from AutomationSolution");
    p.declare<std::string>(solution_prefix_ + "object_frame_id", "initial value seeded from AutomationSolution");
}

void GenerateOffsetPose::reset()
{
    upstream_solutions_.clear();
    MonitoringGenerator::reset();
}

void GenerateOffsetPose::onNewSolution(const SolutionBase& s)
{
    // It's safe to store a pointer to this solution, as the generating stage stores it
    upstream_solutions_.push(&s);
}

bool GenerateOffsetPose::canCompute() const
{
    return !upstream_solutions_.empty();
}

void GenerateOffsetPose::compute()
{
    if (upstream_solutions_.empty())
    {
        return;
    }

    planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

    // Get stage properties
    const auto object_poses = properties().get<std::vector<geometry_msgs::msg::PoseStamped>>("object_poses");

    // Generate solutions
    generateUsingCompleteSearch(scene, object_poses);
}

void GenerateOffsetPose::generateUsingCompleteSearch(const planning_scene::PlanningScenePtr& scene,
                                                     const std::vector<geometry_msgs::msg::PoseStamped>& object_poses)
{
    const auto max_solutions = properties().get<int>("max_solutions");

    // Offset frame is an intermediate frame defining the spherical coordinate origin
    const auto tf_object_pose_to_offset_frame = properties().get<Eigen::Isometry3d>("tf_object_pose_to_offset_frame");
    const auto tf_offset_frame_to_eef_frame = properties().get<Eigen::Isometry3d>("tf_offset_frame_to_eef_frame");

    // Offset/radial distance (offset frame)
    const auto radial_distance_default_m = properties().get<double>("radial_distance_default_m");
    const auto radial_distance_min_m = properties().get<double>("radial_distance_min_m");
    const auto radial_distance_max_m = properties().get<double>("radial_distance_max_m");
    const auto radial_distance_delta_m = properties().get<double>("radial_distance_delta_m");

    // Polar angle (offset frame)
    const auto polar_angle_default_rad = properties().get<double>("polar_angle_default_rad");
    const auto polar_angle_min_rad = properties().get<double>("polar_angle_min_rad");
    const auto polar_angle_max_rad = properties().get<double>("polar_angle_max_rad");
    const auto polar_angle_delta_rad = properties().get<double>("polar_angle_delta_rad");

    // Azimuth angle (offset frame)
    const auto azimuth_angle_default_rad = properties().get<double>("azimuth_angle_default_rad");
    const auto azimuth_angle_min_rad = properties().get<double>("azimuth_angle_min_rad");
    const auto azimuth_angle_max_rad = properties().get<double>("azimuth_angle_max_rad");
    const auto azimuth_angle_delta_rad = properties().get<double>("azimuth_angle_delta_rad");

    // End effector rotation about X axis (eef frame)
    const auto eef_roll_default_rad = properties().get<double>("eef_roll_default_rad");
    const auto eef_roll_min_rad = properties().get<double>("eef_roll_min_rad");
    const auto eef_roll_max_rad = properties().get<double>("eef_roll_max_rad");
    const auto eef_roll_delta_rad = properties().get<double>("eef_roll_delta_rad");

    // End effector rotation about Y axis (eef frame)
    const auto eef_pitch_default_rad = properties().get<double>("eef_pitch_default_rad");
    const auto eef_pitch_min_rad = properties().get<double>("eef_pitch_min_rad");
    const auto eef_pitch_max_rad = properties().get<double>("eef_pitch_max_rad");
    const auto eef_pitch_delta_rad = properties().get<double>("eef_pitch_delta_rad");

    // End effector rotation about Z axis (eef frame)
    const auto eef_yaw_default_rad = properties().get<double>("eef_yaw_default_rad");
    const auto eef_yaw_min_rad = properties().get<double>("eef_yaw_min_rad");
    const auto eef_yaw_max_rad = properties().get<double>("eef_yaw_max_rad");
    const auto eef_yaw_delta_rad = properties().get<double>("eef_yaw_delta_rad");

    // Seed initial values from Job Planner AutomationSolution, use default as fallback (if no AutomationSolution)
    const auto initial_radial_distance_m =
        properties().get<double>(solution_prefix_ + "radial_distance_m", radial_distance_default_m);
    const auto initial_polar_angle_rad =
        properties().get<double>(solution_prefix_ + "polar_angle_rad", polar_angle_default_rad);
    const auto initial_azimuth_angle_rad =
        properties().get<double>(solution_prefix_ + "azimuth_angle_rad", azimuth_angle_default_rad);
    const auto initial_eef_roll_rad = properties().get<double>(solution_prefix_ + "eef_roll_rad", eef_roll_default_rad);
    const auto initial_eef_pitch_rad =
        properties().get<double>(solution_prefix_ + "eef_pitch_rad", eef_pitch_default_rad);
    const auto initial_eef_yaw_rad = properties().get<double>(solution_prefix_ + "eef_yaw_rad", eef_yaw_default_rad);
    const auto object_frame_id = properties().get<std::string>(solution_prefix_ + "object_frame_id", "");

    // If object_frame_id is populated, generate a solution for its matching object_pose first
    auto sorted_object_poses = object_poses;
    if (!object_frame_id.empty())
    {
        auto pivot = std::find_if(sorted_object_poses.begin(), sorted_object_poses.end(),
                                  [&object_frame_id](const auto& object_pose) -> bool
                                  {
                                      if (object_pose.header.frame_id == object_frame_id)
                                      {
                                          Eigen::Isometry3d tf_object_header_to_object_pose;
                                          tf2::fromMsg(object_pose.pose, tf_object_header_to_object_pose);
                                          return tf_object_header_to_object_pose.matrix().isIdentity(1e-5);
                                      }
                                      return false;
                                  });
        if (pivot != sorted_object_poses.end())
        {
            std::rotate(sorted_object_poses.begin(), pivot, pivot + 1);
        }
    }

    // Construct samples from properties
    const std::vector<double> radial_distance_sample = generateSample(initial_radial_distance_m, radial_distance_min_m,
                                                                      radial_distance_max_m, radial_distance_delta_m);
    const std::vector<double> polar_angle_sample =
        generateSample(initial_polar_angle_rad, polar_angle_min_rad, polar_angle_max_rad, polar_angle_delta_rad);
    const std::vector<double> azimuth_angle_sample = generateSample(initial_azimuth_angle_rad, azimuth_angle_min_rad,
                                                                    azimuth_angle_max_rad, azimuth_angle_delta_rad);
    const std::vector<double> eef_roll_sample =
        generateSample(initial_eef_roll_rad, eef_roll_min_rad, eef_roll_max_rad, eef_roll_delta_rad);
    const std::vector<double> eef_pitch_sample =
        generateSample(initial_eef_pitch_rad, eef_pitch_min_rad, eef_pitch_max_rad, eef_pitch_delta_rad);
    const std::vector<double> eef_yaw_sample =
        generateSample(initial_eef_yaw_rad, eef_yaw_min_rad, eef_yaw_max_rad, eef_yaw_delta_rad);

    // Generate solutions for each object, up to max_solutions, and vary parameters in a specific order
    int generated_solutions = 0;
    assert(max_solutions > generated_solutions && "Max solutions must be greater than 0");

    // Vary polar angle last
    for (const double& polar_angle_rad : polar_angle_sample)
    {
        // Vary azimuth angle
        for (const double& azimuth_angle_rad : azimuth_angle_sample)
        {
            if (polar_angle_rad == 0.0 && azimuth_angle_rad != azimuth_angle_default_rad)
            {
                continue;  // don't consider these cases
            }
            // Vary end effector roll
            for (const double& eef_roll_rad : eef_roll_sample)
            {
                // Vary end effector pitch
                for (const double& eef_pitch_rad : eef_pitch_sample)
                {
                    // Vary end effector offset / radial distance
                    for (const double& radial_distance_m : radial_distance_sample)
                    {
                        // Vary end effector yaw first
                        for (const double& eef_yaw_rad : eef_yaw_sample)
                        {
                            // Generate solution for each target frame id using these parameters
                            for (const geometry_msgs::msg::PoseStamped& object_pose : sorted_object_poses)
                            {
                                const bool spawned =
                                    spawnPose(scene, object_pose, tf_object_pose_to_offset_frame,
                                              tf_offset_frame_to_eef_frame, radial_distance_m, polar_angle_rad,
                                              azimuth_angle_rad, eef_roll_rad, eef_pitch_rad, eef_yaw_rad);
                                if (spawned && ++generated_solutions >= max_solutions)
                                {
                                    return;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

std::vector<double> GenerateOffsetPose::generateSample(const double& default_val, const double& min_val,
                                                       const double& max_val, const double& delta_val)
{
    // Initialize sample with default_val
    std::vector<double> sample;
    sample.push_back(default_val);

    // Return sample of one if min_val ~ max_val (not configured to iterate)
    if (std::abs(min_val - max_val) <= std::numeric_limits<double>::epsilon())
    {
        return sample;
    }

    // Assert min_val < default_val < max_val
    assert(min_val <= default_val + std::numeric_limits<double>::epsilon() &&
           "Cannot generate sample: min value is greater than default");
    assert(max_val >= default_val - std::numeric_limits<double>::epsilon() &&
           "Cannot generate sample: max value is less than default");

    // Construct sample as a vector of alternating +/- steps away from default_val, increasing in size
    const int max_steps = static_cast<int>(
        std::ceil(std::max(std::abs(default_val - min_val), std::abs(max_val - default_val)) / delta_val));

    for (int i = 0; i < max_steps; ++i)
    {
        const double step = static_cast<double>(i + 1) * delta_val;

        // Append positive step to sample if less than max_val
        if (default_val + step <= max_val + std::numeric_limits<double>::epsilon())
        {
            sample.push_back(default_val + step);
        }
        // Append negative step to sample if greater than min_val
        if (default_val - step >= min_val - std::numeric_limits<double>::epsilon())
        {
            sample.push_back(default_val - step);
        }
    }

    return sample;
}

bool GenerateOffsetPose::spawnPose(const planning_scene::PlanningScenePtr& scene,
                                   const geometry_msgs::msg::PoseStamped& object_pose,
                                   const Eigen::Isometry3d& tf_object_pose_to_offset_frame,
                                   const Eigen::Isometry3d& tf_offset_frame_to_eef_frame,
                                   const double& radial_distance_m, const double& polar_angle_rad,
                                   const double& azimuth_angle_rad, const double& eef_roll_rad,
                                   const double& eef_pitch_rad, const double& eef_yaw_rad)
{
    auto object_pose_corrected = object_pose;

    // Verify object header frame is a collision object or subframe in the planning scene
    // If the object comes from the world model we may need to prepend world_model_
    if (!scene->knowsFrameTransform(object_pose_corrected.header.frame_id))
    {
        object_pose_corrected.header.frame_id = "world_model_" + object_pose_corrected.header.frame_id;
    }
    if (!scene->knowsFrameTransform(object_pose_corrected.header.frame_id))
    {
        RCLCPP_WARN(LOGGER, "Unknown frame: '%s'", object_pose_corrected.header.frame_id.c_str());
        return false;
    }

    // Projected offset frame is an intermediate frame expressed by spherical coordinates in the offset frame
    const Eigen::Translation3d p_projected_offset_frame = Eigen::Translation3d(
        radial_distance_m * cos(azimuth_angle_rad) * sin(polar_angle_rad),
        radial_distance_m * sin(azimuth_angle_rad) * sin(polar_angle_rad), radial_distance_m * cos(polar_angle_rad));

    // Rotate the projected z-axis to point towards the object
    // When azimuth = 0, rotation is polar about y-axis
    // When azimuth = M_PI, rotation is -1.0 * polar about x-axis
    const Eigen::Quaterniond q_projected_offset_frame =
        Eigen::AngleAxisd(-1.0 * polar_angle_rad * sin(azimuth_angle_rad), Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(polar_angle_rad * cos(azimuth_angle_rad), Eigen::Vector3d::UnitY());

    const Eigen::Isometry3d tf_offset_frame_to_projected_offset_frame =
        p_projected_offset_frame * q_projected_offset_frame;

    // End effector rotation is expressed in extrinsic Euler angles in the eef frame
    const Eigen::Quaterniond q_eef = Eigen::AngleAxisd(eef_yaw_rad, Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(eef_pitch_rad, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(eef_roll_rad, Eigen::Vector3d::UnitX());

    const Eigen::Isometry3d tf_eef_frame_to_eef_pose = Eigen::Translation3d::Identity() * q_eef;

    // Apply transformations to compute target camera pose
    Eigen::Isometry3d tf_object_header_to_object_pose;
    tf2::fromMsg(object_pose_corrected.pose, tf_object_header_to_object_pose);

    // Normalize quaternions of input transforms
    const Eigen::Quaterniond n_q_object_pose_to_offset_frame =
        Eigen::Quaterniond(tf_object_pose_to_offset_frame.rotation()).normalized();
    const Eigen::Isometry3d n_tf_object_pose_to_offset_frame =
        Eigen::Translation3d(tf_object_pose_to_offset_frame.translation()) * n_q_object_pose_to_offset_frame;

    const Eigen::Quaterniond n_q_offset_frame_to_eef_frame =
        Eigen::Quaterniond(tf_offset_frame_to_eef_frame.rotation()).normalized();
    const Eigen::Isometry3d n_tf_offset_frame_to_eef_frame =
        Eigen::Translation3d(tf_offset_frame_to_eef_frame.translation()) * n_q_offset_frame_to_eef_frame;

    const Eigen::Isometry3d tf_object_header_to_eef_pose =
        tf_object_header_to_object_pose * n_tf_object_pose_to_offset_frame * tf_offset_frame_to_projected_offset_frame *
        n_tf_offset_frame_to_eef_frame * tf_eef_frame_to_eef_pose;

    geometry_msgs::msg::PoseStamped eef_pose;
    eef_pose.pose = tf2::toMsg(tf_object_header_to_eef_pose);
    eef_pose.header.frame_id = object_pose_corrected.header.frame_id;

    InterfaceState state(scene);
    state.properties().set("target_pose", eef_pose);

    // Store properties for export
    state.properties().set(export_prefix_ + "radial_distance_m", radial_distance_m);
    state.properties().set(export_prefix_ + "polar_angle_rad", polar_angle_rad);
    state.properties().set(export_prefix_ + "azimuth_angle_rad", azimuth_angle_rad);
    state.properties().set(export_prefix_ + "eef_roll_rad", eef_roll_rad);
    state.properties().set(export_prefix_ + "eef_pitch_rad", eef_pitch_rad);
    state.properties().set(export_prefix_ + "eef_yaw_rad", eef_yaw_rad);

    // If object_pose is defined by a link reference, export the link reference
    if (tf_object_header_to_object_pose.matrix().isIdentity(1e-5))
    {
        state.properties().set(export_prefix_ + "object_frame_id", object_pose_corrected.header.frame_id);
    }
    else
    {
        state.properties().set(export_prefix_ + "object_frame_id", "");
    }

    SubTrajectory trajectory;
    trajectory.setCost(0.0);

    rviz_marker_tools::appendFrame(trajectory.markers(), eef_pose, 0.1, "target pose");
    rviz_marker_tools::appendFrame(trajectory.markers(), object_pose_corrected, 0.05,
                                   object_pose_corrected.header.frame_id);

    spawn(std::move(state), std::move(trajectory));
    return true;
}

void GenerateOffsetPose::setPropertiesFromParameters(const std::vector<rclcpp::Parameter>& parameters)
{
    utils::setPropertyFromParameters<int>(parameters, "max_solutions", properties());

    // Offset frame is an intermediate frame defining the spherical coordinate origin
    utils::setPropertyFromParameters<Eigen::Isometry3d>(parameters, "tf_object_pose_to_offset_frame", properties());
    utils::setPropertyFromParameters<Eigen::Isometry3d>(parameters, "tf_offset_frame_to_eef_frame", properties());

    // Offset/radial distance (offset frame)
    utils::setPropertyFromParameters<double>(parameters, "radial_distance_default_m", properties());
    utils::setPropertyFromParameters<double>(parameters, "radial_distance_min_m", properties());
    utils::setPropertyFromParameters<double>(parameters, "radial_distance_max_m", properties());
    utils::setPropertyFromParameters<double>(parameters, "radial_distance_delta_m", properties());

    // Polar angle (offset frame)
    utils::setPropertyFromParameters<double>(parameters, "polar_angle_default_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "polar_angle_min_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "polar_angle_max_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "polar_angle_delta_rad", properties());

    // Azimuth angle (offset frame)
    utils::setPropertyFromParameters<double>(parameters, "azimuth_angle_default_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "azimuth_angle_min_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "azimuth_angle_max_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "azimuth_angle_delta_rad", properties());

    // End effector rotation about X axis (eef frame)
    utils::setPropertyFromParameters<double>(parameters, "eef_roll_default_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "eef_roll_min_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "eef_roll_max_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "eef_roll_delta_rad", properties());

    // End effector rotation about Y axis (eef frame)
    utils::setPropertyFromParameters<double>(parameters, "eef_pitch_default_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "eef_pitch_min_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "eef_pitch_max_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "eef_pitch_delta_rad", properties());

    // End effector rotation about Z axis (eef frame)
    utils::setPropertyFromParameters<double>(parameters, "eef_yaw_default_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "eef_yaw_min_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "eef_yaw_max_rad", properties());
    utils::setPropertyFromParameters<double>(parameters, "eef_yaw_delta_rad", properties());
}

void GenerateOffsetPose::setPropertiesFromNodeParameters(const rclcpp::Node::SharedPtr& node)
{
    const std::string stage_name = name();  // assume parameters prefixed by stage name
    const std::vector<std::string> parameter_names = node->list_parameters({stage_name}, 2).names;
    const std::vector<rclcpp::Parameter> parameters = node->get_parameters(parameter_names);

    setPropertiesFromParameters(parameters);
}

/*
void GenerateOffsetPose::setPropertiesFromAutomationSolution(
    const std::shared_ptr<job_planner_msgs::msg::AutomationSolution>& automation_solution_ptr)
{
    if (automation_solution_ptr != nullptr)
    {
        const auto& task_description = automation_solution_ptr->task_description;
        const auto& solution = automation_solution_ptr->solution;

        // Find StageDescription matching this stage's name
        const std::string stage_name = name();  // assume parameters prefixed by stage name
        auto it =
            std::find_if(task_description.stages.begin(), task_description.stages.end(),
                         [&stage_name](const auto& stage_description) { return stage_description.name == stage_name; });

        if (it != task_description.stages.end())
        {
            const auto& stage_description = *it;

            // Import properties matching export_prefix_
            for (const auto& ros_msg : stage_description.properties)
            {
                if (ros_msg.name.rfind(export_prefix_, 0) == 0)
                {
                    // Replace import_prefix with solution_prefix
                    const std::string new_name = solution_prefix_ + ros_msg.name.substr(export_prefix_.length());
                    utils::setPropertyFromRosMsg(ros_msg, new_name, properties());
                }
            }
        }
    }
}

*/

}  // namespace dynamic_task_planner::stages
