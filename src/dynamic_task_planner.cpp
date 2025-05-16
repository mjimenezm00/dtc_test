#include <dtc_test/compute_ik.h>
#include <dtc_test/dynamic_task_planner.h>
#include <dtc_test/generate_offset_pose.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// MTC
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("DynamicTaskBase");

using namespace std;
namespace mtc = moveit::task_constructor;
using namespace mtc;

namespace dtc_test
{

DynamicTaskPlanner::DynamicTaskPlanner(const std::string& node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node(node_name, options), logger_(get_logger())
{
    RCLCPP_INFO(LOGGER, "Initializing task base");

    // TODO: Investigate why cartesian planner is not included in PipelinePlanner class
    // TODO: Pass in planner parameters, fetch them from a database (from the RobotType?)
}

/*

    bool DynamicTaskBase::getBestSolution(std::shared_ptr<const moveit::task_constructor::SolutionBase> &solution)
    {
        RCLCPP_INFO(LOGGER, "Get best solution by cost");
        const auto &solutions = task_->solutions();

        if (solutions.empty())
        {
            RCLCPP_INFO(LOGGER, "No solutions were generated. Have you called plan?");
            return false;
        }

        solution = solutions.front();
        RCLCPP_INFO_STREAM(LOGGER, "Best solution has cost of: " << solution->cost());

        return true;
    }

    bool DynamicTaskBase::getBestSolution(moveit_task_constructor_msgs::msg::Solution &solution)
    {
        std::shared_ptr<const moveit::task_constructor::SolutionBase> best_solution;

        if (!getBestSolution(best_solution))
        {
            return false;
        }

        best_solution->appendTo(solution);
        return true;
    }

*/

void DynamicTaskPlanner::run_test()
{
    try
    {
        setup_scene();
        setup_task();
        plan();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, e.what());
    }
}

moveit_msgs::msg::CollisionObject
    DynamicTaskPlanner::place_work_object(const geometry_msgs::msg::Pose pose,
                                          std::vector<geometry_msgs::msg::PoseStamped>& fiducial_poses)
{
    int n_meshes = 4;
    std::string name = "work_object";
    // std::vector<std::string> names = {
    //"work_object", "fid_1", "fid_2", "fid_3"};
    std::vector<std::string> paths = {
        "package://dtc_test/meshes/mesh.stl",
        "package://dtc_test/meshes/marker_41.dae",
        "package://dtc_test/meshes/marker_42.dae",
        "package://dtc_test/meshes/marker_43.dae",
    };

    geometry_msgs::msg::Pose pose_wo;
    pose_wo.position.x = 0, pose_wo.position.y = 0, pose_wo.position.z = 0, pose_wo.orientation.x = 0,
    pose_wo.orientation.y = 0, pose_wo.orientation.z = 0, pose_wo.orientation.w = 0;

    geometry_msgs::msg::Pose pose_fid_1;
    pose_fid_1.position.x = 0.231127;
    pose_fid_1.position.y = -0.0231547;
    pose_fid_1.position.z = 1.86345;
    pose_fid_1.orientation.x = 0;
    pose_fid_1.orientation.y = -0.7071;
    pose_fid_1.orientation.z = 0.7071;  // -
    pose_fid_1.orientation.w = 0;

    geometry_msgs::msg::Pose pose_fid_2;
    pose_fid_2.position.x = 0.681127;
    pose_fid_2.position.y = -0.0231547;
    pose_fid_2.position.z = 1.86345;
    pose_fid_2.orientation.y = -0.7071;
    pose_fid_2.orientation.z = 0.7071;
    pose_fid_2.orientation.w = 0;

    geometry_msgs::msg::Pose pose_fid_3;
    pose_fid_3.position.x = 1.03113;
    pose_fid_3.position.y = -0.0231547;
    pose_fid_3.position.z = 1.86345;
    pose_fid_3.orientation.y = -0.7071;
    pose_fid_3.orientation.z = 0.7071;
    pose_fid_3.orientation.w = 0;

    std::vector<geometry_msgs::msg::Pose> poses = {pose_wo, pose_fid_1, pose_fid_2, pose_fid_3};
    std::vector<geometry_msgs::msg::Pose> poses_to_publish = {pose_fid_1, pose_fid_2, pose_fid_3};

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = name;
    collision_object.meshes.resize(n_meshes);
    collision_object.mesh_poses.resize(n_meshes);

    Eigen::Isometry3d tf_global_pose;
    tf2::fromMsg(pose, tf_global_pose);

    for (int i = 0; i < n_meshes; i++)
    {
        shapes::Mesh* m = shapes::createMeshFromResource(paths[i]);
        shapes::ShapeMsg shape_msg;
        shapes::constructMsgFromShape(m, shape_msg);
        shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(shape_msg);

        collision_object.meshes[i] = mesh;

        Eigen::Isometry3d tf_local_pose;
        tf2::fromMsg(poses[i], tf_local_pose);

        const Eigen::Isometry3d tf_local_pose_updated = tf_global_pose * tf_local_pose;
        geometry_msgs::msg::Pose local_pose_updated = tf2::toMsg(tf_local_pose_updated);
        collision_object.mesh_poses[i] = local_pose_updated;
    }

    for (unsigned int i = 0; i < poses_to_publish.size(); i++)
    {
        Eigen::Isometry3d tf_local_pose;
        tf2::fromMsg(poses_to_publish[i], tf_local_pose);
        const Eigen::Isometry3d tf_local_pose_updated = tf_global_pose * tf_local_pose;
        geometry_msgs::msg::Pose local_pose_updated = tf2::toMsg(tf_local_pose_updated);

        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "world";
        ps.pose = local_pose_updated;
        RCLCPP_FATAL_STREAM(LOGGER, "x" << ps.pose.position.x);
        RCLCPP_FATAL_STREAM(LOGGER, "y" << ps.pose.position.y);
        RCLCPP_FATAL_STREAM(LOGGER, "z" << ps.pose.position.z);

        fiducial_poses.push_back(ps);
    }

    return collision_object;
}

void DynamicTaskPlanner::setup_scene()
{
    rclcpp::Node::SharedPtr node_ptr = shared_from_this();

    // Sampling planner
    sampling_planner_ = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_ptr, "cumotion");
    // sampling_planner_ = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_ptr, "ompl");
    sampling_planner_->setProperty("goal_joint_tolerance", 1e-5);

    // Cartesian planner
    cartesian_planner_ = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
    cartesian_planner_->setMaxVelocityScalingFactor(1.0);
    cartesian_planner_->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner_->setStepSize(.0005);

    // Load Robot
    const std::string robot_urdf = this->get_parameter("robot_description").as_string();
    const std::string robot_srdf = this->get_parameter("robot_description_semantic").as_string();
    robot_model_loader::RobotModelLoader::Options options(robot_urdf, robot_srdf);
    options.robot_description_ = "robot_description";  // it doesn't work
    auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(node_ptr, options);
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader->getModel();

    std::map<std::string, rclcpp::Parameter> params;
    node_ptr->get_parameters("", params);

    // for (auto &p : params)
    // RCLCPP_FATAL_STREAM(LOGGER, p.first << ": " << p.second.value_to_string());

    // Create PlanningScene
    current_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);

    //////////////////////////////////////////////////////////////

    // Create a collision object to represent the model
    geometry_msgs::msg::Pose pose_wo;
    pose_wo.position.x = 1.5258;
    pose_wo.position.y = 0.55341;
    pose_wo.position.z = -0.12717;
    pose_wo.orientation.x = 0;
    pose_wo.orientation.y = 0;
    pose_wo.orientation.z = -0.7071;
    pose_wo.orientation.w = 0.7071;
    auto work_object = place_work_object(pose_wo, this->fiducial_poses);

    current_scene->processCollisionObjectMsg(work_object);

    //////////////////////////////////////////////////////////////
    // Attach end effector
    {
        std::string tip_link = "tool_changer_robot_link";

        // Create a collision object to represent the end effector
        moveit_msgs::msg::AttachedCollisionObject attached_eef_collision_object;
        attached_eef_collision_object.link_name = tip_link;  // attach end effector to tip link

        geometry_msgs::msg::Pose eef_collision_object_pose;
        // eef_collision_object_pose.header.frame_id = tip_link;
        eef_collision_object_pose = tf2::toMsg(Eigen::Isometry3d::Identity());

        moveit_msgs::msg::CollisionObject eef_collision_object;

        eef_collision_object.header.frame_id = tip_link;
        eef_collision_object.id = "ee_hilok";
        eef_collision_object.meshes.resize(1);
        eef_collision_object.mesh_poses.resize(1);
        shapes::Mesh* m = shapes::createMeshFromResource("package://dtc_test/meshes/ee_hilok_collision.stl");
        shapes::ShapeMsg shape_msg;
        shapes::constructMsgFromShape(m, shape_msg);
        shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(shape_msg);

        eef_collision_object.meshes[0] = mesh;
        // geometry_msgs::msg::Pose local_pose;
        eef_collision_object.mesh_poses[0] = eef_collision_object_pose;

        eef_collision_object.subframe_names.clear();
        eef_collision_object.subframe_poses.clear();

        geometry_msgs::msg::Pose tcp;
        tcp.position.x = 0;
        tcp.position.y = 0.1806;
        tcp.position.z = 0.1106;
        tcp.orientation.x = -0.7071067;
        tcp.orientation.y = 0;
        tcp.orientation.z = 0;
        tcp.orientation.w = 0.7071067;

        geometry_msgs::msg::Pose cam;
        cam.position.x = 0;
        cam.position.y = 0.2056;
        cam.position.z = 0.1106;
        cam.orientation.x = -0.7071067;
        cam.orientation.y = 0;
        cam.orientation.z = 0;
        cam.orientation.w = 0.7071067;
        // 0 0.2056 0.1106 -1.570796 0 0

        eef_collision_object.subframe_names.push_back("ee_tcp_link");
        eef_collision_object.subframe_names.push_back("ee_camera_link");
        eef_collision_object.subframe_poses.push_back(tcp);
        eef_collision_object.subframe_poses.push_back(cam);

        // Add allowed collision between EE and robot tip
        RCLCPP_INFO(LOGGER, "Allowing collision between \"%s\" and \"%s\"", tip_link.c_str(),
                    eef_collision_object.id.c_str());
        current_scene->getAllowedCollisionMatrixNonConst().setEntry(tip_link, eef_collision_object.id, true);

        // Spawn model collision object in planning scene (uses ROS service)
        eef_collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
        attached_eef_collision_object.object = std::move(eef_collision_object);

        current_scene->processAttachedCollisionObjectMsg(attached_eef_collision_object);
    }

    current_scene->getCurrentStateNonConst().setToDefaultValues("manipulator", "retracted");

    //////////////////////////////////////////
    // Publish planning scene
    auto psm =
        std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_ptr, current_scene, robot_model_loader);
    psm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_SCENE,
                                      "/monitored_planning_scene");
}

void DynamicTaskPlanner::setup_task()
{
    const std::string stage_name_str = "plan_complete_task";
    RCLCPP_INFO_STREAM(LOGGER, "Planning for: " << stage_name_str + " stage");
    const int stage_enum = getEnum(stage_name_str);

    // Reset ROS introspection before constructing the new object
    task_.reset();
    task_.reset(new moveit::task_constructor::Task());
    task_->stages()->setName("dtc_test");  // set name of stage container, i.e. task
    task_->setRobotModel(current_scene->getRobotModel());

    // Unpack static task parameters
    const string group = "manipulator";  // utils::getParameterValue<std::string>(static_task_parameters, "group");
    const string arm_attachment_link_name =
        "tool_changer_robot_link";  // utils::getParameterValue<std::string>(static_task_parameters,
                                    // "arm_attachment_link_name");
    const string camera_frame_name =
        "ee_hilok/ee_camera_link";  // utils::getParameterValue<std::string>(static_task_parameters,
                                    // "camera_frame_name");
    // camera_frame_name = "tool_changer_robot_link";
    const auto tf_camera_frame_to_camera = Eigen::Isometry3d::Identity();
    const string tcp_frame_name =
        "ee_hilok/ee_tcp_link";  // utils::getParameterValue<std::string>(static_task_parameters, "tcp_frame_name");
    const auto tf_tcp_frame_to_tcp = Eigen::Isometry3d::Identity();
    const double fastener_approach_min_dist = 0.020;
    const double fastener_approach_max_dist = 0.040;
    const double task_timeout_sec = 200;  // utils::getParameterValue<double>(static_task_parameters, "timeout");

    // Set max number of solutions for this task
    this->max_solutions_ = 100;  // utils::getParameterValue<int>(static_task_parameters, "max_solutions");

    geometry_msgs::msg::PoseStamped fastener_pose;
    fastener_pose.header.frame_id = "world";
    fastener_pose.pose.position.x = 1.8418;
    fastener_pose.pose.position.y = -0.82369;
    fastener_pose.pose.position.z = 1.5515;  // 1.5615
    fastener_pose.pose.orientation.x = 0.5;
    fastener_pose.pose.orientation.y = 0.5;
    fastener_pose.pose.orientation.z = 0.5;
    fastener_pose.pose.orientation.w = 0.5;

    // Re-use of the movement stages between pose generators
    auto create_movements_stage = [this](string name, double timeout)
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            name, mtc::stages::Connect::GroupPlannerVector{{"manipulator", sampling_planner_}});

        stage->setTimeout(timeout);
        stage->properties().configureInitFrom(Stage::PARENT);

        return stage;
    };

    // Set task properties
    Task& task = *this->task_;
    task.properties().set("group", "manipulator");
    task.setTimeout(task_timeout_sec);

    /*
    if (automation_solution_ptr != nullptr)
    {
        RCLCPP_WARN(LOGGER, "Seed solution found, it will be used to improve trajectory quality");
    }*/

    Stage* current_state_ptr = nullptr;  // Forward current_state on to pose generators
    {
        auto current_state =
            std::make_unique<moveit::task_constructor::stages::FixedState>("current_state", current_scene);
        current_state_ptr = current_state.get();

        RCLCPP_INFO_STREAM(LOGGER, "Start state for 'current_state': " << current_scene->getCurrentState());

        current_scene->getCurrentState().printStatePositionsWithJointLimits(
            current_scene->getRobotModel()->getJointModelGroup("manipulator"));

        // Check collisions
        std::vector<std::string> collision_links;
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_detection::CollisionResult::ContactMap contacts;

        current_scene->getCollidingPairs(contacts);
        for (auto& c : contacts)
            RCLCPP_WARN(LOGGER, "Link %s is in contact with %s", c.first.first.c_str(), c.first.second.c_str());

        task.add(std::move(current_state));
    }

    if (stage_enum == Stages::fiducial_registration || stage_enum == Stages::plan_complete_task)
    {
        // Move to fiducial task
        double timeout = 1.0;
        // utils::getParameterValue<double>(static_stage_parameters, "move_to_fiducial_registration", "timeout");
        auto move_to_fiducial = create_movements_stage("move_to_fiducial_registration", timeout);

        RCLCPP_INFO_STREAM(LOGGER,
                           "Start state for 'move_to_fiducial_registration': " << current_scene->getCurrentState());
        task.add(std::move(move_to_fiducial));

        // Fiducial pose generator
        {
            const std::string stage_name = "fiducial_registration_pose";
            auto stage = std::make_unique<dynamic_task_planner::stages::GenerateOffsetPose>(stage_name);

            stage->setMonitoredStage(current_state_ptr);  // Hook into current state
            stage->properties().configureInitFrom(Stage::PARENT);
            stage->properties().set("marker_ns", stage_name);
            stage->properties().set("object_poses", fiducial_poses);

            std::vector<rclcpp::Parameter> parameters;
            parameters.push_back(rclcpp::Parameter("max_solutions", rclcpp::ParameterValue(5)));
            parameters.push_back(
                rclcpp::Parameter("tf_object_pose_to_offset_frame",
                                  rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0})));
            parameters.push_back(
                rclcpp::Parameter("tf_offset_frame_to_eef_frame",
                                  rclcpp::ParameterValue(std::vector<double>{0.0, 0.040, 0.0, 0.0, 0.0, 1.0, 0.0})));
            parameters.push_back(rclcpp::Parameter("radial_distance_default_m", rclcpp::ParameterValue(0.150)));
            parameters.push_back(rclcpp::Parameter("radial_distance_min_m", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("radial_distance_max_m", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("radial_distance_delta_m", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("polar_angle_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("polar_angle_min_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("polar_angle_max_rad", rclcpp::ParameterValue(0.5235987755982988)));
            parameters.push_back(
                rclcpp::Parameter("polar_angle_delta_rad", rclcpp::ParameterValue(0.17453292519943295)));
            parameters.push_back(rclcpp::Parameter("azimuth_angle_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(
                rclcpp::Parameter("azimuth_angle_min_rad", rclcpp::ParameterValue(-3.141592653589793)));
            parameters.push_back(rclcpp::Parameter("azimuth_angle_max_rad", rclcpp::ParameterValue(3.141592653589793)));
            parameters.push_back(
                rclcpp::Parameter("azimuth_angle_delta_rad", rclcpp::ParameterValue(0.5235987755982988)));
            parameters.push_back(rclcpp::Parameter("eef_roll_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_roll_min_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_roll_max_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_roll_delta_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_pitch_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_pitch_min_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_pitch_max_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_pitch_delta_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_yaw_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_yaw_min_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_yaw_max_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_yaw_delta_rad", rclcpp::ParameterValue(0.0)));

            stage->setPropertiesFromParameters(parameters);
            // stage->setPropertiesFromAutomationSolution(automation_solution_ptr);

            // Compute IK
            auto wrapper =
                std::make_unique<dynamic_task_planner::stages::ComputeIK>("fiducial_registration", std::move(stage));
            // auto wrapper = std::make_unique<dynamic_task_planner::stages::ComputeIK>("fiducial_registration",
            // std::move(stage));
            wrapper->setMaxIKSolutions(5);
            // utils::getParameterValue<int>(static_stage_parameters, wrapper->name(), "max_solutions"));
            wrapper->setMinSolutionDistance(1.0);
            // utils::getParameterValue<double>(static_stage_parameters, wrapper->name(), "min_solution_distance"));
            wrapper->setIKFrame(tf_camera_frame_to_camera, camera_frame_name);
            // wrapper->properties().configureInitFrom(Stage::PARENT, {"group"});
            wrapper->setGroup("manipulator");
            // wrapper->setVStageJoint("ewellix_lift_top_joint");

            wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
            // wrapper->seedFromAutomationSolution(automation_solution_ptr);
            task.add(std::move(wrapper));
        }
    }

    if (stage_enum == Stages::fastener_registration || stage_enum == Stages::plan_complete_task)
    {
        // Move to fastener reg task
        double timeout = 2.0;
        // utils::getParameterValue<double>(static_stage_parameters, "move_to_fastener_registration", "timeout");
        auto move_to_fastener_registration = create_movements_stage("move_to_fastener_registration", timeout);
        RCLCPP_INFO_STREAM(LOGGER,
                           "Start state for 'move_to_fastener_registration': " << current_scene->getCurrentState());
        task.add(std::move(move_to_fastener_registration));

        // Fastener registration pose generator
        {
            const std::string stage_name = "fastener_registration_pose";
            auto stage = std::make_unique<dynamic_task_planner::stages::GenerateOffsetPose>(stage_name);

            stage->setMonitoredStage(current_state_ptr);  // Hook into current state
            stage->properties().configureInitFrom(Stage::PARENT);
            stage->properties().set("marker_ns", stage_name);
            stage->setObjectPoses({fastener_pose});
            RCLCPP_INFO_STREAM(LOGGER,
                               "Start state for 'fastener_registration_pose': " << current_scene->getCurrentState());
            std::vector<rclcpp::Parameter> parameters;
            parameters.push_back(rclcpp::Parameter("max_solutions", rclcpp::ParameterValue(100)));
            parameters.push_back(rclcpp::Parameter(
                "tf_object_pose_to_offset_frame",
                rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0, 0.707106781, 0.0, 0.707106781, 0.0})));
            parameters.push_back(
                rclcpp::Parameter("tf_offset_frame_to_eef_frame",
                                  rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0})));
            parameters.push_back(rclcpp::Parameter("radial_distance_default_m", rclcpp::ParameterValue(0.040)));
            parameters.push_back(rclcpp::Parameter("radial_distance_min_m", rclcpp::ParameterValue(0.040)));
            parameters.push_back(rclcpp::Parameter("radial_distance_max_m", rclcpp::ParameterValue(0.050)));
            parameters.push_back(rclcpp::Parameter("radial_distance_delta_m", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("polar_angle_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("polar_angle_min_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("polar_angle_max_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("polar_angle_delta_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("azimuth_angle_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("azimuth_angle_min_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("azimuth_angle_max_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("azimuth_angle_delta_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_roll_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_roll_min_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_roll_max_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_roll_delta_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_pitch_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_pitch_min_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_pitch_max_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_pitch_delta_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_yaw_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_yaw_min_rad", rclcpp::ParameterValue(-3.141592653589793)));
            parameters.push_back(rclcpp::Parameter("eef_yaw_max_rad", rclcpp::ParameterValue(3.141592653589793)));
            parameters.push_back(rclcpp::Parameter("eef_yaw_delta_rad", rclcpp::ParameterValue(0.1)));

            stage->setPropertiesFromParameters(parameters);
            // stage->setPropertiesFromAutomationSolution(automation_solution_ptr);

            // Compute IK
            auto wrapper =
                std::make_unique<dynamic_task_planner::stages::ComputeIK>("fastener_registration", std::move(stage));
            wrapper->setMaxIKSolutions(50);
            // utils::getParameterValue<int>(static_stage_parameters, wrapper->name(), "max_solutions"));
            wrapper->setMinSolutionDistance(1.0);
            // utils::getParameterValue<double>(static_stage_parameters, wrapper->name(), "min_solution_distance"));
            wrapper->setIKFrame(tf_camera_frame_to_camera, camera_frame_name);
            // wrapper->properties().configureInitFrom(Stage::PARENT, {"group"});
            wrapper->setGroup("manipulator");
            // wrapper->setVStageJoint("ewellix_lift_top_joint");
            wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
            // wrapper->seedFromAutomationSolution(automation_solution_ptr);
            task.add(std::move(wrapper));
        }
    }

    if (stage_enum == Stages::fastener_dock || stage_enum == Stages::plan_complete_task)
    {
        // Move to fastener dock task
        double timeout = 2.0;
        // utils::getParameterValue<double>(static_stage_parameters, "move_to_fastener_pre_dock", "timeout");
        auto move_to_fastener_dock = create_movements_stage("move_to_fastener_pre_dock", timeout);
        task.add(std::move(move_to_fastener_dock));

        // Fastener dock (actually pre-dock) pose generator
        {
            const std::string stage_name = "fastener_dock_pose";
            auto stage = std::make_unique<dynamic_task_planner::stages::GenerateOffsetPose>(stage_name);

            stage->setMonitoredStage(current_state_ptr);  // Hook into current state
            stage->properties().set("marker_ns", stage_name);
            stage->setObjectPoses({fastener_pose});
            std::vector<rclcpp::Parameter> parameters;
            parameters.push_back(rclcpp::Parameter("max_solutions", rclcpp::ParameterValue(100)));
            parameters.push_back(rclcpp::Parameter(
                "tf_object_pose_to_offset_frame",
                rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0, 0.707106781, 0.0, 0.707106781, 0.0})));
            parameters.push_back(
                rclcpp::Parameter("tf_offset_frame_to_eef_frame",
                                  rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0})));
            parameters.push_back(rclcpp::Parameter("radial_distance_default_m", rclcpp::ParameterValue(0.040)));
            parameters.push_back(rclcpp::Parameter("radial_distance_min_m", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("radial_distance_max_m", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("radial_distance_delta_m", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("polar_angle_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("polar_angle_min_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("polar_angle_max_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("polar_angle_delta_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("azimuth_angle_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("azimuth_angle_min_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("azimuth_angle_max_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("azimuth_angle_delta_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_roll_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_roll_min_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_roll_max_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_roll_delta_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_pitch_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_pitch_min_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_pitch_max_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_pitch_delta_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_yaw_default_rad", rclcpp::ParameterValue(0.0)));
            parameters.push_back(rclcpp::Parameter("eef_yaw_min_rad", rclcpp::ParameterValue(-3.141592653589793)));
            parameters.push_back(rclcpp::Parameter("eef_yaw_max_rad", rclcpp::ParameterValue(3.141592653589793)));
            parameters.push_back(rclcpp::Parameter("eef_yaw_delta_rad", rclcpp::ParameterValue(0.1)));

            stage->setPropertiesFromParameters(parameters);
            // stage->setPropertiesFromAutomationSolution(automation_solution_ptr);

            // Compute IK
            auto wrapper = std::make_unique<dynamic_task_planner::stages::ComputeIK>("fastener_dock", std::move(stage));
            wrapper->setMaxIKSolutions(2);
            // utils::getParameterValue<int>(static_stage_parameters, wrapper->name(), "max_solutions"));
            wrapper->setMinSolutionDistance(1.0);
            // utils::getParameterValue<double>(static_stage_parameters, wrapper->name(), "min_solution_distance"));
            wrapper->setIKFrame(tf_tcp_frame_to_tcp, tcp_frame_name);

            // wrapper->properties().configureInitFrom(Stage::PARENT, {"group"});
            wrapper->setGroup("manipulator");
            // wrapper->setVStageJoint("ewellix_lift_top_joint");
            wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
            // wrapper->seedFromAutomationSolution(automation_solution_ptr);
            task.add(std::move(wrapper));
        }
    }

    if (stage_enum == Stages::plan_complete_task)
    {
        // Allow collision
        /*{
            auto stage =
                std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("allow_fastener_collision");
            const std::string end_effector_name = tcp_frame_name.substr(0, tcp_frame_name.find('/'));
            stage->allowCollisions(end_effector_name, fastener_type, true);
            // stage->allowCollisions(end_effector_name, "work_object", true);
            // stage->allowCollisions(end_effector_name, "work_object_fixture", true);

            task.add(std::move(stage));
        }*/

        // Linear move to approach fastener
        {
            const std::string stage_name = "dock_urscript";  // getName(Stages::move_to_fastener_dock);
            auto stage =
                std::make_unique<moveit::task_constructor::stages::MoveRelative>(stage_name, cartesian_planner_);

            stage->properties().configureInitFrom(Stage::PARENT, {"group"});
            stage->properties().set("marker_ns", stage_name);

            // Set allowable range for approach/withdraw
            stage->setMinMaxDistance(fastener_approach_min_dist, fastener_approach_max_dist);

            // Set IK frame to TCP frame
            geometry_msgs::msg::PoseStamped ik_frame;
            ik_frame.header.frame_id = arm_attachment_link_name;
            ik_frame.pose = tf2::toMsg(Eigen::Isometry3d::Identity());
            stage->setIKFrame(ik_frame);

            // Set direction of movement
            Eigen::Vector3d direction_wrt_tcp =
                tf_tcp_frame_to_tcp.rotation() * Eigen::Vector3d(0.0, 0.0, 1.0);  // approach along TCP z-axis
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = tcp_frame_name;
            vec.vector.x = direction_wrt_tcp.x();
            vec.vector.y = direction_wrt_tcp.y();
            vec.vector.z = direction_wrt_tcp.z();
            stage->setDirection(vec);

            task.add(std::move(stage));
        }

        // Linear move to separate from fastener
        {
            const std::string stage_name = "undock_urscript";  // getName(Stages::move_to_fastener_undock);
            auto stage =
                std::make_unique<moveit::task_constructor::stages::MoveRelative>(stage_name, cartesian_planner_);

            stage->properties().configureInitFrom(Stage::PARENT, {"group"});
            stage->properties().set("marker_ns", stage_name);

            // Set allowable range for approach/withdraw
            stage->setMinMaxDistance(fastener_approach_min_dist, fastener_approach_max_dist);

            // Set IK frame to TCP frame
            geometry_msgs::msg::PoseStamped ik_frame;
            ik_frame.header.frame_id = arm_attachment_link_name;
            ik_frame.pose = tf2::toMsg(Eigen::Isometry3d::Identity());
            stage->setIKFrame(ik_frame);

            // Set direction of movement
            Eigen::Vector3d direction_wrt_tcp =
                tf_tcp_frame_to_tcp.rotation() * Eigen::Vector3d(0.0, 0.0, -1.0);  // withdraw along TCP z-axis
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = tcp_frame_name;
            vec.vector.x = direction_wrt_tcp.x();
            vec.vector.y = direction_wrt_tcp.y();
            vec.vector.z = direction_wrt_tcp.z();
            stage->setDirection(vec);

            task.add(std::move(stage));
        }

        /*
        // Forbid collisions
        {
            auto stage =
                std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("forbid_fastener_collision");
            const std::string end_effector_name = tcp_frame_name.substr(0, tcp_frame_name.find('/'));
            stage->allowCollisions(end_effector_name, fastener_type, false);
            stage->allowCollisions(end_effector_name, "work_object", false);
            stage->allowCollisions(end_effector_name, "work_object_fixture", false);
            task.add(std::move(stage));
        }*/
    }

    if (stage_enum == Stages::retract || stage_enum == Stages::plan_complete_task)
    {
        // Move to retracted pose
        {
            auto stage =
                std::make_unique<moveit::task_constructor::stages::MoveTo>("move_to_retracted", sampling_planner_);
            stage->setTimeout(
                15.0);  // utils::getParameterValue<double>(static_stage_parameters, stage->name(), "timeout"));
            stage->setGroup("manipulator");
            stage->setGoal("retracted");
            task.add(std::move(stage));
        }

        // Move z-axis to retracted pose
        /*{
            auto stage =
                std::make_unique<moveit::task_constructor::stages::MoveTo>("move_to_retracted", sampling_planner_);
            stage->setTimeout(15.0);//utils::getParameterValue<double>(static_stage_parameters, stage->name(),
        "timeout")); stage->setGroup("vertical_stage"); stage->setGoal("retracted"); task.add(std::move(stage));
        }*/
    }
    // Definir fiducial_poses

    // Definir fastener_pose

    // Ejecutar el "build_task_simple"

    return;
}

bool DynamicTaskPlanner::plan()
{
    try
    {
        RCLCPP_INFO(LOGGER, "Start searching for task solutions");
        moveit::core::MoveItErrorCode error_code = task_->plan(max_solutions_);
        return error_code == moveit::core::MoveItErrorCode::SUCCESS;
    }
    catch (mtc::InitStageException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return false;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, e.what());
        return false;
    }
}

}  // namespace dtc_test
