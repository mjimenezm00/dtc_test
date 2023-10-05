// ROS
#include <rclcpp/rclcpp.hpp>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>

// MTC
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>

// JobPlanner
//#include <dynamic_task_planner/utils.h>
//#include <job_planner_msgs/msg/operation.hpp>

// ModelDatabase
//#include <model_database/model_cache.h>

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#pragma once

namespace dtc_test
{

    class DynamicTaskPlanner : public rclcpp::Node
    {
    public:
        DynamicTaskPlanner(const std::string &node_name = "dtc_test",
                           const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        //DynamicTaskPlanner(const std::string &task_name, const rclcpp::Node::SharedPtr &node,
                        //const planning_scene::PlanningScenePtr &current_scene);

        /*
        bool plan();

        bool getBestSolution(std::shared_ptr<const moveit::task_constructor::SolutionBase> &solution);

        bool getBestSolution(moveit_task_constructor_msgs::msg::Solution &solution);

        void buildTask_simple(const planning_scene::PlanningScenePtr &current_scene,
                              const geometry_msgs::msg::PoseStamped &fastener_pose,
                              const std::vector<geometry_msgs::msg::PoseStamped> &fiducial_poses,
                              const std::vector<rclcpp::Parameter> &static_task_parameters,
                              const rclcpp::ParameterMap &static_stage_parameters,
                              const std::shared_ptr<job_planner_msgs::msg::AutomationSolution> &automation_solution_ptr,
                              const std::string &stage_name, const std::string &fastener_type);

        */

        void run_test();
        void setup_scene();
        void setup_task();
        bool plan();

    protected:
        //rclcpp::Node::SharedPtr node_;
        rclcpp::Logger logger_;

        std::string task_name_;
        moveit::task_constructor::TaskPtr task_;
        planning_scene::PlanningScenePtr current_scene;

        std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner> sampling_planner_;
        std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> cartesian_planner_;

        std::vector<geometry_msgs::msg::PoseStamped> fiducial_poses;

        size_t max_solutions_ = 10;

        moveit_msgs::msg::CollisionObject place_work_object(const geometry_msgs::msg::Pose pose, std::vector<geometry_msgs::msg::PoseStamped> & fiducial_poses);

        enum Stages
        {
            plan_complete_task,
            fiducial_registration,
            fastener_registration,
            fastener_dock,
            retract,
            num_stages // number of stages in the task (must be last item)
        };

        const std::unordered_map<std::string, Stages> stage_names_ = {
            {"plan_complete_task", Stages::plan_complete_task},
            {"fiducial_registration", Stages::fiducial_registration},
            {"fastener_registration", Stages::fastener_registration},
            {"fastener_dock", Stages::fastener_dock},
            {"retract", Stages::retract},
        };

        inline int getEnum(const std::string &stage_name)
        {
            auto it = stage_names_.find(stage_name);
            if (it != stage_names_.end())
            {
                return it->second;
            }
            else
            {
                throw std::runtime_error(std::string("Stage " + stage_name + " does not exist in task"));
            }
        }
    };
} // namespace dynamic_task_planner
