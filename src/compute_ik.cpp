/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  Copyright (c) 2017, Hamburg University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/* Authors: Robert Haschke, Michael Goerner */

/* Modified by MTorres & Boeing */

#include <Eigen/Geometry>

#include <dtc_test/compute_ik.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/task_constructor/marker_tools.h>
#include <moveit/task_constructor/storage.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <rclcpp/logging.hpp>

#include <chrono>
#include <functional>
#include <iterator>

namespace dynamic_task_planner::stages
{

namespace mtc = moveit::task_constructor;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ComputeIK");

ComputeIK::ComputeIK(const std::string& name, Stage::pointer&& child) : WrapperBase(name, std::move(child))
{
    auto& p = properties();
    p.declare<std::string>("eef", "name of end-effector group");
    p.declare<std::string>("group", "name of active group (derived from eef if not provided)");
    p.declare<std::string>("default_pose", "", "default joint pose of active group (defines cost of IK)");
    p.declare<uint32_t>("max_ik_solutions", 1);
    p.declare<bool>("ignore_collisions", false);
    p.declare<double>("min_solution_distance", 0.1,
                      "minimum distance between seperate IK solutions for the same target");

    // ik_frame and target_pose are read from the interface
    p.declare<geometry_msgs::msg::PoseStamped>("ik_frame", "frame to be moved towards goal pose");
    p.declare<geometry_msgs::msg::PoseStamped>("target_pose", "goal pose for ik frame");
}

void ComputeIK::setIKFrame(const Eigen::Isometry3d& pose, const std::string& link)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = link;
    pose_msg.pose = tf2::toMsg(pose);
    setIKFrame(pose_msg);
}

void ComputeIK::setTargetPose(const Eigen::Isometry3d& pose, const std::string& frame)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = frame;
    pose_msg.pose = tf2::toMsg(pose);
    setTargetPose(pose_msg);
}

// found IK solutions

struct IKSolution
{
    std::vector<double> joint_positions;
    bool feasible;
    collision_detection::Contact contact;
};

/*
void ComputeIK::seedFromAutomationSolution(
    const std::shared_ptr<job_planner_msgs::msg::AutomationSolution>& automation_solution_ptr)
{
    if (nullptr == automation_solution_ptr)
        return;
    //    automation_solution_joint_positions_
    // Lookup stage ID in TaskDescription by stage name
    uint32_t stage_id;
    {
        const std::string stage_name = this->name();
        const auto& stage_descriptions = automation_solution_ptr->task_description.stages;

        auto it =
            std::find_if(stage_descriptions.begin(), stage_descriptions.end(),
                         [&stage_name](const auto& stage_description) { return stage_description.name == stage_name; });

        if (it == stage_descriptions.end())
        {
            RCLCPP_WARN_STREAM(LOGGER, "Could not find stage description matching name: " << stage_name
                                                                                          << " in AutomationSolution");
            return;
        }

        stage_id = it->id;
    }

    // Lookup robot state in Solution by stage ID
    moveit_msgs::msg::RobotState robot_state_msg;
    {
        const auto& sub_trajectories = automation_solution_ptr->solution.sub_trajectory;

        auto it =
            std::find_if(sub_trajectories.begin(), sub_trajectories.end(),
                         [&stage_id](const auto& sub_trajectory) { return sub_trajectory.info.stage_id == stage_id; });

        if (it == sub_trajectories.end())
        {
            RCLCPP_WARN_STREAM(LOGGER, "Could not find sub trajectory matching stage_id: " << stage_id
                                                                                           << " in AutomationSolution");
            return;
        }

        robot_state_msg = it->scene_diff.robot_state;
    }

    // Populate joint_positions map with robot state positions
    std::map<std::string, double> joint_positions;
    {
        if (robot_state_msg.joint_state.name.empty() ||
            robot_state_msg.joint_state.name.size() != robot_state_msg.joint_state.position.size())
        {
            RCLCPP_WARN_STREAM(LOGGER, "AutomationSolution does not contain a valid joint state");
            return;
        }
        decltype(robot_state_msg.joint_state.name.size()) i;
        for (i = 0; i < robot_state_msg.joint_state.name.size(); i++)
        {
            joint_positions[robot_state_msg.joint_state.name[i]] = robot_state_msg.joint_state.position[i];
        }
    }

    // Apply start state diff if required
    if (robot_state_msg.is_diff)
    {
        const auto& start_state_msg = automation_solution_ptr->solution.start_scene.robot_state;

        if (start_state_msg.joint_state.name.empty() ||
            start_state_msg.joint_state.name.size() != start_state_msg.joint_state.position.size())
        {
            RCLCPP_WARN_STREAM(LOGGER, "AutomationSolution does not contain a valid joint state in its start scene");
            return;
        }
        decltype(start_state_msg.joint_state.name.size()) i;
        for (i = 0; i < start_state_msg.joint_state.name.size(); i++)
        {
            joint_positions[start_state_msg.joint_state.name[i]] += start_state_msg.joint_state.position[i];
        }
    }

    // Store joint positions
    automation_solution_joint_positions_ = joint_positions;
}
*/

using IKSolutions = std::vector<IKSolution>;

namespace
{

// ??? TODO: provide callback methods in PlanningScene class / probably not very useful here though...
// TODO: move into MoveIt core, lift active_components_only_ from fcl to common interface
bool isTargetPoseCollidingInEEF(const planning_scene::PlanningSceneConstPtr& scene,
                                moveit::core::RobotState& robot_state, Eigen::Isometry3d pose,
                                const moveit::core::LinkModel* link,
                                collision_detection::CollisionResult* collision_result = nullptr)
{
    // consider all rigidly connected parent links as well
    const moveit::core::LinkModel* parent = moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(link);
    if (parent != link)  // transform pose into pose suitable to place parent
        pose = pose * robot_state.getGlobalLinkTransform(link).inverse() * robot_state.getGlobalLinkTransform(parent);

    // place links at given pose
    robot_state.updateStateWithLinkAt(parent, pose);
    robot_state.updateCollisionBodyTransforms();

    // disable collision checking for parent links (except links fixed to root)
    auto acm = scene->getAllowedCollisionMatrix();
    std::vector<const std::string*> pending_links;  // parent link names that might be rigidly connected to root
    while (parent)
    {
        pending_links.push_back(&parent->getName());
        link = parent;
        const moveit::core::JointModel* joint = link->getParentJointModel();
        parent = joint->getParentLinkModel();

        if (joint->getType() != moveit::core::JointModel::FIXED)
        {
            for (const std::string* name : pending_links)
                acm.setDefaultEntry(*name, true);
            pending_links.clear();
        }
    }

    // check collision with the world using the padded version
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult result;
    req.contacts = (collision_result != nullptr);
    collision_detection::CollisionResult& res = collision_result ? *collision_result : result;
    scene->checkCollision(req, res, robot_state, acm);
    return res.collision;
}

std::string listCollisionPairs(const collision_detection::CollisionResult::ContactMap& contacts,
                               const std::string& separator)
{
    std::string result;
    for (const auto& contact : contacts)
    {
        if (!result.empty())
            result.append(separator);
        result.append(contact.first.first).append(" - ").append(contact.first.second);
    }
    return result;
}

bool validateEEF(const mtc::PropertyMap& props, const moveit::core::RobotModelConstPtr& robot_model,
                 const moveit::core::JointModelGroup*& jmg, std::string* msg)
{
    try
    {
        const std::string& eef = props.get<std::string>("eef");
        if (!robot_model->hasEndEffector(eef))
        {
            if (msg)
                *msg = "Unknown end effector: " + eef;
            return false;
        }
        else
            jmg = robot_model->getEndEffector(eef);
    }
    catch (const mtc::Property::undefined&)
    {
    }
    return true;
}
bool validateGroup(const mtc::PropertyMap& props, const moveit::core::RobotModelConstPtr& robot_model,
                   const moveit::core::JointModelGroup* eef_jmg, const moveit::core::JointModelGroup*& jmg,
                   std::string* msg)
{
    try
    {
        const std::string& group = props.get<std::string>("group");
        if (!(jmg = robot_model->getJointModelGroup(group)))
        {
            if (msg)
                *msg = "Unknown group: " + group;
            return false;
        }
    }
    catch (const mtc::Property::undefined&)
    {
        if (eef_jmg)
        {
            // derive group from eef
            const auto& parent = eef_jmg->getEndEffectorParentGroup();
            jmg = robot_model->getJointModelGroup(parent.first);
        }
    }
    return true;
}

}  // anonymous namespace

void ComputeIK::reset()
{
    upstream_solutions_.clear();
    WrapperBase::reset();
}

void ComputeIK::init(const moveit::core::RobotModelConstPtr& robot_model)
{
    mtc::InitStageException errors;
    try
    {
        WrapperBase::init(robot_model);
    }
    catch (mtc::InitStageException& e)
    {
        errors.append(e);
    }

    // all properties can be derived from the interface state
    // however, if they are defined already now, we validate here
    const auto& props = properties();
    const moveit::core::JointModelGroup* eef_jmg = nullptr;
    const moveit::core::JointModelGroup* jmg = nullptr;
    std::string msg;

    if (!validateEEF(props, robot_model, eef_jmg, &msg))
        errors.push_back(*this, msg);
    if (!validateGroup(props, robot_model, eef_jmg, jmg, &msg))
        errors.push_back(*this, msg);

    if (errors)
        throw errors;
}

void ComputeIK::onNewSolution(const mtc::SolutionBase& s)
{
    assert(s.start() && s.end());
    assert(s.start()->scene() == s.end()->scene());  // wrapped child should be a generator

    // It's safe to store a pointer to the solution, as the generating stage stores it
    upstream_solutions_.push(&s);
}

bool ComputeIK::canCompute() const
{
    return !upstream_solutions_.empty() || WrapperBase::canCompute();
}

void ComputeIK::compute()
{
    if (WrapperBase::canCompute())
        WrapperBase::compute();

    if (upstream_solutions_.empty())
        return;

    const mtc::SolutionBase& s = *upstream_solutions_.pop();

    // -1 TODO: this should not be necessary in my opinion: Why do you think so?
    // It is, because the properties on the interface might change from call to call...
    // enforced initialization from interface ensures that new target_pose is read
    properties().performInitFrom(INTERFACE, s.start()->properties());
    const auto& props = properties();

    const planning_scene::PlanningSceneConstPtr& scene{s.start()->scene()};

    const bool ignore_collisions = props.get<bool>("ignore_collisions");
    const auto& robot_model = scene->getRobotModel();
    const moveit::core::JointModelGroup* eef_jmg = nullptr;
    const moveit::core::JointModelGroup* jmg = nullptr;
    std::string msg;

    if (!validateEEF(props, robot_model, eef_jmg, &msg))
    {
        RCLCPP_WARN_STREAM(LOGGER, msg);
        return;
    }
    if (!validateGroup(props, robot_model, eef_jmg, jmg, &msg))
    {
        RCLCPP_WARN_STREAM(LOGGER, msg);
        return;
    }
    if (!eef_jmg && !jmg)
    {
        RCLCPP_WARN_STREAM(LOGGER, "Neither eef nor group are well defined");
        return;
    }
    properties().property("timeout").setDefaultValue(jmg->getDefaultIKTimeout());

    // extract target_pose
    geometry_msgs::msg::PoseStamped target_pose_msg = props.get<geometry_msgs::msg::PoseStamped>("target_pose");
    if (target_pose_msg.header.frame_id.empty())  // if not provided, assume planning frame
        target_pose_msg.header.frame_id = scene->getPlanningFrame();

    RCLCPP_FATAL_STREAM(LOGGER, "FRAME: " << target_pose_msg.pose.position.x);
    RCLCPP_FATAL_STREAM(LOGGER, "FRAME: " << target_pose_msg.pose.position.y);
    RCLCPP_FATAL_STREAM(LOGGER, "FRAME: " << target_pose_msg.pose.position.z);

    //RCLCPP_FATAL_STREAM(LOGGER, "SCENE FRAME: " << scene->getPlanningFrame());

    Eigen::Isometry3d target_pose;
    tf2::fromMsg(target_pose_msg.pose, target_pose);
    if (target_pose_msg.header.frame_id != scene->getPlanningFrame())
    {
        if (!scene->knowsFrameTransform(target_pose_msg.header.frame_id))
        {
            RCLCPP_WARN_STREAM(LOGGER, "Unknown reference frame for target pose: " << target_pose_msg.header.frame_id);
            return;
        }
        // transform target_pose w.r.t. planning frame
        target_pose = scene->getFrameTransform(target_pose_msg.header.frame_id) * target_pose;
    }

    // determine IK link from ik_frame
    const moveit::core::LinkModel* link = nullptr;
    geometry_msgs::msg::PoseStamped ik_pose_msg;
    const boost::any& value = props.get("ik_frame");
    if (value.empty())
    {  // property undefined
        //  determine IK link from eef/group
        if (!(link = eef_jmg ? robot_model->getLinkModel(eef_jmg->getEndEffectorParentGroup().second)
                             : jmg->getOnlyOneEndEffectorTip()))
        {
            RCLCPP_WARN_STREAM(LOGGER, "Failed to derive IK target link");
            return;
        }
        ik_pose_msg.header.frame_id = link->getName();
        ik_pose_msg.pose.orientation.w = 1.0;
    }
    else
    {
        ik_pose_msg = boost::any_cast<geometry_msgs::msg::PoseStamped>(value);
        Eigen::Isometry3d ik_pose;
        tf2::fromMsg(ik_pose_msg.pose, ik_pose);

        if (!scene->getCurrentState().knowsFrameTransform(ik_pose_msg.header.frame_id))
        {
            RCLCPP_WARN_STREAM(LOGGER, "ik frame unknown in robot: '" << ik_pose_msg.header.frame_id << "'");
            return;
        }
        ik_pose = scene->getCurrentState().getFrameTransform(ik_pose_msg.header.frame_id) * ik_pose;

        link = scene->getCurrentState().getRigidlyConnectedParentLinkModel(ik_pose_msg.header.frame_id);

        // transform target pose such that ik frame will reach there if link does
        target_pose = target_pose * ik_pose.inverse() * scene->getCurrentState().getFrameTransform(link->getName());
    }

    // validate placed link for collisions
    collision_detection::CollisionResult collisions;
    moveit::core::RobotState sandbox_state{scene->getCurrentState()};
    bool colliding =
        !ignore_collisions && isTargetPoseCollidingInEEF(scene, sandbox_state, target_pose, link, &collisions);

    // frames at target pose and ik frame
    std::deque<visualization_msgs::msg::Marker> frame_markers;
    rviz_marker_tools::appendFrame(frame_markers, target_pose_msg, 0.1, "target frame");
    rviz_marker_tools::appendFrame(frame_markers, ik_pose_msg, 0.1, "ik frame");
    // end-effector markers
    std::deque<visualization_msgs::msg::Marker> eef_markers;
    // visualize placed end-effector
    auto appender = [&eef_markers](visualization_msgs::msg::Marker& marker, const std::string& /*name*/)
    {
        marker.ns = "ik target";
        marker.color.a *= 0.5;
        eef_markers.push_back(marker);
    };
    const auto& links_to_visualize = moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(link)
                                         ->getParentJointModel()
                                         ->getDescendantLinkModels();
    if (colliding)
    {
        mtc::SubTrajectory solution;
        std::copy(frame_markers.begin(), frame_markers.end(), std::back_inserter(solution.markers()));
        mtc::generateCollisionMarkers(sandbox_state, appender, links_to_visualize);
        std::copy(eef_markers.begin(), eef_markers.end(), std::back_inserter(solution.markers()));
        solution.markAsFailure();
        // TODO: visualize collisions
        solution.setComment(s.comment() + " eef in collision: " + listCollisionPairs(collisions.contacts, ", "));
        auto colliding_scene{scene->diff()};
        colliding_scene->setCurrentState(sandbox_state);
        spawn(mtc::InterfaceState(colliding_scene), std::move(solution));
        return;
    }
    else
        mtc::generateVisualMarkers(sandbox_state, appender, links_to_visualize);

    // determine joint values of robot pose to compare IK solution with for costs
    std::vector<double> compare_pose;
    const std::string& compare_pose_name = props.get<std::string>("default_pose");
    if (!compare_pose_name.empty())
    {
        moveit::core::RobotState compare_state(robot_model);
        compare_state.setToDefaultValues(jmg, compare_pose_name);
        compare_state.copyJointGroupPositions(jmg, compare_pose);
    }
    else
        scene->getCurrentState().copyJointGroupPositions(jmg, compare_pose);

    double min_solution_distance = props.get<double>("min_solution_distance");

    IKSolutions ik_solutions;
    auto is_valid = [scene, ignore_collisions, min_solution_distance,
                     &ik_solutions](moveit::core::RobotState* state, const moveit::core::JointModelGroup* jmg,
                                    const double* joint_positions)
    {
        for (const auto& sol : ik_solutions)
        {
            if (jmg->distance(joint_positions, sol.joint_positions.data()) < min_solution_distance)
                return false;  // too close to already found solution
        }
        state->setJointGroupPositions(jmg, joint_positions);
        ik_solutions.emplace_back();
        auto& solution{ik_solutions.back()};
        state->copyJointGroupPositions(jmg, solution.joint_positions);
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        req.contacts = true;
        req.max_contacts = 1;
        scene->checkCollision(req, res, *state);
        solution.feasible = ignore_collisions || !res.collision;
        if (!res.contacts.empty())
        {
            solution.contact = res.contacts.begin()->second.front();
        }
        return solution.feasible;
    };

    uint32_t max_ik_solutions = props.get<uint32_t>("max_ik_solutions");
    bool tried_current_state_as_seed = false;
    bool tried_automation_solution_state_as_seed = false;

    double remaining_time = timeout();
    auto start_time = std::chrono::steady_clock::now();
    while (ik_solutions.size() < max_ik_solutions && remaining_time > 0)
    {
        std::string comment = "Seeded by current state; ";

        if (tried_current_state_as_seed)
        {
            // Seed joint positions randomly
            sandbox_state.setToRandomPositions(jmg);
            comment = "Seeded randomly; ";

            if (!tried_automation_solution_state_as_seed && !automation_solution_joint_positions_.empty())
            {
                const std::vector<std::string> active_joint_names = jmg->getActiveJointModelNames();
                if (active_joint_names.empty())
                {
                    RCLCPP_WARN_STREAM(LOGGER, "No active joints in joint model group: " << jmg->getName());
                }

                // Set as many joint positions as possible from AutomationSolution
                for (const std::string& active_joint_name : active_joint_names)
                {
                    auto it = automation_solution_joint_positions_.find(active_joint_name);
                    if (it == automation_solution_joint_positions_.end())
                    {
                        RCLCPP_WARN_STREAM(LOGGER, "No seed solution available for joint: " << active_joint_name);
                        continue;
                    }
                    const double joint_position = it->second;
                    try
                    {
                        sandbox_state.setJointPositions(active_joint_name, {joint_position});
                    }
                    catch (const std::exception& e)
                    {
                        RCLCPP_WARN_STREAM(LOGGER, "Failed to seed joint \"" << active_joint_name
                                                                             << "\" from solution: " << e.what());
                        continue;
                    }
                    comment = "Seeded by solution; ";
                }
            }
            tried_automation_solution_state_as_seed = true;
        }
        tried_current_state_as_seed = true;

        size_t previous = ik_solutions.size();
        bool succeeded = sandbox_state.setFromIK(jmg, target_pose, link->getName(), remaining_time, is_valid);

        auto now = std::chrono::steady_clock::now();
        remaining_time -= std::chrono::duration<double>(now - start_time).count();
        start_time = now;

        // for all new solutions (successes and failures)
        for (size_t i = previous; i != ik_solutions.size(); ++i)
        {
            // create a new scene for each solution as they will have different robot states
            planning_scene::PlanningScenePtr solution_scene = scene->diff();
            mtc::SubTrajectory solution;
            solution.setComment(s.comment());
            std::copy(frame_markers.begin(), frame_markers.end(), std::back_inserter(solution.markers()));

            if (ik_solutions[i].feasible)
            {
                // compute cost as distance to compare_pose
                solution.setCost(s.cost() + jmg->distance(ik_solutions[i].joint_positions.data(), compare_pose.data()));
                solution.setComment(comment);
            }
            else
            {  // found an IK solution, but this was not valid
                std::stringstream ss;
                ss << "Collision between '" << ik_solutions[i].contact.body_name_1 << "' and '"
                   << ik_solutions[i].contact.body_name_2 << "'";
                solution.markAsFailure(ss.str());
            }
            // set scene's robot state
            moveit::core::RobotState& solution_state = solution_scene->getCurrentStateNonConst();
            solution_state.setJointGroupPositions(jmg, ik_solutions[i].joint_positions.data());
            solution_state.update();

            mtc::InterfaceState state(solution_scene);
            forwardProperties(*s.start(), state);

            // ik target link placement
            std::copy(eef_markers.begin(), eef_markers.end(), std::back_inserter(solution.markers()));

            spawn(std::move(state), std::move(solution));
        }

        // TODO: magic constant should be a property instead ("current_seed_only", or equivalent)
        // Yeah, you are right, these are two different semantic concepts:
        // One could also have multiple IK solutions derived from the same seed
        if (!succeeded && max_ik_solutions == 1)
            break;  // first and only attempt failed
    }

    if (ik_solutions.empty())
    {  // failed to find any solution
        planning_scene::PlanningScenePtr scene = s.start()->scene()->diff();
        mtc::SubTrajectory solution;

        solution.markAsFailure();
        solution.setComment(s.comment() + " no IK found");
        std::copy(frame_markers.begin(), frame_markers.end(), std::back_inserter(solution.markers()));

        // ik target link placement
        std_msgs::msg::ColorRGBA tint_color;
        tint_color.r = 1.0;
        tint_color.g = 0.0;
        tint_color.b = 0.0;
        tint_color.a = 0.5;
        for (auto& marker : eef_markers)
            marker.color = tint_color;
        std::copy(eef_markers.begin(), eef_markers.end(), std::back_inserter(solution.markers()));

        spawn(mtc::InterfaceState(scene), std::move(solution));
    }
}
}  // namespace dynamic_task_planner::stages