//#include <job_planner_msgs/msg/yaml_parameters.hpp>
#include <moveit/task_constructor/properties.h>
#include <moveit/task_constructor/stage.h>
#include <moveit_task_constructor_msgs/msg/property.hpp>
#include <rclcpp/parameter.hpp>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_eigen/tf2_eigen.hpp>

#pragma once

namespace dynamic_task_planner::utils
{

static std::vector<rclcpp::Parameter> getStageParameters(const rclcpp::ParameterMap& parameter_map,
                                                         const std::string& stage_name)
{
    auto it = parameter_map.find(stage_name);
    if (it == parameter_map.end())
    {
        throw std::runtime_error(std::string("Stage \"" + stage_name + "\" is not defined in parameter map"));
    }
    return it->second;
}

/**
 * @brief Mirror rosparam_shortcuts to get parameter value based on expected value type
 * @param parameters - ROS2 parameters
 * @param name - name of parameter to get
 * @param value - resulting loaded values
 */
template <typename T>
void getParameterValue(const std::vector<rclcpp::Parameter>& parameters, const std::string& name, T& value)
{
    // Get property from static properties
    auto it = std::find_if(parameters.begin(), parameters.end(),
                           [&name](const auto& parameter) { return parameter.get_name() == name; });
    if (it == parameters.end())
    {
        throw std::runtime_error(std::string("Failed to get parameter \"" + name + "\""));
    }
    const rclcpp::Parameter& parameter = *it;

    // Return parameter value
    value = parameter.get_value<T>();
}

inline void getParameterValue(const std::vector<rclcpp::Parameter>& parameters, const std::string& name,
                              rclcpp::Duration& value)
{
    double seconds;
    getParameterValue(parameters, name, seconds);
    value = rclcpp::Duration::from_seconds(seconds);
}

inline void getParameterValue(const std::vector<rclcpp::Parameter>& parameters, const std::string& name,
                              Eigen::Isometry3d& value)
{
    std::vector<double> values;
    getParameterValue(parameters, name, values);
    rosparam_shortcuts::convertDoublesToEigen(values, value);
}

inline void getParameterValue(const std::vector<rclcpp::Parameter>& parameters, const std::string& name,
                              geometry_msgs::msg::Pose& value)
{
    Eigen::Isometry3d eigen_pose;
    getParameterValue(parameters, name, eigen_pose);
    tf2::convert(eigen_pose, value);
}

template <typename T> T getParameterValue(const std::vector<rclcpp::Parameter>& parameters, const std::string& name)
{
    T value;
    getParameterValue(parameters, name, value);
    return value;
}

template <typename T>
T getParameterValue(const rclcpp::ParameterMap& parameter_map, const std::string& stage_name,
                    const std::string& property_name)
{
    // Get stage properties
    const auto stage_properties = getStageParameters(parameter_map, stage_name);

    // Get property from stage properties
    return getParameterValue<T>(stage_properties, property_name);
}

template <typename T>
void setPropertyFromParameters(const std::vector<rclcpp::Parameter>& parameters, const std::string& property_name,
                               moveit::task_constructor::PropertyMap& property_map)
{
    T value;
    getParameterValue(parameters, property_name, value);
    property_map.set(property_name, value);
}

inline void setPropertyFromRosMsg(const moveit_task_constructor_msgs::msg::Property& ros_msg,
                                  const std::string& property_name, moveit::task_constructor::PropertyMap& property_map)
{
    boost::any value = moveit::task_constructor::Property::deserialize(ros_msg.type, ros_msg.value);
    property_map.set(property_name, value);
}

}  // namespace dynamic_task_planner::utils