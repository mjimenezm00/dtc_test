import os
import yaml

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.parameters_type import SomeParameters
from launch_ros.utilities import normalize_parameters, evaluate_parameters
from launch_ros.substitutions import FindPackageShare


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        raise Exception('File not found: {}'.format(absolute_file_path))


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        raise Exception('File not found: {}'.format(absolute_file_path))


def dump_yaml(file_path, data):
    try:
        print('Attempting to dump yaml to: {}'.format(file_path))
        with open(file_path, "w") as file:
            return yaml.dump(data, file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        raise Exception('File not found: {}'.format(file_path))


def dump_parameters(file_path, parameters, namespace='/**'):
    print(parameters)

    parameters_to_dump = {}
    for parameter in parameters:
        if isinstance(parameter, dict):
            parameters_to_dump.update(parameter)
        elif isinstance(parameter, Path):
            try:
                with open(parameter, "r") as file:
                    loaded_params = yaml.safe_load(file)  # type: dict
            except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
                raise
            parameters_to_dump.update(
                loaded_params[next(iter(loaded_params))]["ros__parameters"]
            )
        else:
            raise ValueError('Failed to dump unexpected parameter type: {}'.format(type(parameter)))

    dump_yaml(
        file_path,
        {
            namespace: {
                "ros__parameters": parameters_to_dump
            }
        }
    )


def dump_parameters_opaque(
        context: LaunchContext,
        file_path: SomeSubstitutionsType,
        parameters: SomeParameters,
        namespace: SomeSubstitutionsType = '/**'):
    dump_parameters(
        file_path=perform_substitutions(context, normalize_to_list_of_substitutions(file_path)),
        parameters=evaluate_parameters(context, normalize_parameters(parameters)),
        namespace=perform_substitutions(context, normalize_to_list_of_substitutions(namespace))
    )


def generate_robot_description(robot_name_lc_eval, sim_lc_eval, package_name="", relative_path=""):

    if (package_name != "" and relative_path != ""):
        print("Common launch using external robot xacro:")
        robot_xacro = os.path.join(
            get_package_share_directory(package_name),
            relative_path
        )
    else:
        print("Common launch using local robot xacro:")
        robot_xacro = os.path.join(
            get_package_share_directory("bmt_common_bringup"),
            "agv_descriptions",
            robot_name_lc_eval,
            "urdf",
            "main.urdf.xacro"
        )
    # print(robot_xacro)

    script_filename = PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "ros_control.urscript"])
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"])
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"])

    print("Common launch evaluating robot xacro: {}".format(robot_xacro))
    robot_description_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            robot_xacro, " ",
            "name:=", robot_name_lc_eval, " ",
            "sim:=", str(sim_lc_eval), " ",
            "script_filename:=", script_filename, " ",
            "input_recipe_filename:=", input_recipe_filename, " ",
            "output_recipe_filename:=", output_recipe_filename, " ",
            # "prefix:=", "ur", " ",
            # TODO: Add physical_params parameter to include actual calibration file
        ]
    )

    print("Common launch defining move group configuration files from AGV profile: {}".format(
        "bmt_common_bringup/agv_descriptions/{}/config/move_group/".format(robot_name_lc_eval)
    ))
    robot_description_semantic = load_file(
        "bmt_common_bringup",
        "agv_descriptions/{}/config/move_group/semantic_description.srdf".format(robot_name_lc_eval))

    return {
        "urdf": robot_description_xml,
        "srdf": robot_description_semantic,
    }


class MoveGroupConfiguration:
    planning_pipeline_config: dict = {
        "default_planning_pipeline": "",
        "planning_pipelines": []
    }

    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.0,
        "trajectory_execution.allowed_goal_duration_margin": 10.0,
        "trajectory_execution.allowed_start_tolerance": 0.03,
        "trajectory_execution.trajectory_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description_semantic": True,
        "publish_robot_description": True,
    }

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    caps = [
        "move_group/ExecuteTaskSolutionCapability",
        "ee_manager/EndEffectorMonitor"
    ]

    moveit_controllers = {
        "moveit_simple_controller_manager": {},  # controllers_yaml
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    motion_planning_parameters: dict = {}

    def __init__(self, urdf, srdf, kinematics_yaml, controllers_yaml, load_world_state, sim):

        self.urdf = urdf
        self.srdf = srdf
        self.kinematics = kinematics_yaml

        MoveGroupConfiguration.moveit_controllers["moveit_simple_controller_manager"].update(controllers_yaml)

        if load_world_state:
            MoveGroupConfiguration.caps.append(
                "moveit_world_model_monitor/WorldModelMonitorCapability"
            )

        self.sim = sim

        self.move_group_capabilities = {
            "capabilities": " ".join(MoveGroupConfiguration.caps)
        }

    def add_planning_pipeline(self, name, pipeline_yaml):

        MoveGroupConfiguration.planning_pipeline_config["planning_pipelines"].append(name)
        MoveGroupConfiguration.planning_pipeline_config.update(
            {name: pipeline_yaml}
        )

    def set_default_planning_pipeline(self, name):
        MoveGroupConfiguration.planning_pipeline_config["default_planning_pipeline"] = name

    def get_move_group_config(self):

        data = [
            {"robot_description": self.urdf},
            {"robot_description_semantic": self.srdf},
            {"robot_description_kinematics": self.kinematics},
            MoveGroupConfiguration.planning_pipeline_config,
            MoveGroupConfiguration.trajectory_execution,
            MoveGroupConfiguration.moveit_controllers,
            self.move_group_capabilities,
            MoveGroupConfiguration.planning_scene_monitor_parameters,
            {"use_sim_time": self.sim},
            {"sensors_3d": {}}
        ]

        return data

    def get_motion_planning_config(self):

        data: dict = {}
        data.update(self.kinematics)
        data.update(self.planning_pipeline_config)

        return data
