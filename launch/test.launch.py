import os
import typing
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix
import launch_ros
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import SetEnvironmentVariable


PACKAGE_NAME = 'dtc_test'


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


def generate_robot_description(sim_lc_eval):

    robot_xacro = os.path.join(
        get_package_share_directory("dtc_test"),
        "data",
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
            "name:=", "rbrobout", " ",
            "sim:=", str(sim_lc_eval), " ",
            "script_filename:=", script_filename, " ",
            "input_recipe_filename:=", input_recipe_filename, " ",
            "output_recipe_filename:=", output_recipe_filename, " ",
            # "prefix:=", "ur", " ",
            # TODO: Add physical_params parameter to include actual calibration file
        ]
    )

    robot_description_semantic = load_file(
        "dtc_test",
        "config/semantic_description.srdf"
    )

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
    ]

    moveit_controllers = {
        "moveit_simple_controller_manager": {},  # controllers_yaml
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    motion_planning_parameters: dict = {}

    def __init__(self, urdf, srdf, kinematics_yaml, controllers_yaml, sim):

        self.urdf = urdf
        self.srdf = srdf
        self.kinematics = kinematics_yaml

        MoveGroupConfiguration.moveit_controllers["moveit_simple_controller_manager"].update(controllers_yaml)

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
            {"_kinematics": self.kinematics},  # todo this is needed by dtc_test
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


def declare_launch_arguments() -> typing.List[DeclareLaunchArgument]:
    package_share_path = get_package_share_directory('dtc_test')

    return [
        DeclareLaunchArgument(name='sim',
                              default_value='true',
                              description='Execute in simulation mode'),
        DeclareLaunchArgument(name='rviz',
                              default_value='true',
                              description='Launch RVIZ'),
    ]


def launch_setup(context, *args, **kwargs):
    # Create all LaunchConfigurations
    sim_lc = LaunchConfiguration('sim')
    rviz_lc = LaunchConfiguration('rviz')
    rviz_config_lc = LaunchConfiguration('rviz_config')

    # Perform substitutions using the provided launch context
    sim = str(sim_lc.perform(context)).lower() == 'true'
    # load_world_state = str(load_world_state_lc.perform(context)).lower() == 'true'

    ##########################################################
    # Prepare Robot model (load URDF, SDRF)

    robot_description = generate_robot_description(sim)
    kinematics_yaml = load_yaml(PACKAGE_NAME, os.path.join('config', 'kinematics.yaml'))
    ompl_planning_yaml = load_yaml(PACKAGE_NAME, os.path.join('config', 'ompl_planning.yaml'))
    chomp_planning_yaml = load_yaml(PACKAGE_NAME, os.path.join('config', 'chomp_planning.yaml'))
    controllers_yaml = load_yaml(PACKAGE_NAME, os.path.join('config', 'controllers.yaml'))

    ##########################################################
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output='full',
        parameters=[{
            "robot_description": robot_description['urdf'],
            'use_sim_time': sim
        }],
        on_exit=Shutdown()
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "robot_description": robot_description['urdf'],
                "zeros": {
                    "ewellix_lift_top_joint": 0.0,
                    "ur_shoulder_pan_joint": 0.0,
                    "ur_shoulder_lift_joint": -2.6,
                    "ur_elbow_joint": 2.25,
                    "ur_wrist_1_joint": -1.16,
                    "ur_wrist_2_joint": -1.5708,
                    "ur_wrist_3_joint": 0.3745,
                }
            }
        ]
    )

    ##########################################################

    # Move Group
    move_group_config = MoveGroupConfiguration(
        robot_description['urdf'],
        robot_description['srdf'],
        kinematics_yaml,
        controllers_yaml,
        sim
    )
    move_group_config.add_planning_pipeline('ompl', ompl_planning_yaml)
    move_group_config.add_planning_pipeline('chomp', chomp_planning_yaml)
    move_group_config.set_default_planning_pipeline('ompl')

    print(move_group_config.get_move_group_config()[0])

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="full",
        parameters=move_group_config.get_move_group_config(),
        on_exit=Shutdown(),
    )

    ##########################################################
    # Dynamic_task_planner node
    dynamic_task_planner_node = Node(
        package="dtc_test",
        executable="dtc_test",
        output={'full'},
        on_exit=Shutdown(),
        arguments=["--ros-args", "--log-level", "DEBUG"],
        parameters=move_group_config.get_move_group_config(),
        # {"robot_description": robot_description['urdf']},
        # {"robot_description_semantic": robot_description['srdf']},


        # parameters=moveit_config,
        # prefix=["gdbserver localhost:3000"],
    )
    dtc_ld = TimerAction(period=10.0, actions=[dynamic_task_planner_node])

    tf_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output={'full'},
        on_exit=Shutdown(),
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        # {"robot_description": robot_description['urdf']},
        # {"robot_description_semantic": robot_description['srdf']},


        # parameters=moveit_config,
        # prefix=["gdbserver localhost:3000"],
    )

    ##########################################################
    # RVIZ
    rviz_config_file = (get_package_share_directory("dtc_test") + "/config/rviz_config.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output='full',
        arguments=["-d", rviz_config_file,
                   "--ros-args", "--log-level", "FATAL"],
        parameters=[
            {"robot_description": robot_description['urdf']},
            {"robot_description_semantic": robot_description['srdf']},
            # move_group_config.ompl_planning_pipeline_config,
            # kinematics_yaml,
        ],
        condition=IfCondition(rviz_lc),
    )
    rviz_ld = TimerAction(period=5.0, actions=[rviz_node])

    gazebo_server_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch',
                         'gzserver.launch.py')
        ),
        launch_arguments={
            "verbose": "true",
            'gui': '0',
            # 'world': world_file_path,
            # 'params_file': gazebo_file_path,
        }.items(),
    )

    ########################################
    # Gazebo client
    gazebo_client_ld = \
        TimerAction(period=3.0,  # Delay gazebo client coming up to give everything else a chance
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(get_package_share_directory('gazebo_ros'), 'launch',
                                             'gzclient.launch.py')
                            ),
                            # launch_arguments={'gui': '0'}.items(),  # removed temporarily to work in virtual machines
                            # condition=IfCondition(gazebo_client_lc)
                        )])

    #########################################
    # Other nodes for sim

    spawn_entity_node = Node(package='gazebo_ros', name='robot_spawn_entity_node', executable='spawn_entity.py',
                             arguments=['-entity', 'robot', '-topic', 'robot_description'],
                             output={'full'},
                             )

    gazebo_pkg_list = ['dtc_test',
                       'ur_description']

    if 'GAZEBO_MODEL_PATH' in os.environ:
        gz_model_path = os.environ['GAZEBO_MODEL_PATH']
        temp = os.environ['GAZEBO_MODEL_PATH'].split(':')
        for pkg in gazebo_pkg_list:
            pkg_path = get_package_prefix(pkg) + '/share'
            if pkg_path not in temp:
                gz_model_path = gz_model_path + ':' + pkg_path
    else:
        gz_model_path = ''
        for pkg in gazebo_pkg_list:
            gz_model_path = gz_model_path + ':' + get_package_prefix(pkg) + '/share'

    print('Setting GAZEBO_MODEL_PATH = ' + gz_model_path)
    print('Setting GAZEBO_MODEL_DATABASE_URI = http://localhost:8099/models')



    print("Common launch completed launch description definition. Returning descriptions to ROS launch.")
    return [

        # Set env_var
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=gz_model_path),
        SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value='http://localhost:8099/models'),

        # simulation_ld,
        # agv_ld,  # Make sure this is early so the robot joint state pubs come up
        rviz_ld,

        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        move_group_node,

        tf_world,
        # dynamic_task_planner_node,
        dtc_ld,

        #gazebo_server_ld,
        #spawn_entity_node,
        #gazebo_client_ld,


        launch_ros.actions.SetParameter(name='use_sim_time', value=sim)
    ]


def generate_launch_description():
    return LaunchDescription(declare_launch_arguments() + [OpaqueFunction(function=launch_setup)])
