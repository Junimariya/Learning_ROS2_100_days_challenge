from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.launch_description_sources import IncludelLaunchDescription


def generate_launch_description():

    IncludeLaunchDescription(