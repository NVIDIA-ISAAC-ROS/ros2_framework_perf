import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
import pytest


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description for testing EmitterNode."""
    # Get the launch directory
    launch_dir = os.path.join(get_package_share_directory('ros2_framework_perf'), 'launch')

    # Include the emitter launch file
    emitter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'emitter.launch.py')),
    )

    return LaunchDescription([
        emitter_launch,
        # Tell launch when to start the test
        ReadyToTest()
    ])