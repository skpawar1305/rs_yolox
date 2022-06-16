import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    realsense2_dir = get_package_share_directory('realsense2_camera')
    rs_yolox_dir = get_package_share_directory('rs_yolox')
    use_rviz = LaunchConfiguration('rviz', default=False)    

    realsense2_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [realsense2_dir, '/launch/rs_launch.py']),
        launch_arguments={
            'align_depth.enable': 'true',
        }.items()
    )

    rs_yolox = launch_ros.actions.Node(
        package="rs_yolox", executable="detect_yolox_onnx",
        parameters=[
            {"input_shape/width": 416},
            {"input_shape/height": 416},

            {"with_p6" : False},
            {"model_path" : rs_yolox_dir+"/weights/yolox_nano.onnx"},
            {"conf" : 0.3},
        ],
    )

    rviz_config = os.path.join(rs_yolox_dir, 'resource', 'default.rviz')
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        condition=launch.conditions.IfCondition(use_rviz)
    )
    video_device = LaunchConfiguration('video_device', default='/dev/video0')
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device'
    )    
    webcam = launch_ros.actions.Node(
        package="v4l2_camera", executable="v4l2_camera_node",
        parameters=[
            {"image_size": [640,480]},
            {"video_device": video_device},
        ],
    )
    return launch.LaunchDescription([
        realsense2_camera_launch,
        rs_yolox,
        rviz,
        webcam,
    ])
