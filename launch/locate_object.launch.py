import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    yolox_ros_share_dir = get_package_share_directory('yolox_ros_py')
    realsense2_dir = get_package_share_directory('realsense2_camera')
    rs_locate_object_dir = get_package_share_directory('rs_locate_object')
    use_rviz = LaunchConfiguration('rviz', default=False)    

    realsense2_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [realsense2_dir, '/launch/rs_launch.py']),
        launch_arguments={
            'align_depth.enable': 'true',
        }.items()
    )

    yolox_ros = launch_ros.actions.Node(
        package="yolox_ros_py", executable="yolox_ros",
        parameters=[
            {"yolox_exp_py" : yolox_ros_share_dir+'/yolox_nano.py'},
            {"device" : 'cpu'},
            {"fp16" : True},
            {"fuse" : False},
            {"legacy" : False},
            {"trt" : False},
            {"ckpt" : yolox_ros_share_dir+"/yolox_nano.pth"},
            {"conf" : 0.3},
            {"threshold" : 0.65},
            {"resize" : 640},
        ],
        remappings=[
            ('/image_raw', '/camera/color/image_raw'),
        ],
    )

    rs_locate_object = launch_ros.actions.Node(
        package="rs_locate_object", executable="locate_yolox",
    )

    rviz_config = os.path.join(rs_locate_object_dir, 'resource', 'default.rviz')
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    return launch.LaunchDescription([
        realsense2_camera_launch,
        yolox_ros,
        rs_locate_object,
        rviz,
    ])
