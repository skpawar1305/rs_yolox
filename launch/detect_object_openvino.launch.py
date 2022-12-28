import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rs_yolox_dir = get_package_share_directory('rs_yolox')

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

    yolox_openvino = launch_ros.actions.Node(
        package="rs_yolox", executable="detect_yolox_openvino",output="screen",
        parameters=[
            {"device" : 'CPU'},
            {"model_path" : rs_yolox_dir+"/weights/yolov5s.onnx"},
            {"conf" : 0.3},
        ],
    )

    rqt_graph = launch_ros.actions.Node(
        package="rqt_graph", executable="rqt_graph",
    )

    return launch.LaunchDescription([
        video_device_arg,
        webcam,
        yolox_openvino,
        # rqt_graph
    ])
