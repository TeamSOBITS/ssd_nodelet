
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    voc_object_prototxt_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object.prototxt')
    voc_object_caffemodel_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object.caffemodel')
    voc_object_names_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object_names.txt')

    # 顔認識用
    # voc_object_prototxt_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'face.prototxt')
    # voc_object_caffemodel_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'face.caffemodel')
    # voc_object_names_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'face_names.txt')

    image_show_flag = LaunchConfiguration("image_show_flag")
    image_show_flag_cmd = DeclareLaunchArgument(
        "image_show_flag",
        description="is image show?",
        default_value="true",
    )

    execute_default = LaunchConfiguration("execute_default")
    execute_default_cmd = DeclareLaunchArgument(
        "execute_default",
        description="Whether to start SSD enabled",
        default_value="true",
    )

    base_frame_name = LaunchConfiguration("base_frame_name")
    base_frame_name_cmd = DeclareLaunchArgument(
        "base_frame_name",
        description="Base frame name for TF and 3D detection",
        default_value="base_footprint",
    )

    image_topic_name = LaunchConfiguration("image_topic_name")
    image_topic_name_cmd = DeclareLaunchArgument(
        "image_topic_name",
        description="ROS Topic Name of sensor_msgs/msg/Image message",
        # default_value="/camera/color/image_raw",      ## realsense
        default_value="/image_raw",                      ## azure_kinect
        # default_value="",             ## orbbec_series
        # default_value="/camera/rgb/image_raw",            ## xtion
    )

    point_cloud_topic_name = LaunchConfiguration("point_cloud_topic_name")
    point_cloud_topic_name_cmd = DeclareLaunchArgument(
        "point_cloud_topic_name",
        description="ROS Topic Name of sensor_msgs/msg/PointCloud2 message",
        # default_value="/camera/depth/color/points",   ## realsense
        # default_value="/points2",                            ## azure_kinect
        # default_value="",     ## orbbec_series
        default_value="/camera/depth_registered/points",     ## xtion
    )

    depth_image_topic_name = LaunchConfiguration("depth_image_topic_name")
    depth_image_topic_name_cmd = DeclareLaunchArgument(
        "depth_image_topic_name",
        description="ROS Topic Name of sensor_msgs/msg/Image message",
        # default_value="/camera/depth/image_raw",      ## realsense
        # default_value="",                      ## azure_kinect
        # default_value="",             ## orbbec_series
        default_value="/camera/depth/image_raw",    ## xtion
    )

    info_topic_name = LaunchConfiguration("info_topic_name")
    info_topic_name_cmd = DeclareLaunchArgument(
        "info_topic_name",
        description="ROS Topic Name of sensor_msgs/msg/Image message",
        # default_value="/camera/depth/image_raw",      ## realsense
        # default_value="",                      ## azure_kinect
        # default_value="",             ## orbbec_series
        default_value="/camera/rgb/camera_info", ## xtion
    )

    in_scale_factor = LaunchConfiguration("in_scale_factor")
    in_scale_factor_cmd = DeclareLaunchArgument(
        "in_scale_factor",
        description="Caffemodelで扱う際の変換時スケールパラメータ",
        default_value="0.007843",
        # default_value="1.00",
    )

    confidence_threshold = LaunchConfiguration("confidence_threshold")
    confidence_threshold_cmd = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.5",
        description="Minimum probability of a detection to be published",
    )

    ssd_prototxt_name = LaunchConfiguration("ssd_prototxt_name")
    ssd_prototxt_name_cmd = DeclareLaunchArgument(
        "ssd_prototxt_name",
        description="ニューラルネットの構造を記述したtxt",
        default_value=voc_object_prototxt_path,
    )

    ssd_caffemodel_name = LaunchConfiguration("ssd_caffemodel_name")
    ssd_caffemodel_name_cmd = DeclareLaunchArgument(
        "ssd_caffemodel_name",
        description="学習済みモデル",
        default_value=voc_object_caffemodel_path,
    )

    ssd_class_names_file = LaunchConfiguration("ssd_class_names_file")
    ssd_class_names_file_cmd = DeclareLaunchArgument(
        "ssd_class_names_file",
        description="物体名リスト",
        default_value=voc_object_names_path,
    )

    object_specified_enabled = LaunchConfiguration("object_specified_enabled")
    object_specified_enabled_cmd = DeclareLaunchArgument(
        "object_specified_enabled",
        description="特定の物体検出フラグ",
        default_value='true',
    )

    specified_object_name = LaunchConfiguration("specified_object_name")
    specified_object_name_cmd = DeclareLaunchArgument(
        "specified_object_name",
        description="特定する場合に検出する物体名",
        default_value='person',
    )

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        description="Namespace for the nodes",
        default_value="ssd_ros",
    )

    positioning_detection_mode = LaunchConfiguration("positioning_detection_mode")
    positioning_detection_mode_cmd = DeclareLaunchArgument(
        "positioning_detection_mode",
        description="Detection mode for 3D positioning",
        # default_value="point_cloud",
        # default_value="depth_image",
        default_value="fast_point",
    )

    ssd_ros_node_cmd = Node(
        package="ssd_ros",
        executable="single_shot_multibox_detector",
        name="ssd_ros",
        namespace=namespace,
        parameters=[
            {
                "image_show_flag": image_show_flag,
                "execute_default": execute_default,
                "image_topic_name": image_topic_name,
                "in_scale_factor": in_scale_factor,
                "confidence_threshold": confidence_threshold,
                "ssd_prototxt_name": ssd_prototxt_name,
                "ssd_caffemodel_name": ssd_caffemodel_name,
                "ssd_class_names_file": ssd_class_names_file,
                "object_specified_enabled": object_specified_enabled,
                "specified_object_name": specified_object_name,
            },
        ],
        output="screen"
    )

    use_3d = LaunchConfiguration("use_3d")
    use_3d_cmd = DeclareLaunchArgument(
        "use_3d", default_value="true", description="Whether to activate 3D detections"
    )

    bbox_to_3d_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("image_to_position"),
                "launch",
                "bbox_to_3d.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "base_frame_name": base_frame_name,
            "bbox_topic_name": "/ssd_ros/objects_rect",
            "cloud_topic_name": point_cloud_topic_name,
            "depth_image_topic_name": depth_image_topic_name,
            "info_topic_name": info_topic_name,
            "execute_default": execute_default,
            "enable_id": "False",
            "positioning_detection_mode": positioning_detection_mode,
        }.items(),
        condition=IfCondition(use_3d),  # use_3dがTrueのときのみ実行
    )

    return LaunchDescription(
        [
            image_show_flag_cmd,
            execute_default_cmd,
            image_topic_name_cmd,
            point_cloud_topic_name_cmd,
            depth_image_topic_name_cmd,
            info_topic_name_cmd,
            in_scale_factor_cmd,
            confidence_threshold_cmd,
            ssd_prototxt_name_cmd,
            ssd_caffemodel_name_cmd,
            ssd_class_names_file_cmd,
            object_specified_enabled_cmd,
            specified_object_name_cmd,
            namespace_cmd,
            positioning_detection_mode_cmd,
            base_frame_name_cmd,
            ssd_ros_node_cmd,
            use_3d_cmd,
            bbox_to_3d_cmd,
        ]
    )