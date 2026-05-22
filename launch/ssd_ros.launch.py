
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    # SSD Configfile path
    config_file = os.path.join(get_package_share_directory('ssd_ros'), 'config', 'detection_config.yaml')

    # SSD (Face Models 'Directory')
    face_model_dir = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'face_model')

    # SSD (Objects Models 'Directory') (example: person, chair, etc...)
    objects_model_dir = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'objects_model')



    execute_default = LaunchConfiguration("execute_default")
    execute_default_cmd = DeclareLaunchArgument(
        "execute_default",
        description="Whether to start SSD enabled",
        default_value="true",
    )


    model_directory = LaunchConfiguration("model_directory")
    model_directory_cmd = DeclareLaunchArgument(
        "model_directory",
        description="Directory containing the model files (prototxt, caffemodel, class names)",
        default_value=objects_model_dir, # objects_model_dir or face_model_dir
    )


    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        description="Namespace for the nodes",
        default_value="ssd_ros",
    )


    ssd_ros_node_cmd = Node(
        package="ssd_ros",
        executable="single_shot_multibox_detector",
        name="ssd_ros",
        namespace=namespace,
        parameters=[
            {
                "execute_default": execute_default,
                "model_directory": model_directory,
            },
            config_file,
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
            "execute_default": execute_default,
            "params_file": config_file,
        }.items(),
        condition=IfCondition(use_3d),  # use_3dがTrueのときのみ実行
    )

    return LaunchDescription(
        [
            execute_default_cmd,
            model_directory_cmd,
            namespace_cmd,
            ssd_ros_node_cmd,
            use_3d_cmd,
            bbox_to_3d_cmd,
        ]
    )