import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    ssd_ros_share_dir = get_package_share_directory('ssd_ros')

    voc_object_prototxt_path = os.path.join(ssd_ros_share_dir, 'models', 'voc_object.prototxt')
    voc_object_caffemodel_path = os.path.join(ssd_ros_share_dir, 'models', 'voc_object.caffemodel')
    voc_object_names_path = os.path.join(ssd_ros_share_dir, 'models', 'voc_object_names.txt')
    # 顔認識用
    # face_prototxt_path = os.path.join(ssd_ros_share_dir, 'models', 'face.prototxt')
    # face_caffemodel_path = os.path.join(ssd_ros_share_dir, 'models', 'face.caffemodel')
    # face_names_path = os.path.join(ssd_ros_share_dir, 'models', 'face_names.txt')

    # 全ての物体
    # bbox_to_tf_path = os.path.join(get_package_share_directory('bbox_to_tf'),'launch','bbox_to_tf.launch.xml')
    # 単一種類の物体
    bbox_to_tf_fast_shot_path = os.path.join(get_package_share_directory('bbox_to_tf'),'launch','bbox_to_tf_fast_shot.launch.xml')

    launch_arguments = [
        # 検出画像の描画フラグ
        DeclareLaunchArgument('img_show_flag', default_value='true'),
        # 起動時の検出開始フラグ
        DeclareLaunchArgument('execute_default', default_value='true'),
        # 入力
        # real_sense?
        # DeclareLaunchArgument('img_topic_name', default_value='/camera/camera/color/image_raw'),
        # azrue_kinect
        # DeclareLaunchArgument('image_topic_name', default_value='/rgb/image_raw'),
        # web camera
        DeclareLaunchArgument('image_topic_name', default_value='/image_raw'),

        # 点群情報
        # real_sense
        # DeclareLaunchArgument('point_cloud_topic_name', default_value='/camera/camera/depth/color/points'),
        # azrue_kinect
        DeclareLaunchArgument('point_cloud_topic_name', default_value='/points2'),

        # Caffemodelで扱う際の変換時スケールパラメータ
        DeclareLaunchArgument('in_scale_factor', default_value='0.007843'),
        # 顔認識用
        # DeclareLaunchArgument('in_scale_factor', default_value='1.00'),
        # 検出結果の信頼度しきい値
        DeclareLaunchArgument('confidence_threshold', default_value='0.5'),

        # ニューラルネットの構造を記述したtxt
        DeclareLaunchArgument('ssd_prototxt_name', default_value=voc_object_prototxt_path),
        # 学習済みモデル
        DeclareLaunchArgument('ssd_caffemodel_name', default_value=voc_object_caffemodel_path),
        # 物体名リスト
        DeclareLaunchArgument('ssd_class_names_file', default_value=voc_object_names_path),
        # 顔認識用
        # DeclareLaunchArgument('ssd_prototxt_name', default_value=face_prototxt_path),
        # DeclareLaunchArgument('ssd_caffemodel_name', default_value=face_caffemodel_path),
        # DeclareLaunchArgument('ssd_class_names_file', default_value=face_names_path),

        # 特定の物体検出フラグ
        DeclareLaunchArgument('object_specified_enabled', default_value='true'),
        # 検出する物体名(names.txtにない場合は棄却)
        DeclareLaunchArgument('specified_object_name', default_value='person')
    ]

    # ノード定義
    ssd_node = Node(
        package='ssd_ros',
        executable='single_shot_multibox_detector',
        name='ssd_node',
        parameters=[{
            'img_show_flag': LaunchConfiguration('img_show_flag'),
            'execute_default': LaunchConfiguration('execute_default'),

            'image_topic_name': LaunchConfiguration('image_topic_name'),
            'in_scale_factor': LaunchConfiguration('in_scale_factor'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),

            'ssd_prototxt_name': LaunchConfiguration('ssd_prototxt_name'),
            'ssd_caffemodel_name': LaunchConfiguration('ssd_caffemodel_name'),
            'ssd_class_names_file': LaunchConfiguration('ssd_class_names_file'),

            'object_specified_enabled': LaunchConfiguration('object_specified_enabled'),
            'specified_object_name': LaunchConfiguration('specified_object_name')
        }]
    )

    return LaunchDescription([
        *launch_arguments,
        ssd_node,
        # IncludeLaunchDescription(
        #     XMLLaunchDescriptionSource(bbox_to_tf_fast_shot_path), # bbox_to_tf_path, bbox_to_tf_fast_shot_path
        #     launch_arguments=[
        #         ('node_name', 'ssd_ros_bbox_to_tf'),
        #         ('base_frame_name', 'base_footprint'),
        #         ('bbox_topic_name', '/ssd_ros/objects_rect'),
        #         ('cloud_topic_name', LaunchConfiguration('point_cloud_topic_name')),
        #         ('image_topic_name', LaunchConfiguration('image_topic_name')),
        #         ('execute_default', 'true'),
        #         ('cluster_tolerance', '0.008'),
        #         ('min_clusterSize', '10'),
        #         ('max_clusterSize', '2000000'),
        #         ('noise_point_cloud_range', '0.03'),
        #         ('rviz', 'true')
        #     ],
        # )
    ])


    # MEMO
        # ssd_ros
            # single shot multibox detector
                # 入力画像1枚から複数のバウンディングボックスの検出ができるシステム
                    # 映像の場合は、フレームごとにやっているはず（何フレームごととかは今の所わからん。調べたらわかるかも？？？）
                # SOBITSのssd_rosはcaffeというCNNモデルベース

        # roscore
            # ROSにおいてノード間の通信の中核となるシステム
            # 通常：node_A→roscore, roscore→node_Bで2回のコピーを経てデータの授受が行われる

        # nodelet(ROS)
            # ROSにおいて、nodelet_managerと呼ばれるプロセスの配下で複数のノードを同一プロセスとして管理（同じメモリ領域で実行）するシステム
            # 複数のプロセスを同じメモリ領域で実行することで、共有するデータ(topic,service,etc...)の読み書きにおいてコピー回数を削減
            # nodelet：node_A, node_Bはトピックを共有（メモリ領域が同一）したがって、コピーが発生しない　

        # DDS   
            # ROS2は、DDS(Data Distribution System)と呼ばれるノード間での直接通信を誘導・効率化するシステムがある
            # この場合、ノード間で直接データがやり取りされることから、コピーはnode_A→node_Bの1回

        # component_container(ROS2)
            # ROS2におけるnodeletのようなシステム
            # component_container_mtとcomponent_containerがあり、mtはマルチスレッド所利用(リアルタイム処理向きらしい)

        #tf
            # rosでは、bbox_to_tfを使用していない（そもそもbbox_to_tfのパッケージの導入前？）ので、以下のtopicを使用している
                # /ssd_object_detect/ssd_nodelet/use_tf	                tfによる座標登録するか
                # /ssd_object_detect/ssd_nodelet/target_frame	        基準座標フレーム名
                # /ssd_object_detect/ssd_nodelet/ssd_cloud_topic_name	入力点群のトピック名

            # azrue_kinect用に分けられたlaunchがあるが、基本的には今のSOBITSでは分ける必要性がなくなっていると思われる

        # caffe
            # https://qiita.com/yka2ki/items/4acb648666ad628459db
            # https://qiita.com/Hiroki11x/items/7017ac0c03df8011b53c
        
        # in_scale_factor
            # 物体認識や、顔認識用のcaffeモデルが期待する入力に入力スケールを合わせるためのパラメーターらしい
        
        # IncludeLaunchDescription
            # 第1引数：呼び出すlaunchファイルのファイルパス
            # 第2引数：タプル（トピック名，値）のリスト

        # XMLLaunchDescriptionSource
            # launchファイルがxml形式のときLaunchDescriptionを生成するのに利用

        # LaunchDescription
            # launchファイルが実行されると、メモリ上に生成されるデータ
            # ノードの起動や他のlaunchファイルの起動に関する情報であり、実行されたものから、メモリ上で処理される 

        # アンパック演算子 * リストやタプルの中身を展開する
            # イテラブルオブジェクトの分解を行う演算子
            # *launch_arguments
            # 直接launchファイルを立ち上げる場合、LaunchDescriptionに引数を読み込ませなければ起動しない
                # トピックが未定義になり起動できない
            # 直接実行しない場合は冗長
            # Nodeの定義自体をreturn文の中に書き込むほうが一般的なよう
            # そのほうが楽
    