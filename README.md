# SSD Nodelet
<div align="center">
    <img src="doc/ssd_nodelet.png" width="1080">
</div> 
<div align="center">
    <img src="doc/ssd_nodelet_pose.png" width="1080"> 
</div> 

## About
* SSD(Single Shot MultiBox Detector)による物体検出
* Nodelet実装による高速化
* 処理速度は、SSD Nodeの約4倍
* 50Hzでカメラ画像を入力した場合
    SSD Node
    ```
    average rate: 12.962
        min: 0.050s max: 0.088s std dev: 0.00711s window: 77
    ```
    SSD Nodelet
    ```
    average rate: 49.889
        min: 0.015s max: 0.027s std dev: 0.00243s window: 49
    ```

## Setup
### ホスト環境の場合
```python
$ cd ~/catkin_ws/src/ssd_nodelet/
$ bash setup.sh
$ cd ~/catkin_ws
$ catkin_make
```

### Docker環境の場合
- ホスト環境
```python 
$ cd ~/〇〇/〇〇/src/ssd_nodelet/
$ bash setup_rules.sh
```
- Docker環境
```python 
$ cd ~/catkin_ws/src/ssd_nodelet/
$ bash setup.sh
$ cd ~/catkin_ws
$ catkin_make
```

## How To Use
### Camera
```bash
$ roslaunch ssd_nodelet camera_720p_16_9.launch
# 他に
# camera_1080p_16_9.launch  camera_480p_16_9.launch   camera_720p_16_9.launch
# camera_1080p_3_2.launch   camera_480p_3_2.launch    camera_720p_3_2.launch
```
※以下のようなエラーが発生した場合
```python
[ERROR] [1663911409.917317256]: Permission denied opening /dev/bus/usb/001/002
```
次のを実行してください
```python
sudo chmod o+w /dev/bus/usb/001/002
```

### Object Detect
```bash
$ roslaunch ssd_nodelet face_detect.launch  <- face detect mode
$ roslaunch ssd_nodelet object_detect.launch  <- object detect mode
```
### Object Pose Detect
```bash
$ roslaunch ssd_nodelet face_detect_pose.launch  <- face detect mode
$ roslaunch ssd_nodelet object_detect_pose.launch  <- object detect mode
```

### Publications:
|トピック名|型|意味|
|---|---|---|
|/ssd_object_detect/detect_result|sensor_msgs/Image|出力画像(検出結果)|
|/ssd_object_detect/object_name|sobit_common_msg/StringArray|検出物体の名前のリスト|
|/ssd_object_detect/object_rect|sobit_common_msg/BoundingBoxes|検出物体のバウンディングボックス情報|

#### Ooly Object Pose
|トピック名|型|意味|
|---|---|---|
|/ssd_object_detect/object_pose|sobit_common_msg/ObjectPoseArray|検出物体の位置|

### Subscriptions:
|トピック名|型|意味|
|---|---|---|
|/camera/rgb/image_raw|sensor_msgs/Image|入力画像|
|/ssd_object_detect/detect_ctrl|std_msgs/Bool|検出のオンオフ|

#### Ooly Object Pose
|トピック名|型|意味|
|---|---|---|
|/camera/depth/points|sensor_msgs/PointCloud2|入力点群|

### Parameters:
|パラメータ名|型|意味|
|---|---|---|
|/ssd_object_detect/ssd_nodelet/ssd_img_show_flag|bool|検出画像の描画をするか|
|/ssd_object_detect/ssd_nodelet/ssd_execute_default|bool|起動時に検出を開始するか|
|/ssd_object_detect/ssd_nodelet/ssd_pub_result_image|bool|/detect_resultをパブリッシュをするかどうか|
|/ssd_object_detect/ssd_nodelet/ssd_image_topic_name|string|入力画像のトピック名|
|/ssd_object_detect/ssd_nodelet/ssd_in_scale_factor|double|Caffeで扱うBlob形式の変換時のスケールパラメータ|
|/ssd_object_detect/ssd_nodelet/ssd_confidence_threshold|double|検出結果リストに追加される結果の信頼度の閾値|
|/ssd_object_detect/ssd_nodelet/ssd_prototxt_name|double|string|prototxtファイルパス|
|/ssd_object_detect/ssd_nodelet/ssd_caffemodel_name|string|caffeモデルファイルパス|
|/ssd_object_detect/ssd_nodelet/ssd_class_names_file|string|検出する物体名リストファイルパス|
|/ssd_object_detect/ssd_nodelet/object_specified_enabled|bool|特定の物体のみ検出を行うか|
|/ssd_object_detect/ssd_nodelet/specified_object_name|string|検出する特定の物体名(物体ラベルにない名前は却下されます)|

#### Ooly Object Pose
|パラメータ名|型|意味|
|---|---|---|
|/ssd_object_detect/ssd_nodelet/use_tf|bool|tfによる座標登録するか|
|/ssd_object_detect/ssd_nodelet/target_frame|string|基準座標フレーム名|
|/ssd_object_detect/ssd_nodelet/ssd_cloud_topic_name|string|入力点群のトピック名|