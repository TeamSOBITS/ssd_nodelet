# SSD Nodelet：SSD(Single Shot MultiBox Detector)による物体検出（Nodelet実装による高速化）
<div align="center">
    <img src="doc/ssd_nodelet.png" width="1080">
</div> 
<div align="center">
    <img src="doc/ssd_nodelet_pose.png" width="1080"> 
</div> 

## How To Use
### Object Detect
```bash
$ roslaunch ssd_nodelet face_detect.launch  <- face detect mode
$ roslaunch ssd_nodelet object_detect.launch  <- object detect mode
```

### Publications:
|トピック名|型|意味|
|---|---|---|
|/ssd_object_detect/detect_result|sensor_msgs/Image|検出結果の画像|
|/ssd_object_detect/objects_name|sobit_common_msg/StringArray|検出物体の名前のリスト|
|/ssd_object_detect/objects_rect|sobit_common_msg/BoundingBoxes|検出物体のバウンディングボックス情報|

### Subscriptions:
|トピック名|型|意味|
|---|---|---|
|/camera/rgb/image_raw|sensor_msgs/Image|検出される画像|
|/ssd_object_detect/detect_ctrl|std_msgs/Bool|検出のオンオフ|

### Parameters:
|パラメータ名|型|意味|
|---|---|---|
|/ssd_object_detect/ssd_nodelet/ssd_img_show_flag|bool|検出画像の描画をするか|
|/ssd_object_detect/ssd_nodelet/ssd_execute_default|bool|起動時に検出を開始するか|
|/ssd_object_detect/ssd_nodelet/ssd_pub_result_image|bool|/detect_resultをパブリッシュをするかどうか|
|/ssd_object_detect/ssd_nodelet/ssd_image_topic_name|string|検出される画像のトピック名|
|/ssd_object_detect/ssd_nodelet/ssd_in_scale_factor|double|Caffeで扱うBlob形式の変換時のスケールパラメータ|
|/ssd_object_detect/ssd_nodelet/ssd_confidence_threshold|double|検出結果リストに追加される結果の信頼度の閾値|
|/ssd_object_detect/ssd_nodelet/ssd_prototxt_name|double|string|prototxtファイルパス|
|/ssd_object_detect/ssd_nodelet/ssd_caffemodel_name|string|caffeモデルファイルパス|
|/ssd_object_detect/ssd_nodelet/ssd_class_names_file|string|検出する物体名リストファイルパス|