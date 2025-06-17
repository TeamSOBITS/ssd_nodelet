<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SSD for ROS

<!-- 目次 -->
<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#環境構築">環境構築</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#実行操作方法">実行・操作方法</a></li>
      <li><a href="#パラメーター">パラメーター</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>



<!-- レポジトリの概要 -->
## 概要

本レポジトリは，Single Shot MultiBox Detector(SSD)による物体検出をROS2環境で行うためのパッケージです．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 環境構築 -->
## 環境構築

本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 環境条件

まず，以下の環境を整えてから，次のインストール方法に進んでください．

| System | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill    |
| Python | >=3.10              |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法
1. はじめにROS2の`src`フォルダに移動します．
    ```sh
    cd ~/colcon_ws/src
    ```
2. `src`フォルダ内にROSパッケージ`ssd_nodelet`をクローンします．
    ```sh
    git clone -b humble-devel https://github.com/TeamSOBITS/ssd_nodelet.git
    ```
3. クローンしたレポジトリフォルダの中へ移動します．
    ```sh
    cd ssd_nodelet
    ```
4. 依存パッケージをインストールします．
    ```sh
    bash install.sh
    ```
5. パッケージをビルドします
    ```sh
    cd ~/colcon_ws/
    ```
    ```sh
    colcon build --symlink-install
    ```    
    ```sh
    source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- 実行・操作方法 -->
## 実行・操作方法
パッケージのビルドまで完了したら，以下の手順で動作確認を行うことができます．

1. カメラを起動します．

2. [ssd.launch.py](launch/ssd_ros.launch.py)の**image_topic_name**を使用するカメラのトピック名に書き換えます．

    例
    ```python
    default_value="/camera/camera/color/image_raw",           ## realsense
    ```

3. RGBDカメラを使用する場合は，[ssd.launch.py](launch/ssd_ros.launch.py)の**point_cloud_topic_name**を使用するカメラの点群トピック名に書き換えます．

    例
    ```python
    default_value="/camera/camera/depth/color/points",      　## realsense
    ```

4. 物体検出（人を含む）するか，顔検出をするかに応じて使用するモデルを変更するため，[ssd.launch.py](launch/ssd_ros.launch.py)内の以下のパスを書き換えます．
    - voc_object_prototxt_path
    - voc_object_caffemodel_path
    - voc_object_names_path

    例：物体検出時
    ```python
    voc_object_prototxt_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object.prototxt')
    voc_object_caffemodel_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object.caffemodel')
    voc_object_names_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object_names.txt')
    ```

6. 顔の検出をする場合は，パスの変更に加えて[ssd.launch.py](launch/ssd_ros.launch.py)の**in_scale_factor**を書き換えます．
    ```python
    # default_value="0.007843",     # 物体検出時
    default_value="1.00",           # 顔検出時
    ```


7. 必要な変更が完了したら,[ssd.launch.py](launch/ssd_ros.launch.py)を起動して動作確認することが可能です．
    ```sh
    ros2 launch ssd_ros ssd_ros.launch.py
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## パラメーター
[ssd.launch.py](launch/ssd_ros.launch.py)内で設定可能なパラメータは以下のとおりです．
| パラメーター名  | 説明 | デフォルト値 |
| ------------- | ------------- | ------------- |
|image\_show\_flag|画像を表示するかどうかの制御|`true`|
|execute\_default|SSD (Single Shot MultiBox Detector) がデフォルトで起動するかどうかを決定|`true`|
|image\_topic\_name|`sensor_msgs/msg/Image`型メッセージのROSトピック名を指定|`/camera/camera/color/image_raw`（realsense用）|
|point\_cloud\_topic\_name|`sensor_msgs/msg/PointCloud2`型メッセージのROSトピック名を指定|`/camera/camera/depth/color/points`（realsense用）|
|in\_scale\_factor|Caffemodelでデータを処理する際の変換時に使用されるスケールパラメータ|`0.007843`（物体検出用）|
|confidence\_threshold|検出結果の信頼度に対するしきい値|`0.5`|
|ssd_prototxt_name|Caffemodelの構造を記述したファイルパス|/home/user_name/colcon_ws/install/ssd_ros/share/ssd_ros/models/voc_object.prototxt|
|ssd_caffemodel_name|学習済みモデルのファイルパス|/home/user_name/colcon_ws/install/ssd_ros/share/ssd_ros/models/voc_object.caffemodel|
|ssd_class_names_file|学習済み物体名リストのファイルパス|/home/user_name/colcon_ws/install/ssd_ros/share/ssd_ros/models/voc_object_names.txt|
|object\_specified\_enabled|特定の物体検出の有効化フラグ|`true`|
|specified\_object\_name|`object_specified_enabled`が`true`に設定されている場合に検出する物体の名前を指定|`person`|
|use\_3d|3D検出の有効化フラグ|`true`|

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## マイルストーン
現時点のbugや新規機能の依頼を確認するために[Issueページ][issues-url] をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## 参考文献
* [SSD: Single Shot MultiBox Detector](https://www.cs.unc.edu/~wliu/papers/ssd.pdf)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/ssd_nodelet.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/ssd_nodelet/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/ssd_nodelet.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/ssd_nodelet/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/ssd_nodelet.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/ssd_nodelet/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/ssd_nodelet.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/ssd_nodelet/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/ssd_nodelet.svg?style=for-the-badge
[license-url]: LICENSE
