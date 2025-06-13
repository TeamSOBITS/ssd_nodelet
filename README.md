<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SSD for ROS

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#セットアップ">セットアップ</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#実行・操作方法">実行・操作方法</a></li>
      <!-- <ul>
        <li><a href="#モデルのダウンロード">モデルのダウンロード</a></li>
        <li><a href="#会話をする">会話をする</a></li>
      </ul> -->
    </li>
    <li><a href="#パラメーター">パラメーター</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>



<!-- レポジトリの概要 -->
## 概要

本レポジトリは、Single Shot MultiBox Detector(SSD)による物体検出をROS2環境で行うためのパッケージです。

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 環境構築 -->
## 環境構築

ここで，本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill    |
| Python | >=3.10              |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法
1. ROS2の`src`フォルダに移動します。
    ```console
    cd ~/colcon_ws/src
    ```
2. レポジトリの中へ移動します。
    ```console
    git clone -b humble-devel https://github.com/TeamSOBITS/ssd_nodelet.git
    ```
3. レポジトリの中へ移動します．
    ```console
    cd ssd_nodelet
    ```
4. 維新パッケージをインストールします．
    ```console
    bash install.sh
    ```
5. パッケージをビルドします
    ```console
    cd ~/colcon_ws/
    ```
    ```console
    colcon build --symlink-install
    ```    
    ```console
    source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- 実行・操作方法 -->
## 実行・操作方法
1. カメラを起動し、[ssd.launch.py](launch/ssd_ros.launch.py)の**image_topic_name**を使用するカメラのトピック名に書き換える。

    例
    ```sh
    default_value="/camera/camera/color/image_raw",          ##   realsense
    ```

2. RGBDカメラを使用する場合は，[ssd.launch.py](launch/ssd_ros.launch.py)のpoint_cloud_topic_nameも使用するカメラの点群のトピック名に書き換える．

    例
    ```sh
    default_value="/camera/camera/depth/color/points",      ## realsense
    ```

3. 検出したい対象に応じて[ssd.launch.py](launch/ssd_ros.launch.py)の使用するモデルの以下のパスを書き換える。
    - voc_object_prototxt_path
    - voc_object_caffemodel_path
    - voc_object_names_path

    例：物体検出時
    ```sh
    voc_object_prototxt_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object.prototxt')
    voc_object_caffemodel_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object.caffemodel')
    voc_object_names_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object_names.txt')
    ```

4. 顔の検出をする場合は[ssd.launch.py](launch/ssd_ros.launch.py)の**in_scale_factor**を書き換える。
    ```sh
    # default_value="0.007843",     # 物体検出時
    default_value="1.00",           # 顔検出時
    ```


5. [ssd.launch.py](launch/ssd_ros.launch.py)を起動する
    ```console
    ros2 launch ssd_ros ssd_ros.launch.py
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## パラメーター
### 鋭意作成中

* **image\_show\_flag**: この真偽値パラメータは、画像を表示するかどうかを制御します。デフォルトは`true`です。
* **execute\_default**: この真偽値パラメータは、SSD (Single Shot MultiBox Detector) がデフォルトで起動するかどうかを決定します。デフォルトは`true`です。
* **image\_topic\_name**: この文字列パラメータは、`sensor_msgs/msg/Image`メッセージのROSトピック名を指定します。デフォルトは`/image_raw`（Webカメラ用）です。
* **point\_cloud\_topic\_name**: この文字列パラメータは、`sensor_msgs/msg/PointCloud2`メッセージのROSトピック名を指定します。デフォルトは`/hsrb/head_rgbd_sensor/depth_registered/points`（Orbbecシリーズカメラ用）です。
* **in\_scale\_factor**: この浮動小数点パラメータは、Caffemodelでデータを処理する際の変換時に使用されるスケールパラメータを表します。デフォルトは`1.00`です。
* **confidence\_threshold**: この浮動小数点パラメータは、検出結果が公開されるために必要な最小の確率を設定します。デフォルトは`0.5`です。
* **ssd\_prototxt\_name**: この文字列パラメータは、ニューラルネットワークの構造を記述した`.prototxt`ファイルへのパスを指定します。デフォルトは`ssd_ros`パッケージの`models`ディレクトリにある`voc_object.prototxt`です。
* **ssd\_caffemodel\_name**: この文字列パラメータは、学習済みCaffemodelへのパスを指定します。デフォルトは`ssd_ros`パッケージの`models`ディレクトリにある`voc_object.caffemodel`です。
* **ssd\_class\_names\_file**: この文字列パラメータは、物体名のリストを含むファイルへのパスを指定します。デフォルトは`ssd_ros`パッケージの`models`ディレクトリにある`voc_object_names.txt`です。
* **object\_specified\_enabled**: この真偽値パラメータは、特定の物体の検出を有効または無効にします。デフォルトは`false`です。
* **specified\_object\_name**: この文字列パラメータは、`object_specified_enabled`が`true`に設定されている場合に検出する物体の名前を指定します。デフォルトは空の文字列です。
* **namespace**: この文字列パラメータは、このファイルによって起動されるノードの名前空間を定義します。デフォルトは`ssd_ros`です。
* **use\_3d**: この真偽値パラメータは、3D検出を有効にするかどうかを制御します。デフォルトは`true`です。

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
