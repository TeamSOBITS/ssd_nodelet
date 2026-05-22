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

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS | Jazzy Jalisco |
| Python | 3.12 |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法
1. はじめにROS2の`src`フォルダに移動します．
    ```sh
    cd ~/colcon_ws/src
    ```
2. `src`フォルダ内にROSパッケージ`ssd_ros`をクローンします．
    ```sh
    git clone -b jazzy-devel https://github.com/TeamSOBITS/ssd_ros.git
    ```
3. クローンしたレポジトリフォルダの中へ移動します．
    ```sh
    cd ssd_ros
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

1. カメラを起動

2. [detection_config.yaml](config/detection_config.yaml)の**image_topic_name**を使用するカメラのトピック名に書き換え

    例
    ```sh
    image_topic_name: "/camera/color/image_raw" # Default Realsense Topic Name
    ```


3. [ssd.launch.py](launch/ssd_ros.launch.py)を起動して動作可能
    ```sh
    ros2 launch ssd_ros ssd_ros.launch.py
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## パラメーター
[ssd.launch.py](launch/ssd_ros.launch.py)内および[detection_config.yaml](config/detection_config.yaml)で設定可能なパラメータは以下のとおりです．
| パラメーター名  | 説明 | デフォルト値 |
| ------------- | ------------- | ------------- |
|image\_show\_flag|画像を表示するかどうかの制御|`true`|
|execute\_default|SSD (Single Shot MultiBox Detector) がデフォルトで起動するかどうかを決定|`true`|
|image\_topic\_name|`sensor_msgs/msg/Image`型メッセージのROSトピック名を指定|`/camera/camera/color/image_raw`（realsense用）|
|confidence\_threshold|検出結果の信頼度に対するしきい値|`0.5`|
|specified\_object\_class|検出する物体の名前を指定する文字列リスト。指定しない場合は`[None]`または`[dummy]`とする| [`person`] |
|use\_3d|3D検出の有効化フラグ|`true`|
|model\_directory|Caffemodelの構造を記述したファイルや学習済みモデルのファイル、モデルごとの詳細設定ファイルらがある**ディレクトリパス** | `../models/objects_model` |

<!-- 以下モデルごとの設定ファイル(拡張子がyamlのもの)へ移動または廃止 -->
<!-- |in\_scale\_factor|Caffemodelでデータを処理する際の変換時に使用されるスケールパラメータ|`0.007843`（物体検出用）| -->
<!-- |ssd_prototxt_name|Caffemodelの構造を記述したファイルパス|/install/ssd_ros/share/ssd_ros/models/voc_object.prototxt| -->
<!-- |ssd_caffemodel_name|学習済みモデルのファイルパス|/install/ssd_ros/share/ssd_ros/models/voc_object.caffemodel| -->
<!-- |ssd_class_names_file|学習済み物体名リストのファイルパス|/install/ssd_ros/share/ssd_ros/models/voc_object_names.txt| -->
<!-- |object\_specified\_enabled|特定の物体検出の有効化フラグ|`true`| -->
<!-- 以下パラメータはここでは詳しく書かずにimage_to_positionを参照 -->
<!-- |cluster_tolerance|どの程度離れた点群までは同一の物体とみなすかのしきい値|`0.01`| -->
<!-- |min_clusterSize|一定数以下の点群クラスタを対象から棄却するかのしきい値|`100`| -->
<!-- |max_clusterSize|一定数以上の点群クラスタを対象から棄却するかのしきい値|`20000`| -->
<!-- |noise_point_cloud_range|対象の物体の点群からノイズ面を除去する量|`0.01`| -->
<!-- |fast_shot|fast_shotの有効化フラグ|`true`| -->
<!-- |enable_id|検出した物体のラベルにIDを付与するかのフラグ|`false`| -->

> [!NOTE]
> model\_directory内の構成は以下の3つとする。
> 1. 検出できる物体を、学習された順番に羅列されたリスト(class)とCaffemodelでデータを処理する際の変換時に使用されるスケールパラメータ(in\_scale\_factor)を記述したYAMLファイル
> 2. 学習済み物体名リストのファイルパス(拡張子が`.caffemodel`のもの)
> 3. Caffemodelの構造を記述したファイルパス(拡張子が`prototxt`のもの)\
> これら3つのみにしてください。

> [!NOTE]
> その他のパラメータ(特にbbox_to_3d)は[image_to_position](https://github.com/TeamSOBITS/image_to_position)を参照してください。
> 基本的にはRealsenseに焦点が合わせられている。


<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## マイルストーン
* image_to_positionの更新に伴うlaunchパラメータに更新の可能性あり．： 解決

現時点のbugや新規機能の依頼を確認するために[Issueページ][issues-url] をご覧ください．


<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## 参考文献
* [SSD: Single Shot MultiBox Detector](https://www.cs.unc.edu/~wliu/papers/ssd.pdf)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/ssd_ros.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/ssd_ros/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/ssd_ros.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/ssd_ros/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/ssd_ros.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/ssd_ros/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/ssd_ros.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/ssd_ros/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/ssd_ros.svg?style=for-the-badge
[license-url]: LICENSE
