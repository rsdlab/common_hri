# Common HRI Components（common_hri）

[English](README.md) | **日本語**

## 概要

- RoIS 2.0に基づくHRI（Human-Robot Interaction）コンポーネントを管理するリポジトリ
- 本リポジトリでは，ロボットに共通するHRI機能を提供し，ロボット固有の動作処理は各ロボット内に配置する実行ノードに分離する
- common_hri_ros1はROS1対応，common_hri_ros2はROS2対応したものである
- [RT System Integration Framework（RTSI-FW）](https://github.com/rsdlab/RTSI_FW) と連携して使用する

## 開発環境

- 開発言語：Python
- ROS：ROS 1 Noetic / ROS 2 Humble
- 対応OS：Ubuntu 20.04 / 22.04

## リポジトリ構成

```text
common_hri/
├── README.md
├── README.ja.md
├── common_hri_ros1/
│   ├── config/
│   ├── include/
│   ├── launch/
│   ├── rtc/
│   ├── script/
│   ├── yaml/
│   ├── CMakeLists.txt
│   ├── hri.xml
│   ├── mgr.py
│   ├── package.xml
│   ├── robot.yaml
│   └── setting.yaml
└── common_hri_ros2/
```

| ディレクトリ・ファイル | 役割 |
| --- | --- |
| `common_hri_ros1/` | ROS1用の共通HRIパッケージ |
| `common_hri_ros2/` | ROS2用の共通HRIパッケージ |
| `config/` | ロボットごとの設定ファイル |
| `script/` | HRIコンポーネントおよびMainEngineの実装 |
| `yaml/` | HRIコンポーネントの収集に使用する設定ファイル |
| `launch/` | ノード起動用ファイル |
| `robot.yaml` | 使用するロボットの設定，configで管理しているものをコピー＆ペ−ストして使用する |
| `hri.xml` | HRIパッケージの定義情報，configで管理しているものをコピー＆ペ−ストして使用する |

## システム構成
本パッケージを用いたRTSI-FWのシステム構成
```text
RTSI-FW / Service Application
             ↓
         MainEngine
             ↓
    共通HRIコンポーネント
   (common_hri_ros1 / ros2)
             ↓
   ロボット固有実機ノード
    (airec_hri_action など)
             ↓
  ロボット実機・シミュレータ
```

共通HRIコンポーネントは，上位層からの要求を受け取り，ロボット固有Actionノードを介して動作を実行する．これにより，ロボットごとの実装の違いをAction層に分離する．

## ロボット設定

ロボットごとの設定ファイルは，各パッケージの `config/` ディレクトリ内で管理する．
`robot.yaml` には，使用するHRIコンポーネントやロボットに応じたサービス名などを記述する．
本パッケージでは，例としてAIREC，HSR，SEED-Noidを使用することを想定したファイルを配置してある．

```text
config/
├── airec/
│   ├── robot.yaml
│   └── hri.xml
├── hsr/
│   ├── robot.yaml
│   └── hri.xml
└── seed/
    ├── robot.yaml
    └── hri.xml
```

## 注意事項

- ROS1用とROS2用のパッケージは，それぞれ対応するROSワークスペースで使用する．
- Person関連のコンポーネントについて，設定ファイル等には記載があるが，現在の公開対象に含まれていない．
- 一部のコンポーネントには，実装・動作確認が完了していない機能がある．

## 関連リポジトリ

- [RTSI-FW](https://github.com/rsdlab/RTSI_FW)
- [rois_env](https://github.com/rsdlab/rois_env/tree/ros1-ros2)
- [Wasanbon](https://github.com/rsdlab/wasanbon/tree/ros1-ros2)
- [Wasanbon Web Framework](https://github.com/rsdlab/wasanbon_webframework)
