# Unitree L2 LiDAR Setup and Configuration Guide

このドキュメントでは、Unitree L2 LiDARをUDP（イーサネット）とシリアルポートの両方で動作させるための設定手順と変更点をまとめています。

## 前提条件

- Ubuntu 20.04/22.04
- ROS2 Humble
- PCL-1.10
- LiDAR IP: 192.168.1.62
- ユーザーが `dialout` グループに所属していること

```bash
# dialoutグループの確認
groups
# 必要に応じて追加
sudo usermod -a -G dialout $USER
```

## 1. UDP（イーサネット）接続での設定

### 1.1 ネットワーク設定の確認

LiDARのデフォルトIP（192.168.1.62）に接続できるよう、ローカルIPを設定します。

```bash
# 現在のIP設定確認
ip addr show

# LiDARとの接続確認
ping -c 3 192.168.1.62
```

### 1.2 UDP接続用ファイル修正

**ファイル**: `unitree_lidar_sdk/examples/example_lidar_udp.cpp`

```cpp
// 修正前
std::string local_ip = "192.168.1.2";
unsigned short local_port = 6201;

// 修正後（環境に合わせて調整）
std::string local_ip = "192.168.1.3";     // 実際のローカルIPに変更
unsigned short local_port = 6202;          // ポート競合を回避
```

**ファイル**: `unitree_lidar_ros2/src/unitree_lidar_ros2/launch/launch.py`

```python
# UDP接続用設定
def generate_launch_description():
    node1 = Node(
        parameters= [
            {'initialize_type': 2},          # UDP通信
            {'local_ip': '192.168.1.3'},     # 実際のローカルIPに変更
            {'local_port': 6203},            # ポート競合を回避
        ]
    )
```

### 1.3 UDP接続でのビルドと実行

```bash
# C++ SDKのテスト
cd unitree_lidar_sdk/build
cmake .. && make -j2
cd ..
./bin/example_lidar_udp

# ROS2パッケージでの実行
cd unitree_lidar_ros2
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

## 2. シリアルポート接続での設定

### 2.1 LiDARをシリアルモードに切り替え

**重要**: LiDARは工場出荷時にUDPモードで設定されているため、まずUDP接続でシリアルモードに変更する必要があります。

```bash
cd unitree_lidar_sdk
./bin/set_to_serial_mode
```

### 2.2 シリアルポート用ファイル修正

**ファイル**: `unitree_lidar_ros2/src/unitree_lidar_ros2/launch/launch.py`

```python
# シリアル接続用設定
def generate_launch_description():
    node1 = Node(
        parameters= [
            {'initialize_type': 1},          # シリアル通信
            {'serial_port': '/dev/ttyACM0'}, # シリアルポートデバイス
            {'baudrate': 4000000},           # ボーレート設定
        ]
    )
```

### 2.3 シリアルポートデバイスの確認

```bash
# シリアルポートデバイスの確認
ls -la /dev/ttyACM*
ls -la /dev/ttyUSB*

# 権限確認（dialoutグループに所属していることを確認）
groups
```

### 2.4 シリアル接続でのテストと実行

**重要**: シリアル通信では、LiDAR電源投入後に毎回C++ SDKでのテストが必要です。

```bash
# 1. C++ SDKでのシリアル接続テスト（L2の電源投入後には必須）
cd unitree_lidar_sdk
./bin/example_lidar_serial

# 2. ROS2パッケージでの実行（シリアルテスト成功後）
cd unitree_lidar_ros2
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

**シリアル通信での推奨手順**:

1. LiDAR電源投入
2. `example_lidar_serial`でシリアル通信を初期化
3. テスト成功後にROS2を起動

### 2.5 シリアル通信でのC++ SDKテスト不要化

電源投入毎にC++ SDKテストが必要な根本原因は、**ROS2ノードの初期化処理に重要なステップが不足**していることです。

**ROS2ノードで不足している処理**:

1. `startLidarRotation()` - LiDARモーター回転開始
2. `resetLidar()` - デバイスリセット
3. コマンド間の適切な待機時間（1秒）


#### 2.5.1. `unitree_lidar_ros2/src/unitree_lidar_ros2/launch/launch.py`の以下の項目を修正

```python
# 動作モード修正前
        parameters= [                
                {'work_mode': 0},
        ]

# 動作モード修正後
        parameters= [                
                {'work_mode': 8},
        ]
```

#### 2.5.2 `unitree_lidar_ros2/src/unitree_lidar_ros2/include/unitree_lidar_ros2.h`の149行目付近に追記：

```cpp
lsdk_->setLidarWorkMode(work_mode_);

// シリアル通信用の追加初期化処理
if (initialize_type_ == 1) {  // シリアルモードのみ
    lsdk_->startLidarRotation();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    lsdk_->resetLidar();
    std::this_thread::sleep_for(std::chrono::seconds(1));
}
```

修正後に下記を実行

```bash
cd unitree_lidar_ros2
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

## 3. Rviz2での可視化

### 3.1 Rviz2の起動

```bash
# 自動でRviz2も起動される（launch.pyに含まれている）
ros2 launch unitree_lidar_ros2 launch.py

# または個別で起動
source /opt/ros/humble/setup.bash
rviz2 -d install/unitree_lidar_ros2/share/unitree_lidar_ros2/view.rviz
```

### 3.2 トピック確認

```bash
# 利用可能なトピックの確認
ros2 topic list

# 期待されるトピック
# /unilidar/cloud (sensor_msgs/msg/PointCloud2)
# /unilidar/imu (sensor_msgs/msg/Imu)

# トピック情報の確認
ros2 topic info /unilidar/cloud
ros2 topic info /unilidar/imu

# データ頻度の確認
ros2 topic hz /unilidar/cloud
ros2 topic hz /unilidar/imu
```

## 4. 動作モードの設定

LiDARの動作モードは`uint32_t`の各ビットで制御されます：

| ビット位置 | 機能 | 値0 | 値1 |
|---|---|---|---|
| 0 | FOV切り替え | 標準FOV (180°) | 広角FOV (192°) |
| 1 | 測定モード | 3D測定モード | 2D測定モード |
| 2 | IMU有効化 | IMU有効 | IMU無効 |
| 3 | 通信モード | イーサネットモード | シリアルモード |
| 4 | 起動モード | 電源投入時自動開始 | 電源投入時待機 |

```cpp
// 例：シリアル通信 + 3D + IMU有効 + 自動開始
uint32_t workMode = 8;  // ビット3のみ1（シリアルモード）

// 例：UDP通信 + 3D + IMU有効 + 自動開始
uint32_t workMode = 0;  // すべて0（デフォルト）
```

### 4.1 ROS2での動作モード設定

**ファイル**: `unitree_lidar_ros2/src/unitree_lidar_ros2/launch/launch.py`

```python
# work_modeパラメータで設定（デフォルト値：0）
{'work_mode': 0},    # UDP通信 + 3D + IMU有効 + 自動開始

# 例：シリアル通信 + 2D測定モード + IMU無効
{'work_mode': 14},   # ビット1,2,3 = 1 (2D + IMU無効 + シリアル)

# 例：UDP通信 + 広角FOV + 3D測定
{'work_mode': 1},    # ビット0 = 1 (広角FOV)
```

### 4.2 C++ SDKでの動作モード設定

**ファイル**: `unitree_lidar_sdk/examples/example_lidar_udp.cpp`

```cpp
// setWorkModeメソッドで設定
unitree_lidar_sdk::UnitreeLidarReader lidar_reader;
uint32_t work_mode = 0;  // 設定したいモード値

// 接続後にモード設定を実行
lidar_reader.setWorkMode(work_mode);
```

### 4.3 動作モード設定の適用方法

```bash
# 1. ROS2での設定変更
# launch.pyのwork_modeパラメータを編集後、再起動
cd unitree_lidar_ros2
colcon build
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py

# 2. C++ SDKでの設定変更
# ソースコード編集後、リビルドして実行
cd unitree_lidar_sdk/build
cmake .. && make -j2
cd ..
./bin/example_lidar_udp
```

## 5. トラブルシューティング

### 5.1 UDP接続の問題

```bash
# ポート使用状況の確認
sudo netstat -tuln | grep 6201

# ファイアウォール設定の確認
sudo ufw status

# LiDAR接続確認
ping -c 3 192.168.1.62
```

### 5.2 シリアル接続の問題

```bash
# シリアルデバイスの確認
dmesg | grep ttyACM
ls -la /dev/ttyACM*

# 権限の確認
groups
ls -la /dev/ttyACM0

# 必要に応じて権限追加
sudo chmod 666 /dev/ttyACM0
```

### 5.3 ROS2の問題

```bash
# ROS2デーモンのリスタート
ros2 daemon stop
ros2 daemon start

# ノードの確認
ros2 node list

# パッケージの再ビルド
colcon build --packages-select unitree_lidar_ros2
```

## 6. 座標系とデータフォーマット

### 6.1 座標系定義

- **LiDAR座標系**: 右手座標系、原点は底面マウント部中央
- **IMU座標系**: LiDAR座標系に対して平行移動のみ
- **変換行列**: LiDARからIMUへの変換は `[-0.007698, -0.014655, 0.00667]` の平行移動

### 6.2 データ仕様

- **点群データ**: 18リング構成、最大100m測定範囲
- **IMUデータ**: 6軸（加速度・角速度）+ クォータニオン
- **フレーム名**: `unilidar_lidar` (点群), `unilidar_imu` (IMU)

## 7. 設定ファイルの保存場所

修正が必要な主要ファイル：

1. `unitree_lidar_sdk/examples/example_lidar_udp.cpp` - UDP接続設定
2. `unitree_lidar_ros2/src/unitree_lidar_ros2/launch/launch.py` - ROS2起動設定
3. `unitree_lidar_ros2/src/unitree_lidar_ros2/rviz/view.rviz` - Rviz2設定

これらの設定により、Unitree L2 LiDARをUDP/シリアル両方の通信方式で利用できるようになります。