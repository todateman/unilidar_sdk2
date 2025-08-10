# 選択的TOPIC送信機能

## 概要

FastDDSで大きな点群データ（~170KB/メッセージ）を送信する際の問題を解決するため、必要なTOPICのみを選択してDDS経由で送信する機能を実装しました。

## 機能詳細

### 対応パラメータ

| パラメータ名 | デフォルト値 | 説明 |
|-------------|-------------|------|
| `enable_lidar` | `true` | LiDARノード全体の有効/無効 |
| `enable_rviz` | `true` | RViz2の起動有効/無効 |
| `enable_cloud` | `true` | 点群データ送信の有効/無効 |
| `enable_imu` | `true` | IMUデータ送信の有効/無効 |

### 推奨設定

#### ネットワーク経由での軽量データのみ送信
```bash
# 点群データを無効化し、IMUとTFデータのみ送信
docker compose run --rm unitree_lidar ros2 launch unitree_lidar_ros2 launch.py \
  enable_cloud:=false \
  enable_imu:=true \
  enable_rviz:=false
```

#### ローカル可視化用
```bash
# 全データを有効化してローカルでRViz2表示
docker compose run --rm unitree_lidar ros2 launch unitree_lidar_ros2 launch.py \
  enable_cloud:=true \
  enable_imu:=true \
  enable_rviz:=true
```

## 実装詳細

### C++コード修正点

1. **パラメータ宣言**（`unitree_lidar_ros2.h`）:
```cpp
// Topic publishing control parameters
declare_parameter<bool>("enable_cloud_publish", true);
declare_parameter<bool>("enable_imu_publish", true);
```

2. **条件付きパブリッシャー作成**:
```cpp
// Create publishers only if enabled
if (enable_cloud_publish_) {
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, qos_profile);
    RCLCPP_INFO(this->get_logger(), "Point cloud publishing enabled");
} else {
    RCLCPP_INFO(this->get_logger(), "Point cloud publishing disabled");
}
```

3. **条件付きメッセージ送信**:
```cpp
// Only publish if enabled
if (enable_cloud_publish_ && pub_cloud_) {
    pub_cloud_->publish(cloud_msg);
}

if (enable_imu_publish_ && pub_imu_) {
    pub_imu_->publish(imuMsg);
}
```

### Launch ファイル修正点

1. **引数定義**:
```python
enable_cloud_arg = DeclareLaunchArgument(
    'enable_cloud',
    default_value='true',
    description='Enable point cloud publishing (true/false)'
)
```

2. **パラメータ渡し**:
```python
parameters=[
    # ... 他のパラメータ
    {'enable_cloud_publish': enable_cloud},
    {'enable_imu_publish': enable_imu},
]
```

## 使用例

### 他のPCから軽量データのみを受信

```bash
# LiDARホストPC側
docker compose run --rm unitree_lidar ros2 launch unitree_lidar_ros2 launch.py \
  enable_cloud:=false \
  enable_imu:=true \
  enable_rviz:=false

# 他のPC側（環境変数設定後）
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

# IMUデータの確認
ros2 topic echo /unilidar/imu

# TFデータの確認  
ros2 topic echo /tf
```

### Foxglove Studioでの可視化

点群データが必要な場合は、FastDDS経由ではなくFoxglove Studio経由での可視化を推奨：

```bash
# LiDARホストPC側：Foxglove Bridge起動
docker compose up foxglove_bridge

# Webブラウザで接続
# URL: https://studio.foxglove.dev/
# WebSocket URL: ws://LiDARホストPCのIPアドレス:8765
```

## データサイズ比較

| データ種別 | サイズ | FastDDS推奨度 |
|-----------|-------|-------------|
| IMUデータ | ~500B | ✅ 推奨 |
| TFデータ | ~200B | ✅ 推奨 |
| 点群データ | ~170KB | ❌ 非推奨（セグメンテーション違反の原因） |

## トラブルシューティング

### パラメータが反映されない場合

1. **Docker rebuild**:
```bash
docker compose build
```

2. **パラメータ確認**:
```bash
ros2 param list /unitree_lidar_ros2_node
ros2 param get /unitree_lidar_ros2_node enable_cloud_publish
```

### ログメッセージ確認

正常に動作している場合、以下のログが表示されます：

```
[INFO] [unitree_lidar_ros2_node]: Point cloud publishing disabled
[INFO] [unitree_lidar_ros2_node]: IMU publishing enabled on topic: unilidar/imu
```

## まとめ

この選択的TOPIC送信機能により：

- ✅ **FastDDSの制限を回避**: 大きな点群データを無効化
- ✅ **必要なデータは確保**: IMU、TF、診断データはネットワーク経由で利用可能
- ✅ **柔軟な設定**: 用途に応じてパラメータを調整可能
- ✅ **安定した動作**: セグメンテーション違反を回避

FastDDSの制約を理解した上で、適切なデータのみをネットワーク経由で配信することで、安定したROS2分散システムを構築できます。