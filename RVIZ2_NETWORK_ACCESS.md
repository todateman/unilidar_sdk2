# 他のPCからRviz2でUnitree LiDARデータを確認する方法

## 概要

ローカルネットワーク内の他のPCから、Rviz2を使用してUnitree LiDARの点群データを確認できます。

## 前提条件

- LiDARホストPC: Dockerコンテナ実行中
- クライアントPC: 同一ネットワーク内、ROS2 Humbleインストール済み

## 方法1: LiDAR付きでDockerコンテナを起動（推奨）

### LiDARホストPC側

```bash
# 通常の起動（LiDAR + RViz有効）
docker compose up unitree_lidar

# または環境変数で制御
ENABLE_LIDAR=true ENABLE_RVIZ=false docker compose up unitree_lidar
```

### 他のPC側

```bash
# 1. 環境変数設定（必須）
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# 2. トピック確認
ros2 topic list
# /unilidar/cloud と /unilidar/imu が見えることを確認

# 3. RViz2起動
rviz2

# または設定ファイル指定で起動（設定ファイルを事前にコピーが必要）
rviz2 -d unitree_lidar_config.rviz
```

## 方法2: LiDARなしでブリッジのみ起動

LiDARの実ハードウェアがない環境や、既に別でLiDARノードが動作している場合

### LiDARホストPC側

```bash
# LiDARノード無効、RVizも無効でブリッジのみ
ENABLE_LIDAR=false ENABLE_RVIZ=false docker compose up unitree_lidar

# この場合は別途、直接ros2 launchでLiDARを起動
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# LiDARのみ起動（シリアルデバイス競合に注意）
ros2 launch unitree_lidar_ros2 launch.py enable_rviz:=false
```

## RViz2での設定手順

### 1. 基本設定

1. **Fixed Frame**を`unilidar_lidar`に設定
2. **Add**ボタンから`PointCloud2`を追加
3. **Topic**を`/unilidar/cloud`に設定

### 2. PointCloud2表示設定

- **Color Transformer**: `AxisColor` または `Intensity`
- **Size (Pixels)**: `3`
- **Style**: `Points`
- **Use rainbow**: `true`

### 3. TF表示（オプション）

- **Add**ボタンから`TF`を追加
- LiDARフレームを確認可能

## 設定ファイルの共有

### ホストPCから設定ファイルをコピー

```bash
# 設定ファイルのコピー
scp user@192.168.11.210:/home/tomohirokusu/unilidar_sdk2/unitree_lidar_ros2/src/unitree_lidar_ros2/rviz/view.rviz ./unitree_lidar_config.rviz

# コピーした設定で起動
rviz2 -d unitree_lidar_config.rviz
```

## トラブルシューティング

### トピックが見えない場合

```bash
# DDS Discovery確認
ros2 daemon stop
ros2 daemon start

# 環境変数確認
echo $ROS_DOMAIN_ID          # 0
echo $RMW_IMPLEMENTATION     # rmw_fastrtps_cpp
echo $ROS_LOCALHOST_ONLY     # 0

# ネットワーク疎通確認
ping 192.168.11.210

# マルチキャスト確認
ping 239.255.0.1
```

### RVizでポイントクラウドが表示されない場合

1. **Fixed Frame**を`unilidar_lidar`に設定
2. **Topic**名が`/unilidar/cloud`になっているか確認
3. **Use Fixed Frame**を`false`に設定
4. **Reliability Policy**を`Best Effort`に変更
5. データが流れているか確認: `ros2 topic hz /unilidar/cloud`

### TFフレームの問題がある場合

ローカルネットワーク越しのDDS通信でTFデータが正常に配信されない場合：

1. **Fixed Frame**を`unilidar_lidar`に設定
2. PointCloud2設定で**Use Fixed Frame**を`false`に変更
3. 各点群データは自身のフレーム情報を持っているため、TFなしでも表示可能

### データの遅延がある場合

- **Reliability Policy**を`Best Effort`に変更  
- **History Policy**を`Keep Last`、**Depth**を`1`に設定
- **Durability Policy**を`Volatile`に設定

## 起動オプション一覧

| オプション | 説明 |
|-----------|-----|
| `enable_lidar:=true` | LiDARノードを起動（デフォルト） |
| `enable_lidar:=false` | LiDARノードを起動しない |
| `enable_rviz:=true` | RVizを起動（デフォルト） |
| `enable_rviz:=false` | RVizを起動しない |

### 使用例

```bash
# LiDARのみ起動（RViz無し）
ros2 launch unitree_lidar_ros2 launch.py enable_rviz:=false

# RVizのみ起動（LiDAR無し）※別途データソースが必要
ros2 launch unitree_lidar_ros2 launch.py enable_lidar:=false

# 両方無効（何も起動しない）
ros2 launch unitree_lidar_ros2 launch.py enable_lidar:=false enable_rviz:=false
```

## 性能考慮事項

- **点群データ**: 約12Hz、5000-6000点/frame
- **ネットワーク帯域**: 約10-20Mbps
- **レイテンシ**: DDSマルチキャスト経由で50-100ms程度

## セキュリティ考慮事項

- ROS2 DDSトラフィックは暗号化されていません
- 信頼できるネットワーク内でのみ使用してください
- ファイアウォール設定でUDPポート7400-7420を適切に制御してください