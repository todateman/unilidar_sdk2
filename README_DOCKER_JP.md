# Unitree LiDAR ROS2 Docker セットアップ

このDockerセットアップでは、Docker ComposeでUnitree L2 LiDARをシリアルモードでROS2で動作させることができ、Foxglove Studioやその他のROSクライアントからのリモートアクセス用のネットワークブリッジも含まれています。

## クイックスタート

1. **シリアルデバイスの権限設定:**
   
   ```bash
   ./scripts/setup-udev.sh
   ```

   *注意: グループ変更を有効にするために、ログアウトして再ログインが必要な場合があります。*

2. **Unitree L2 LiDARをUSBケーブルで接続**

3. **ネットワークブリッジ付きでLiDARサービスを開始:**
   
   ```bash
   ./scripts/start-lidar.sh
   ```

4. **ネットワークエンドポイントを確認:**
   
   ```bash
   ./scripts/network-info.sh
   ```

5. **サービス停止:**
   
   ```bash
   ./scripts/stop-lidar.sh
   ```

## 利用可能なスクリプト

### `./scripts/start-lidar.sh`

メイン起動スクリプトで以下のオプションがあります:

- `--build`: Dockerイメージを強制的に再構築
- `--detach` または `-d`: バックグラウンドモード（デタッチモード）で実行
- `--no-bridges`: Foxglove BridgeとROS Bridgeの起動をスキップ
- `--rviz-only`: Rvizのみを起動（メインサービスが実行中である必要があります）
- `--help`: ヘルプメッセージを表示

使用例:

```bash
./scripts/start-lidar.sh                # ブリッジとRviz付きで起動
./scripts/start-lidar.sh --build        # 全サービスを再構築して起動
./scripts/start-lidar.sh --detach       # バックグラウンドで起動
./scripts/start-lidar.sh --no-bridges   # LiDARノードのみ起動（ネットワークブリッジなし）
./scripts/start-lidar.sh --rviz-only    # Rvizのみ起動
```

### `./scripts/stop-lidar.sh`

停止スクリプトで以下のオプションがあります:

- `--volumes` または `-v`: ボリュームも削除
- `--images`: イメージも削除

### `./scripts/setup-udev.sh`

適切なシリアルデバイスアクセス用のudevルールを設定します。

### `./scripts/network-info.sh`

ブリッジのネットワークエンドポイント情報と接続状況を表示します。

## 手動でのDockerコマンド

Docker Composeを直接使用したい場合:

```bash
# 構築して起動
docker-compose up --build

# バックグラウンドで起動
docker-compose up -d

# サービス停止
docker-compose down

# ログを表示
docker-compose logs -f
```

## 設定

### シリアルデバイス

システムが一般的なシリアルデバイスを自動検出します:

- `/dev/ttyACM0`, `/dev/ttyACM1` (USB CDCデバイス)
- `/dev/ttyUSB0`, `/dev/ttyUSB1` (USB-シリアルアダプター)
- `/dev/unitree_lidar` (udevルールが設定されている場合)

### パラメータ

LiDARパラメータを以下で変更できます:

- `config/lidar_params.yaml` - ROS2パラメータファイル
- `unitree_lidar_ros2/src/unitree_lidar_ros2/launch/launch.py` - 起動ファイルパラメータ

### 環境変数

`.env.example`を`.env`にコピーして必要に応じて編集:

```bash
cp .env.example .env
# .envファイルを希望の設定で編集
```

## ROS2トピックとネットワークアクセス

### ROS2トピック

実行時に以下のトピックが利用可能になります:

- `/unilidar/cloud` - 点群データ (sensor_msgs/PointCloud2)
- `/unilidar/imu` - IMUデータ (sensor_msgs/Imu)

### ネットワークブリッジ

システムにはリモートアクセス用の2つのネットワークブリッジが含まれています:

#### Foxglove Bridge (ポート 8765)

- **URL**: `ws://[HOST_IP]:8765`
- **互換性**: Foxglove Studio、その他のFoxglove互換クライアント
- **プロトコル**: Foxglove WebSocketプロトコル
- **使用方法**: 
  1. Foxglove Studioを開く
  2. WebSocketに接続
  3. ホストIPアドレスを含むURLを入力

#### ROS Bridge (ポート 9090)

- **URL**: `ws://[HOST_IP]:9090`
- **互換性**: Webアプリケーション、カスタムROSクライアント
- **プロトコル**: ROSBridge WebSocketプロトコル
- **使用方法**: ROSBridge互換クライアントをWebSocketエンドポイントに接続

### 選択的TOPIC送信

FastDDSで大きな点群データを送信する際の制限を回避するため、選択的TOPIC送信機能を追加しました:

```bash
# 軽量データのみをネットワーク送信（推奨）
docker compose run --rm unitree_lidar ros2 launch unitree_lidar_ros2 launch.py \
  enable_cloud:=false \
  enable_imu:=true \
  enable_rviz:=false

# 全データを有効化（ローカル用）
docker compose run --rm unitree_lidar ros2 launch unitree_lidar_ros2 launch.py \
  enable_cloud:=true \
  enable_imu:=true \
  enable_rviz:=true
```

詳細な使用方法は `SELECTIVE_TOPIC_PUBLISHING.md` を参照してください。

### 他のマシンからのアクセス

1. ホストマシンのIPアドレスを確認: `./scripts/network-info.sh`
2. ファイアウォールでポート8765と9090を開放:
   
   ```bash
   sudo ufw allow 8765
   sudo ufw allow 9090
   sudo ufw reload
   ```

3. 上記のURLを使用してリモートマシンから接続

### 接続例

#### Foxglove Studio接続

1. Foxglove Studioを開く
2. "Open connection"をクリック
3. **"Foxglove WebSocket"** を選択
4. URL入力: `ws://[HOST_IP]:8765`
5. "Open"をクリック
6. `/unilidar/cloud`と`/unilidar/imu`トピックが利用可能になります

#### ROS Bridge接続

1. Foxglove StudioまたはROS互換クライアントを開く
2. "Open connection"をクリック
3. **"Rosbridge (ROS 1 & ROS 2)"**を選択
4. URL入力: `ws://[HOST_IP]:9090`
5. "Open"をクリック
6. トピックが購読可能になります

**注意**: 両方の接続方法は同じROS2トピックへのアクセスを提供しますが、パフォーマンス特性が異なる場合があります。Foxglove WebSocketはFoxglove Studio用に最適化されており、ROS Bridgeはより広い互換性を提供します。

## トラブルシューティング

### シリアルデバイスが見つからない

1. デバイスが接続されているか確認: `ls -la /dev/tty* | grep -E '(ACM|USB)'`
2. udevセットアップスクリプトを実行: `./scripts/setup-udev.sh`
3. ユーザーがdialoutグループに属しているか確認: `groups $USER`

### アクセス権限拒否

1. udevセットアップスクリプトを実行: `./scripts/setup-udev.sh`
2. ログアウトして再ログイン
3. またはsudoで実行（推奨されません）: `sudo ./scripts/start-lidar.sh`

### Rvizが表示されない

1. X11フォワーディングが設定されているか確認: `echo $DISPLAY`
2. DockerにX11へのアクセスを許可: `xhost +local:docker`
3. DISPLAY環境変数が設定されているか確認

### ビルドエラー

1. Dockerに十分なリソース（RAM/ディスク容量）があることを確認
2. クリーンアップを試行: `docker system prune -a`
3. 最初から再構築: `./scripts/start-lidar.sh --build`

### ネットワークブリッジの問題

1. ポートが利用可能か確認: `netstat -tuln | grep -E '(8765|9090)'`
2. 接続性をテスト: `./scripts/network-info.sh`
3. ファイアウォール設定を確認: `sudo ufw status`
4. 他のサービスがポート8765や9090を使用していないことを確認

### リモート接続の問題

1. ホストIPアドレスを確認: `./scripts/network-info.sh`
2. リモートマシンからポートアクセス性をテスト: `telnet [HOST_IP] 8765`
3. ネットワークルーティングとファイアウォールルールを確認
4. 外部ネットワークの前に同じネットワークから接続を試行

## ハードウェア要件

- Unitree L2 LiDAR
- シリアル接続用のUSBケーブル
- Dockerがインストールされたコンピュータ
- Linuxシステム（Ubuntu 20.04+でテスト済み）

## システム要件

- Docker Engine 20.10+
- Docker Compose 2.0+ (または docker-compose 1.29+)
- Rviz表示用のX11サーバー（オプション）
- LiDAR接続用のUSBポート
- リモートアクセス用のネットワーク接続

## 機能

✅ **シリアルモード接続** - Unitree L2 LiDARへの直接USB接続  
✅ **ROS2統合** - 標準メッセージタイプによる完全なROS2 Humbleサポート  
✅ **デュアルネットワークブリッジ** - Foxglove WebSocketとROS Bridgeプロトコルの両方  
✅ **リモートアクセス** - ネットワーク上の任意のデバイスからアクセス  
✅ **点群可視化** - リアルタイム3D点群データ  
✅ **IMUデータ** - クォータニオン姿勢を含む6軸IMUデータ  
✅ **TF配信** - 適切な座標フレーム変換  
✅ **Docker コンテナ化** - 簡単なデプロイメントと一貫した環境  
✅ **選択的TOPIC送信** - ネットワーク負荷を軽減するための軽量データ選択機能  
✅ **FastDDS最適化** - 大きな点群データ問題の回避