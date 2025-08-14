# Foxglove Studioで点群データを確認する方法

## はじめに

RViz2でセグメンテーション違反が発生する場合、**Foxglove Studio**を使用することで、Webブラウザ経由で安定した点群データ可視化が可能です。

## 方法1: Webブラウザ版（推奨）

### 1. Foxglove Studioにアクセス

```bash
# LiDARホスト側でコンテナが起動していることを確認
docker compose up unitree_lidar
```

### 2. Webブラウザで接続

```
URL: https://studio.foxglove.dev/
```

### 3. データソース接続設定

1. **Open connection**をクリック
2. **Foxglove WebSocket**を選択
3. **WebSocket URL**に入力:
   ```
   ws://LiDARホストPCのIPアドレス:8765
   ```
4. **Open**をクリック

### 4. 点群データ表示設定

1. **Add panel** → **3D Scene**を追加
2. **3D Scene**設定で:
   - **Topic**: `/unilidar/cloud`を選択
   - **Point size**: `2`
   - **Color mode**: `colorField` または `rainbow`

## 方法2: デスクトップ版

### 1. Foxglove Studioをダウンロード

```bash
# Ubuntu/Debian
wget https://get.foxglove.dev/desktop/latest/foxglove-studio-linux-amd64.deb
sudo dpkg -i foxglove-studio-linux-amd64.deb
```

### 2. 起動と接続

```bash
foxglove-studio
```

接続設定は方法1と同じです。

## 方法3: ローカルファイル再生

### 1. ROSbag2で点群データを記録

```bash
# 環境変数設定
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# 点群データを記録
ros2 bag record /unilidar/cloud /unilidar/imu /tf /tf_static
```

### 2. Foxglove Studioで再生

1. **Open local file**を選択
2. 作成されたbagファイルを選択
3. 点群データが再生される

## トラブルシューティング

### 接続できない場合

1. **ファイアウォール確認**:
   ```bash
   sudo ufw allow 8765/tcp
   ```

2. **コンテナのポート確認**:
   ```bash
   docker compose ps
   # 8765ポートが公開されているか確認
   ```

3. **ネットワーク疎通確認**:
   ```bash
   curl http://LiDARホストPCのIPアドレス:8765
   ```

### データが表示されない場合

1. **トピック確認**:
   - Foxglove Studio画面左の**Topics**パネルでトピック一覧を確認
   - `/unilidar/cloud`が表示されているか確認

2. **データ頻度確認**:
   ```bash
   ros2 topic hz /unilidar/cloud
   ```

## 期待される結果

- **リアルタイム点群表示**: 約12Hzで更新される3D点群
- **インタラクティブ操作**: マウスで回転・ズーム可能
- **カラーマッピング**: 距離や強度に応じた色分け表示
- **安定した動作**: RViz2のセグメンテーション違反を回避

## 設定例

### 最適な3D Scene設定

```json
{
  "topics": {
    "/unilidar/cloud": {
      "pointSize": 2,
      "pointShape": "circle",
      "colorField": "intensity",
      "colorMode": "rainbow",
      "colorMap": "turbo"
    }
  },
  "transforms": {
    "frame": "unilidar_lidar"
  }
}
```

## その他の可視化オプション

### 1. IMUデータの表示

- **Add panel** → **Plot**
- **Topics**: `/unilidar/imu/linear_acceleration/*`

### 2. TFフレームの表示

- **3D Scene**設定で**Show TF**を有効化

### 3. 複数視点の表示

- **Add panel** → **3D Scene**を複数追加
- 異なるカメラアングルで表示

## まとめ

Foxglove Studioは：
- ✅ **Webブラウザで動作**（インストール不要）
- ✅ **ROS2のセグメンテーション違反を回避**
- ✅ **リアルタイム3D可視化**
- ✅ **直感的なUI**
- ✅ **豊富な可視化オプション**

RViz2で問題が発生する場合の最適な代替手段です。