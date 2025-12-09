# LSM6DS3TR Measurement System - Python版（シンプル構造）

ProcessingからPythonに移植した加速度センサー測定システムです。
**1ファイルに全機能を統合したシンプル版**です。

## ファイル構成

```
measurement-system/
├── measurement_system.py  # メインプログラム（全機能統合）
├── requirements.txt       # 依存パッケージ
└── data/                  # データ保存先（自動作成）
    └── sample/
        ├── sample-measurement-20231226_120000.csv
        └── sample-qd-20231226_120100.csv
```

## クイックスタート

### 1. インストール

```bash
pip install numpy pyserial matplotlib
```

または

```bash
pip install -r requirements.txt
```

### 2. 設定変更

`measurement_system.py`の先頭部分を編集：

```python
class Config:
    COM_PORT = "COM9"  # ← あなたのCOMポートに変更
    BAUD_RATE = 921600
    SENSOR_NUM = 2  # センサー数
    VIBRATION_FREQ = 70.0  # 振動周波数 [Hz]
```

### 3. 実行

```bash
python measurement_system.py
```

または、ファイル名を指定：

```bash
python measurement_system.py my_experiment
```

## 使い方

### コマンド一覧

プログラム実行中に以下のコマンドを入力：

| コマンド | 機能 |
|---------|------|
| `s` | データ送信開始 |
| `e` | データ送信停止 |
| `r` | 記録開始 |
| `o` | 記録停止＆保存 |
| `c` | データクリア |
| `q` | 終了 |
| `h` | ヘルプ表示 |

### 基本的な流れ

1. プログラム起動
2. `s`を入力してデータ受信開始
3. グラフでリアルタイム表示を確認
4. `r`で記録開始
5. `o`で記録停止＆CSVファイルに保存
6. `q`で終了

## コード構造

`measurement_system.py`は以下のクラスで構成：

```python
Config              # 設定管理
Accel               # 加速度データ
VibrationDetector   # 直交検波
DataManager         # データ収集
SerialComm          # シリアル通信
Visualizer          # 可視化
FileWriter          # ファイル保存
MeasurementSystem   # メインシステム
```

約400行で全機能を実装しています。

## カスタマイズ例

### 周波数を変更

```python
# measurement_system.py 内
class Config:
    VIBRATION_FREQ = 100.0  # 70 → 100 Hz
```

### センサー数を変更

```python
class Config:
    SENSOR_NUM = 4  # 2 → 4個
```

### データ長を変更

```python
class Config:
    VIBRATION_CYCLES = 20.0  # 10 → 20サイクル
```

## 出力ファイル

### 測定データ（measurement）

```csv
timestamp,ax,ay,az
1234567890,0.123,0.456,9.789
1234567891,0.124,0.457,9.790
...
```

### 直交検波結果（qd）

```csv
freq,amp_set,qd_amp,max_amp
70.0,0.00,0.123,0.145
70.0,0.01,0.234,0.256
...
```

## トラブルシューティング

### シリアルポートエラー

```
Serial error: [Errno 2] No such file or directory: 'COM9'
```

**解決方法**: `Config.COM_PORT`を正しいポート名に変更

**ポートの確認方法**:
- Windows: デバイスマネージャー
- Linux: `ls /dev/ttyUSB*`
- macOS: `ls /dev/cu.*`

### データが表示されない

1. `s`コマンドを実行したか確認
2. センサーの接続を確認
3. ボーレートが正しいか確認（921600）

### グラフが動かない

Matplotlibのバックエンドの問題の可能性があります：

```python
# measurement_system.py の先頭に追加
import matplotlib
matplotlib.use('TkAgg')  # または 'Qt5Agg'
```

## 依存パッケージ

- **numpy**: 数値計算
- **pyserial**: シリアル通信
- **matplotlib**: グラフ表示

```bash
pip install numpy>=1.21.0 pyserial>=3.5 matplotlib>=3.5.0
```

## 高度な使い方

### プログラムから直接使用

```python
from measurement_system import MeasurementSystem, Config

# 設定変更
Config.SENSOR_NUM = 4
Config.VIBRATION_FREQ = 100.0

# システム起動
system = MeasurementSystem("my_test")
system.start()
```

### データの直接アクセス

```python
# データマネージャーから直接データ取得
data = system.data_manager.get_data(sensor_id=0)
for accel in data:
    print(f"ax={accel.ax}, ay={accel.ay}, az={accel.az}")
```

## Processing版との違い

| 項目 | Processing | Python（シンプル版） |
|-----|-----------|-------------------|
| ファイル数 | 8ファイル | 1ファイル |
| 総行数 | 約800行 | 約400行 |
| クラス数 | 10+ | 8 |
| 可視化 | 組み込み | Matplotlib |

## ライセンス

（プロジェクトのライセンスを記載）

## 作者

Original: keigo ushiyama (Processing版)  
Python Port: (移植者名)

## サポート

問題が発生した場合は、以下を確認してください：

1. Pythonバージョン（3.8以上推奨）
2. 依存パッケージのバージョン
3. COMポート設定
4. センサーの接続状態

それでも解決しない場合は、GitHubのIssuesで報告してください。