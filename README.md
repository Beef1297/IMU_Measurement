# IMU_Measurement

## 📦 概要

IMU_Measurement は、IMU（例: LSM6DS3）を使って、加速度・角速度などの慣性データを取得／記録／可視化するためのソフトウェアです。
Arduino（組み込み）、Processing（可視化・リアルタイム確認）、Python（可視化・リアルタイム確認）など複数のモジュールで構成されており、次のような用途を想定しています：

- センサから raw IMU データを取得
- センサデータをリアルタイムでモニタ／可視化

## 🧩 ディレクトリ構成
/arduino/lsm6ds3_esp32c6   … Arduino（ESP32C6）用スケッチ（IMU センサ読み取り）  
/processing/measurement_LSM6DS3TR  … Processing スケッチ（リアルタイム可視化／データ受信）  
/python/measurement_imu     … Python スクリプト（リアルタイム可視化／データ受信）  

現状はpythonでしか動きません (Processingはchecksumやseqの判定にまだ対応していません)

## 🚀 動作環境 & 依存関係
Arduino
- プラットフォーム: ESP32C6（または互換ボード）
- センサ: LSM6DS3 または類似の I²C/SPI 対応 6-axis／9-axis MEMS IMU
- Arduino IDE または PlatformIO でビルド／書き込み

Processing

- Processing 3.x 系推奨
- シリアル通信 (Arduino から送られてくる IMU データ) を受け取るためのポート権限

Python

- Python 3.x
- 主な依存ライブラリ例: numpy, pandas, matplotlib など (スクリプト内容に応じて適宜)
- ログファイル (CSV など) の入出力および解析

⚠️ ※実際の依存ライブラリおよびバージョンは、Python スクリプト内の import 部分やコメントを参考にしてください。

## 🛠️ セットアップ & 使用手順

### マイコン側

- /arduino/lsm6ds3_esp32c6 ディレクトリを Arduino IDE (または PlatformIO) で開く
- 接続する IMU を LSM6DS3 など適切な慣性センサに設定
- ESP32C6 ボードを選んでスケッチを書き込み

### Processing 側（リアルタイム可視化）
⚠️現状では、最新のシリアル通信のプロトコル (packet) にまだ対応していません。⚠️

- /processing/measurement_LSM6DS3TR を Processing で開く
- Arduino が送る IMU データを受け取るシリアルポートを指定して実行
- センサの加速度／角速度などをリアルタイムで画面に表示

### Python 側（ログ処理／解析）
(推奨)

- /python/measurement_imu 内のスクリプトを実行 (または修正)
- Arduino が送る IMU データを受け取るシリアルポートを指定して実行
- センサの加速度／角速度などをリアルタイムで画面に表示




## 📝 注意事項

現状は主に LSM6DS3 など特定の IMU 向けに実装されています。別のセンサを使う場合はスケッチや通信フォーマットの見直しが必要です。


