"""
LSM6DS3TR加速度センサーによる振動測定システム - PyQtGraph版（高速）

PyQtGraphを使用して60FPS以上の高速描画を実現

依存パッケージ:
pip install numpy pyserial pyqtgraph PyQt6

@author keigo ushiyama (Pythonバージョン)
"""
import sys
import time
import csv
import struct
import threading
from pathlib import Path
from datetime import datetime
from collections import deque
from typing import List, Tuple, Optional

import numpy as np
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets


# ============================================================
# 設定
# ============================================================
class Config:
    """システム設定"""
    COM_PORT = "COM7"
    BAUD_RATE = 460800
    
    FULL_SCALE = 16.0  # [g]
    SAMPLING_RATE = 1660.0  # [Hz]
    SENSOR_NUM = 7  # 実際に使用するセンサー数
    MAX_SENSOR_NUM = 8  # 最大センサー数（グラフは常に8個表示）
    
    VIBRATION_FREQ = 70.0  # [Hz]
    VIBRATION_CYCLES = 10.0
    DURATION = VIBRATION_CYCLES / VIBRATION_FREQ
    DATA_LEN = int(SAMPLING_RATE * DURATION)
    
    ADC_RESOLUTION = 1 << 16
    GRAVITY_MS2 = 9.8
    
    DATA_DIR = Path("./data")
    
    FRAMERATE = 15  # PyQtGraphなら60FPS可能


# ============================================================
# 加速度データクラス（NumPy対応版）
# ============================================================
class Accel:
    """加速度データ（シンプル版）"""
    __slots__ = ['ax', 'ay', 'az']  # メモリ最適化
    
    def __init__(self, raw_x: int = None, raw_y: int = None, raw_z: int = None,
                 ax: float = None, ay: float = None, az: float = None):
        if raw_x is not None:
            self.ax = self._convert(raw_x)
            self.ay = self._convert(raw_y)
            self.az = self._convert(raw_z)
        else:
            self.ax = ax or 0.0
            self.ay = ay or 0.0
            self.az = az or 0.0
    
    @staticmethod
    def _convert(raw_value: int) -> float:
        """生データをm/s²に変換"""
        # if raw_value > 32767:
        #     raw_value -= 65536
        g_value = raw_value * (2.0 * Config.FULL_SCALE) / Config.ADC_RESOLUTION
        return g_value * Config.GRAVITY_MS2


def convert_raw_to_ms2(raw_values: np.ndarray) -> np.ndarray:
    """
    生データ配列をm/s²に一括変換（NumPy版）
    
    Args:
        raw_values: shape=(N, 3) の生データ配列
    Returns:
        shape=(N, 3) のm/s²配列
    """
    # 符号付き16bit整数で送られてくる
    signed_values = raw_values.copy()
    # signed_values[signed_values > 32767] -= 65536
    
    # G単位に変換
    g_values = signed_values * (2.0 * Config.FULL_SCALE) / Config.ADC_RESOLUTION
    
    # m/s²に変換
    ms2_values = g_values * Config.GRAVITY_MS2
    
    return ms2_values


# ============================================================
# 直交検波（振動検出）
# ============================================================
class VibrationDetector:
    """直交検波による振動検出"""
    
    def __init__(self):
        self.SS = np.zeros(Config.DATA_LEN)
        self.CC = np.zeros(Config.DATA_LEN)
        self._init_bases()
    
    def _init_bases(self):
        """sin/cos基底を初期化"""
        t = np.arange(Config.DATA_LEN) / Config.SAMPLING_RATE
        w = 2 * np.pi * Config.VIBRATION_FREQ
        self.SS = np.sin(w * t)
        self.CC = np.cos(w * t)
    
    def detect(self, az) -> Tuple[float, float]:
        """振幅と位相を検出"""
        if len(az) < Config.DATA_LEN:
            return 0.0, 0.0
        
        # az = np.array([az for az in accel_list[:Config.DATA_LEN]])
        ss_sum = np.sum(self.SS * az)
        cc_sum = np.sum(self.CC * az)
        
        amplitude = np.sqrt(ss_sum**2 + cc_sum**2) * 2.0 / Config.DATA_LEN
        phase = np.arctan2(cc_sum, ss_sum)
        
        return amplitude, phase


# ============================================================
# データ収集・管理（NumPy版）
# ============================================================
class DataManager:
    """データ収集と管理（NumPy配列使用）"""
    
    def __init__(self):
        # NumPy配列でデータ管理（shape: [MAX_SENSOR_NUM, DATA_LEN, 3]）
        self.accel_data = np.zeros((Config.MAX_SENSOR_NUM, Config.DATA_LEN, 3), dtype=np.float32)
        self.data_index = np.zeros(Config.MAX_SENSOR_NUM, dtype=np.int32)  # 各センサーの書き込み位置
        self.data_count = np.zeros(Config.MAX_SENSOR_NUM, dtype=np.int32)  # 各センサーのデータ数
        
        self.measurement_list = []
        self.timestamps = []
        self.max_amps = np.zeros(Config.MAX_SENSOR_NUM, dtype=np.float32)
        self.calc_amps = np.zeros(Config.MAX_SENSOR_NUM, dtype=np.float32)
        self.is_recording = False
        self._lock = threading.Lock()
    
    def add_data(self, sensor_id: int, ax: float, ay: float, az: float):
        """
        データを追加（NumPy版）
        
        Args:
            sensor_id: センサーID
            ax, ay, az: 加速度 [m/s²]
        """
        if sensor_id >= Config.MAX_SENSOR_NUM:
            return
        
        with self._lock:
            idx = self.data_index[sensor_id]
            
            # リングバッファとして使用
            self.accel_data[sensor_id, idx, 0] = ax
            self.accel_data[sensor_id, idx, 1] = ay
            self.accel_data[sensor_id, idx, 2] = az
            
            # インデックス更新
            self.data_index[sensor_id] = (idx + 1) % Config.DATA_LEN
            
            # データ数更新（最大DATA_LEN）
            if self.data_count[sensor_id] < Config.DATA_LEN:
                self.data_count[sensor_id] += 1
            
            # 記録中なら追加
            if self.is_recording and sensor_id < Config.SENSOR_NUM:
                self.measurement_list.append([ax, ay, az])
                self.timestamps.append(int(time.time_ns() / 1000))
    
    def get_data_numpy(self, sensor_id: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        データを取得（NumPy版）
        
        Returns:
            (x_data, y_data, z_data) のタプル
        """
        with self._lock:
            count = self.data_count[sensor_id]
            if count == 0:
                return np.array([]), np.array([]), np.array([])
            
            idx = self.data_index[sensor_id]
            
            # リングバッファから正しい順序でデータを取得
            if count < Config.DATA_LEN:
                # まだバッファが満たされていない
                x_data = self.accel_data[sensor_id, :count, 0].copy()
                y_data = self.accel_data[sensor_id, :count, 1].copy()
                z_data = self.accel_data[sensor_id, :count, 2].copy()
            else:
                # バッファが満杯：古いデータから順に並べ替え
                x_data = np.concatenate([
                    self.accel_data[sensor_id, idx:, 0],
                    self.accel_data[sensor_id, :idx, 0]
                ])
                y_data = np.concatenate([
                    self.accel_data[sensor_id, idx:, 1],
                    self.accel_data[sensor_id, :idx, 1]
                ])
                z_data = np.concatenate([
                    self.accel_data[sensor_id, idx:, 2],
                    self.accel_data[sensor_id, :idx, 2]
                ])
            
            return x_data, y_data, z_data
    
    def start_recording(self):
        """記録開始"""
        with self._lock:
            self.measurement_list.clear()
            self.timestamps.clear()
            self.is_recording = True
        print("Recording started")
    
    def stop_recording(self) -> Tuple[np.ndarray, List[int]]:
        """記録停止"""
        with self._lock:
            self.is_recording = False
            data = np.array(self.measurement_list) if self.measurement_list else np.array([])
            return data, self.timestamps.copy()
    
    def clear_all(self):
        """全データクリア"""
        with self._lock:
            self.accel_data.fill(0)
            self.data_index.fill(0)
            self.data_count.fill(0)
            self.measurement_list.clear()
            self.timestamps.clear()


# ============================================================
# シリアル通信
# ============================================================
class SerialComm:
    """シリアル通信"""
    
    def __init__(self, data_manager: DataManager):
        self.data_manager = data_manager
        self.port = None
        self.is_running = False
        self.thread = None
        
        self.last_seq = None  # 直前の seq を保持してギャップ検出に使う

        try:
            self.port = serial.Serial(Config.COM_PORT, Config.BAUD_RATE, timeout=1)
            print(f"Serial opened: {Config.COM_PORT}")
            time.sleep(2)
        except Exception as e:
            print(f"Serial error: {e}")
    
    @staticmethod
    def calc_checksum(data: bytes) -> int:
        """Arduino 側と同じ: バイトの和を 16bit にマスク"""
        return sum(data) & 0xFFFF


    def start(self):
        """通信開始"""
        if not self.port:
            return
        self.is_running = True
        self.port.reset_input_buffer()
        self.port.write(b's')
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print("Serial started")
    
    def stop(self):
        """通信停止"""
        if not self.port:
            return
        self.is_running = False
        try:
            self.port.write(b'e')
        except:
            pass
        print("Serial stopped")
    
    def _sync_serial(self, consecutive_errors: int, max_errors: int):
        """
        シリアル通信の同期を再確立
        
        Args:
            consecutive_errors: 連続エラー回数
            max_errors: 最大連続エラー数
        """
        # print(f"Too many consecutive errors ({consecutive_errors}). Resetting serial buffer...")
        
        if (consecutive_errors > max_errors) :
            # 送信を停止
            print("Sending 'e' to stop transmission...")
            self.port.write(b'e')
            # time.sleep(1)  # マイコンの処理待ち
            
            # # バッファをクリア
            self.port.reset_input_buffer()
            # self.port.reset_output_buffer()
            # time.sleep(0.1)
            
            # 送信を再開
            print("Sending 's' to restart transmission...")
            self.port.write(b's')
            # time.sleep(1)
            
            # print("Serial communication restarted successfully.")

            return 0
            # エラーカウントリセット（呼び出し元でもリセットされる
        else :
            # do nothing
            return consecutive_errors



    def _read_loop(self):
        """データ読み取りループ（ヘッダ探索 + seq/CRC チェック付き）"""
        try:
            import ctypes
            ctypes.windll.kernel32.SetThreadPriority(
                ctypes.windll.kernel32.GetCurrentThread(), 2
            )
        except:
            pass

        # パケット構造:
        # [0] HEADER(0x7F)
        # [1..2] SEQ (uint16 big-endian)
        # [3..(3+SENSOR_NUM*6-1)] DATA
        # [..+0..+1] CHECKSUM (uint16 big-endian)
        # [最後] FOOTER(0xFE)
        header_len = 1
        seq_len = 2
        data_len = Config.SENSOR_NUM * 6
        checksum_len = 2
        footer_len = 1

        bytes_expected = header_len + seq_len + data_len + checksum_len + footer_len

        print(f"Serial reading loop started. Expecting {bytes_expected} bytes per packet")
        print(
            f"Format: HEADER(0x7F) + SEQ(2) + DATA({data_len} bytes) "
            f"+ CHECKSUM(2) + FOOTER(0xFE)"
        )

        buffer = bytearray()
        consecutive_errors = 0
        max_consecutive_errors = 50  # 必要なら調整

        while self.is_running:
            try:
                # 1バイトずつ読んで HEADER を探す
                b = self.port.read(1)
                if not b:
                    # タイムアウト。状況によっては sleep を少し入れてもよい
                    continue

                if b[0] != 0x7F:
                    # ヘッダじゃないので捨てて次へ
                    continue

                # ヘッダは見つかったので、残りをまとめて読む
                rest = self.port.read(bytes_expected - 1)
                if len(rest) != bytes_expected - 1:
                    print(f"Incomplete packet: got {len(rest)+1} bytes, expected {bytes_expected}")
                    consecutive_errors += 1
                    if consecutive_errors > max_consecutive_errors:
                        print("Too many errors, resetting input buffer...")
                        try:
                            self.port.reset_input_buffer()
                        except Exception as e:
                            print(f"reset_input_buffer error: {e}")
                        consecutive_errors = 0
                    continue

                packet = b + rest  # 長さ bytes_expected の 1パケット

                # FOOTER チェック
                if packet[-1] != 0xFE:
                    print(f"Invalid footer: 0x{packet[-1]:02X}, expected 0xFE")
                    consecutive_errors += 1
                    continue

                # seq 抜き出し（big-endian）
                seq = (packet[1] << 8) | packet[2]
                if self.last_seq is not None:
                    diff = (seq - self.last_seq) & 0xFFFF
                    if diff != 1:
                        print(f"Sequence jump detected: prev={self.last_seq}, now={seq}, diff={diff}")
                self.last_seq = seq

                # チェックサム確認
                payload_end = header_len + seq_len + data_len  # checksum計算に含める長さ
                checksum_start = payload_end
                checksum_end = checksum_start + checksum_len

                calc = SerialComm.calc_checksum(packet[:payload_end])
                recv = (packet[checksum_start] << 8) | packet[checksum_start + 1]

                if calc != recv:
                    print(f"Checksum mismatch: calc=0x{calc:04X}, recv=0x{recv:04X}")
                    consecutive_errors += 1
                    continue  # このパケットは捨てる

                # ここまで来ればパケットは正常
                consecutive_errors = 0

                # データ部だけを取り出して解析へ渡す
                data_start = header_len + seq_len
                data_end = data_start + data_len
                data_buffer = packet[data_start:data_end]  # length = SENSOR_NUM * 6

                self._parse_data(data_buffer)

            except Exception as e:
                print(f"Read error: {e}")
                import traceback
                traceback.print_exc()
                break

    
    def _parse_data(self, buffer: bytes):
        """データ解析（NumPy版）"""
        try:
            raw_data = np.frombuffer(buffer, dtype='>i2').reshape(Config.SENSOR_NUM, 3)
            accel_data = convert_raw_to_ms2(raw_data)
            for sensor_id in range(Config.SENSOR_NUM):
                self.data_manager.add_data(
                    sensor_id,
                    accel_data[sensor_id, 0],
                    accel_data[sensor_id, 1],
                    accel_data[sensor_id, 2],
                )
        except Exception as e:
            print(f"Parse error: {e}")
            print(f"Buffer length: {len(buffer)}, expected: {Config.SENSOR_NUM * 6}")
            import traceback
            traceback.print_exc()

    
    def dispose(self):
        """クリーンアップ"""
        self.stop()
        if self.port and self.port.is_open:
            self.port.close()


# ============================================================
# 可視化（PyQtGraph - 高速）
# ============================================================
class Visualizer(QtWidgets.QWidget):
    """PyQtGraphによる高速リアルタイム可視化"""
    
    def __init__(self, data_manager: DataManager, detector: VibrationDetector):
        super().__init__()
        self.data_manager = data_manager
        self.detector = detector
        
        # FPS計測
        self.frame_count = 0
        self.last_time = time.time()
        self.fps = 0.0
        
        self._init_ui()
    
    def _init_ui(self):
        """UI初期化"""
        self.setWindowTitle('LSM6DS3TR Measurement System')
        self.resize(1400, 900)
        
        # PyQtGraphのグローバル設定（高速化）
        pg.setConfigOptions(
            antialias=False,  # アンチエイリアス無効化で高速化
            useOpenGL=True,   # OpenGL使用で高速化
            enableExperimental=True
        )
        pg.setConfigOption('foreground', 'k')  # 文字色を黒に
        
        # レイアウト
        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)
        
        # 情報パネル
        info_layout = QtWidgets.QHBoxLayout()
        
        # FPSラベル
        self.fps_label = QtWidgets.QLabel('FPS: --')
        self.fps_label.setStyleSheet('font-size: 14px; font-weight: bold; color: red;')
        info_layout.addWidget(self.fps_label)
        
        # センサー情報ラベル
        self.sensor_info_label = QtWidgets.QLabel(
            f'Active Sensors: {Config.SENSOR_NUM} / {Config.MAX_SENSOR_NUM}'
        )
        self.sensor_info_label.setStyleSheet('font-size: 14px; font-weight: bold; color: blue;')
        info_layout.addWidget(self.sensor_info_label)
        
        info_layout.addStretch()
        layout.addLayout(info_layout)
        
        # グラフウィジェット
        self.graphics_layout = pg.GraphicsLayoutWidget()
        # ウィンドウ全体の背景色を設定
        self.graphics_layout.setBackground('#FFFFFF')  # 白色
        # または他の色: '#E8E8E8'（薄いグレー）, '#F5F5F5'（ほぼ白）など
        layout.addWidget(self.graphics_layout)
        
        # 8個のプロット（2行4列）
        self.plots = []
        self.curves = []
        self.text_items = []
        
        y_range = Config.FULL_SCALE * Config.GRAVITY_MS2
        
        for i in range(Config.MAX_SENSOR_NUM):
            row = i // 4  # 0, 1
            col = i % 4   # 0, 1, 2, 3
            
            # プロット作成
            plot = self.graphics_layout.addPlot(row=row, col=col)
            
            # アクティブかどうかで表示を変える
            is_active = i < Config.SENSOR_NUM
            title_color = '#000000' if is_active else '#888888'  # 黒 or グレー
            label_color = '#000000' if is_active else '#888888'
            
            plot.setTitle(f'Sensor {i + 1}', color=title_color, size='12pt')
            plot.setLabel('left', 'Accel', units='m/s²', color=label_color)
            plot.setYRange(-y_range, y_range)
            plot.showGrid(x=True, y=True, alpha=0.3)
            
            # 軸の色を設定
            axis_color = '#000000' if is_active else '#CCCCCC'
            plot.getAxis('left').setPen(axis_color)
            plot.getAxis('bottom').setPen(axis_color)
            plot.getAxis('left').setTextPen(axis_color)
            plot.getAxis('bottom').setTextPen(axis_color)
            
            # 非アクティブなセンサーは背景を暗くする
            if not is_active:
                plot.getViewBox().setBackgroundColor('#F0F0F0')
            
            # 凡例（アクティブセンサーのみ）
            if is_active:
                legend = plot.addLegend(offset=(5, 5))
                # 凡例のスタイル設定
                legend.setParentItem(plot.getViewBox())
                legend.anchor((0, 0), (0, 0))  # 左上に固定
            
            # X, Y, Z軸のカーブ
            # 高速化: downsample と clipToView を有効化
            curve_x = plot.plot(pen=pg.mkPen('r', width=1), name='X' if is_active else None)
            curve_y = plot.plot(pen=pg.mkPen('g', width=1), name='Y' if is_active else None)
            curve_z = plot.plot(pen=pg.mkPen('b', width=1), name='Z' if is_active else None)
            
            # データアイテムの高速化設定
            for curve in [curve_x, curve_y, curve_z]:
                curve.setClipToView(True)  # 表示範囲外のデータを描画しない
                curve.setDownsampling(auto=True, method='peak')  # 自動ダウンサンプリング
            
            # ゼロライン
            plot.plot([0, Config.DATA_LEN], [0, 0], 
                     pen=pg.mkPen('k', width=1, style=QtCore.Qt.PenStyle.DashLine))
            
            # 振幅表示テキスト（右上に配置）
            text_color = '#000000' if is_active else '#AAAAAA'
            text = pg.TextItem('', anchor=(1, 0), color=text_color)  # anchor=(1,0)で右上基準
            # ViewBoxの右上に配置（座標は後で更新時に設定）
            plot.addItem(text)
            
            # 非アクティブなセンサーには "Not Active" と表示
            if not is_active:
                text.setText('Not Active')
                text.setColor('#AAAAAA')
                # 中央に配置
                text.setPos(Config.DATA_LEN / 2, 0)
                text.setAnchor((0.5, 0.5))
            
            self.plots.append(plot)
            self.curves.append({'x': curve_x, 'y': curve_y, 'z': curve_z})
            self.text_items.append(text)
        
        # タイマー設定（高速更新）
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        # 高速化: 30FPSに制限（60FPSは過剰）
        self.timer.start(33)  # 約30FPS (1000/30 ≈ 33ms)
    
    def update_plots(self):
        """プロット更新（NumPy最適化版）"""
        # FPS計算
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.last_time
        
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.fps_label.setText(f'FPS: {self.fps:.1f}')
            self.frame_count = 0
            self.last_time = current_time
        
        # アクティブなセンサーのみ更新
        for sid in range(Config.SENSOR_NUM):
            # NumPy配列で直接取得
            x_data, y_data, z_data = self.data_manager.get_data_numpy(sid)
            
            if len(x_data) == 0:
                continue
            
            # 重力除去（NumPy高速演算）
            x_data = x_data - np.mean(x_data)
            y_data = y_data - np.mean(y_data)
            z_data = z_data - np.mean(z_data)
            
            n = len(x_data)
            x_idx = np.arange(n)
            
            # カーブ更新（高速化）
            self.curves[sid]['x'].setData(x_idx, x_data, connect='finite')
            self.curves[sid]['y'].setData(x_idx, y_data, connect='finite')
            self.curves[sid]['z'].setData(x_idx, z_data, connect='finite')
            
            # 振幅計算（NumPy）
            max_amp = np.max(np.abs(z_data)) if len(z_data) > 0 else 0
            self.data_manager.max_amps[sid] = max_amp
            
            # 直交検波（データが十分にある場合）
            if len(z_data) >= Config.DATA_LEN:
                calc_amp, _ = self.detector.detect(z_data)
                self.data_manager.calc_amps[sid] = calc_amp
            else:
                calc_amp = 0
            
            # テキスト更新（0.5秒に1回程度に制限）
            # if elapsed >= 0.5:
            text_str = f"Calc: {calc_amp:.3f} m/s²\nMax: {max_amp:.3f} m/s²"
            self.text_items[sid].setText(text_str)
            # 右上の座標を設定（データ座標系）
            y_range = Config.FULL_SCALE * Config.GRAVITY_MS2
            self.text_items[sid].setPos(Config.DATA_LEN - 10, y_range * 0.95)


# ============================================================
# ファイル保存
# ============================================================
class FileWriter:
    """ファイル保存"""
    
    def __init__(self, base_name: str):
        self.base_name = base_name
        (Config.DATA_DIR / base_name).mkdir(parents=True, exist_ok=True)
    
    def _create_filename(self, suffix: str) -> Path:
        """ファイル名生成"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return Config.DATA_DIR / self.base_name / f"{self.base_name}-{suffix}-{timestamp}.csv"
    
    def save_measurement(self, accel_data: np.ndarray, timestamps: List[int]):
        """
        測定データ保存（NumPy版）
        
        Args:
            accel_data: shape=(N, 3) の加速度データ配列
            timestamps: タイムスタンプリスト
        """
        if len(accel_data) == 0:
            print("No data to save")
            return
        
        filename = self._create_filename("measurement")
        
        # NumPyでCSV保存（高速）
        header = 'timestamp,ax,ay,az'
        data_with_ts = np.column_stack([
            timestamps[:len(accel_data)],
            accel_data
        ])
        
        np.savetxt(filename, data_with_ts, delimiter=',', 
                   header=header, comments='', fmt='%d,%.3f,%.3f,%.3f')
        
        print(f"Saved: {filename}")


# ============================================================
# メインシステム
# ============================================================
class MeasurementSystem:
    """測定システム"""
    
    def __init__(self, file_name: str = "sample"):
        print("=== System Initialization ===")
        print(f"Data length: {Config.DATA_LEN}")
        print(f"Sampling rate: {Config.SAMPLING_RATE} Hz")
        print(f"Vibration freq: {Config.VIBRATION_FREQ} Hz")
        print(f"Target FPS: {Config.FRAMERATE}")
        
        self.data_manager = DataManager()
        self.detector = VibrationDetector()
        self.serial = SerialComm(self.data_manager)
        self.file_writer = FileWriter(file_name)
        
        # Qt アプリケーション
        self.app = QtWidgets.QApplication(sys.argv)
        self.visualizer = Visualizer(self.data_manager, self.detector)
        
        print("=== Initialization Complete ===\n")
        self._print_help()
    
    def _print_help(self):
        """ヘルプ表示"""
        print("Commands:")
        print("  s - Start data transmission")
        print("  e - Stop data transmission")
        print("  r - Start recording")
        print("  o - Stop recording & save")
        print("  c - Clear data")
        print("  q - Quit")
    
    def start(self):
        """システム起動"""
        
        # コマンド入力スレッド
        cmd_thread = threading.Thread(target=self._command_loop, daemon=True)
        cmd_thread.start()
        
        # ウィンドウ表示
        self.visualizer.show()

        # シリアルスタート
        # 's' を送ることで、データが送られてくるので、いろいろ準備ができてから start() する
        self.serial.start()
        
        # イベントループ開始
        sys.exit(self.app.exec())
    
    def _command_loop(self):
        """コマンド処理ループ"""
        while True:
            try:
                cmd = input().strip().lower()
                
                if cmd == 's':
                    self.serial.start()
                elif cmd == 'e':
                    self.serial.stop()
                elif cmd == 'r':
                    self.data_manager.start_recording()
                elif cmd == 'o':
                    data, ts = self.data_manager.stop_recording()
                    self.file_writer.save_measurement(data, ts)
                elif cmd == 'c':
                    self.data_manager.clear_all()
                    print("Data cleared")
                elif cmd == 'q':
                    self.shutdown()
                    break
                
            except EOFError:
                break
            except Exception as e:
                print(f"Error: {e}")
    
    def shutdown(self):
        """終了処理"""
        print("\n=== Shutting down ===")
        self.serial.dispose()
        self.app.quit()


# ============================================================
# エントリーポイント
# ============================================================
def main():
    file_name = sys.argv[1] if len(sys.argv) > 1 else "sample"
    system = MeasurementSystem(file_name)
    system.start()


if __name__ == "__main__":
    main()
