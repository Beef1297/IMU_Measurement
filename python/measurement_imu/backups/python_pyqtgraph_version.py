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
    BAUD_RATE = 921600
    
    FULL_SCALE = 16.0  # [g]
    SAMPLING_RATE = 1660.0  # [Hz]
    SENSOR_NUM = 4
    MAX_SENSOR_NUM = 8
    
    VIBRATION_FREQ = 70.0  # [Hz]
    VIBRATION_CYCLES = 5.0
    DURATION = VIBRATION_CYCLES / VIBRATION_FREQ
    DATA_LEN = int(SAMPLING_RATE * DURATION)
    
    ADC_RESOLUTION = 1 << 16
    GRAVITY_MS2 = 9.8
    
    DATA_DIR = Path("./data")
    
    FRAMERATE = 60  # PyQtGraphなら60FPS可能


# ============================================================
# 加速度データクラス
# ============================================================
class Accel:
    """加速度データ"""
    
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
        if raw_value > 32767:
            raw_value -= 65536
        g_value = raw_value * (2.0 * Config.FULL_SCALE) / Config.ADC_RESOLUTION
        return g_value * Config.GRAVITY_MS2


def calc_average_accel(accel_list: List[Accel]) -> Accel:
    """平均加速度を計算（重力除去用）"""
    if not accel_list:
        return Accel(ax=0, ay=0, az=0)
    sum_x = sum(a.ax for a in accel_list)
    sum_y = sum(a.ay for a in accel_list)
    sum_z = sum(a.az for a in accel_list)
    n = len(accel_list)
    return Accel(ax=sum_x/n, ay=sum_y/n, az=sum_z/n)


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
    
    def detect(self, accel_list: List[Accel]) -> Tuple[float, float]:
        """振幅と位相を検出"""
        if len(accel_list) < Config.DATA_LEN:
            return 0.0, 0.0
        
        az = np.array([a.az for a in accel_list[:Config.DATA_LEN]])
        ss_sum = np.sum(self.SS * az)
        cc_sum = np.sum(self.CC * az)
        
        amplitude = np.sqrt(ss_sum**2 + cc_sum**2) * 2.0 / Config.DATA_LEN
        phase = np.arctan2(cc_sum, ss_sum)
        
        return amplitude, phase


# ============================================================
# データ収集・管理
# ============================================================
class DataManager:
    """データ収集と管理"""
    
    def __init__(self):
        self.accel_lists = [deque(maxlen=Config.DATA_LEN) 
                           for _ in range(Config.SENSOR_NUM)]
        self.measurement_list = []
        self.timestamps = []
        self.max_amps = [0.0] * Config.SENSOR_NUM
        self.calc_amps = [0.0] * Config.SENSOR_NUM
        self.is_recording = False
        self._lock = threading.Lock()
    
    def add_data(self, sensor_id: int, accel: Accel):
        """データを追加"""
        with self._lock:
            self.accel_lists[sensor_id].append(accel)
            if self.is_recording:
                self.measurement_list.append(accel)
                self.timestamps.append(int(time.time_ns() / 1000))
    
    def get_data(self, sensor_id: int) -> List[Accel]:
        """データを取得"""
        with self._lock:
            return list(self.accel_lists[sensor_id])
    
    def start_recording(self):
        """記録開始"""
        with self._lock:
            self.measurement_list.clear()
            self.timestamps.clear()
            self.is_recording = True
        print("Recording started")
    
    def stop_recording(self) -> Tuple[List[Accel], List[int]]:
        """記録停止"""
        with self._lock:
            self.is_recording = False
            return self.measurement_list.copy(), self.timestamps.copy()


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
        
        try:
            self.port = serial.Serial(Config.COM_PORT, Config.BAUD_RATE, timeout=1)
            print(f"Serial opened: {Config.COM_PORT}")
            time.sleep(2)
        except Exception as e:
            print(f"Serial error: {e}")
    
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
    
    def _read_loop(self):
        """データ読み取りループ"""
        bytes_expected = Config.SENSOR_NUM * 6
        while self.is_running:
            try:
                if self.port.in_waiting >= bytes_expected:
                    buffer = self.port.read(bytes_expected)
                    self._parse_data(buffer)
            except Exception as e:
                print(f"Read error: {e}")
                break
    
    def _parse_data(self, buffer: bytes):
        """データ解析"""
        for sensor_id in range(Config.SENSOR_NUM):
            offset = sensor_id * 6
            raw = [struct.unpack('>h', buffer[offset+i*2:offset+i*2+2])[0] 
                   for i in range(3)]
            accel = Accel(raw_x=raw[0], raw_y=raw[1], raw_z=raw[2])
            self.data_manager.add_data(sensor_id, accel)
    
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
        self.resize(1200, 800)
        
        # PyQtGraphのグローバル設定（文字色など）
        pg.setConfigOptions(antialias=True)  # アンチエイリアス有効化
        pg.setConfigOption('foreground', 'k')  # 文字色を黒に（'k'=black, 'w'=white）
        # pg.setConfigOption('foreground', 'w')  # ダークモード用は白

        # レイアウト
        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)
        
        # FPSラベル
        self.fps_label = QtWidgets.QLabel('FPS: --')
        self.fps_label.setStyleSheet('font-size: 14px; font-weight: bold; color: red;')
        layout.addWidget(self.fps_label)
        
        # グラフウィジェット
        self.graphics_layout = pg.GraphicsLayoutWidget()
        
        self.graphics_layout.setBackground('#F5F5F5')

        layout.addWidget(self.graphics_layout)
        

        # 各センサー用のプロット
        self.plots = []
        self.curves = []
        self.text_items = []
        
        y_range = Config.FULL_SCALE * Config.GRAVITY_MS2
        
        for i in range(Config.MAX_SENSOR_NUM):
            # アクティブかどうかで表示を変える
            is_active = i < Config.SENSOR_NUM

            # プロット作成
            plot = self.graphics_layout.addPlot(row=i // 2, col=i % 2)
            plot.setTitle(f'Sensor {i + 1}')
            plot.setLabel('bottom', 'Sample')
            plot.setLabel('left', 'Accel', units='m/s²')
            plot.setYRange(-y_range, y_range)
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.addLegend()
    
            # 非アクティブなセンサーは背景を暗くする
            if not is_active:
                plot.getViewBox().setBackgroundColor('#F0F0F0')
            
            # 凡例（アクティブセンサーのみ）
            if is_active:
                legend = plot.addLegend(offset=(5, 5))
                # 凡例のスタイル設定
                legend.setParentItem(plot.getViewBox())
                legend.anchor((0, 0), (0, 0))  # 左上に固定

            plot.getViewBox().setBackgroundColor('#F0F0F0')
            
            # X, Y, Z軸のカーブ
            curve_x = plot.plot(pen=pg.mkPen('r', width=1), name='X')
            curve_y = plot.plot(pen=pg.mkPen('g', width=1), name='Y')
            curve_z = plot.plot(pen=pg.mkPen('b', width=1), name='Z')
            
            # ゼロライン
            plot.plot([0, Config.DATA_LEN], [0, 0], 
                     pen=pg.mkPen('k', width=1, style=QtCore.Qt.PenStyle.DashLine))
            
            # 振幅表示テキスト（右上に配置）
            text_color = '#000000' if is_active else '#AAAAAA'
            text = pg.TextItem('', anchor=(1, 0), color=text_color)  # anchor=(1,0)で右上基準
            # ViewBoxの右上に配置（座標は後で更新時に設定）
            plot.addItem(text)
            
            self.plots.append(plot)
            self.curves.append({'x': curve_x, 'y': curve_y, 'z': curve_z})
            self.text_items.append(text)
        
        # タイマー設定（高速更新）
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(1000 // Config.FRAMERATE)  # ミリ秒
    
    def update_plots(self):
        """プロット更新"""
        # FPS計算
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.last_time
        
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.fps_label.setText(f'FPS: {self.fps:.1f}')
            self.frame_count = 0
            self.last_time = current_time
        
        # 各センサーのデータを更新
        for sid in range(Config.SENSOR_NUM):
            data = self.data_manager.get_data(sid)
            if not data:
                continue
            
            # 重力除去
            avg = calc_average_accel(data)
            n = len(data)
            x_idx = np.arange(n)
            x_data = np.array([a.ax - avg.ax for a in data])
            y_data = np.array([a.ay - avg.ay for a in data])
            z_data = np.array([a.az - avg.az for a in data])
            
            # カーブ更新（PyQtGraphは非常に高速）
            self.curves[sid]['x'].setData(x_idx, x_data)
            self.curves[sid]['y'].setData(x_idx, y_data)
            self.curves[sid]['z'].setData(x_idx, z_data)
            
            # 振幅計算
            max_amp = np.max(np.abs(z_data)) if len(z_data) > 0 else 0
            self.data_manager.max_amps[sid] = max_amp
            
            if len(data) >= Config.DATA_LEN:
                calc_amp, _ = self.detector.detect(data)
                self.data_manager.calc_amps[sid] = calc_amp
            else:
                calc_amp = 0
            
            # テキスト更新
            text_str = f"Calc: {calc_amp:.3f} m/s²\nMax amp: {max_amp:.3f} m/s²"
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
    
    def save_measurement(self, accel_list: List[Accel], timestamps: List[int]):
        """測定データ保存"""
        if not accel_list:
            print("No data to save")
            return
        
        filename = self._create_filename("measurement")
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'ax', 'ay', 'az'])
            for i, a in enumerate(accel_list):
                ts = timestamps[i] if i < len(timestamps) else 0
                writer.writerow([ts, f"{a.ax:.3f}", f"{a.ay:.3f}", f"{a.az:.3f}"])
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
        self.serial.start()
        
        # コマンド入力スレッド
        cmd_thread = threading.Thread(target=self._command_loop, daemon=True)
        cmd_thread.start()
        
        # ウィンドウ表示
        self.visualizer.show()
        
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
                    for lst in self.data_manager.accel_lists:
                        lst.clear()
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
