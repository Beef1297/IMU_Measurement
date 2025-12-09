"""
LSM6DS3TR加速度センサーによる振動測定システム - シンプル統合版

機能:
- 複数センサーからのリアルタイム加速度データ取得
- 直交検波による振動振幅・位相の検出
- データの可視化とファイル出力

@author keigo ushiyama (Pythonバージョン)
@date 2023/12/26
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
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ============================================================
# 設定
# ============================================================
class Config:
    """システム設定"""
    # シリアル通信
    COM_PORT = "COM7"
    BAUD_RATE = 921600
    
    # センサー設定
    FULL_SCALE_IN_G = 16.0  # [g]
    FULL_SCALE_IN_MS = FULL_SCALE_IN_G * 9.8 # [m/s2]
    SAMPLING_RATE = 1660.0  # [Hz]
    SENSOR_NUM = 2
    MAX_SENSOR_NUM = 8
    
    # 振動設定
    VIBRATION_FREQ = 70.0  # [Hz]
    VIBRATION_CYCLES = 3.0
    DURATION = VIBRATION_CYCLES / VIBRATION_FREQ
    DATA_LEN = int(SAMPLING_RATE * DURATION)
    
    # 定数
    ADC_RESOLUTION = 1 << 16
    GRAVITY_MS2 = 9.8
    
    # ファイル設定
    DATA_DIR = Path("./data")
    
    # 表示設定
    FRAMERATE = 30


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
        g_value = raw_value * (2.0 * Config.FULL_SCALE_IN_G) / Config.ADC_RESOLUTION
        return g_value * Config.GRAVITY_MS2
    
    def __repr__(self):
        return f"Accel({self.ax:.3f}, {self.ay:.3f}, {self.az:.3f})"


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
        
        # Z軸データを抽出
        az = np.array([a.az for a in accel_list[:Config.DATA_LEN]])
        
        # 畳み込み
        ss_sum = np.sum(self.SS * az)
        cc_sum = np.sum(self.CC * az)
        
        # 振幅と位相
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
# 可視化
# ============================================================
class Visualizer:
    """リアルタイム可視化"""
    
    def __init__(self, data_manager: DataManager, detector: VibrationDetector):
        self.data_manager = data_manager
        self.detector = detector
        self.fig = None
        self.axes = []
        self.lines = {}
        self.texts = {} # 振幅表示用
        # FPS計測用
        self.frame_count = 0
        self.last_time = time.time()
        self.fps_text = None
    
    def setup(self):
        """プロット初期化"""
        self.fig = plt.figure(figsize=(12, 8))
        
        # FPS計測用
        self.frame_count = 0
        self.last_time = time.time()

        for i in range(Config.MAX_SENSOR_NUM):
            # r = (i % 2) + 1
            # c = int(i // 2) + 1
            # print(f"r: {r}, c: {c}")
            ax = self.fig.add_subplot(Config.MAX_SENSOR_NUM//2, 2, i+1)
            ax.set_xlim(0, Config.DATA_LEN)
            ax.set_ylim(-Config.FULL_SCALE_IN_MS, Config.FULL_SCALE_IN_MS)
            # ax.set_xlabel('Sample')
            ax.set_ylabel('Accel [m/s²]')
            ax.set_title(f'Sensor {i+1}')
            ax.grid(True, alpha=0.3)
            
            # ライン
            lx, = ax.plot([], [], 'r-', label='X', linewidth=1)
            ly, = ax.plot([], [], 'g-', label='Y', linewidth=1)
            lz, = ax.plot([], [], 'b-', label='Z', linewidth=1)
            ax.plot([0, Config.DATA_LEN], [0, 0], 'k-', linewidth=0.5, alpha=0.5)
            ax.legend(loc='upper right')
            
            self.axes.append(ax)
            self.lines[i] = {'x': lx, 'y': ly, 'z': lz}
            
            # テキスト
            text = ax.text(0.02, 0.95, '', transform=ax.transAxes,
                          verticalalignment='top',
                          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            self.texts[i] = text
        
        # FPSテキストを追加
        self.fps_text = self.fig.text(0.02, 0.98, 'FPS: --', 
                                       transform=self.axes[0].transAxes,horizontalalignment='right',
                                       fontsize=12,
                                       bbox=dict(boxstyle='round', 
                                               facecolor='yellow', 
                                               alpha=0.7))

        self.fig.tight_layout()
        return self.fig
    
    def update(self, frame):
        """表示更新"""

        # FPS計算
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.last_time
        
        if elapsed >= 1.0:  # 1秒ごとに更新
            fps = self.frame_count / elapsed
            self.fps_text.set_text(f'FPS: {fps:.1f}')
            self.frame_count = 0
            self.last_time = current_time
            print(fps)

        # 更新処理
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
            
            # ライン更新
            self.lines[sid]['x'].set_data(x_idx, x_data)
            self.lines[sid]['y'].set_data(x_idx, y_data)
            self.lines[sid]['z'].set_data(x_idx, z_data)
            
            # Y軸調整
            # all_data = np.concatenate([x_data, y_data, z_data])
            # if len(all_data) > 0:
            #     y_min, y_max = all_data.min(), all_data.max()
            #     margin = (y_max - y_min) * 0.1 if y_max != y_min else 1
            #     self.axes[sid].set_ylim(y_min - margin, y_max + margin)
            
            # 振幅計算
            # print(z_data)
            max_amp = np.max(np.abs(z_data)) if len(z_data) > 0 else 0
            # print(max_amp)
            self.data_manager.max_amps[sid] = max_amp
            
            if len(data) >= Config.DATA_LEN:
                calc_amp, _ = self.detector.detect(data)
                self.data_manager.calc_amps[sid] = calc_amp
            else:
                calc_amp = 0
            
            # テキスト更新
            self.texts[sid].set_text(
                f"Calc: {calc_amp:.3f} m/s²\nMeans: {max_amp:.3f} m/s²"
            )
        
        # when blit=True, we need to return updated "artist" objects
        # ラインとテキストの両方を返す
        all_artists = []
        
        # すべてのラインを追加
        for lines_dict in self.lines.values():
            all_artists.extend(lines_dict.values())
        
        # すべてのテキストを追加
        all_artists.extend(self.texts.values())
        # all_artists.append(self.fps_text)
        
        return all_artists
    
    def start(self):
        """アニメーション開始"""
        ani = FuncAnimation(self.fig, self.update,
                           interval=1000//Config.FRAMERATE,
                           blit=True, cache_frame_data=False)
        return ani


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
    
    def save_qd_result(self, results: List[dict]):
        """直交検波結果保存"""
        filename = self._create_filename("qd")
        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['freq', 'amp_set', 'qd_amp', 'max_amp'])
            writer.writeheader()
            writer.writerows(results)
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
        
        self.data_manager = DataManager()
        self.detector = VibrationDetector()
        self.serial = SerialComm(self.data_manager)
        self.visualizer = Visualizer(self.data_manager, self.detector)
        self.file_writer = FileWriter(file_name)
        self.animation = None
        
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
        print("  h - Help")
    
    def start(self):
        """システム起動"""
        self.serial.start()
        self.visualizer.setup()
        self.animation = self.visualizer.start()
        
        # コマンド入力スレッド
        cmd_thread = threading.Thread(target=self._command_loop, daemon=True)
        cmd_thread.start()
        
        try:
            plt.show()
        except KeyboardInterrupt:
            self.shutdown()
    
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
                elif cmd == 'h':
                    self._print_help()
                
            except EOFError:
                break
            except Exception as e:
                print(f"Error: {e}")
    
    def shutdown(self):
        """終了処理"""
        print("\n=== Shutting down ===")
        self.serial.dispose()
        plt.close('all')
        print("Done")
        sys.exit(0)


# ============================================================
# エントリーポイント
# ============================================================
def main():
    file_name = sys.argv[1] if len(sys.argv) > 1 else "sample"
    system = MeasurementSystem(file_name)
    system.start()


if __name__ == "__main__":
    main()