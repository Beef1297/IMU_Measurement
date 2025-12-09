#ifndef LSM6DS3_H
#define LSM6DS3_H

#include <Arduino.h>
#include <SPI.h>

// -------------------- レジスタアドレス --------------------
#define LSM6DS3_REG_WHO_AM_I 0x0F
#define LSM6DS3_REG_CTRL1_XL 0x10
#define LSM6DS3_REG_CTRL2_G 0x11
#define LSM6DS3_REG_CTRL3_C 0x12
#define LSM6DS3_REG_OUTX_L_G 0x22
#define LSM6DS3_REG_OUTX_L_XL 0x28

// -------------------- 設定値の列挙体 --------------------
// CTRL1_XL: 加速度センサの帯域幅 (BW_XL)
enum AccelBW {
  ACC_BW_400Hz = 0x00,
  ACC_BW_200Hz = 0x01,
  ACC_BW_100Hz = 0x02,
  ACC_BW_50Hz = 0x03
};

// CTRL1_XL: 加速度センサのフルスケール (FS_XL)
enum AccelScale {
  ACC_FS_2G = 0x00,
  ACC_FS_16G = 0x04,
  ACC_FS_4G = 0x08,
  ACC_FS_8G = 0x0C
};

// CTRL1_XL: 加速度センサの出力データレート (ODR_XL)
enum AccelODR {
  ACC_ODR_POWERDOWN = 0x00,
  ACC_ODR_13Hz = 0x10,
  ACC_ODR_26Hz = 0x20,
  ACC_ODR_52Hz = 0x30,
  ACC_ODR_104Hz = 0x40,
  ACC_ODR_208Hz = 0x50,
  ACC_ODR_416Hz = 0x60,
  ACC_ODR_833Hz = 0x70,
  ACC_ODR_1660Hz = 0x80,
  ACC_ODR_3330Hz = 0x90,
  ACC_ODR_6660Hz = 0xA0,
  ACC_ODR_13330Hz = 0xB0
};

// CTRL2_G: ジャイロのフルスケール (FS_G)
enum GyroScale {
  GYR_FS_245DPS = 0x00,
  GYR_FS_500DPS = 0x04,
  GYR_FS_1000DPS = 0x08,
  GYR_FS_2000DPS = 0x0C
};

// CTRL2_G: ジャイロの出力データレート (ODR_G)
enum GyroODR {
  GYR_ODR_POWERDOWN = 0x00,
  GYR_ODR_13Hz = 0x10,
  GYR_ODR_26Hz = 0x20,
  GYR_ODR_52Hz = 0x30,
  GYR_ODR_104Hz = 0x40,
  GYR_ODR_208Hz = 0x50,
  GYR_ODR_416Hz = 0x60,
  GYR_ODR_833Hz = 0x70,
  GYR_ODR_1660Hz = 0x80
};

class LSM6DS3 {
public:
  // コンストラクタ
  LSM6DS3(uint8_t csPin);
  
  // 初期化
  bool begin(AccelODR aOdr, AccelScale aScale, AccelBW aBw,
             GyroODR gOdr, GyroScale gScale);
  
  // データ読み取り
  void readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az);
  void readGyroRaw(int16_t &gx, int16_t &gy, int16_t &gz);
  
  // 物理単位への変換
  float convertAccel(int16_t raw);
  float convertGyro(int16_t raw);
  
  // CSピンの取得
  uint8_t getCSPin() const { return _csPin; }

private:
  uint8_t _csPin;
  AccelScale _accelScale;
  GyroScale _gyroScale;
  
  // SPI通信の基本関数
  void writeByte(uint8_t reg, uint8_t value);
  uint8_t readByte(uint8_t reg);
  void readBytes(uint8_t startReg, uint8_t *buffer, uint8_t len);
};

#endif // LSM6DS3_H