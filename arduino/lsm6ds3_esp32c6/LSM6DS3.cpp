#include "LSM6DS3.h"

// コンストラクタ
LSM6DS3::LSM6DS3(uint8_t csPin) : _csPin(csPin), _accelScale(ACC_FS_2G), _gyroScale(GYR_FS_245DPS) {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
}

// 書き込み関数: アドレスの MSB は 0 にする（書き込み）
void LSM6DS3::writeByte(uint8_t reg, uint8_t value) {
  digitalWrite(_csPin, LOW);
  SPI.transfer(reg & 0x7F);  // MSB=0 で書き込み
  SPI.transfer(value);
  digitalWrite(_csPin, HIGH);
}

// 読み取り関数: アドレスの MSB を 1 にする（読み出し）
uint8_t LSM6DS3::readByte(uint8_t reg) {
  digitalWrite(_csPin, LOW);
  SPI.transfer(reg | 0x80);  // MSB=1 で読み出し
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(_csPin, HIGH);
  return val;
}

// 連続読み出し（自動インクリメント有効）
void LSM6DS3::readBytes(uint8_t startReg, uint8_t *buffer, uint8_t len) {
  digitalWrite(_csPin, LOW);
  SPI.transfer(startReg | 0x80);
  for (uint8_t i = 0; i < len; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(_csPin, HIGH);
}

// センサ初期化：WHO_AM_I確認後，設定を書き込む
bool LSM6DS3::begin(AccelODR aOdr, AccelScale aScale, AccelBW aBw,
                    GyroODR gOdr, GyroScale gScale) {
  uint8_t whoami = readByte(LSM6DS3_REG_WHO_AM_I);
  if (whoami != 0x6A) {  // LSM6DS3 の WHO_AM_I は 0x6A のはず
    Serial.print("WHO_AM_I mismatch: ");
    Serial.println(whoami, HEX);
    return false;
  }
  
  // スケール設定を保存
  _accelScale = aScale;
  _gyroScale = gScale;
  
  // 加速度設定（ODR + FS + BW）
  uint8_t ctrl1 = (uint8_t)aOdr | (uint8_t)aScale | (uint8_t)aBw;
  writeByte(LSM6DS3_REG_CTRL1_XL, ctrl1);
  
  // ジャイロ設定（ODR + FS）
  uint8_t ctrl2 = (uint8_t)gOdr | (uint8_t)gScale;
  writeByte(LSM6DS3_REG_CTRL2_G, ctrl2);
  
  // CTRL3_C: ブロックデータ更新（BDU）とレジスタ自動インクリメント（IF_INC）を有効にする
  // 0x40 = BDU, 0x04 = IF_INC
  writeByte(LSM6DS3_REG_CTRL3_C, 0x40 | 0x04);
  
  return true;
}

// 生の加速度データ読み出し
void LSM6DS3::readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t buf[6];
  readBytes(LSM6DS3_REG_OUTX_L_XL, buf, 6);
  ax = (int16_t)((buf[1] << 8) | buf[0]);
  ay = (int16_t)((buf[3] << 8) | buf[2]);
  az = (int16_t)((buf[5] << 8) | buf[4]);
}

// 生のジャイロデータ読み出し
void LSM6DS3::readGyroRaw(int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t buf[6];
  readBytes(LSM6DS3_REG_OUTX_L_G, buf, 6);
  gx = (int16_t)((buf[1] << 8) | buf[0]);
  gy = (int16_t)((buf[3] << 8) | buf[2]);
  gz = (int16_t)((buf[5] << 8) | buf[4]);
}

// 加速度を物理単位に変換
float LSM6DS3::convertAccel(int16_t raw) {
  float sens;
  // mg/LSB (datasheet Table.3)
  switch (_accelScale) {
    case ACC_FS_2G: sens = 0.061f; break;   // ±2G
    case ACC_FS_4G: sens = 0.122f; break;   // ±4G
    case ACC_FS_8G: sens = 0.244f; break;   // ±8G
    case ACC_FS_16G: sens = 0.488f; break;  // ±16G
    default: sens = 0.061f; break;
  }
  return raw * sens * 0.001f;  // g 単位に変換
}

// ジャイロを物理単位に変換
float LSM6DS3::convertGyro(int16_t raw) {
  float sens;
  switch (_gyroScale) {
    case GYR_FS_245DPS: sens = 8.75f; break;  // mdps/LSB
    case GYR_FS_500DPS: sens = 17.50f; break;
    case GYR_FS_1000DPS: sens = 35.0f; break;
    case GYR_FS_2000DPS: sens = 70.0f; break;
    default: sens = 8.75f; break;
  }
  return raw * sens * 0.001f;  // dps 単位に変換
}