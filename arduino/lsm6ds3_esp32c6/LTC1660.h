#ifndef LTC1660_H
#define LTC1660_H

#include <Arduino.h>
#include <SPI.h>

// DACチャンネルアドレス
enum DACChannel {
  DAC_A = 0b0001,
  DAC_B = 0b0010,
  DAC_C = 0b0011,
  DAC_D = 0b0100,
  DAC_E = 0b0101,
  DAC_F = 0b0110,
  DAC_G = 0b0111,
  DAC_H = 0b1000,
  DAC_ALL = 0b1111,      // 全チャンネル同時更新
  DAC_NO_CHANGE = 0b0000, // 出力変更なし（Wake用）
  DAC_SLEEP = 0b1110     // スリープモード
};

class LTC1660 {
public:
  // コンストラクタ
  LTC1660(uint8_t csPin);
  
  // 初期化
  void begin();
  
  // DACに値を書き込み（10ビット: 0-1023）
  void writeDAC(DACChannel channel, uint16_t value);
  
  // スリープモードに入る
  void sleep();
  
  // スリープモードから復帰（Wake）
  void wake();
  
  // 全DACをクリア（0Vに設定）
  void clearAll();
  
  // 電圧値から10ビットコードに変換（VREF基準）
  uint16_t voltageToCode(float voltage, float vref);
  
  // 10ビットコードから電圧値に変換
  float codeToVoltage(uint16_t code, float vref);
  
  // CSピンの取得
  uint8_t getCSPin() const { return _csPin; }

private:
  uint8_t _csPin;
  bool _isSleeping;
  
  // 16ビットワードを送信
  void sendWord(uint8_t address, uint16_t data);
  
  // アドレスとデータから16ビットワードを構成
  uint16_t buildWord(uint8_t address, uint16_t data);
};

#endif // LTC1660_H