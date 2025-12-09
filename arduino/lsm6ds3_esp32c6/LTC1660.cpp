#include "LTC1660.h"

// コンストラクタ
LTC1660::LTC1660(uint8_t csPin) : _csPin(csPin), _isSleeping(false) {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
}

// 初期化
void LTC1660::begin() {
  digitalWrite(_csPin, HIGH);
  _isSleeping = false;
  // パワーオンリセットにより、出力は0スケールに初期化される
}

// 16ビットワードを構成
// フォーマット: [A3 A2 A1 A0 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0 X1 X0]
// A3-A0: アドレス/コントロール (4ビット)
// D9-D0: データ (10ビット)
// X1-X0: Don't care (2ビット)
uint16_t LTC1660::buildWord(uint8_t address, uint16_t data) {
  // 10ビットデータに制限
  data &= 0x3FF;
  
  // 16ビットワードを構成
  // [A3 A2 A1 A0][D9 D8 D7 D6 D5 D4 D3 D2 D1 D0][X1 X0]
  uint16_t word = 0;
  word |= ((uint16_t)address & 0x0F) << 12;  // アドレス: ビット15-12
  word |= (data & 0x3FF) << 2;               // データ: ビット11-2
  // ビット1-0はDon't care (0のまま)
  
  return word;
}

// 16ビットワードを送信
void LTC1660::sendWord(uint8_t address, uint16_t data) {
  uint16_t word = buildWord(address, data);
  
  // CS/LDをLOWにして転送開始
  digitalWrite(_csPin, LOW);
  
  // 16ビットを送信 (MSBファースト)
  SPI.transfer16(word);
  
  // CS/LDをHIGHにしてDACレジスタを更新
  digitalWrite(_csPin, HIGH);
}

// DACに値を書き込み
void LTC1660::writeDAC(DACChannel channel, uint16_t value) {
  // 値を10ビットに制限
  if (value > 1023) {
    value = 1023;
  }
  
  sendWord((uint8_t)channel, value);
  
  // スリープモード以外のコマンドでWake
  if (channel != DAC_SLEEP) {
    _isSleeping = false;
  }
}

// スリープモードに入る
void LTC1660::sleep() {
  sendWord((uint8_t)DAC_SLEEP, 0);
  _isSleeping = true;
}

// スリープモードから復帰
void LTC1660::wake() {
  if (_isSleeping) {
    // NO_CHANGEアドレスでWake（出力は変更されない）
    sendWord((uint8_t)DAC_NO_CHANGE, 0);
    _isSleeping = false;
  }
}

// 全DACをクリア
void LTC1660::clearAll() {
  // 全チャンネルを0に設定
  writeDAC(DAC_ALL, 0);
}

// 電圧値から10ビットコードに変換
// VOUT = (code / 1024) * VREF
// code = (VOUT / VREF) * 1024
uint16_t LTC1660::voltageToCode(float voltage, float vref) {
  if (vref <= 0) {
    return 0;
  }
  
  float code_float = (voltage / vref) * 1024.0f;
  
  // 範囲制限
  if (code_float < 0) {
    return 0;
  } else if (code_float > 1023) {
    return 1023;
  }
  
  return (uint16_t)(code_float + 0.5f);  // 四捨五入
}

// 10ビットコードから電圧値に変換
float LTC1660::codeToVoltage(uint16_t code, float vref) {
  // コードを10ビットに制限
  code &= 0x3FF;
  
  // VOUT = (code / 1024) * VREF
  return ((float)code / 1024.0f) * vref;
}