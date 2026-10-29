#pragma once
#include <stdint.h>

// Arduinoの自動関数宣言より前に型を定義
// センサー記録
enum Kind : uint8_t { BOOT, PIR, MOTION, MIC, WARMUP, LOST };
struct Record {
  uint32_t seq, uptime, epoch, count, period, samples, speakerWindows, lost;
  int32_t pir, p2p;
  float rms, peak, bias, clipped;
  Kind kind;
};

// メロディーの音符
struct Note {
  float hz;
  uint16_t durationMs;
  uint16_t gapMs;
};
