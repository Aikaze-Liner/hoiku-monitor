#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <math.h>
#include <time.h>
#include "trusted_roots.h"
#include "SensorTypes.h"
#include "sdkconfig.h"
#include "driver/dac_continuous.h"
#include "DoorbellPcm.h"

// =================================================================
// 1. 各種設定（Wi-Fi・GASのURL）
// =================================================================
// Wi-Fi接続情報
const char *WIFI_SSID = "Testify-AP.E";
const char *WIFI_PASSWORD = "11451419";

// 友人の元システム用設定（そのまま残す）
const char *GAS_URL = "https://script.google.com/macros/s/YOUR_DEPLOYMENT_ID/exec";
const char *INGEST_KEY = "YOUR_INGEST_KEY";
const char *DEVICE_ID = "ESP32-02-PIR"; 

// ★追加：土田さんの見守りアプリ用GAS URL（即時通知用）
const char *WEBAPP_GAS_URL = "https://script.google.com/macros/s/AKfycby2zjdxpoYufVbzLhNkcAVZFqA1bOsAOFAVe4wZSpjsOlrcPqFMyc_MqlH2RoYxQnIe/exec";

// =================================================================
// ピン配置・定数設定
// =================================================================
#ifndef ENABLE_SOUND
#define ENABLE_SOUND 1
#endif
constexpr int PIR_PIN = 33, MIC_PIN = 34, LED_PIN = 23, DAC_PIN = 25;
constexpr int MUSIC_AMPLITUDE = 12;  
constexpr uint32_t WARMUP_MS = 30000, FILTER_MS = 50, REARM_MS = 1000;
constexpr uint32_t MIC_REPORT_MS = 60000, UPLOAD_MS = 60000, RETRY_MS = 15000;
constexpr size_t QUEUE_CAPACITY = 128, BATCH_SIZE = 20;

const char *kindNames[] = {"boot", "pir", "motion", "mic", "warmup", "lost"};
QueueHandle_t logQueue, beepQueue;
char bootId[33];
uint32_t sequence = 0, dropped = 0, triggerCount = 0;
uint32_t bootAt, rawChangedAt, stableChangedAt, micReportAt, noisePrintedAt = 0;
uint32_t micSamples = 0, micClipped = 0, micSpeakerWindows = 0;
int rawLast = LOW, stableState = LOW, micMaxP2P = 0;
bool warmedUp = false, armed = false;
volatile bool speakerActive = false;
double micEnergy = 0, micMeanSum = 0;
float micPeak = 0, latestNoiseRms = 0;
int latestNoiseP2P = 0;

// 独自型を使う関数宣言
Record stamp(Record r);
void publish(Record r);
void publishKind(Kind kind);
String recordJson(const Record &r);
#if ENABLE_SOUND
bool writeAudioBlock(uint8_t *data, size_t count);
bool playMelody();
#endif

// =================================================================
// ★追加：見守りアプリへ即時ステータスを送る関数
// =================================================================
void sendToWebAppGAS(String pirState) {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClientSecure client;
    client.setInsecure(); // 即時通知のため証明書検証をスキップ
    HTTPClient http;
    String url = String(WEBAPP_GAS_URL) + "?pir=" + pirState;
    http.begin(client, url);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = http.GET();
    Serial.printf("WebApp Notified: %s -> HTTP Code: %d\n", pirState.c_str(), httpCode);
    http.end();
  }
}

// =================================================================
// ユーティリティ・通信処理
// =================================================================
String newUuid() {
  uint8_t b[16];
  esp_fill_random(b, sizeof(b));
  b[6] = (b[6] & 15) | 0x40;
  b[8] = (b[8] & 63) | 0x80;
  char out[37];
  snprintf(out, sizeof(out),
      "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
      b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
      b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
  return String(out);
}

String quoteJson(const String &s) {
  String out = "\"";
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else if ((uint8_t)c >= 32) out += c;
  }
  return out + "\"";
}

Record stamp(Record r) {
  r.seq = ++sequence;
  r.uptime = millis();
  time_t t = time(nullptr);
  r.epoch = t > 1700000000 ? (uint32_t)t : 0;
  r.count = triggerCount;
  r.pir = stableState;
  return r;
}

void publish(Record r) {
  if (dropped) {
    Record warning = {};
    warning.kind = LOST;
    warning.lost = dropped;
    warning = stamp(warning);
    if (xQueueSend(logQueue, &warning, 0) == pdTRUE) dropped = 0;
  }
  r = stamp(r);
  Serial.printf("seq=%lu kind=%s PIR=%d count=%lu RMS=%.1f peak=%.1f p2p=%ld clip=%.2f%%\n",
      (unsigned long)r.seq, kindNames[r.kind], r.pir, (unsigned long)r.count,
      r.rms, r.peak, (long)r.p2p, r.clipped);
  if (xQueueSend(logQueue, &r, 0) != pdTRUE) {
    ++dropped;
    Serial.println("QUEUE FULL: record not queued; lost count will be reported.");
  }
}

void publishKind(Kind kind) { Record r = {}; r.kind = kind; publish(r); }

String recordJson(const Record &r) {
  char text[680];
  snprintf(text, sizeof(text),
      "{\"id\":\"%s-%lu\",\"kind\":\"%s\",\"device_epoch_s\":%lu,"
      "\"uptime_ms\":%lu,\"pir\":%d,\"count\":%lu,\"rms\":%.2f,\"peak\":%.2f,"
      "\"p2p\":%ld,\"bias\":%.2f,\"clipped_pct\":%.3f,\"period_ms\":%lu,"
      "\"samples\":%lu,\"speaker_windows\":%lu,\"lost\":%lu}",
      bootId, (unsigned long)r.seq, kindNames[r.kind], (unsigned long)r.epoch,
      (unsigned long)r.uptime, r.pir, (unsigned long)r.count, r.rms, r.peak,
      (long)r.p2p, r.bias, r.clipped, (unsigned long)r.period,
      (unsigned long)r.samples, (unsigned long)r.speakerWindows, (unsigned long)r.lost);
  return String(text);
}

bool uploadBatch(const String &body, const String &batchId) {
  WiFiClientSecure client;
  client.setCACert(TRUSTED_ROOTS);
  client.setHandshakeTimeout(10);
  HTTPClient http;
  http.setConnectTimeout(10000);
  http.setTimeout(20000);
  http.setReuse(false);
  http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  if (!http.begin(client, GAS_URL)) return false;
  const char *headers[] = {"Location"};
  http.collectHeaders(headers, 1);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  String result;
  if (code == 302 || code == 303) {
    String target = http.header("Location");
    http.end();
    if (!target.startsWith("https://script.googleusercontent.com/")) {
      Serial.println("GAS redirect rejected. Check public /exec deployment.");
      return false;
    }
    if (!http.begin(client, target)) return false;
    code = http.GET();
  }
  if (code == 200) result = http.getString();
  http.end();
  String expected = "\"batch_id\":" + quoteJson(batchId);
  bool ok = code == 200 && result.startsWith("{\"ok\":true,") &&
            result.indexOf(expected) >= 0;
  Serial.printf("GAS HTTP=%d stored_ack=%s\n", code, ok ? "yes" : "no");
  if (!ok) Serial.println("Upload not acknowledged; keeping batch for retry.");
  return ok;
}

void networkTask(void *) {
  // ★改修：見守りアプリ用のURLが設定されていればWi-Fi接続を許可する
  if (String(WIFI_SSID).startsWith("YOUR_") || String(WEBAPP_GAS_URL).indexOf("YOUR_") >= 0) {
    Serial.println("Error: Please set Wi-Fi and WebApp GAS URL properly.");
    vTaskDelete(nullptr); return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  configTime(9 * 3600, 0, "pool.ntp.org", "time.google.com");
  uint32_t lastUpload = millis(), lastTry = 0, lastConnect = millis(), lastClockLog = 0;
  bool attempted = false;
  String pending, batchId;
  for (;;) {
    uint32_t now = millis();
    if (WiFi.status() != WL_CONNECTED) {
      if ((uint32_t)(now - lastConnect) >= 30000) {
        WiFi.reconnect(); lastConnect = now;
        Serial.println("WiFi reconnecting; sensors continue.");
      }
      vTaskDelay(pdMS_TO_TICKS(200)); continue;
    }
    if (time(nullptr) < 1700000000) {
      if ((uint32_t)(now - lastClockLog) >= 30000) {
        Serial.println("Waiting for NTP; TLS upload paused, sensors continue.");
        lastClockLog = now;
      }
      vTaskDelay(pdMS_TO_TICKS(200)); continue;
    }
    
    // 友人の元のLINE通知システム部分（設定されていなければスキップしてキューを捨てる）
    if (String(GAS_URL).indexOf("YOUR_") < 0) {
      if (pending.isEmpty() && (uint32_t)(now - lastUpload) >= UPLOAD_MS) {
        Record r;
        if (xQueueReceive(logQueue, &r, 0) == pdTRUE) {
          batchId = newUuid();
          pending = "{\"key\":" + quoteJson(INGEST_KEY) +
            ",\"batch_id\":" + quoteJson(batchId) + ",\"device_id\":" + quoteJson(DEVICE_ID) +
            ",\"boot_id\":" + quoteJson(bootId) + ",\"events\":[" + recordJson(r);
          for (size_t i = 1; i < BATCH_SIZE && xQueueReceive(logQueue, &r, 0) == pdTRUE; ++i)
            pending += "," + recordJson(r);
          pending += "]}";
          attempted = false;
        } else lastUpload = now;
      }
      if (!pending.isEmpty() && (!attempted || (uint32_t)(now - lastTry) >= RETRY_MS)) {
        lastTry = now; attempted = true;
        if (uploadBatch(pending, batchId)) {
          pending = "";
          lastUpload = millis();
        }
      }
    } else {
      Record dummy;
      while(xQueueReceive(logQueue, &dummy, 0) == pdTRUE) {}
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// =================================================================
// 音声・センサー処理
// =================================================================
#if ENABLE_SOUND
static dac_continuous_handle_t audioHandle = nullptr;
static_assert(MUSIC_AMPLITUDE >= 0 && MUSIC_AMPLITUDE <= 12,
              "Keep MUSIC_AMPLITUDE between 0 and 12.");

bool writeAudioBlock(uint8_t *data, size_t count) {
#if CONFIG_DAC_DMA_AUTO_16BIT_ALIGN
  return dac_continuous_write(audioHandle, data, count, nullptr, 1000) == ESP_OK;
#else
  uint16_t aligned[256];
  if (count > 256) return false;
  for (size_t i = 0; i < count; ++i) aligned[i] = (uint16_t)data[i] << 8;
  return dac_continuous_write(audioHandle, (uint8_t *)aligned,
                              count * sizeof(uint16_t), nullptr, 1000) == ESP_OK;
#endif
}

bool playMelody() {
  speakerActive = true;
  uint8_t block[256];
  for (size_t offset = 0; offset < DOORBELL_PCM_SIZE; offset += sizeof(block)) {
    size_t count = DOORBELL_PCM_SIZE - offset;
    if (count > sizeof(block)) count = sizeof(block);
    for (size_t i = 0; i < count; ++i) {
      int centered = (int)DOORBELL_PCM[offset + i] - 128;
      block[i] = 128 + centered * MUSIC_AMPLITUDE / DOORBELL_BASE_AMPLITUDE;
    }
    if (!writeAudioBlock(block, count)) {
      speakerActive = false;
      return false;
    }
  }
  vTaskDelay(pdMS_TO_TICKS(200));
  speakerActive = false;
  return true;
}

void soundTask(void *) {
  dac_continuous_config_t config = {};
  config.chan_mask = DAC_CHANNEL_MASK_CH0;  // GPIO25
  config.desc_num = 6;
  config.buf_size = 1024;
  config.freq_hz = DOORBELL_SAMPLE_RATE;
  config.offset = 0;
  config.clk_src = DAC_DIGI_CLK_SRC_DEFAULT;
  config.chan_mode = DAC_CHANNEL_MODE_SIMUL;
  esp_err_t error = dac_continuous_new_channels(&config, &audioHandle);
  if (error == ESP_OK) error = dac_continuous_enable(audioHandle);
  if (error != ESP_OK) {
    Serial.printf("AUDIO DMA INIT ERROR=%d; sensors continue.\n", (int)error);
    if (audioHandle) dac_continuous_del_channels(audioHandle);
    audioHandle = nullptr;
    vTaskDelete(nullptr);
    return;
  }
  uint8_t silence[256];
  memset(silence, 128, sizeof(silence));
  if (!writeAudioBlock(silence, sizeof(silence))) {
    dac_continuous_disable(audioHandle);
    dac_continuous_del_channels(audioHandle);
    audioHandle = nullptr;
    Serial.println("AUDIO DMA START ERROR; sensors continue.");
    vTaskDelete(nullptr);
    return;
  }
  Serial.println("Audio ready: 24kHz DAC DMA / GPIO25.");
  uint8_t event;
  for (;;) {
    xQueueReceive(beepQueue, &event, portMAX_DELAY);
    if (!playMelody()) {
      vTaskDelay(pdMS_TO_TICKS(200));
      dac_continuous_disable(audioHandle);
      dac_continuous_del_channels(audioHandle);
      audioHandle = nullptr;
      Serial.println("AUDIO DMA WRITE ERROR; sensors continue.");
      vTaskDelete(nullptr);
      return;
    }
  }
}
#endif

void servicePir() {
  uint32_t now = millis();
  int raw = digitalRead(PIR_PIN);
  if (raw != rawLast) { rawLast = raw; rawChangedAt = now; }
  if (raw != stableState && (uint32_t)(now - rawChangedAt) >= FILTER_MS) {
    stableState = raw; stableChangedAt = now; publishKind(PIR);
    
    // ★追加：誰もいなくなって安定したらアプリへ「安全(OFF)」を通知
    if (stableState == LOW) {
      sendToWebAppGAS("DANGER_OFF");
    }
  }
  if (!warmedUp && (uint32_t)(now - bootAt) >= WARMUP_MS) {
    warmedUp = true; publishKind(WARMUP);
  }
  if (warmedUp) {
    if (!armed && stableState == LOW &&
        (uint32_t)(now - stableChangedAt) >= REARM_MS) armed = true;
    if (armed && stableState == HIGH) {
      armed = false; ++triggerCount; publishKind(MOTION);
      
      // ★追加：人を検知した瞬間にアプリへ「危険(ON)」を即時通知
      sendToWebAppGAS("DANGER_ON");
      
#if ENABLE_SOUND
      uint8_t event = 1;
      if (xQueueSend(beepQueue, &event, 0) != pdTRUE)
        Serial.println("MUSIC queue full; motion was still recorded.");
#endif
    }
  }
  digitalWrite(LED_PIN, warmedUp && stableState == HIGH ? HIGH : LOW);
}

void sampleMic() {
  uint32_t start = micros(), count = 0;
  double sum = 0, squares = 0;
  int lo = 4095, hi = 0;
  bool overlap = speakerActive;
  while ((uint32_t)(micros() - start) < 25000) {
    int value = analogRead(MIC_PIN);
    sum += value; squares += (double)value * value; ++count;
    if (value < lo) lo = value;
    if (value > hi) hi = value;
    if (value <= 5 || value >= 4090) ++micClipped;
    overlap = overlap || speakerActive;
    delayMicroseconds(200);
  }
  if (!count) return;
  double mean = sum / count, variance = squares / count - mean * mean;
  if (variance < 0) variance = 0;
  latestNoiseRms = sqrt(variance);
  latestNoiseP2P = hi - lo;
  micEnergy += variance * count; micMeanSum += sum; micSamples += count;
  if (hi - lo > micMaxP2P) micMaxP2P = hi - lo;
  if (sqrt(variance) > micPeak) micPeak = sqrt(variance);
  if (overlap) ++micSpeakerWindows;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT); pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
#if !ENABLE_SOUND
  pinMode(DAC_PIN, OUTPUT);
  digitalWrite(DAC_PIN, LOW);
#endif
  analogReadResolution(12); analogSetPinAttenuation(MIC_PIN, ADC_11db);
  logQueue = xQueueCreate(QUEUE_CAPACITY, sizeof(Record));
  beepQueue = xQueueCreate(1, sizeof(uint8_t));
  if (!logQueue || !beepQueue) {
    Serial.println("ERROR: queue allocation failed.");
    while (true) delay(1000);
  }
  uint8_t randomBytes[16]; esp_fill_random(randomBytes, sizeof(randomBytes));
  for (int i = 0; i < 16; ++i) snprintf(bootId + 2 * i, 3, "%02x", randomBytes[i]);
  for (int i = 0; i < 3; ++i) {
    digitalWrite(LED_PIN, HIGH); delay(200); digitalWrite(LED_PIN, LOW); delay(200);
  }
  bootAt = rawChangedAt = stableChangedAt = micReportAt = millis();
  rawLast = stableState = digitalRead(PIR_PIN);
  publishKind(BOOT);
  if (xTaskCreate(networkTask, "gas", 12288, nullptr, 1, nullptr) != pdPASS)
    Serial.println("ERROR: network task could not start.");
#if ENABLE_SOUND
  if (xTaskCreatePinnedToCore(soundTask, "pam8012", 4096, nullptr, 2, nullptr, 0) != pdPASS)
    Serial.println("ERROR: sound task could not start.");
#endif
}

void loop() {
  servicePir(); sampleMic();
  uint32_t now = millis();
  if ((uint32_t)(now - noisePrintedAt) >= 1000) {
    noisePrintedAt = now;
    Serial.printf("NOISE_WINDOW rms=%.1f p2p=%d speaker=%d (ADC units)\n",
        latestNoiseRms, latestNoiseP2P, speakerActive ? 1 : 0);
  }
  if ((uint32_t)(now - micReportAt) >= MIC_REPORT_MS && micSamples) {
    Record r = {}; r.kind = MIC;
    r.rms = sqrt(micEnergy / micSamples); r.peak = micPeak; r.p2p = micMaxP2P;
    r.bias = micMeanSum / micSamples; r.clipped = 100.0 * micClipped / micSamples;
    r.period = now - micReportAt; r.samples = micSamples; r.speakerWindows = micSpeakerWindows;
    publish(r);
    micReportAt = now; micEnergy = micMeanSum = 0;
    micSamples = micClipped = micSpeakerWindows = 0; micPeak = 0; micMaxP2P = 0;
  }
  delay(1);
}