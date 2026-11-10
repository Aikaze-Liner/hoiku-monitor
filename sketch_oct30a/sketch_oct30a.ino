#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h> // ← 追加: HTTPS通信を確実にするため

// ==========================================
// Wi-FiとGASの設定
// ==========================================
const char* ssid = "Testify-AP.E";
const char* password = "11451419";

// 見守りアプリ共通のGAS URL
String gasUrl = "https://script.google.com/macros/s/AKfycby2zjdxpoYufVbzLhNkcAVZFqA1bOsAOFAVe4wZSpjsOlrcPqFMyc_MqlH2RoYxQnIe/exec";

// ==========================================
// ピン設定
// ==========================================
// 光センサーモジュールのDO（デジタル出力）ピンを繋ぐGPIO
const int LDR_PIN = 4; 
int lastState = -1;

void setup() {
  Serial.begin(115200);
  pinMode(LDR_PIN, INPUT);
  
  // Wi-Fi接続開始 (確実に接続させるためのリセット処理を追加)
  WiFi.mode(WIFI_STA); // モードを子機に固定
  WiFi.disconnect(true); // 過去のバグった接続情報を一度消去
  delay(100);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected! (Light Sensor Unit)");
}

// GASへ状態を送信する関数
void sendToGAS(String state) {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClientSecure client;
    client.setInsecure(); // ← 追加: Googleの証明書エラーを無視して確実に届ける

    HTTPClient http;
    String url = gasUrl + "?state=" + state;
    http.begin(client, url); // ← 修正: セキュアクライアントを使うように変更
    
    // GAS特有のリダイレクト(302)を自動で追従する設定
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS); 
    
    int httpCode = http.GET();
    Serial.printf("Sent %s -> HTTP Code: %d\n", state.c_str(), httpCode);
    http.end();
  } else {
    // もし途中でWi-Fiが切れたら自動で再接続を試みる
    Serial.println("WiFi connection lost. Reconnecting...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
  }
}

void loop() {
  // センサーの現在の状態を読み取る (光を検知=LOW(0), 暗い=HIGH(1))
  int currentState = digitalRead(LDR_PIN);
  
  // 状態が変化した時だけ送信する
  if (currentState != lastState) {
    if (currentState == LOW) {
      Serial.println("Sensor: LOW -> LIGHT_ON (明るい)");
      sendToGAS("LIGHT_ON");
    } else {
      Serial.println("Sensor: HIGH -> LIGHT_OFF (暗い)");
      sendToGAS("LIGHT_OFF");
    }
    lastState = currentState;
  }
  
  // 100ミリ秒ごとに監視
  delay(100);
}