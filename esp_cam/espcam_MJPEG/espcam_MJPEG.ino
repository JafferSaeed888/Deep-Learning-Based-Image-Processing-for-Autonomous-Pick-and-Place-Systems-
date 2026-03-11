#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>

const char* ssid = "";
const char* password = "";

const char* serverIP = "";
const int serverPort = 5000;

WebServer server(80);
bool streaming = false;
bool registered = false;
unsigned long lastRegisterAttempt = 0;
unsigned long lastWiFiCheck = 0;
const unsigned long REGISTER_RETRY_INTERVAL = 5000;  // 5 seconds
const unsigned long WIFI_CHECK_INTERVAL = 10000;     // 10 seconds
const unsigned long WIFI_RECONNECT_TIMEOUT = 30000;  // 30 seconds

// 💡 LED Flash Pin (GPIO 4 on ESP32-CAM)
#define LED_PIN 4

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void setupLED() {
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(LED_PIN, 5000, 8);
  #else
    ledcSetup(7, 5000, 8);
    ledcAttachPin(LED_PIN, 7);
  #endif
  
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(LED_PIN, 0);
  #else
    ledcWrite(7, 0);
  #endif
  
  Serial.println("💡 LED initialized (OFF)");
}

void setLED(bool on) {
  int pwm = on ? 204 : 0;
  
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(LED_PIN, pwm);
  #else
    ledcWrite(7, pwm);
  #endif
  
  Serial.printf("💡 LED: %s\n", on ? "ON (80%)" : "OFF");
}

void configCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);
  s->set_special_effect(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 0);
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 0);
  s->set_gainceiling(s, (gainceiling_t)0);
  s->set_bpc(s, 0);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_lenc(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  s->set_dcw(s, 1);
  s->set_colorbar(s, 0);
}

// ========== NEW: WiFi Connection/Reconnection ==========
bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  
  Serial.println("\n🔌 Connecting to WiFi...");
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  
  unsigned long startAttempt = millis();
  
  while (WiFi.status() != WL_CONNECTED && 
         millis() - startAttempt < WIFI_RECONNECT_TIMEOUT) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("✅ WiFi Connected! IP: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println();
    Serial.println("❌ WiFi connection failed!");
    return false;
  }
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi disconnected! Attempting reconnection...");
    
    // Stop streaming if active
    if (streaming) {
      streaming = false;
      setLED(false);
      Serial.println("⏹️ Streaming stopped due to WiFi loss");
    }
    
    // Mark as unregistered since we lost connection
    registered = false;
    
    // Try to reconnect
    if (connectWiFi()) {
      // Successfully reconnected, try to register again
      Serial.println("🔄 WiFi restored, attempting server registration...");
      registered = registerWithServer();
    }
  }
}
// ======================================================

bool registerWithServer() {
  // Can't register without WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ Cannot register - WiFi not connected");
    return false;
  }
  
  HTTPClient http;
  String myIP = WiFi.localIP().toString();
  
  String url = String("http://") + serverIP + ":" + serverPort + "/register";
  String jsonData = "{\"device\":\"esp32cam\",\"ip\":\"" + myIP + "\"}";
  
  Serial.printf("⏳ Attempting registration with %s:%d...\n", serverIP, serverPort);
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(3000);
  
  int httpCode = http.POST(jsonData);
  
  if (httpCode == 200) {
    Serial.println("✅ Registered with server!");
    http.end();
    return true;
  } else if (httpCode > 0) {
    Serial.printf("⚠️ Registration failed: HTTP %d\n", httpCode);
  } else {
    Serial.printf("⚠️ Server not reachable: %s\n", http.errorToString(httpCode).c_str());
  }
  
  http.end();
  return false;
}

// ========== NEW: Periodic Health Check for Server ==========
void checkServerConnection() {
  if (!registered) {
    return; // Already trying to register in main loop
  }
  
  // Periodically ping server to verify connection
  HTTPClient http;
  String url = String("http://") + serverIP + ":" + serverPort + "/";
  
  http.begin(url);
  http.setTimeout(2000);
  
  int httpCode = http.GET();
  http.end();
  
  if (httpCode <= 0) {
    Serial.println("⚠️ Server connection lost! Will attempt re-registration...");
    registered = false;
    
    // Stop streaming if server is down
    if (streaming) {
      streaming = false;
      setLED(false);
      Serial.println("⏹️ Streaming stopped due to server loss");
    }
  }
}
// ===========================================================

void handleStream() {
  if (!streaming) {
    server.send(503, "text/plain", "Not streaming");
    return;
  }
  
  WiFiClient client = server.client();
  
  // Send MJPEG HTTP headers
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Access-Control-Allow-Origin: *");
  client.println();
  
  Serial.println(" MJPEG stream started");
  
  while (streaming && client.connected() && WiFi.status() == WL_CONNECTED) {
    camera_fb_t * fb = esp_camera_fb_get();
    
    if (!fb) {
      Serial.println("⚠️ Frame capture failed");
      delay(10);
      continue;
    }
    
    // Send MJPEG frame boundary
    client.println("--frame");
    client.println("Content-Type: image/jpeg");
    client.printf("Content-Length: %u\r\n\r\n", fb->len);
    
    // Send JPEG data
    client.write(fb->buf, fb->len);
    client.println();
    
    esp_camera_fb_return(fb);
    
    // Control frame rate (~30 FPS)
    delay(33);
  }
  
  // If WiFi disconnected during stream
  if (WiFi.status() != WL_CONNECTED) {
    streaming = false;
    setLED(false);
    Serial.println("📹 MJPEG stream ended - WiFi disconnected");
  } else {
    Serial.println("📹 MJPEG stream ended");
  }
}

void handleCapture() {
  if (!streaming) {
    server.send(503, "text/plain", "Not streaming");
    return;
  }
  
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }
  
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  
  esp_camera_fb_return(fb);
}

void handleStart() {
  if (WiFi.status() != WL_CONNECTED) {
    server.send(503, "text/plain", "WiFi not connected");
    return;
  }
  
  streaming = true;
  setLED(true);
  Serial.println("▶️ Streaming started");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "Streaming started");
}

void handleStop() {
  streaming = false;
  setLED(false);
  Serial.println("⏹️ Streaming stopped");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "Streaming stopped");
}

void handleRoot() {
  String wifiStatus = WiFi.status() == WL_CONNECTED ? 
                      "✅ Connected (" + WiFi.localIP().toString() + ")" : 
                      "❌ Disconnected";
  
  String html = "<html><body><h1>ESP32-CAM</h1>";
  html += "<p>WiFi: " + wifiStatus + "</p>";
  html += "<p>Server: " + String(registered ? "✅ Registered" : "⏳ Waiting...") + "</p>";
  html += "<p>Stream: " + String(streaming ? "🟢 Active (LED ON)" : "🔴 Stopped (LED OFF)") + "</p>";
  html += "<p><a href='/start'>Start</a> | <a href='/stop'>Stop</a></p>";
  html += "<p><a href='/stream'>MJPEG Stream</a> | <a href='/capture'>Single Capture</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  setupLED();
  
  // Initial WiFi connection
  connectWiFi();
  
  configCamera();
  Serial.println("Camera initialized");
  
  server.on("/", handleRoot);
  server.on("/capture", handleCapture);
  server.on("/stream", handleStream);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  
  server.begin();
  Serial.println("HTTP server started");
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("📹 MJPEG available at: http://" + WiFi.localIP().toString() + "/stream");
    
    Serial.println("\n🔄 Starting auto-registration...");
    registered = registerWithServer();
    lastRegisterAttempt = millis();
    
    if (!registered) {
      Serial.println("⚠️ Server not available yet - will keep trying every 5 seconds");
    }
  }
  
  lastWiFiCheck = millis();
  Serial.println("✅ Ready!");
}

void loop() {
  server.handleClient();
  
  unsigned long currentMillis = millis();
  
  // Check WiFi connection periodically
  if (currentMillis - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    checkWiFiConnection();
    lastWiFiCheck = currentMillis;
  }
  
  // Try to register with server if not registered
  if (!registered && WiFi.status() == WL_CONNECTED) {
    if (currentMillis - lastRegisterAttempt >= REGISTER_RETRY_INTERVAL) {
      registered = registerWithServer();
      lastRegisterAttempt = currentMillis;
    }
  }
  
  // Periodically check server health (every 30 seconds when registered)
  static unsigned long lastServerCheck = 0;
  if (registered && currentMillis - lastServerCheck >= 5000) {
    checkServerConnection();
    lastServerCheck = currentMillis;
  }
}