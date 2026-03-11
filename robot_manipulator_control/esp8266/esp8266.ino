#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <SoftwareSerial.h>

const char* ssid = "bridge";
const char* password = "password";
const char* serverIP = "192.168.137.1";
const int serverPort = 5000;

const float ROBOT_BASE_X = 0.0;
const float ROBOT_BASE_Y = -120.0;

SoftwareSerial arduinoSerial(D7, D8);  // RX = D7, TX = D8

// SIMPLIFIED: Only handle ONE object at a time
struct CurrentObject {
  String name;
  float x;
  float y;
  bool active;
  bool waitingForResponse;
  unsigned long sentTime;
};

CurrentObject currentObject = {"", 0, 0, false, false, 0};
const unsigned long TIMEOUT = 7000;  // 7 seconds timeout
bool registered = false;

ESP8266WebServer server(80);

void setup() {
  Serial.begin(115200);
  arduinoSerial.begin(38400);
  delay(2000);

  Serial.println("\n================================");
  Serial.println("ESP8266");
  Serial.println("================================");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.on("/add_object", HTTP_POST, handleAddObject);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();
  Serial.println("Web server started");

  delay(2000);
  registerWithServer();

  Serial.println("================================");
  Serial.println("READY: One object at a time\n");
}

void registerWithServer() {
  WiFiClient client;
  HTTPClient http;
  String url = String("http://") + serverIP + ":" + serverPort + "/register";
  String json = "{\"device\":\"esp8266\",\"ip\":\"" + WiFi.localIP().toString() + "\"}";
  
  Serial.print("Registering with Flask... ");
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(json);
  
  if (code == 200) {
    registered = true;
    Serial.println("OK");
  } else {
    Serial.print("Failed (");
    Serial.print(code);
    Serial.println(")");
  }
  http.end();
}

void notifyPickSuccess(String name, float x, float y) {
  WiFiClient client;
  HTTPClient http;
  String url = String("http://") + serverIP + ":" + serverPort + "/pick_success";
    
  StaticJsonDocument<256> doc;
  doc["name"] = name;
  doc["x"] = x;
  doc["y"] = y;
    
  String json;
  serializeJson(doc, json);
    
  Serial.print("Notifying Flask: SUCCESS ");
  Serial.print(name);
  Serial.print(" at (");
  Serial.print(x/10.0, 1);
  Serial.print(", ");
  Serial.print(y/10.0, 1);
  Serial.println(")cm");
    
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(json);
  
  if (code == 200) {
    Serial.println("Flask acknowledged - will re-detect");
  } else {
    Serial.print("Notification failed (");
    Serial.print(code);
    Serial.println(")");
  }
  http.end();
}

void notifyPickFailed(String name, float x, float y) {
  WiFiClient client;
  HTTPClient http;
  String url = String("http://") + serverIP + ":" + serverPort + "/retry_object";
  
  StaticJsonDocument<256> doc;
  doc["name"] = name;
  doc["x"] = x;
  doc["y"] = y;
  
  String json;
  serializeJson(doc, json);
  
  Serial.print("Notifying Flask: FAILED ");
  Serial.print(name);
  Serial.print(" at (");
  Serial.print(x/10.0, 1);
  Serial.print(", ");
  Serial.print(y/10.0, 1);
  Serial.println(")cm");
  
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(json);
  
  if (code == 200) {
    Serial.println("Flask acknowledged - will re-detect");
  } else {
    Serial.print("Notification failed (");
    Serial.print(code);
    Serial.println(")");
  }
  http.end();
}

void sendToArduino() {
  if (!currentObject.active || currentObject.waitingForResponse) {
    return;
  }

  // Clear Arduino serial buffer
  while(arduinoSerial.available()) arduinoSerial.read();
  
  // Send object command
  String command = "<" + currentObject.name + "," + 
                   String(currentObject.x, 1) + "," + 
                   String(currentObject.y, 1) + ">";
  
  Serial.println("\nSENDING TO ARDUINO:");
  Serial.print("   Object: ");
  Serial.println(currentObject.name);
  Serial.print("   Position: (");
  Serial.print(currentObject.x/10.0, 1);
  Serial.print(", ");
  Serial.print(currentObject.y/10.0, 1);
  Serial.println(")cm");
  Serial.print("   Command: ");
  Serial.println(command);
  
  arduinoSerial.println(command);
  arduinoSerial.flush();

  currentObject.waitingForResponse = true;
  currentObject.sentTime = millis();
}

void clearCurrentObject() {
  currentObject.name = "";
  currentObject.x = 0;
  currentObject.y = 0;
  currentObject.active = false;
  currentObject.waitingForResponse = false;
  currentObject.sentTime = 0;
}

void loop() {
  server.handleClient();
  
  // Send to Arduino if we have an object and haven't sent yet
  if (currentObject.active && !currentObject.waitingForResponse) {
    delay(100);  // Small delay before sending
    sendToArduino();
  }
  
  // Check for Arduino response
  if (currentObject.waitingForResponse) {
    if (arduinoSerial.available()) {
      String response = arduinoSerial.readStringUntil('\n');
      response.trim();
      
      Serial.println("\n ARDUINO RESPONSE:");
      Serial.print("   Raw: ");
      Serial.println(response);
      
      if (response.startsWith("OK")) {
        Serial.println("   Status: SUCCESS");
        notifyPickSuccess(currentObject.name, currentObject.x, currentObject.y);
        clearCurrentObject();
        Serial.println("   Cleared - ready for next object\n");
      } 
      else if (response.startsWith("FAIL")) {
        Serial.println("   Status: FAILED");
        notifyPickFailed(currentObject.name, currentObject.x, currentObject.y);
        clearCurrentObject();
        Serial.println("   Cleared - ready for retry\n");
      }
    }
    
    // Timeout check
    if (millis() - currentObject.sentTime > TIMEOUT) {
      Serial.println("\n TIMEOUT - no response from Arduino");
      notifyPickFailed(currentObject.name, currentObject.x, currentObject.y);
      clearCurrentObject();
      Serial.println("   Cleared - ready for retry\n");
    }
  }

  delay(10);
}

void handleAddObject() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "No data");
    return;
  }
  
  String body = server.arg("plain");
  StaticJsonDocument<256> doc;
  
  if (deserializeJson(doc, body)) {
    Serial.println("JSON parse error");
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }

  String name = doc["name"].as<String>();
  float x = doc["x"];
  float y = doc["y"];
  
  // If we're busy, reject
  if (currentObject.active) {
    Serial.println("⚠️ Busy - rejecting new object");
    server.send(503, "text/plain", "Busy with current pick");
    return;
  }
  
  // Accept the new object
  currentObject.name = name;
  currentObject.x = x;
  currentObject.y = y;
  currentObject.active = true;
  currentObject.waitingForResponse = false;
  
  // Calculate distance for display
  float dx = x - ROBOT_BASE_X;
  float dy = y - ROBOT_BASE_Y;
  float distance = sqrt(dx*dx + dy*dy);
  
  Serial.println("\n📥 NEW OBJECT RECEIVED:");
  Serial.print("   Name: ");
  Serial.println(name);
  Serial.print("   Position: (");
  Serial.print(x/10.0, 1);
  Serial.print(", ");
  Serial.print(y/10.0, 1);
  Serial.println(")cm");
  Serial.print("   Distance: ");
  Serial.print(distance/10.0, 1);
  Serial.println("cm from base");
  
  server.send(200, "text/plain", "Object accepted");
}

void handleStatus() {
  StaticJsonDocument<512> doc;
  doc["registered"] = registered;
  doc["busy"] = currentObject.active;
  doc["waiting_response"] = currentObject.waitingForResponse;
  doc["ip"] = WiFi.localIP().toString();
  doc["timeout_ms"] = TIMEOUT;
  
  if (currentObject.active) {
    JsonObject obj = doc.createNestedObject("current_object");
    obj["name"] = currentObject.name;
    obj["x"] = currentObject.x / 10.0;
    obj["y"] = currentObject.y / 10.0;
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}