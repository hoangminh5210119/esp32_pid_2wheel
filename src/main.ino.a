// #include <OneButton.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

// #include <PID_AutoTune_v0.h>
#include <PID_v1.h>

// ================= WIFI ==========================
const char *ssid = "Mshop";
const char *password = "0354545185";
WebServer server(80);

// ================= EEPROM =========================
Preferences prefs;

// ================= L298N PINS ====================
#define IN1 26
#define IN2 25
#define IN3 33
#define IN4 32

// ================= MPU6050 ========================
const int MPU = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

float angle = 0, accAngle = 0, gyroRate = 0;
unsigned long lastTime;

// ================= PID ============================
double Input, Output, Setpoint = 0;
double Kp = 10, Ki = 0.15, Kd = 0.6;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ==================================================
//                     MOTOR CONTROL
// ==================================================
void motorRun(double pwm) {
  int sp = abs(pwm);
  if (sp > 255)
    sp = 255;

  if (pwm > 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, sp);
    analogWrite(IN3, 0);
    analogWrite(IN4, sp);
  } else if (pwm < 0) {
    analogWrite(IN1, sp);
    analogWrite(IN2, 0);
    analogWrite(IN3, sp);
    analogWrite(IN4, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
  }
}

// ==================================================
//           MPU6050 HIGH SPEED CONFIG (1kHz)
// ==================================================
void configMPU() {
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(); // DLPF 42Hz

  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(); // gyro ±250

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(); // acc ±2g
}

// ==================================================
//                     MPU READ FIXED
// ==================================================
void readMPU() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU, 14, true);

  AcX = (Wire.read() << 8) | Wire.read();
  AcY = (Wire.read() << 8) | Wire.read();
  AcZ = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read(); // skip temp

  GyX = (Wire.read() << 8) | Wire.read();
  GyY = (Wire.read() << 8) | Wire.read();
  GyZ = (Wire.read() << 8) | Wire.read();

  accAngle = atan2(AcY, AcZ) * 57.2958;
  gyroRate = GyX / 131.0;
}

// ==================================================
//                   EEPROM
// ==================================================
void savePID() {
  prefs.begin("pid", false);
  prefs.putDouble("kp", Kp);
  prefs.putDouble("ki", Ki);
  prefs.putDouble("kd", Kd);
  prefs.putDouble("set", Setpoint);
  prefs.end();
}

void loadPID() {
  prefs.begin("pid", true);
  Kp = prefs.getDouble("kp", Kp);
  Ki = prefs.getDouble("ki", Ki);
  Kd = prefs.getDouble("kd", Kd);
  Setpoint = prefs.getDouble("set", Setpoint);
  prefs.end();
}

// ==================================================
//                   PID TASK REALTIME
// ==================================================
void PIDTask(void *param) {
  const TickType_t xFreq = pdMS_TO_TICKS(5);
  TickType_t xLast = xTaskGetTickCount();

  double iTerm = 0;
  lastTime = micros();

  while (1) {
    readMPU();

    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    if (dt <= 0 || dt > 0.02)
      dt = 0.005;
    lastTime = now;

    // Complementary filter
    angle = 0.99f * (angle + gyroRate * dt) + 0.01f * accAngle;

    // PID
    double error = Setpoint - angle;

    double pOut = Kp * error;
    if (abs(angle) < 35)
      iTerm += Ki * error * dt;
    iTerm = constrain(iTerm, -180, 180);

    double dOut = -Kd * gyroRate;

    Output = pOut + iTerm + dOut;
    Output = constrain(Output, -255, 255);

    // Motor control
    if (abs(angle) > 45)
      motorRun(0);
    else {
      double pwm = Output;
      if (pwm > 0)
        pwm += 45;
      else if (pwm < 0)
        pwm -= 45;
      motorRun(pwm);
    }

    vTaskDelayUntil(&xLast, xFreq);
  }
}

// ==================================================
//                NEW BEAUTIFUL WEB UI
// ==================================================
String htmlPage() {
  String h =
      "<!DOCTYPE html><html><head>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>ESP32 BalanceBot</title>"
      "<style>"
      "body{font-family:Segoe UI,Arial;background:#f0f2f5;margin:0;padding:0}"
      ".header{background:#007bff;padding:18px;color:white;text-align:center;"
      "font-size:26px;font-weight:600;letter-spacing:1px;}"
      ".card{background:white;margin:14px;padding:20px;border-radius:14px;"
      "box-shadow:0 4px 15px rgba(0,0,0,0.12);}"
      ".title{font-size:20px;font-weight:600;margin-bottom:12px;color:#333}"
      ".slider{width:100%}"
      ".minmax{width:48%;padding:8px;font-size:16px;margin-bottom:10px;border-"
      "radius:8px;border:1px solid #ccc;}"
      "input[type=number]{padding:10px;font-size:18px;width:100%;border-radius:"
      "10px;border:1px solid #ccc;margin-bottom:12px}"
      "button{padding:14px;width:100%;background:#28a745;color:white;font-size:"
      "18px;border-radius:10px;border:none;margin-top:10px}"
      "</style>"
      "</head><body>";

  h += "<div class='header'>🤖 ESP32 BalanceBot PID</div>";

  // PID Section
  h += "<div class='card'><div class='title'>PID Tuning</div>";

  auto slider = [&](String n, double v) {
    return "<b>" + n + " = <span id='" + n + "v'>" + String(v) +
           "</span></b><br>"
           "<div style='display:flex;justify-content:space-between;'>"
           "<input id='" +
           n +
           "min' class='minmax' type='number' step='0.01' value='-20' "
           "oninput='updateRange(\"" +
           n +
           "\")'>"
           "<input id='" +
           n +
           "max' class='minmax' type='number' step='0.01' value='20'  "
           "oninput='updateRange(\"" +
           n +
           "\")'>"
           "</div>"
           "<input id='" +
           n +
           "s' type='range' class='slider' min='-20' max='20' step='0.01' "
           "value='" +
           v + "' oninput='" + n + "v.innerHTML=this.value;updatePID(\"" + n +
           "\",this.value)'><br><br>";
  };

  h += slider("kp", Kp);
  h += slider("ki", Ki);
  h += slider("kd", Kd);
  h += slider("set", Setpoint);
  h += "</div>";

  // Manual input
  h += "<div class='card'><div class='title'>Manual Input</div>"
       "<form action='/apply'>"
       "Kp:<input type='number' step='any' name='kp' value='" +
       String(Kp) +
       "'>"
       "Ki:<input type='number' step='any' name='ki' value='" +
       String(Ki) +
       "'>"
       "Kd:<input type='number' step='any' name='kd' value='" +
       String(Kd) +
       "'>"
       "Setpoint:<input type='number' step='any' name='set' value='" +
       String(Setpoint) +
       "'>"
       "<button type='submit'>APPLY</button>"
       "</form></div>";

  // Status
  h += "<div class='card'><div class='title'>Status</div>"
       "Angle: <b id='ang'>0</b><br>"
       "PWM Output: <b id='out'>0</b>"
       "</div>";

  // Script
  h += "<script>"
       "function updateRange(n){"
       " let mn=document.getElementById(n+'min').value;"
       " let mx=document.getElementById(n+'max').value;"
       " let s=document.getElementById(n+'s');"
       " s.min=mn; s.max=mx;"
       " if(s.value<mn) s.value=mn;"
       " if(s.value>mx) s.value=mx;"
       " document.getElementById(n+'v').innerHTML=s.value;"
       " updatePID(n,s.value);"
       "}"

       "function updatePID(k,v){ fetch('/pid?'+k+'='+v); }"

       "setInterval(()=>{fetch('/status').then(r=>r.json()).then(d=>{"
       " ang.innerHTML=d.angle.toFixed(2);"
       " out.innerHTML=d.out.toFixed(0);"
       "})},100);"
       "</script>";

  h += "</body></html>";
  return h;
}

// ==================================================
//                   API HANDLERS
// ==================================================
void handlePID() {
  if (server.hasArg("kp"))
    Kp = server.arg("kp").toDouble();
  if (server.hasArg("ki"))
    Ki = server.arg("ki").toDouble();
  if (server.hasArg("kd"))
    Kd = server.arg("kd").toDouble();
  if (server.hasArg("set"))
    Setpoint = server.arg("set").toDouble();
  savePID();
  server.send(200, "text/plain", "OK");
}

void handleApply() {
  Kp = server.arg("kp").toDouble();
  Ki = server.arg("ki").toDouble();
  Kd = server.arg("kd").toDouble();
  Setpoint = server.arg("set").toDouble();
  savePID();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStatus() {
  String j = "{";
  j += "\"angle\":" + String(angle, 2) + ",";
  j += "\"out\":" + String(Output, 0);
  j += "}";
  server.send(200, "application/json", j);
}

// ==================================================
//                      SETUP
// ==================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22, 400000);

  configMPU();
  loadPID();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
  }
  Serial.println("\nConnected!");
  Serial.println(WiFi.localIP());

  server.on("/", []() { server.send(200, "text/html", htmlPage()); });
  server.on("/pid", handlePID);
  server.on("/apply", handleApply);
  server.on("/status", handleStatus);
  server.begin();

  xTaskCreatePinnedToCore(PIDTask, "PID", 12000, NULL, 20, NULL, 1);
}

// ==================================================
void loop() { server.handleClient(); }
