#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <PID_v1.h>

// ================= WIFI ==========================
const char *ssid = "ESP32_Balance_Bot";
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

// ================= MOVEMENT CONTROL ===============
double speedBias = 0;      // Tốc độ tới/lui (-100 đến 100)
double turnBias = 0;       // Xoay trái/phải (-100 đến 100)
double baseSetpoint = 0;   // Setpoint gốc

// ==================================================
//                     MOTOR CONTROL
// ==================================================
void motorRun(double leftPWM, double rightPWM) {
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);
  
  int leftSpeed = abs(leftPWM);
  int rightSpeed = abs(rightPWM);
  
  // Motor trái (IN1, IN2)
  if (leftPWM > 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, leftSpeed);
  } else if (leftPWM < 0) {
    analogWrite(IN1, leftSpeed);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
  }
  
  // Motor phải (IN3, IN4)
  if (rightPWM > 0) {
    analogWrite(IN3, 0);
    analogWrite(IN4, rightSpeed);
  } else if (rightPWM < 0) {
    analogWrite(IN3, rightSpeed);
    analogWrite(IN4, 0);
  } else {
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
  }
}

// ==================================================
//           MPU6050 CONFIG
// ==================================================
void configMPU() {
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
}

// ==================================================
//                     MPU READ
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
  Wire.read();

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
  prefs.putDouble("set", baseSetpoint);
  prefs.end();
}

void loadPID() {
  prefs.begin("pid", true);
  Kp = prefs.getDouble("kp", Kp);
  Ki = prefs.getDouble("ki", Ki);
  Kd = prefs.getDouble("kd", Kd);
  baseSetpoint = prefs.getDouble("set", baseSetpoint);
  Setpoint = baseSetpoint;
  prefs.end();
}

// ==================================================
//                   PID TASK
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

    angle = 0.99f * (angle + gyroRate * dt) + 0.01f * accAngle;

    // Điều chỉnh setpoint để di chuyển tới/lui
    // speedBias > 0: đi tới (nghiêng về trước = setpoint âm)
    // speedBias < 0: đi lui (nghiêng về sau = setpoint dương)
    double dynamicSetpoint = baseSetpoint - (speedBias * 0.08);
    
    double error = dynamicSetpoint - angle;

    double pOut = Kp * error;
    if (abs(angle) < 35)
      iTerm += Ki * error * dt;
    iTerm = constrain(iTerm, -180, 180);

    double dOut = -Kd * gyroRate;

    Output = pOut + iTerm + dOut;
    Output = constrain(Output, -255, 255);

    if (abs(angle) > 45) {
      motorRun(0, 0);
    } else {
      double basePWM = Output;
      
      if (basePWM > 0)
        basePWM += 45;
      else if (basePWM < 0)
        basePWM -= 45;
      
      // Áp dụng turn bias (trái/phải)
      double leftPWM = basePWM - turnBias;
      double rightPWM = basePWM + turnBias;
      
      motorRun(leftPWM, rightPWM);
    }

    vTaskDelayUntil(&xLast, xFreq);
  }
}

// ==================================================
//                WEB UI
// ==================================================
String htmlPage() {
  String h = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name='viewport' content='width=device-width,initial-scale=1'>
  <meta charset='UTF-8'>
  <title>ESP32 Balance Bot</title>
  <style>
    * {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
    }
    
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Arial, sans-serif;
      background: #f5f5f5;
      padding: 15px;
      color: #333;
    }
    
    .container {
      max-width: 650px;
      margin: 0 auto;
    }
    
    .header {
      background: white;
      padding: 25px;
      border-radius: 12px;
      text-align: center;
      margin-bottom: 15px;
      box-shadow: 0 2px 8px rgba(0,0,0,0.1);
    }
    
    .header h1 {
      font-size: 26px;
      color: #2196F3;
      margin-bottom: 15px;
    }
    
    .header .status {
      display: flex;
      justify-content: space-around;
      margin-top: 15px;
    }
    
    .status-item {
      text-align: center;
    }
    
    .status-item label {
      display: block;
      color: #777;
      font-size: 13px;
      margin-bottom: 5px;
    }
    
    .status-item .value {
      font-size: 22px;
      font-weight: bold;
      color: #2196F3;
    }
    
    .card {
      background: white;
      border-radius: 12px;
      padding: 20px;
      margin-bottom: 15px;
      box-shadow: 0 2px 8px rgba(0,0,0,0.1);
    }
    
    .card-title {
      font-size: 18px;
      font-weight: bold;
      margin-bottom: 18px;
      color: #2196F3;
      border-bottom: 2px solid #f0f0f0;
      padding-bottom: 10px;
    }
    
    .control-group {
      margin-bottom: 20px;
    }
    
    .control-label {
      display: flex;
      justify-content: space-between;
      margin-bottom: 8px;
      font-weight: 500;
      font-size: 14px;
    }
    
    .control-label .name {
      color: #555;
    }
    
    .control-label .value {
      color: #2196F3;
      font-weight: bold;
    }
    
    .slider {
      width: 100%;
      height: 6px;
      border-radius: 5px;
      background: #ddd;
      outline: none;
      -webkit-appearance: none;
    }
    
    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 22px;
      height: 22px;
      border-radius: 50%;
      background: #2196F3;
      cursor: pointer;
      box-shadow: 0 2px 5px rgba(0,0,0,0.2);
    }
    
    .slider::-moz-range-thumb {
      width: 22px;
      height: 22px;
      border-radius: 50%;
      background: #2196F3;
      cursor: pointer;
      border: none;
      box-shadow: 0 2px 5px rgba(0,0,0,0.2);
    }
    
    .joystick-container {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 15px;
      margin-bottom: 15px;
    }
    
    .joystick {
      aspect-ratio: 1;
      position: relative;
      background: #fafafa;
      border-radius: 12px;
      border: 2px solid #e0e0e0;
      overflow: hidden;
    }
    
    .joystick-label {
      text-align: center;
      font-weight: 600;
      color: #2196F3;
      margin-bottom: 10px;
      font-size: 14px;
    }
    
    .btn-group {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
      height: 100%;
      padding: 15px;
    }
    
    .btn {
      background: white;
      border: 2px solid #2196F3;
      border-radius: 10px;
      font-size: 32px;
      color: #2196F3;
      cursor: pointer;
      transition: all 0.2s;
      box-shadow: 0 2px 5px rgba(0,0,0,0.1);
      display: flex;
      align-items: center;
      justify-content: center;
      user-select: none;
      -webkit-user-select: none;
      touch-action: manipulation;
      font-weight: bold;
    }
    
    .btn:active {
      transform: scale(0.95);
      background: #2196F3;
      color: white;
      box-shadow: 0 1px 3px rgba(0,0,0,0.2);
    }
    
    .btn-stop {
      background: #f44336;
      color: white;
      font-size: 16px;
      font-weight: bold;
      border-color: #f44336;
    }
    
    .btn-stop:active {
      background: #d32f2f;
      border-color: #d32f2f;
    }
    
    .speed-indicator {
      display: flex;
      justify-content: space-around;
      margin-top: 10px;
      font-size: 13px;
      padding: 12px;
      background: #f9f9f9;
      border-radius: 8px;
    }
    
    .speed-indicator div {
      text-align: center;
    }
    
    .speed-indicator .label {
      color: #777;
      margin-bottom: 5px;
    }
    
    .speed-indicator .val {
      font-weight: bold;
      color: #2196F3;
      font-size: 18px;
    }
    
    .input-group {
      display: grid;
      grid-template-columns: 80px 1fr;
      gap: 10px;
      align-items: center;
      margin-bottom: 12px;
    }
    
    .input-group label {
      font-weight: 500;
      font-size: 14px;
      color: #555;
    }
    
    .input-group input {
      padding: 10px;
      border: 2px solid #e0e0e0;
      border-radius: 8px;
      font-size: 14px;
    }
    
    .input-group input:focus {
      outline: none;
      border-color: #2196F3;
    }
    
    .btn-apply {
      width: 100%;
      padding: 14px;
      background: #2196F3;
      color: white;
      border: none;
      border-radius: 10px;
      font-size: 16px;
      font-weight: bold;
      cursor: pointer;
      margin-top: 10px;
      box-shadow: 0 3px 10px rgba(33,150,243,0.3);
      transition: all 0.3s;
    }
    
    .btn-apply:active {
      transform: scale(0.98);
      box-shadow: 0 2px 5px rgba(33,150,243,0.3);
    }
  </style>
</head>
<body>
  <div class='container'>
    <div class='header'>
      <h1>🤖 ESP32 Balance Bot</h1>
      <div class='status'>
        <div class='status-item'>
          <label>Angle</label>
          <div class='value' id='ang'>0.0°</div>
        </div>
        <div class='status-item'>
          <label>PWM</label>
          <div class='value' id='out'>0</div>
        </div>
        <div class='status-item'>
          <label>Status</label>
          <div class='value' style='color:#4CAF50'>●</div>
        </div>
      </div>
    </div>
    
    <div class='card'>
      <div class='card-title'>🎮 Movement Control</div>
      
      <div class='joystick-container'>
        <div>
          <div class='joystick-label'>Forward/Back</div>
          <div class='joystick'>
            <div class='btn-group'>
              <div></div>
              <button class='btn' id='btnFwd'>▲</button>
              <div></div>
              <div></div>
              <button class='btn btn-stop' id='btnStop'>STOP</button>
              <div></div>
              <div></div>
              <button class='btn' id='btnBack'>▼</button>
              <div></div>
            </div>
          </div>
        </div>
        
        <div>
          <div class='joystick-label'>Turn Left/Right</div>
          <div class='joystick'>
            <div class='btn-group'>
              <div></div>
              <button class='btn' id='btnLeft'>◄</button>
              <div></div>
              <div></div>
              <div></div>
              <div></div>
              <div></div>
              <button class='btn' id='btnRight'>►</button>
              <div></div>
            </div>
          </div>
        </div>
      </div>
      
      <div class='speed-indicator'>
        <div>
          <div class='label'>Speed</div>
          <div class='val' id='speedVal'>0</div>
        </div>
        <div>
          <div class='label'>Turn</div>
          <div class='val' id='turnVal'>0</div>
        </div>
      </div>
    </div>
    
    <div class='card'>
      <div class='card-title'>⚙️ PID Parameters</div>
      
      <div class='control-group'>
        <div class='control-label'>
          <span class='name'>Kp (Proportional)</span>
          <span class='value' id='kpVal'>)rawliteral" + String(Kp) + R"rawliteral(</span>
        </div>
        <input type='range' class='slider' id='kpSlider' min='0' max='30' step='0.1' value=')rawliteral" + String(Kp) + R"rawliteral('>
      </div>
      
      <div class='control-group'>
        <div class='control-label'>
          <span class='name'>Ki (Integral)</span>
          <span class='value' id='kiVal'>)rawliteral" + String(Ki) + R"rawliteral(</span>
        </div>
        <input type='range' class='slider' id='kiSlider' min='0' max='2' step='0.01' value=')rawliteral" + String(Ki) + R"rawliteral('>
      </div>
      
      <div class='control-group'>
        <div class='control-label'>
          <span class='name'>Kd (Derivative)</span>
          <span class='value' id='kdVal'>)rawliteral" + String(Kd) + R"rawliteral(</span>
        </div>
        <input type='range' class='slider' id='kdSlider' min='0' max='5' step='0.01' value=')rawliteral" + String(Kd) + R"rawliteral('>
      </div>
      
      <div class='control-group'>
        <div class='control-label'>
          <span class='name'>Setpoint (Target Angle)</span>
          <span class='value' id='setVal'>)rawliteral" + String(Setpoint) + R"rawliteral(</span>
        </div>
        <input type='range' class='slider' id='setSlider' min='-10' max='10' step='0.1' value=')rawliteral" + String(Setpoint) + R"rawliteral('>
      </div>
    </div>
    
    <div class='card'>
      <div class='card-title'>✏️ Manual Input</div>
      <form action='/apply' method='get'>
        <div class='input-group'>
          <label>Kp:</label>
          <input type='number' name='kp' step='any' value=')rawliteral" + String(Kp) + R"rawliteral('>
        </div>
        <div class='input-group'>
          <label>Ki:</label>
          <input type='number' name='ki' step='any' value=')rawliteral" + String(Ki) + R"rawliteral('>
        </div>
        <div class='input-group'>
          <label>Kd:</label>
          <input type='number' name='kd' step='any' value=')rawliteral" + String(Kd) + R"rawliteral('>
        </div>
        <div class='input-group'>
          <label>Setpoint:</label>
          <input type='number' name='set' step='any' value=')rawliteral" + String(Setpoint) + R"rawliteral('>
        </div>
        <button type='submit' class='btn-apply'>APPLY CHANGES</button>
      </form>
    </div>
  </div>
  
  <script>
    const kpSlider = document.getElementById('kpSlider');
    const kiSlider = document.getElementById('kiSlider');
    const kdSlider = document.getElementById('kdSlider');
    const setSlider = document.getElementById('setSlider');
    
    const kpVal = document.getElementById('kpVal');
    const kiVal = document.getElementById('kiVal');
    const kdVal = document.getElementById('kdVal');
    const setVal = document.getElementById('setVal');
    
    kpSlider.oninput = function() {
      kpVal.textContent = this.value;
      fetch('/pid?kp=' + this.value);
    }
    
    kiSlider.oninput = function() {
      kiVal.textContent = this.value;
      fetch('/pid?ki=' + this.value);
    }
    
    kdSlider.oninput = function() {
      kdVal.textContent = this.value;
      fetch('/pid?kd=' + this.value);
    }
    
    setSlider.oninput = function() {
      setVal.textContent = this.value;
      fetch('/pid?set=' + this.value);
    }
    
    const btnFwd = document.getElementById('btnFwd');
    const btnBack = document.getElementById('btnBack');
    const btnLeft = document.getElementById('btnLeft');
    const btnRight = document.getElementById('btnRight');
    const btnStop = document.getElementById('btnStop');
    
    const speedVal = document.getElementById('speedVal');
    const turnVal = document.getElementById('turnVal');
    
    let currentSpeed = 0;
    let currentTurn = 0;
    
    function updateMovement(speed, turn) {
      currentSpeed = speed;
      currentTurn = turn;
      speedVal.textContent = speed;
      turnVal.textContent = turn;
      fetch('/move?speed=' + speed + '&turn=' + turn);
    }
    
    function addButtonEvents(btn, speed, turn) {
      btn.addEventListener('mousedown', () => updateMovement(speed, turn));
      btn.addEventListener('touchstart', (e) => {
        e.preventDefault();
        updateMovement(speed, turn);
      });
      
      btn.addEventListener('mouseup', () => updateMovement(0, 0));
      btn.addEventListener('touchend', (e) => {
        e.preventDefault();
        updateMovement(0, 0);
      });
      
      btn.addEventListener('mouseleave', () => updateMovement(0, 0));
    }
    
    addButtonEvents(btnFwd, 60, 0);
    addButtonEvents(btnBack, -60, 0);
    addButtonEvents(btnLeft, 0, -40);
    addButtonEvents(btnRight, 0, 40);
    
    btnStop.addEventListener('click', () => updateMovement(0, 0));
    
    document.addEventListener('keydown', (e) => {
      if (e.repeat) return;
      switch(e.key) {
        case 'w':
        case 'W':
        case 'ArrowUp':
          updateMovement(60, 0);
          break;
        case 's':
        case 'S':
        case 'ArrowDown':
          updateMovement(-60, 0);
          break;
        case 'a':
        case 'A':
        case 'ArrowLeft':
          updateMovement(0, -40);
          break;
        case 'd':
        case 'D':
        case 'ArrowRight':
          updateMovement(0, 40);
          break;
        case ' ':
          updateMovement(0, 0);
          break;
      }
    });
    
    document.addEventListener('keyup', (e) => {
      switch(e.key) {
        case 'w':
        case 'W':
        case 's':
        case 'S':
        case 'a':
        case 'A':
        case 'd':
        case 'D':
        case 'ArrowUp':
        case 'ArrowDown':
        case 'ArrowLeft':
        case 'ArrowRight':
          updateMovement(0, 0);
          break;
      }
    });
    
    setInterval(() => {
      fetch('/status')
        .then(r => r.json())
        .then(d => {
          document.getElementById('ang').textContent = d.angle.toFixed(1) + '°';
          document.getElementById('out').textContent = d.out.toFixed(0);
        })
        .catch(() => {});
    }, 100);
  </script>
</body>
</html>
)rawliteral";
  
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
  if (server.hasArg("set")) {
    baseSetpoint = server.arg("set").toDouble();
    Setpoint = baseSetpoint;
  }
  savePID();
  server.send(200, "text/plain", "OK");
}

void handleApply() {
  Kp = server.arg("kp").toDouble();
  Ki = server.arg("ki").toDouble();
  Kd = server.arg("kd").toDouble();
  baseSetpoint = server.arg("set").toDouble();
  Setpoint = baseSetpoint;
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

void handleMove() {
  if (server.hasArg("speed")) {
    speedBias = server.arg("speed").toDouble();
  }
  if (server.hasArg("turn")) {
    turnBias = server.arg("turn").toDouble();
  }
  server.send(200, "text/plain", "OK");
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

  // WiFi.begin(ssid, password);
  // Serial.print("Connecting to WiFi");
  // while (WiFi.status() != WL_CONNECTED) {
  //   Serial.print(".");
  //   delay(250);
  // }
  // Serial.println("\nWiFi Connected!");
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.localIP());

  // start Access Point
  WiFi.softAP(ssid);

  server.on("/", []() { server.send(200, "text/html", htmlPage()); });
  server.on("/pid", handlePID);
  server.on("/apply", handleApply);
  server.on("/status", handleStatus);
  server.on("/move", handleMove);
  server.begin();

  xTaskCreatePinnedToCore(PIDTask, "PID", 12000, NULL, 20, NULL, 1);
  
  Serial.println("Balance Bot Ready!");
}

// ==================================================
void loop() { 
  server.handleClient(); 
}