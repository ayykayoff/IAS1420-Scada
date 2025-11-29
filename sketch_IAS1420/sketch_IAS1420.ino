#include <WiFiS3.h>

const char* WIFI_SSID = "iPhone";
const char* WIFI_PASS = "123456789";

const uint16_t MB_PORT = 502;
WiFiServer mbServer(MB_PORT);

static inline uint16_t be16(const uint8_t* p){ return (uint16_t)p[0] << 8 | p[1]; }
static inline void     wbe16(uint8_t* p, uint16_t v){ p[0] = v >> 8; p[1] = v & 0xFF; }

const int enA    = 9;    // L298N ENA (PWM)
const int in1    = 8;    // Direction
const int in2    = 7;    // Direction
const int potPin = A0;   // Potentiometer
const int tempPin= A1;   // Temperature sensor
const int ledPin = 2;    // Over-temp LED

const int   PWM_MIN = 245;      // fan starts only from ~240
const int   PWM_MAX = 255;
const float FULL_SPEED_SPAN = 20.0;  // °C above setpoint to reach full speed
const float LED_HYST       = 0.5;

// HR0: setpointC * 10 (0.1°C)
// HR1: temperatureC * 10
// HR2: PWM (0..255)
// HR3: LED (0/1)
// HR4: MODE command FROM SCADA (0 = MANUAL, 1 = AUTO)
#define HREG_COUNT 5

#define HR_SETPOINT 0
#define HR_TEMP     1
#define HR_PWM      2
#define HR_LED      3
#define HR_MODE     4

int   lastPWM = 0;
bool  ledOn   = false;

// HR4 starts as 0 (MANUAL by default). Arduino does NOT force it later.
uint16_t hreg[HREG_COUNT] = {0,0,0,0,0};

// Variable reflecting the switch (from HR4). 0/1 only.
volatile uint8_t switchCmd = 0;
uint8_t lastSwitchCmd = 255;

void updateFanAndRegs()
{
  static uint32_t lastUpdate = 0;
  uint32_t now = millis();
  if (now - lastUpdate < 100)   // 10 Hz
    return;
  lastUpdate = now;

  // 0 = MANUAL, 1 = AUTO (из HR4)
  switchCmd = (hreg[HR_MODE] != 0) ? 1 : 0;
  bool autoMode = (switchCmd == 1);

  if (switchCmd != lastSwitchCmd) {
    lastSwitchCmd = switchCmd;
    Serial.print("SWITCH CMD (from HR4) = ");
    Serial.println(switchCmd);  // 0 or 1
  }

  int potValue = analogRead(potPin);   // 0..1023 (ручка)
  int rawTemp  = analogRead(tempPin);  // 0..1023 (датчик)

  // Температура всегда одна
  float temperatureC = (rawTemp * 100.0) / 1023.0;  // 0..100°C

  // --- setpoint ВСЕГДА из HR0 (Ignition), и в AUTO, и в MANUAL ---
  float setpointC = hreg[HR_SETPOINT] / 10.0f;   // HR0 хранится как *10

  float error = temperatureC - setpointC;
  int   targetPWM = 0;

  if (autoMode) {
    // --- AUTO: по температуре и setpoint'у из SCADA ---
    if (error > 0) {
      float ratio = error / FULL_SPEED_SPAN;
      if (ratio > 1.0) ratio = 1.0;
      targetPWM = PWM_MIN + (int)((PWM_MAX - PWM_MIN) * ratio);
      if (targetPWM < PWM_MIN) targetPWM = PWM_MIN;
      if (targetPWM > PWM_MAX) targetPWM = PWM_MAX;
    } else {
      targetPWM = 0;
    }
  } else {
    // --- MANUAL: скорость от потенциометра, setpoint НЕ трогаем ---
    float ratio = potValue / 1023.0f;   // 0..1

    if (ratio < 0.05f) {
      targetPWM = 0;
    } else {
      float r = (ratio - 0.05f) / 0.95f;
      if (r < 0)   r = 0;
      if (r > 1.0) r = 1.0;
      targetPWM = PWM_MIN + (int)((PWM_MAX - PWM_MIN) * r);
    }
  }

  analogWrite(enA, targetPWM);
  lastPWM = targetPWM;

  // LED по сравнению T и выбранного setpointC
  if (!ledOn && (temperatureC > setpointC + LED_HYST)) {
    ledOn = true;
    digitalWrite(ledPin, HIGH);
  } else if (ledOn && (temperatureC < setpointC - LED_HYST)) {
    ledOn = false;
    digitalWrite(ledPin, LOW);
  }

  // --- Обновляем регистры Modbus ---
  hreg[HR_TEMP] = (uint16_t)(temperatureC * 10.0f);  // HR1
  hreg[HR_PWM]  = (uint16_t)targetPWM;              // HR2
  hreg[HR_LED]  = ledOn ? 1 : 0;                    // HR3

  // Debug
  Serial.print("MODE: ");
  Serial.print(autoMode ? "AUTO" : "MANUAL");
  Serial.print(" | SP: ");
  Serial.print(setpointC, 1);
  Serial.print(" C | T: ");
  Serial.print(temperatureC, 1);
  Serial.print(" C | PWM: ");
  Serial.print(targetPWM);
  Serial.print(" | LED: ");
  Serial.println(ledOn ? "ON" : "OFF");
}

void handleClient(WiFiClient& c){
  uint8_t buf[256];

  while (c.connected()){
    updateFanAndRegs();

    if (c.available() < 8){
      delay(1);
      continue;
    }

    int n = c.read(buf, sizeof(buf));
    if (n < 8) continue;

    uint16_t len   = (buf[4] << 8) | buf[5];
    uint8_t  unit  = buf[6];
    uint8_t  fc    = buf[7];

    uint8_t* out = buf;
    out[0] = buf[0];
    out[1] = buf[1];
    out[2] = 0;
    out[3] = 0;
    out[6] = unit;

    // --- FC3: Read Holding Registers ---
    if (fc == 3 && len >= 5){
      uint16_t start = be16(&buf[8]);
      uint16_t qty   = be16(&buf[10]);

      if (qty == 0 || (start + qty) > HREG_COUNT){
        out[7] = fc | 0x80;
        out[8] = 0x02;
        wbe16(&out[4], 3);
        c.write(out, 7 + 3);
      } else {
        out[7] = 3;
        out[8] = qty * 2;

        for (uint16_t i = 0; i < qty; i++){
          wbe16(&out[9 + 2*i], hreg[start + i]);
        }

        wbe16(&out[4], 3 + 2*qty);
        c.write(out, 7 + 3 + 2*qty);
      }
      continue;
    }

    // --- FC6: Write Single Register ---
    if (fc == 6 && len >= 5){
      uint16_t addr = be16(&buf[8]);
      uint16_t val  = be16(&buf[10]);

      if (addr < HREG_COUNT){
        hreg[addr] = val;

        Serial.print("FC6 write HR");
        Serial.print(addr);
        Serial.print(" = ");
        Serial.println(val);
      }

      c.write(buf, 12); // echo
      continue;
    }

    // --- FC16: Write Multiple Registers ---
    if (fc == 16 && len >= 6){
      uint16_t start = be16(&buf[8]);
      uint16_t qty   = be16(&buf[10]);
      uint8_t  byteCount = buf[12];

      if (qty == 0 || (start + qty) > HREG_COUNT || byteCount != qty*2){
        out[7] = fc | 0x80;
        out[8] = 0x02;
        wbe16(&out[4], 3);
        c.write(out, 7 + 3);
        continue;
      }

      const uint8_t* p = &buf[13];
      for (uint16_t i = 0; i < qty; i++){
        uint16_t v = be16(p + 2*i);
        hreg[start + i] = v;

        Serial.print("FC16 write HR");
        Serial.print(start + i);
        Serial.print(" = ");
        Serial.println(v);
      }

      out[7] = 16;
      out[8] = (start >> 8) & 0xFF;
      out[9] = start & 0xFF;
      out[10] = (qty >> 8) & 0xFF;
      out[11] = qty & 0xFF;

      wbe16(&out[4], 6);
      c.write(out, 12);
      continue;
    }

    // Unsupported function → exception
    out[7] = fc | 0x80;
    out[8] = 0x01;
    wbe16(&out[4], 3);
    c.write(out, 7 + 3);
  }
}

bool connectWifi(uint32_t timeout_ms = 15000){
  Serial.print("Connecting to "); Serial.println(WIFI_SSID);
  WiFi.disconnect();
  delay(300);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeout_ms){
    delay(250);
  }

  if (WiFi.status() != WL_CONNECTED){
    Serial.println("WiFi failed!");
    return false;
  }

  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());
  return true;
}

void setup(){
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
  digitalWrite(ledPin, LOW);

  Serial.begin(115200);

  if (connectWifi()){
    mbServer.begin();
    Serial.println("Modbus TCP server started on port 502");
  }
}

void loop(){
  updateFanAndRegs();

  WiFiClient cli = mbServer.available();
  if (cli){
    Serial.print("Client connected: ");
    Serial.println(cli.remoteIP());
    handleClient(cli);
    cli.stop();
    Serial.println("Client disconnected");
  }

  delay(5);
}
