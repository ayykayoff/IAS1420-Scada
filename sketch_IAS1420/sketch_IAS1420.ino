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
const int potPin = A0;   // Potentiometer -> setpoint (0..100 °C)
const int tempPin= A1;   // Temperature sensor (0..5V = 0..100 °C)
const int ledPin = 2;    // Over-temp LED

const int   PWM_MIN = 245;      // fan starts only from ~240
const int   PWM_MAX = 255;
const float FULL_SPEED_SPAN = 20.0;  // °C above setpoint to reach full speed
const float LED_HYST       = 0.5;

// HR0: setpointC * 10 (0.1°C)
// HR1: temperatureC * 10
// HR2: PWM (0..255)
// HR3: LED (0/1)
// HR4: MODE (0 = fan OFF, 1 = AUTO temperature control)
#define HREG_COUNT 5

#define HR_SETPOINT 0
#define HR_TEMP     1
#define HR_PWM      2
#define HR_LED      3
#define HR_MODE     4

int   lastPWM = 0;
bool  ledOn   = false;

// default: AUTO mode enabled (HR4 = 1)
uint16_t hreg[HREG_COUNT] = {0,0,0,0,1};

void updateFanAndRegs()
{
  static uint32_t lastUpdate = 0;
  uint32_t now = millis();
  if (now - lastUpdate < 100)   // 10 Hz
    return;
  lastUpdate = now;

  int potValue = analogRead(potPin);   // 0..1023
  int rawTemp  = analogRead(tempPin);  // 0..1023

  float setpointC    = (potValue * 100.0) / 1023.0;  // 0..100°C
  float temperatureC = (rawTemp  * 100.0) / 1023.0;  // 0..100°C

  float error = temperatureC - setpointC;
  int   targetPWM = 0;

  // 1 = AUTO (по температуре), 0 = MANUAL (скорость задаёт потенциометр)
  bool autoMode = (hreg[HR_MODE] != 0);

  if (autoMode) {
    // --- AUTO: как было ---
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
    // --- MANUAL: скорость от потенциометра ---
    // делаем зону, где вентилятор полностью выключен,
    // а дальше масштабируем в диапазон PWM_MIN..PWM_MAX
    float ratio = potValue / 1023.0f;   // 0..1

    if (ratio < 0.05f) {
      // нижние ~5% хода потенциометра — вентилятор off
      targetPWM = 0;
    } else {
      float r = (ratio - 0.05f) / 0.95f;  // 0..1 после "мертвой зоны"
      if (r < 0)   r = 0;
      if (r > 1.0) r = 1.0;
      targetPWM = PWM_MIN + (int)((PWM_MAX - PWM_MIN) * r);
    }
  }

  analogWrite(enA, targetPWM);
  lastPWM = targetPWM;

  // LED hysteresis (оставляем как было — по температуре и setpoint)
  if (!ledOn && (temperatureC > setpointC + LED_HYST)) {
    ledOn = true;
    digitalWrite(ledPin, HIGH);
  } else if (ledOn && (temperatureC < setpointC - LED_HYST)) {
    ledOn = false;
    digitalWrite(ledPin, LOW);
  }

  // обновляем регистры Modbus
  hreg[HR_SETPOINT] = (uint16_t)(setpointC    * 10.0f);  // HR0
  hreg[HR_TEMP]     = (uint16_t)(temperatureC * 10.0f);  // HR1
  hreg[HR_PWM]      = (uint16_t)targetPWM;              // HR2 (фактический PWM)
  hreg[HR_LED]      = ledOn ? 1 : 0;                    // HR3
  // HR4 (MODE) не трогаем — им рулит SCADA

  // Debug в Serial
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

    // Ждём хотя бы 7 байт MBAP (Transaction + Protocol + Length + UnitId)
    if (c.available() < 7){
      delay(1);
      continue;
    }

    // Читаем 7 байт MBAP
    int n = c.read(buf, 7);
    if (n != 7) {
      // что-то странное, пробуем заново
      continue;
    }

    // MBAP: [0..1] TID, [2..3] PID, [4..5] Length (UnitId + PDU), [6] UnitId
    uint16_t len = be16(&buf[4]);
    if (len < 2 || len > (sizeof(buf) - 7)) {
      Serial.println("Bad MBAP length, closing client");
      break;
    }

    // len = 1 (UnitId) + PDU_length, но UnitId мы уже прочитали в buf[6]
    uint16_t pdu_len = len - 1;

    // Ждём, пока придут все байты PDU
    uint32_t t0 = millis();
    while (c.available() < pdu_len){
      if (millis() - t0 > 1000){
        Serial.println("Timeout waiting PDU, closing client");
        return;
      }
      delay(1);
    }

    // Читаем PDU (fc + данные)
    int m = c.read(buf + 7, pdu_len);
    if (m != pdu_len){
      // не дочитали — пропускаем
      continue;
    }

    uint8_t unit = buf[6];
    uint8_t fc   = buf[7];

    uint8_t* out = buf;

    // TransactionId (0..1) и ProtocolId (2..3) оставляем как есть,
    // но ProtocolId по Modbus TCP = 0
    out[2] = 0;
    out[3] = 0;
    out[6] = unit;  // UnitId

    // ---------- FC3: Read Holding Registers ----------
    if (fc == 3 && pdu_len >= 5) {
      uint16_t start = be16(&buf[8]);   // starting address
      uint16_t qty   = be16(&buf[10]);  // number of registers

      if (qty == 0 || (start + qty) > HREG_COUNT){
        // exception: illegal data address
        out[7] = fc | 0x80;
        out[8] = 0x02;  // ILLEGAL DATA ADDRESS
        uint16_t pdu_resp_len = 2;           // fc + exception code
        uint16_t len_resp     = 1 + pdu_resp_len; // UnitId + PDU
        wbe16(&out[4], len_resp);
        c.write(out, 7 + pdu_resp_len);      // MBAP(7) + PDU
      } else {
        out[7] = 3;            // fc
        out[8] = qty * 2;      // byte count

        for (uint16_t i = 0; i < qty; i++){
          wbe16(&out[9 + 2*i], hreg[start + i]);
        }

        uint16_t pdu_resp_len = 2 + 2*qty;  // fc + byteCount + data
        uint16_t len_resp     = 1 + pdu_resp_len; // UnitId + PDU
        wbe16(&out[4], len_resp);
        c.write(out, 7 + pdu_resp_len);     // MBAP(7) + PDU
      }
      continue;
    }

    // ---------- FC6: Write Single Register ----------
    if (fc == 6 && pdu_len >= 5) {
      uint16_t addr = be16(&buf[8]);
      uint16_t val  = be16(&buf[10]);

      if (addr < HREG_COUNT){
        hreg[addr] = val;

        Serial.print("FC6 write HR");
        Serial.print(addr);
        Serial.print(" = ");
        Serial.println(val);
      }

      // Ответ по FC6 — эхо: fc + addr + val
      uint16_t pdu_resp_len = 5;          // fc + addr(2) + val(2)
      uint16_t len_resp     = 1 + pdu_resp_len; // UnitId + PDU
      wbe16(&out[4], len_resp);
      // buf уже содержит fc/addr/val на тех же позициях
      c.write(out, 7 + pdu_resp_len);     // MBAP(7) + PDU
      continue;
    }

    // ---------- FC16: Write Multiple Registers ----------
    if (fc == 16 && pdu_len >= 6) {
      uint16_t start = be16(&buf[8]);   // starting address
      uint16_t qty   = be16(&buf[10]);  // how many registers
      uint8_t  byteCount = buf[12];

      if (qty == 0 || (start + qty) > HREG_COUNT || byteCount != qty * 2){
        out[7] = fc | 0x80;  // exception
        out[8] = 0x02;       // illegal data address
        uint16_t pdu_resp_len = 2;           // fc + exception code
        uint16_t len_resp     = 1 + pdu_resp_len;
        wbe16(&out[4], len_resp);
        c.write(out, 7 + pdu_resp_len);
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

      // Ответ: fc + start + qty
      out[7]  = 16;
      out[8]  = (start >> 8) & 0xFF;
      out[9]  = start & 0xFF;
      out[10] = (qty >> 8) & 0xFF;
      out[11] = qty & 0xFF;

      uint16_t pdu_resp_len = 5;           // fc + start(2) + qty(2)
      uint16_t len_resp     = 1 + pdu_resp_len;
      wbe16(&out[4], len_resp);
      c.write(out, 7 + pdu_resp_len);
      continue;
    }

    // ---------- Unsupported function → exception ----------
    out[7] = fc | 0x80;
    out[8] = 0x01;  // ILLEGAL FUNCTION
    {
      uint16_t pdu_resp_len = 2;           // fc + exc.code
      uint16_t len_resp     = 1 + pdu_resp_len;
      wbe16(&out[4], len_resp);
      c.write(out, 7 + pdu_resp_len);
    }
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
