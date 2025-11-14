#include <WiFiS3.h>

// ===== Wi-Fi =====
const char* WIFI_SSID = "iPhone";      // твой SSID
const char* WIFI_PASS = "123456789";   // твой пароль

// ===== Modbus TCP =====
const uint16_t MB_PORT = 502;          // стандартный порт Modbus TCP
WiFiServer mbServer(MB_PORT);

// ===== Данные =====
uint16_t hreg0 = 6;  // HR0 всегда 6

// ---- вспомогательные функции ----
static inline uint16_t be16(const uint8_t* p){ return (uint16_t)p[0] << 8 | p[1]; }
static inline void wbe16(uint8_t* p, uint16_t v){ p[0] = v >> 8; p[1] = v & 0xFF; }

// ---- обработка клиента ----
void handleClient(WiFiClient& c){
  uint8_t buf[256];

  while (c.connected()){
    if (c.available() < 8){ delay(1); continue; }
    int n = c.read(buf, sizeof(buf));
    if (n < 8) continue;

    uint16_t trans = (buf[0] << 8) | buf[1];
    uint16_t len   = (buf[4] << 8) | buf[5];
    uint8_t  unit  = buf[6];
    uint8_t  fc    = buf[7];

    uint8_t* out = buf;
    out[0] = buf[0]; out[1] = buf[1]; out[2] = 0; out[3] = 0; out[6] = unit;

    // --- FC3: Read Holding Registers ---
    if (fc == 3 && len >= 5){
      uint16_t start = be16(&buf[8]);
      uint16_t qty   = be16(&buf[10]);

      if (start == 0 && qty == 1){
        out[7] = 3;       // Function
        out[8] = 2;       // byte count
        wbe16(&out[9], hreg0);
        wbe16(&out[4], 3 + 2);  // length
        c.write(out, 7 + 3 + 2);
      }
      continue;
    }

    // --- FC6: Write Single Register (optional) ---
    if (fc == 6 && len >= 5){
      uint16_t addr = be16(&buf[8]);
      uint16_t val  = be16(&buf[10]);
      if (addr == 0){ hreg0 = val; }   // разрешаем запись в HR0
      c.write(buf, 12);                // echo
      continue;
    }
  }
}

// ---- подключение Wi-Fi ----
bool connectWifi(uint32_t timeout_ms = 15000){
  Serial.print("Connecting to "); Serial.println(WIFI_SSID);
  WiFi.disconnect(); delay(300);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeout_ms){ delay(250); }
  if (WiFi.status() != WL_CONNECTED){ Serial.println("WiFi failed!"); return false; }
  Serial.print("Connected! IP: "); Serial.println(WiFi.localIP());
  return true;
}

void setup(){
  Serial.begin(115200);
  if (connectWifi()){
    mbServer.begin();
    Serial.println("Modbus TCP server started on port 502");
  }
}

void loop(){
  WiFiClient cli = mbServer.available();
  if (cli){
    Serial.print("Client connected: "); Serial.println(cli.remoteIP());
    handleClient(cli);
    cli.stop();
  }
}
