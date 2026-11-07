#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "driver/i2s.h"
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <WebServer.h>

// ================= ПИНЫ =================
#define I2C_SDA_WM 4
#define I2C_SCL_WM 5
#define I2C_SDA_DISP 11
#define I2C_SCL_DISP 12
#define I2C_SDA_MPU 9
#define I2C_SCL_MPU 10

#define I2S_MCLK 1
#define I2S_BCLK 2
#define I2S_LRCK 3
#define I2S_DOUT 6
#define I2S_DIN  7

#define BAT_PIN 8   
#define TEMP_PIN 43 
#define BTN_BOOT 0
#define BTN_EXT 13

// ================= BLE NUS UUIDs =================
#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

// ================= ОБЪЕКТЫ =================
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL_DISP, I2C_SDA_DISP);
Adafruit_MPU6050 mpu;
OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);
WebServer server(80);

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// ================= ПЕРЕМЕННЫЕ =================
enum AppMode { MODE_BLE_USB, MODE_WIFI_AP };
AppMode currentAppMode = MODE_BLE_USB;

String currentSensorMode = "MIC";
bool isPowerOn = true;
bool isStreaming = false;
bool btnBootState = false;

float currentTempC = 0.0;
float currentBatVolts = 0.0;
int currentBatPercent = 0;
int peakMic = 0;
int peakPiezo = 0;
float vibX = 0, vibY = 0, vibZ = 0;
char statusMsg[16] = "Ready";
bool codecOk = true;
bool useInternalMic = true; // По умолчанию слушаем микрофон на плате
int64_t sumL = 0;
int64_t sumR = 0;
int rmsSamples = 0;
// ================= ФОРВАРД-ДЕКЛАРАЦИИ =================
void updateDisplay();
void processCommand(String cmd);
void sendTelemetryData();
String getTelemetryJson();
void handleWebRoot();
void handleWebCommand();
void handleWebStatus();

// ================= ВЕБ-ИНТЕРФЕЙС (HTML/JS) =================
const char WEB_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>NVH Scanner PRO</title>
  <style>
    body { background: #12161F; color: #FFF; font-family: 'Courier New', Courier, monospace; margin: 0; padding: 15px; }
    .wrap { max-width: 600px; margin: 0 auto; }
    h2 { color: #00E5FF; text-align: center; border-bottom: 2px solid #3182CE; padding-bottom: 10px; margin-top: 5px; }
    .card { background: #0A0D14; border: 2px solid #2D3748; border-radius: 8px; padding: 15px; margin-bottom: 15px; }
    .row { display: flex; justify-content: space-between; margin-bottom: 10px; font-weight: bold; font-size: 1.1em; }
    .val { color: #00E676; }
    .lbl { color: #CBD5E0; }
    .vu-row { display: flex; justify-content: space-between; margin-top: 15px; font-weight: bold; }
    .vu-bg { background: #000; height: 20px; border: 1px solid #2D3748; border-radius: 4px; width: 100%; margin-top: 5px; overflow: hidden; }
    .vu-fill { background: #00E676; height: 100%; width: 0%; transition: width 0.1s, background 0.1s; }
    .vib { color: #718096; text-align: center; margin-top: 15px; font-weight: bold; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 15px; }
    .grid-3 { grid-template-columns: 1fr 1fr 1fr; }
    .btn { padding: 12px; border: none; border-radius: 4px; font-weight: bold; cursor: pointer; color: #FFF; font-family: inherit; font-size: 0.95em; transition: 0.2s; }
    .btn:active { opacity: 0.8; }
    .btn-scan { background: #2B6CB0; }
    .btn-cal { background: #B7791F; }
    .btn-log-off { background: #2D3748; color: #CBD5E0; }
    .btn-log-on { background: #9B2C2C; }
    .btn-mode { background: #1A202C; color: #CBD5E0; border: 1px solid #2D3748; }
    .radio-box { background: #0A0D14; border: 1px solid #2D3748; padding: 12px; border-radius: 4px; display: flex; flex-direction: column; gap: 10px; margin-bottom: 15px; }
    .radio-title { color: #CBD5E0; font-weight: bold; margin-bottom: 5px; }
    .radio-options { display: flex; justify-content: space-around; }
    .radio-options label { color: #00E5FF; font-weight: bold; cursor: pointer; display: flex; align-items: center; gap: 5px; }
    input[type="radio"] { accent-color: #00E5FF; transform: scale(1.2); }
  </style>
</head>
<body>
  <div class="wrap">
    <h2>CAR NVH SCANNER</h2>
    <div class="card">
      <div class="row">
        <span class="lbl">Status: <span id="st" class="val">Ready</span></span>
        <span class="lbl">Mode: <span id="md" style="color:#00E5FF">MIC</span></span>
      </div>
      <div class="row">
        <span class="lbl">Temp: <span id="tm" class="val">0.0</span>&deg;C</span>
        <span class="lbl">Bat: <span id="bt" style="color:#FFD600">0.0</span>V</span>
      </div>
      <hr style="border-color:#2D3748; margin: 15px 0;">
      
      <div class="vu-row"><span class="lbl">MIC AIR</span><span id="mv">0%</span></div>
      <div class="vu-bg"><div id="mb" class="vu-fill"></div></div>
      
      <div class="vu-row"><span class="lbl">PIEZO (METAL)</span><span id="pv">0%</span></div>
      <div class="vu-bg"><div id="pb" class="vu-fill"></div></div>
      
      <div class="vib">VIB X:<span id="vx">0.00</span> Y:<span id="vy">0.00</span> Z:<span id="vz">0.00</span></div>
    </div>

    <div class="grid">
      <button class="btn btn-scan" onclick="cmd('MEAS')">🔍 SCAN</button>
      <button class="btn btn-cal" onclick="cmd('CAL')">🎯 CALIBRATE</button>
      <button id="btn_log" class="btn btn-log-off" onclick="toggleLog()">📡 LOG: OFF</button>
      <button class="btn btn-scan" onclick="cmd('STATUS')" style="background:#276749">🔄 STATUS</button>
    </div>

    <div class="radio-box">
      <div class="radio-title">MIC SOURCE:</div>
      <div class="radio-options">
        <label><input type="radio" name="src" id="r_int" onclick="cmd('MIC INT')"> INTERNAL</label>
        <label><input type="radio" name="src" id="r_ext" onclick="cmd('MIC EXT')"> EXTERNAL</label>
      </div>
    </div>

    <div class="grid grid-3">
      <button class="btn btn-mode" onclick="cmd('MODE MIC')">MIC</button>
      <button class="btn btn-mode" onclick="cmd('MODE PIEZO')">PIEZO</button>
      <button class="btn btn-mode" onclick="cmd('MODE VIB')">VIB</button>
    </div>
  </div>

  <script>
    let isLog = false;
    function cmd(c) { fetch('/command?cmd=' + c); }
    function toggleLog() {
      if(isLog) cmd('STOP'); else cmd('START');
    }
    function getColor(v) {
      if(v > 85) return '#FF1744'; // Red
      if(v > 65) return '#FFD600'; // Yellow
      return '#00E676';            // Green
    }
    setInterval(() => {
      fetch('/status').then(r => r.json()).then(d => {
        document.getElementById('st').innerText = d.status;
        document.getElementById('md').innerText = d.mode;
        document.getElementById('tm').innerText = d.temp.toFixed(1);
        document.getElementById('bt').innerText = d.bat.toFixed(1);
        
        // MIC Bar
        document.getElementById('mv').innerText = d.mic + '%';
        let mb = document.getElementById('mb');
        mb.style.width = d.mic + '%';
        mb.style.background = getColor(d.mic);
        
        // PIEZO Bar
        document.getElementById('pv').innerText = d.piezo + '%';
        let pb = document.getElementById('pb');
        pb.style.width = d.piezo + '%';
        pb.style.background = getColor(d.piezo);
        
        // VIB
        document.getElementById('vx').innerText = d.vib_x.toFixed(2);
        document.getElementById('vy').innerText = d.vib_y.toFixed(2);
        document.getElementById('vz').innerText = d.vib_z.toFixed(2);
        
        // Sync MIC SRC Radio
        document.getElementById('r_int').checked = d.mic_int;
        document.getElementById('r_ext').checked = !d.mic_int;

        // Sync LOG Button
        isLog = d.stream;
        let bl = document.getElementById('btn_log');
        if(isLog) {
          bl.className = 'btn btn-log-on'; bl.innerText = '📡 LOG: ON';
        } else {
          bl.className = 'btn btn-log-off'; bl.innerText = '📡 LOG: OFF';
        }
      }).catch(e => console.log("Conn Error"));
    }, 500);
  </script>
</body>
</html>
)rawliteral";
// ================= BLE CALLBACKS =================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      snprintf(statusMsg, sizeof(statusMsg), "BLE Connected");
      updateDisplay();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      isStreaming = false;
      snprintf(statusMsg, sizeof(statusMsg), "BLE Wait...");
      updateDisplay();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue().c_str();
      if (rxValue.length() > 0) {
        processCommand(rxValue);
      }
    }
};

// ================= ИНИЦИАЛИЗАЦИЯ ЖЕЛЕЗА =================
void initBLE() {
  BLEDevice::init("ESP32-S3-CarDiag");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RX,
                                           BLECharacteristic::PROPERTY_WRITE
                                         );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
}

void writeWM8978(uint8_t reg, uint16_t val) {
  // 1. Освобождаем шину от дисплея
  Wire.end(); 
  
  // 2. Общаемся с аудиокодеком Вильсона
  Wire.begin(I2C_SDA_WM, I2C_SCL_WM);
  Wire.beginTransmission(0x1A);
  Wire.write((reg << 1) | ((val >> 8) & 0x01)); // Сдвиг 9 бит
  Wire.write(val & 0xFF);
  byte err = Wire.endTransmission();
  
  if (err == 0) codecOk = true; 
  
  // 3. ОБЯЗАТЕЛЬНО возвращаем шину OLED дисплею, чтобы плата не зависла!
  Wire.end();
  Wire.begin(I2C_SDA_DISP, I2C_SCL_DISP);
  Wire.setClock(400000); 
}

void setAudioRouting() {
  if (!codecOk) return;
  
  if (useInternalMic) {
    // INTERNAL (Внутренний микрофон на плате)
    // 0x000 отключает все внешние гнезда от микрофонных усилителей.
    // Работают только жестко запаянные LINPUT1 (Mic Air) и RINPUT1 (Piezo).
    // Никаких висящих в воздухе контактов, чистый звук!
    writeWM8978(44, 0x000); 
  } else {
    // EXTERNAL (Щуп в гнезде Line-In)
    // Ваш на 100% рабочий и проверенный вариант
    writeWM8978(44, 0x033); 
  }
}
void initWM8978() {
  codecOk = false;
  delay(100);
  writeWM8978(0, 0);       // Soft reset
  delay(50);
  if(codecOk) {
    // 1. Рег 1: Питание. Включаем VMID, BIASEN и главное — MICBEN (питание микрофона!)
    writeWM8978(1, 0x03F);   
    // 2. Рег 2: Включаем АЦП (ADCEN) и предусилители микрофонов (INPGAEN) обоих каналов
    writeWM8978(2, 0x03F);   
    // 3. Рег 3: ЦАП (DAC) для вывода звука оставим включенным
    writeWM8978(3, 0x06C);   
    // 4. Рег 4: Формат аудио: I2S, 16 бит
    writeWM8978(4, 0x010);   
    // 5. Рег 6: Кодек работает в режиме Slave (частоту задает ESP32)
    writeWM8978(6, 0x000);   
    // 6. Рег 14: Отключаем фильтры высоких частот (чтобы писать чистый сырой звук)
    writeWM8978(14, 0x000);  
    // 7. Рег 44: Маршрутизация. Подключаем пины капсюля прямо к усилителю
    setAudioRouting();  // или внутренний микрофон или внешний
    // 8. Рег 45 и 46: Чувствительность микрофонов (PGA Gain). 0x03F = +30dB (Максимум!)
    writeWM8978(45, 0x03F);  // Левый канал (Микрофон)
    writeWM8978(46, 0x03F);  // Правый канал (Пьезо)
    // 9. Рег 47 и 48: Дополнительный цифровой Boost АЦП на +20dB
    writeWM8978(47, 0x100);  
    writeWM8978(48, 0x100);  
  }
}
void initI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll = true,
    .tx_desc_auto_clear = true
  };

  i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_MCLK,
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRCK,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_DIN
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}
void readBattery() {
  int adcValue = analogRead(BAT_PIN);
  float pinVoltage = (adcValue / 4095.0) * 3.1;
  currentBatVolts = pinVoltage * 2.0; 
  currentBatPercent = (currentBatVolts - 3.2) * 100.0;
  if (currentBatPercent > 100) currentBatPercent = 100;
  if (currentBatPercent < 0) currentBatPercent = 0;
}

// ================= ЛОГИКА СВЯЗИ =================
String getTelemetryJson() {
  StaticJsonDocument<256> doc;
  doc["temp"] = currentTempC;
  doc["bat"] = currentBatVolts;
  doc["mic"] = peakMic;
  doc["piezo"] = peakPiezo;
  doc["vib_x"] = vibX;
  doc["vib_y"] = vibY;
  doc["vib_z"] = vibZ;
  doc["status"] = statusMsg;
  doc["codec"] = codecOk;
  doc["mic_int"] = useInternalMic;
  doc["stream"] = isStreaming; 
  doc["mode"] = currentSensorMode;
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}
void sendTelemetryData() {
  String jsonStr = getTelemetryJson() + "\r\n";
  
  // 1. Отправляем в USB
  Serial.print(jsonStr);

  // 2. Отправляем в BLE
  if (currentAppMode == MODE_BLE_USB && deviceConnected) {
    pTxCharacteristic->setValue(jsonStr.c_str());
    pTxCharacteristic->notify();
  }
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "STATUS" || cmd == "MEAS") {
    sendTelemetryData();
  } 
  else if (cmd == "START") {
    isStreaming = true;
    snprintf(statusMsg, sizeof(statusMsg), "LOG: ON");
    sendTelemetryData();
  } 
  else if (cmd == "STOP") {
    isStreaming = false;
    snprintf(statusMsg, sizeof(statusMsg), "Ready");
    sendTelemetryData();
  } 
  else if (cmd == "CAL") {
    snprintf(statusMsg, sizeof(statusMsg), "Calibrating");
    updateDisplay();
    delay(500); 
    snprintf(statusMsg, sizeof(statusMsg), "Ready");
    sendTelemetryData();
  } 
  else if (cmd.startsWith("MODE ")) {
    currentSensorMode = cmd.substring(5);
    snprintf(statusMsg, sizeof(statusMsg), "Mode SET");
    sendTelemetryData();
  }
  else if (cmd == "MIC INT") {
    useInternalMic = true;
    setAudioRouting();
    snprintf(statusMsg, sizeof(statusMsg), "Mic: IN-BOARD");
    sendTelemetryData();
  } 
  else if (cmd == "MIC EXT") {
    useInternalMic = false;
    setAudioRouting();
    snprintf(statusMsg, sizeof(statusMsg), "Mic: LINE-IN");
    sendTelemetryData();
  }
}

// ================= WEBSERVER НА CORE 0 =================
void handleWebRoot() {
  server.send(200, "text/html", WEB_PAGE);
}

void handleWebStatus() {
  server.send(200, "application/json", getTelemetryJson());
}

void handleWebCommand() {
  if (server.hasArg("cmd")) {
    processCommand(server.arg("cmd"));
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing cmd");
  }
}

// Задача для второго ядра (Core 0)
void webServerTask(void *pvParameters) {
  for (;;) {
    if (currentAppMode == MODE_WIFI_AP) {
      server.handleClient();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Отдаем время планировщику
  }
}

// ================= ДИСПЛЕЙ =================
void drawInterface() {
  u8g2.drawFrame(102, 0, 22, 6);      
  u8g2.drawBox(124, 2, 2, 2);         
  int barWidth = map(currentBatPercent, 0, 100, 0, 18);
  if (barWidth > 0) u8g2.drawBox(104, 2, barWidth, 2); 

  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.setCursor(82, 5);
  u8g2.print(currentBatPercent); u8g2.print("%");

  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 10);
  if (currentAppMode == MODE_BLE_USB) {
    u8g2.print("BLE/USB "); u8g2.print(deviceConnected ? "[CON]" : "[WAIT]");
  } else {
    u8g2.print("WiFi AP: CarDiag");
  }

  u8g2.setCursor(0, 25);
  u8g2.print("Mode: "); u8g2.print(currentSensorMode);

  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.setCursor(0, 45);
  u8g2.print(currentTempC, 1); u8g2.print(" C");

  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 60);
  u8g2.print(statusMsg);
}

void updateDisplay() {
  u8g2.firstPage();
  do {
    drawInterface();
  } while (u8g2.nextPage());
}

// ================= ЛОГИКА КНОПКИ BOOT =================
void handleBootButton() {
  bool currentBootState = !digitalRead(BTN_BOOT);
  if (currentBootState && !btnBootState) {
    if (currentAppMode == MODE_BLE_USB) {
      // ПЕРВЫЙ КЛИК: ПЕРЕХОД В WIFI AP
      currentAppMode = MODE_WIFI_AP;
      snprintf(statusMsg, sizeof(statusMsg), "Starting Wi-Fi");
      updateDisplay();
      
      // 1. Безопасно отключаем видимость Bluetooth (чтобы плата не крашилась)
      if (pServer) {
          pServer->getAdvertising()->stop();
      }
      delay(100);
      
      // 2. Правильно включаем режим точки доступа Wi-Fi
      WiFi.mode(WIFI_AP);
      WiFi.softAP("CarDiag-Pro", "12345678"); // Пароль точки доступа
      
      server.on("/", handleWebRoot);
      server.on("/status", handleWebStatus);
      server.on("/command", handleWebCommand);
      server.begin();

      // Запускаем веб-сервер на фоновом ядре 0 (Core 0)
      xTaskCreatePinnedToCore(webServerTask, "WebTask", 4096, NULL, 1, NULL, 0);
      
      snprintf(statusMsg, sizeof(statusMsg), "IP: 192.168.4.1");
      updateDisplay();
      
    } else {
      // ВТОРОЙ КЛИК: ЖЕСТКАЯ ПЕРЕЗАГРУЗКА
      snprintf(statusMsg, sizeof(statusMsg), "Rebooting...");
      updateDisplay(); 
      delay(500);
      ESP.restart();
    }
    delay(500); // Антидребезг
  }
  btnBootState = currentBootState;
}
// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  pinMode(BTN_BOOT, INPUT_PULLUP);
  pinMode(BTN_EXT, INPUT_PULLUP);
  analogReadResolution(12);

  initWM8978();
  initI2S();     // Вызов добавлен!
  
  u8g2.begin();

  Wire1.begin(I2C_SDA_MPU, I2C_SCL_MPU);
  if (mpu.begin(0x68, &Wire1)) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  }
  sensors.begin();
  
  // По умолчанию стартует BLE
  initBLE();

  readBattery();
  updateDisplay();
}

// ================= LOOP (CORE 1) =================
void loop() {
  handleBootButton();

  // Обработка команд из USB Serial
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }

  // --- НЕПРЕРЫВНОЕ ЧТЕНИЕ I2S (РЕШАЕТ ПРОБЛЕМУ ОТВАЛОВ И ПЕРЕЗАГРУЗОК) ---
  size_t bytesRead = 0;
  int16_t i2sBuffer[128]; // Читаем мелкими порциями
  
  // timeout = 0 (не блокирует процессор, просто забирает то, что накопилось)
  if (i2s_read(I2S_NUM_0, i2sBuffer, sizeof(i2sBuffer), &bytesRead, 0) == ESP_OK && bytesRead > 0) {
    int samplesRead = bytesRead / 2; 
    for (int i = 0; i < samplesRead; i += 2) {
      sumL += (int64_t)i2sBuffer[i] * i2sBuffer[i];
      sumR += (int64_t)i2sBuffer[i + 1] * i2sBuffer[i + 1];
      rmsSamples++;
    }
  }

  // Обновление датчиков и отправка телеметрии строго 1 раз в секунду
  static uint32_t lastSensorUpdate = 0;
  if (millis() - lastSensorUpdate > 1000) {
    lastSensorUpdate = millis();
    
    sensors.requestTemperatures();
    currentTempC = sensors.getTempCByIndex(0);
    readBattery();

    sensors_event_t a, g, temp_mpu;
    mpu.getEvent(&a, &g, &temp_mpu);
    vibX = a.acceleration.x / 9.81;
    vibY = a.acceleration.y / 9.81;
    vibZ = a.acceleration.z / 9.81;

    // Вычисляем RMS за всю накопившуюся секунду (максимальная точность звука!)
    if (rmsSamples > 0) {
      int rmsL = sqrt(sumL / rmsSamples);
      int rmsR = sqrt(sumR / rmsSamples);
      
      peakMic = map(rmsL, 0, 4000, 0, 100);
      peakPiezo = map(rmsR, 0, 4000, 0, 100);
      
      if (peakMic > 100) peakMic = 100;
      if (peakPiezo > 100) peakPiezo = 100;

      // Обязательно сбрасываем накопители для следующей секунды
      sumL = 0;
      sumR = 0;
      rmsSamples = 0;
    }

    // Если включен режим стриминга (START)
    if (isStreaming) {
      sendTelemetryData();
    }
    
    updateDisplay();
  }

  // Восстановление Advertising для BLE, если дисконнект
  if (currentAppMode == MODE_BLE_USB) {
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); 
        pServer->startAdvertising();
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
  }
}