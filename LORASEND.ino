#include <SPI.h>
#include <Adafruit_MAX31865.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <math.h>
//#include <LiquidCrystal_I2C.h>
#include <Wire.h> 
#include <BluetoothSerial.h>

// cấu hình bluethooth
#define RELAY_PIN 2
#define SOIL_SENSOR_PIN 32
BluetoothSerial SerialBT;

// Cấu hình cảm biến PT100
#define CS_PT100_1  18 //g5 trên schematic
#define MAX1_MOSI 21  //g18
#define MAX1_MISO 5 //g19
#define MAX1_SCK 19  //21

#define CS_PT100_2 4  //g2
#define MAX2_MOSI 17  //g4
#define MAX2_MISO 2 //g16
#define MAX2_SCK 16  //g17

#define WIRE_TYPE MAX31865_3WIRE

// LoRa pins
#define LORA_SCK     14 //g26
#define LORA_MISO    26 //g25
#define LORA_MOSI    12 //g12
#define LORA_SS      25 //g14
#define LORA_RST     27 //g27
#define LORA_DIO0    13 //g13

// NTC
#define NTC_PIN  34 //g8
#define SERIES_RESISTOR 10000
#define NOMINAL_RESISTANCE 10000
#define NOMINAL_TEMPERATURE 25
#define B_COEFFICIENT 4200

// Tiệm cận
#define SENSOR_PIN  15 //g15

// 4-20mA sensor
#define SEN420_PIN  35

/* I2C LCD CONFIGURATION
//--------------------------------------------------
#define I2C_SDA     23  // free GPIO
#define I2C_SCL     22  // free GPIO
#define LCD_ADDR    0x27
//LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);  // 16×2 display
*/
// Đối tượng PT100
Adafruit_MAX31865 pt100_1 = Adafruit_MAX31865(CS_PT100_1, MAX1_MOSI, MAX1_MISO, MAX1_SCK);
Adafruit_MAX31865 pt100_2 = Adafruit_MAX31865(CS_PT100_2, MAX2_MOSI, MAX2_MISO, MAX2_SCK);

// Thời gian không chặn
unsigned long previousMillis_pt100 = 0;
const long interval_pt100 = 1000;

unsigned long previousMillis_ntc = 0;
const long interval_ntc = 1000;

unsigned long previousMillis_sen420 = 0;
const long interval_sen420 = 1000;

unsigned long previousMillis_sendLoRa = 0;
const long interval_sendLoRa = 1450;  // Gửi mỗi 1 giây

unsigned long previousMillis_lcd = 0;
const long interval_lcd_update = 1000; // Cập nhật giá trị mới lên chuỗi chữ 1s/lần

const long interval_scroll = 1000; // tốc độ dịch chữ trên LCD (nhỏ hơn nhanh hơn)

unsigned long previousMillis_scroll = 0;
int scrollIndex = 0;

// Biến lưu dữ liệu cảm biến
float temp1_val = 0, temp2_val = 0, ntc_val = 0, sen420_val = 0;

// Biến đếm đối tượng và trạng thái loading
int object_count = 0;
String loading = "YES";

//khai báo blth
int rawADC = 0, avgADC = 0;
int moisturePercent = 0;
bool manualMode = false;    // true = điều khiển bằng Bluetooth, false = tự động
bool relayState = false;

const int DRY_THRESHOLD = 20;   // Nếu độ ẩm % < ngưỡng này, bật relay
unsigned long lastMillis = 0;
const unsigned long interval = 3000;

// Chống dội theo thời gian
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 2;
bool lastRawState = HIGH;
bool lastObjectState = false;  // TRUE = có vật, FALSE = không có vật

// Chuỗi chạy chữ
String scrollText1 = "";
String scrollText2 = "";

void updateProximitySensor() {
  bool reading = digitalRead(SENSOR_PIN);

  if (reading != lastRawState) {
    lastDebounceTime = millis();
    lastRawState = reading;
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    bool currentObjectState = (reading == LOW); // LOW = có vật (NPN)

    if (currentObjectState != lastObjectState) {
      lastObjectState = currentObjectState;

      if (currentObjectState == true) {
        object_count++;
        Serial.print("Object count increased: ");
        Serial.println(object_count);
      }
    }

    loading = currentObjectState ? "NO" : "YES";
  }
}

void bluetooth(){
//Đọc lệnh từ Bluetooth
  if (SerialBT.available()) {
    char cmd = SerialBT.read();

    if (cmd == '1') {
      manualMode = true;
      relayState = true;
      Serial.println("Manual Mode: Relay ON");
    } 
    else if (cmd == '0') {
      manualMode = true;
      relayState = false;
      Serial.println("Manual Mode: Relay OFF");
    } 
    else if (cmd == '2' || cmd == 'a') {
      manualMode = false;
      Serial.println("Switched to Auto Mode");
    }

    digitalWrite(RELAY_PIN, relayState);
  }

  // Gửi giá trị độ ẩm mỗi 3 giây
  if (millis() - lastMillis > interval) {
    lastMillis = millis();

    int sum = 0;
    for (int i = 0; i < 10; i++) {
      sum += analogRead(SOIL_SENSOR_PIN);
      delay(5);
    }
    avgADC = sum / 10;

    // Cập nhật giá trị độ ẩm % (cần chỉnh dải tùy cảm biến của bạn)
    moisturePercent = map(avgADC, 3000, 1200, 0, 100);
    moisturePercent = constrain(moisturePercent, 0, 100);

    Serial.print("Soil Moisture: ");
    Serial.print(moisturePercent);
    Serial.println(" %");

    SerialBT.print("Soil Moisture: ");
    SerialBT.print(moisturePercent);
    SerialBT.println(" %");

    // Điều khiển tự động nếu không ở chế độ thủ công
    if (!manualMode) {
      if (moisturePercent < DRY_THRESHOLD) {
        relayState = true;
      } else {
        relayState = false;
      }
      digitalWrite(RELAY_PIN, relayState);
    }
  }

}
void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  analogReadResolution(12);  // 0 - 4095
  // Khởi tạo LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  Serial.println("LoRa init succeeded.");

  // Khởi tạo cảm biến PT100
  pt100_1.begin(WIRE_TYPE);
  pt100_2.begin(WIRE_TYPE);

  /* Khởi tạo LCD
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();

  Serial.println("Ready!");*/
}

void loop() {
  unsigned long currentMillis = millis();

  // Luôn kiểm tra cảm biến tiệm cận
  updateProximitySensor();
  bluetooth();

  // Cập nhật PT100
  if (currentMillis - previousMillis_pt100 >= interval_pt100) {
    previousMillis_pt100 = currentMillis;
    temp1_val = pt100_1.temperature(100.0, 430.0);
    temp2_val = pt100_2.temperature(100.0, 430.0);
  }

  // Cập nhật NTC
  if (currentMillis - previousMillis_ntc >= interval_ntc) {
    previousMillis_ntc = currentMillis;
    int adcValue = analogRead(NTC_PIN);
    float voltage = adcValue * (3.3 / 4095.0);
    float resistance = (SERIES_RESISTOR * voltage) / (3.3 - voltage);
    float steinhart = NOMINAL_RESISTANCE / resistance;
    steinhart = log(steinhart);
    steinhart /= B_COEFFICIENT;
    steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;
    ntc_val = steinhart;
  }

  // Cập nhật 4-20mA
  if (currentMillis - previousMillis_sen420 >= interval_sen420) {
    previousMillis_sen420 = currentMillis;
    int adcValue420 = analogRead(SEN420_PIN);
    float voltage420 = adcValue420 * (3.3 / 4095.0);
    float current_mA = (voltage420 / 3.3) * 20.0;
    float temperature = (current_mA - 4.0) * (100.0 / 16.0);
    sen420_val = temperature;
  }

  // Gửi dữ liệu LoRa
  if (currentMillis - previousMillis_sendLoRa >= interval_sendLoRa) {
    previousMillis_sendLoRa = currentMillis;

    StaticJsonDocument<512> doc;
    doc["device"] = "ESP1";
    doc["p1"] = temp1_val;
    doc["p2"] = temp2_val;
    doc["ntc"] = ntc_val;
    doc["sen420"] = sen420_val;
    doc["object"] = object_count;
    doc["loading"] = loading;

    char buffer[512];
    serializeJson(doc, buffer);

    LoRa.beginPacket();
    LoRa.print("~");
    LoRa.print(buffer);
    LoRa.print("~");
    LoRa.endPacket();

    Serial.println("ESP1 sent LoRa:");
    Serial.println(buffer);
  }

  /* Cập nhật chuỗi chữ cho LCD mỗi 1s để lấy giá trị mới
  if (currentMillis - previousMillis_lcd >= interval_lcd_update) {
    previousMillis_lcd = currentMillis;

    scrollText1 = "T1:" + String(temp1_val, 1) + " T2:" + String(temp2_val, 1) + "  ";
    scrollText2 = "NTC:" + String(ntc_val, 1) + " 4-20:" + String(sen420_val, 1) + " OBJ:" + String(object_count) + " LOAD:" + loading + "  ";

   // scrollIndex = 0;  // reset vị trí dịch khi cập nhật giá trị mới
  }

  // Dịch chữ liên tục trên LCD, mỗi 500ms dịch sang trái 1 ký tự vòng quanh chuỗi dài
  if (currentMillis - previousMillis_scroll >= interval_scroll) {
    previousMillis_scroll = currentMillis;

    int maxLen1 = scrollText1.length();
    int maxLen2 = scrollText2.length();

    // Lấy chuỗi dài nhất trong 2 dòng
    int maxLen = max(maxLen1, maxLen2);

    // Nếu chuỗi ngắn hơn 16 thì bổ sung khoảng trắng cho đủ 16
    if (maxLen < 16) maxLen = 16;

    // Hàm lấy 16 ký tự liên tục theo vòng từ vị trí scrollIndex, có bổ sung khoảng trắng nếu thiếu
    auto getDisplayText = [](String &text, int startPos) -> String {
      String result = "";
      int len = text.length();
      if (len < 16) {
        // Nếu chuỗi ngắn hơn 16 ký tự thì lấy nguyên chuỗi + khoảng trắng
        result = text + String(' ', 16 - len);
      } else {
        if (startPos + 16 <= len) {
          result = text.substring(startPos, startPos + 16);
        } else {
          int part1_len = len - startPos;
          result = text.substring(startPos) + text.substring(0, 16 - part1_len);
        }
        // Nếu kết quả chưa đủ 16 ký tự thì bổ sung khoảng trắng
        if (result.length() < 16) {
          result += String(' ', 16 - result.length());
        }
      }
      return result;
    };

    String disp1 = getDisplayText(scrollText1, scrollIndex);
    String disp2 = getDisplayText(scrollText2, scrollIndex);

    lcd.setCursor(0, 0);
    lcd.print(disp1);

    lcd.setCursor(0, 1);
    lcd.print(disp2);

    scrollIndex++;
    if (scrollIndex >= maxLen) {
      scrollIndex = 0;  // Khi dịch hết chuỗi thì quay về 0 để tiếp tục vòng
    }
  }

  delay(1);  // delay nhỏ giúp chạy mượt loop, không gây gián đoạn hiển thị LCD
  */
}
