/*
 * =====================================================================================
 * HE THONG GIAM SAT NONG DO KHI CO2 & DIEU KHIEN QUAT THONG GIO TU DONG (ARDUINO UNO)
 * PHIEN BAN TIN HIEU SONG DONG LIVE: DO THI DAO DONG LIEN TUC THEO CAM BIEN (MOC 400 PPM)
 * KHONG KEP CUNG GIA TRI - PHAN HOI 100% THEO TIN HIEU ANALOG THOI GIAN THUC
 * TUONG THICH HOAN HAO ARDUINO SERIAL PLOTTER & LCD 1602 I2C
 * =====================================================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// --- CAU HINH CHAN PHAN CUNG ---
#define PIN_MQ135      A0      // Tin hieu Analog MQ-135 (Cam vao chan AO tren module)
#define PIN_DHT        2       // Tin hieu Digital DHT11
#define DHTTYPE        DHT11   // Cam bien DHT11
#define PIN_FAN_PWM    9       // PWM dieu khien MOSFET D4184 (Quat 5V)

LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(PIN_DHT, DHTTYPE);

// =====================================================================================
// --- THAM SO CALIB CHUAN MOC 400 PPM KHONG KHI SACH ---
// =====================================================================================
const float RL = 10.0;               // Dien tro tai tren module MQ-135 (10 kOhm)
const float T_CAL = 31.4;            // Nhiet do luc Calib (do C)
const float H_CAL = 62.4;            // Do am luc Calib (% RH)
const float CO2_EXPONENT = -2.76903; // He so mu chuan Winsen MQ-135

float R0_BASE = 108.0;               // Tu dong neo chuan theo dien tro phong de moc 400ppm dao dong song dong

// --- GIOI HAN CONG SUAT QUAT TOI DA (FULL 255 PWM = 100% 5V) ---
const int PWM_MAX_FAN = 255;         // Cong suat toi da 100%
const int HYST_PPM    = 25;          // Tre Schmitt Trigger chong rung ro-le/dao dong quat

const float DHT_TEMP_OFFSET = -2.0; 
const float DHT_HUMI_OFFSET = 0.0;

// Biến trạng thái
float temperature = T_CAL;
float humidity = H_CAL;
int currentLevel = 0;                // 0: TOT, 1: TB, 2: CAO, 3: NGUY
int currentFanPWM = 0;
bool lcdReady = false;

// Trạng thái kiểm tra sức khỏe cảm biến DHT11
bool dhtOk = true;
int dhtErrorCount = 0;
bool dhtHeartbeat = false;

unsigned long lastDhtTime = 0;

// --- HAM TINH HE SO BU NHIET AM HANWEI ---
float getMQ135CorrectionFactor(float t, float h) {
  float cor = (0.00035 * t * t - 0.02718 * t + 1.39538) - ((h - 33.0) * 0.0018);
  if (cor < 0.35) cor = 0.35;
  if (cor > 2.50) cor = 2.50;
  return cor;
}

// --- BO TRUNG VI 5 MAU: GIU NGUYEN DAO DONG ANALOG THOI GIAN THUC ---
float readLiveADC() {
  const int NUM_SAMPLES = 5;
  float samples[NUM_SAMPLES];
  for (int i = 0; i < NUM_SAMPLES; i++) {
    samples[i] = analogRead(PIN_MQ135);
    delayMicroseconds(100);
  }
  for (int i = 1; i < NUM_SAMPLES; i++) {
    float key = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }
  return samples[2]; // Lay mau o giua de loai nhieu xung nhung giu tron dao dong song
}

// --- KIEM TRA VA KHOI TAO LCD ---
bool checkAndInitLCD() {
  Wire.beginTransmission(0x27);
  if (Wire.endTransmission() == 0) {
    if (!lcdReady) {
      lcd.init();
      lcd.backlight();
      lcdReady = true;
    }
    return true;
  }
  Wire.beginTransmission(0x3F);
  if (Wire.endTransmission() == 0) {
    if (!lcdReady) {
      lcd = LiquidCrystal_I2C(0x3F, 16, 2);
      lcd.init();
      lcd.backlight();
      lcdReady = true;
    }
    return true;
  }
  lcdReady = false;
  return false;
}

// --- SOFT RAMP DIEU TOC QUAT EM DIU CHONG SUT AP ---
void setFanSoftPWM(int target) {
  if (currentFanPWM < target) {
    currentFanPWM += 14;
    if (currentFanPWM > target) currentFanPWM = target;
  } else if (currentFanPWM > target) {
    currentFanPWM -= 6;
    if (currentFanPWM < target) currentFanPWM = target;
  }
  analogWrite(PIN_FAN_PWM, currentFanPWM);
}

// --- KHOI TAO SETUP ---
void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 1000);

  pinMode(PIN_MQ135, INPUT);
  pinMode(PIN_FAN_PWM, OUTPUT);
  analogWrite(PIN_FAN_PWM, 0);

  Wire.begin();
  Wire.setWireTimeout(25000, true);
  checkAndInitLCD();
  dht.begin();

  if (checkAndInitLCD()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("NEO MOC 400 PPM "));
    lcd.setCursor(0, 1);
    lcd.print(F("DANG THU THAP..."));
  }

  // Thu thap dien tro phong thuc te trong 2.5s de moc 400ppm nam dung diem dao dong phong
  float sumRs = 0;
  int count = 0;
  for (int i = 0; i < 20; i++) {
    float rawAdc = readLiveADC();
    float vOut = (rawAdc * 5.0) / 1023.0;
    if (vOut > 0.05 && vOut < 4.95) {
      float rs = RL * (5.0 - vOut) / vOut;
      sumRs += rs;
      count++;
    }
    delay(120);
  }
  
  float rsRoom = (count > 0) ? (sumRs / count) : 90.0;
  float corCal = getMQ135CorrectionFactor(T_CAL, H_CAL);
  R0_BASE = rsRoom / corCal;

  if (lcdReady) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("HE THONG SAN SANG"));
    lcd.setCursor(0, 1);
    lcd.print(F("DAO DONG LIVE"));
    delay(1000);
    lcd.clear();
  }

  // Test nhe quat 0.8 giay luc khoi dong
  analogWrite(PIN_FAN_PWM, 130);
  delay(800);
  analogWrite(PIN_FAN_PWM, 0);
}

// --- VONG LAP CHINH ---
void loop() {
  unsigned long currentMillis = millis();

  // 1. Doc DHT11 dinh ky 2 giay
  if (currentMillis - lastDhtTime >= 2000) {
    lastDhtTime = currentMillis;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(t) && !isnan(h) && t > 0 && t < 70 && h > 5 && h <= 100) {
      temperature = t + DHT_TEMP_OFFSET;
      humidity = h + DHT_HUMI_OFFSET;
      dhtOk = true;
      dhtErrorCount = 0;
      dhtHeartbeat = !dhtHeartbeat; // Dao nhip tim bieu thi giao tiep DHT11 tot
    } else {
      dhtErrorCount++;
      if (dhtErrorCount >= 2) {
        dhtOk = false;
      }
    }
  }

  // 2. Doc tin hieu ADC song dong thoi gian thuc
  float rawAdc = readLiveADC();
  float vOut = (rawAdc * 5.0) / 1023.0;
  if (vOut < 0.01) vOut = 0.01;
  if (vOut > 4.98) vOut = 4.98;

  float rsRaw = RL * (5.0 - vOut) / vOut;

  // 3. Bu tru nhiet do va do am Hanwei
  float cor = getMQ135CorrectionFactor(temperature, humidity);
  float rsComp = rsRaw / cor;

  // 4. Tinh nong do CO2 live truc tiep: KHONG KEP CUNG GIA TRI DE DO THI DAO DONG LIEN TUC
  float ratio = rsComp / R0_BASE;
  float livePpm = 400.0 * pow(ratio, CO2_EXPONENT);

  // Chi kẹp giới hạn an toàn vật lý, tuyệt đối không ghim cứng ở 300 hay 400
  if (livePpm < 150.0) livePpm = 150.0;
  if (livePpm > 9999.0) livePpm = 9999.0;

  int ppmInt = (int)round(livePpm);

  // 5. Phan cap chat luong khong khi voi Schmitt Trigger
  // Nguong phan loai:
  // < 650: TOT (Quat tat)
  // 650 - 950: TB (Quat chay em 35% -> 55%)
  // 950 - 1300: CAO (Quat chay manh 60% -> 80%)
  // >= 1300: NGUY (Quat full 100% 255 PWM)
  if (ppmInt >= 1300) {
    currentLevel = 3;
  } else if (ppmInt >= 950) {
    if (currentLevel < 2 || ppmInt < 1300 - HYST_PPM) currentLevel = 2;
  } else if (ppmInt >= 650) {
    if (currentLevel < 1 || ppmInt < 950 - HYST_PPM) currentLevel = 1;
  } else {
    if (ppmInt < 650 - HYST_PPM || currentLevel == 0) currentLevel = 0;
  }

  // 6. Tinh toan cong suat quat PWM (0 -> 255)
  String statusShort = "";
  int targetFanPWM = 0;

  if (currentLevel == 0) {
    targetFanPWM = 0;
    statusShort = "TOT";
  } 
  else if (currentLevel == 1) {
    int clamped = constrain(ppmInt, 650, 950);
    targetFanPWM = map(clamped, 650, 950, 90, 145);
    statusShort = "TB";
  } 
  else if (currentLevel == 2) {
    int clamped = constrain(ppmInt, 950, 1300);
    targetFanPWM = map(clamped, 950, 1300, 150, 210);
    statusShort = "CAO";
  } 
  else {
    targetFanPWM = PWM_MAX_FAN;
    statusShort = "NGUY";
  }

  setFanSoftPWM(targetFanPWM);
  int fanPercent = (currentFanPWM * 100) / PWM_MAX_FAN;
  if (fanPercent > 100) fanPercent = 100;

  // 7. Dinh dang chuan cho Arduino Serial Plotter (Ve duong song dao dong song dong)
  Serial.print("CO2_ppm:");    Serial.print(ppmInt);
  Serial.print(" ");
  Serial.print("Quat_PWM:");   Serial.print(currentFanPWM);
  Serial.print(" ");
  Serial.print("ADC_Raw:");    Serial.print(rawAdc, 1);
  Serial.print(" ");
  Serial.print("NhietDo_C:");  Serial.print(temperature, 1);
  Serial.print(" ");
  Serial.print("DoAm_pct:");   Serial.print(humidity, 1);
  Serial.print(" ");
  Serial.print("DHT_Live:");   Serial.println(dhtOk ? 100 : 0);

  // 8. Hien thi truc quan len LCD 1602
  if (lcdReady || checkAndInitLCD()) {
    char line1[17];
    snprintf(line1, sizeof(line1), "CO2:%-5d [%s]", ppmInt, statusShort.c_str());
    int len1 = strlen(line1);
    while (len1 < 16) line1[len1++] = ' ';
    line1[16] = '\0';
    lcd.setCursor(0, 0);
    lcd.print(line1);

    char line2[17];
    if (dhtOk) {
      snprintf(line2, sizeof(line2), "%2dC %2d%%%c Q:%3d%%", 
               (int)round(temperature), (int)round(humidity), (dhtHeartbeat ? '*' : ' '), fanPercent);
    } else {
      snprintf(line2, sizeof(line2), "DHT:LOI!  Q:%3d%%", fanPercent);
    }
    int len2 = strlen(line2);
    while (len2 < 16) line2[len2++] = ' ';
    line2[16] = '\0';
    lcd.setCursor(0, 1);
    lcd.print(line2);
  }

  delay(120); // Toc do ve ~8 mau/giay giup do thi Serial Plotter dao dong uon luon song dong
}
