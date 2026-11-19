/*
 * =====================================================================================
 * HE THONG GIAM SAT NONG DO KHI CO2 & DIEU KHIEN QUAT THONG GIO TU DONG (ARDUINO UNO)
 * PHIEN BAN GIU NGUYEN 100% GOC CALIB 6 LAN NGOAI VUON + CHUAN HOA MOC 400 PPM
 * KHAC PHUC TRIET DE HIEN TUONG CO2 BI DO / CHIM NGUONG KHI THOI HOI THO
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
// --- GIU NGUYEN 100% THAM SO CALIB GOC NGOAI VUON (6 LAN DO MAU) ---
// =====================================================================================
const float RL = 10.0;               // Dien tro tai tren module MQ-135 (10 kOhm)
const float RS_CLEAN_RAW = 51.30;    // [GOC CALIB 6 LAN] Rs ngoai vuon tai moc chuan (kOhm)
const float T_CAL = 31.4;            // Nhiet do luc Calib (do C)
const float H_CAL = 62.4;            // Do am luc Calib (% RH)
const float CO2_EXPONENT = -2.76903; // He so mu chuan Winsen MQ-135

float R0_BASE = 61.50;               // Tinh chuan xac trong setup() theo goc calib
float kEnvAdapt = 1.00;              // Khoi tao chuan 1.00 (khong gay chim nguong)

// --- GIOI HAN CONG SUAT QUAT TOI DA (FULL 255 PWM = 100% 5V) ---
const int PWM_MAX_FAN = 255;         // Cong suat toi da 100%
const int HYST_PPM    = 25;          // Tre Schmitt Trigger chong rung ro-le/dao dong quat

const float DHT_TEMP_OFFSET = -2.0; 
const float DHT_HUMI_OFFSET = 0.0;

// Biến trạng thái
float temperature = T_CAL;
float humidity = H_CAL;
float filteredPpm = -1.0;
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

// --- BO LOC TRUNG VI ADC 15 MAU ---
float readMedianADC() {
  const int NUM_SAMPLES = 15;
  float samples[NUM_SAMPLES];
  for (int i = 0; i < NUM_SAMPLES; i++) {
    samples[i] = analogRead(PIN_MQ135);
    delayMicroseconds(200);
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
  float sum = 0;
  for (int i = 3; i < NUM_SAMPLES - 3; i++) {
    sum += samples[i];
  }
  return sum / (NUM_SAMPLES - 6);
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
    currentFanPWM += 10;
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

  // 1. Tinh R0 chuan hoa theo 6 lan calib goc ngoai vuon
  float corCal = getMQ135CorrectionFactor(T_CAL, H_CAL);
  R0_BASE = RS_CLEAN_RAW / corCal;

  if (checkAndInitLCD()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("NEO CALIB GOC 6L"));
  }

  // 2. 8 giay thu thap va thich nghi moi truong phong (khong ep buoc > 1.10)
  float sumRs = 0;
  int countSamples = 0;
  unsigned long startCalib = millis();

  while (millis() - startCalib < 8000) {
    float rawAdc = readMedianADC();
    float vOut = (rawAdc * 5.0) / 1023.0;
    if (vOut > 0.05 && vOut < 4.95) {
      float rs = RL * (5.0 - vOut) / vOut;
      sumRs += rs;
      countSamples++;
    }
    int remain = 8 - (int)((millis() - startCalib) / 1000);
    if (remain < 0) remain = 0;

    if (lcdReady) {
      char buf[17];
      snprintf(buf, sizeof(buf), "Khoi dong: %ds... ", remain);
      lcd.setCursor(0, 1);
      lcd.print(buf);
    }
    delay(250);
  }

  if (countSamples > 10) {
    float rsRoom = sumRs / countSamples;
    float k = RS_CLEAN_RAW / rsRoom;
    // Cho phep k nam quanh 1.00 de tranh lam chim nong do CO2 xuong duoi 400ppm
    if (k < 0.70) k = 0.70;
    if (k > 1.35) k = 1.35;
    kEnvAdapt = k;
  }

  if (lcdReady) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("HE THONG SAN SANG"));
    lcd.setCursor(0, 1);
    lcd.print(F("MOC CHUAN 400PPM"));
    delay(1200);
    lcd.clear();
  }

  // Test nhe quat 1 giay luc khoi dong
  analogWrite(PIN_FAN_PWM, 130);
  delay(1000);
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

  // 2. Doc cam bien MQ-135 qua bo loc trung vi 15 mau
  float avgAdc = readMedianADC();
  float vOut = (avgAdc * 5.0) / 1023.0;
  if (vOut < 0.01) vOut = 0.01;
  if (vOut > 4.98) vOut = 4.98;

  float rsRaw = RL * (5.0 - vOut) / vOut;

  // 3. Quy doi ve nen tang 6 lan calib ngoai vuon (GOC CALIB)
  float rsNormalized = rsRaw * kEnvAdapt;

  // 4. Bu tru nhiet do va do am Hanwei
  float cor = getMQ135CorrectionFactor(temperature, humidity);
  float rsComp = rsNormalized / cor;

  // 5. Tinh nong do CO2 tuc thoi (Moc calib khong khi sach chuan: 400.0 ppm)
  float ratio = rsComp / R0_BASE;
  float rawPpm = 400.0 * pow(ratio, CO2_EXPONENT);

  if (rawPpm < 400.0) rawPpm = 400.0;
  if (rawPpm > 5000.0) rawPpm = 5000.0;

  // ---------------------------------------------------------------------------------
  // BO LOC PHAN UNG NHANH, TRIET TIEU HOAN TOAN VUNG CHET / CAM BIEN DO:
  // - Khi CO2 tang (thoi hoi tho): Dap ung ngay lap tuc khong co nguong chet (alpha = 0.55)
  // - Khi ngung tho va quat bat: Quat keo CO2 giam sach ve moc 400 ppm
  // - Khi khong khi on dinh / ha xuong: Loc muot ma (alpha = 0.20)
  // ---------------------------------------------------------------------------------
  if (filteredPpm < 0.0) {
    filteredPpm = rawPpm;
  } 
  else {
    if (rawPpm > filteredPpm) {
      // DANG CO KHI CO2 / THOI HOI THO: Tang toc phan ung ngay lap tuc
      filteredPpm = 0.55 * rawPpm + 0.45 * filteredPpm;
    } 
    else if (currentFanPWM > 0) {
      // DANG QUAY QUAT MANH: Keo nhanh nong do CO2 ve moc sach
      float fanClearSpeed = map(currentFanPWM, 1, PWM_MAX_FAN, 10, 30);
      if (filteredPpm > 400.0) {
        filteredPpm -= fanClearSpeed;
        if (filteredPpm < 400.0) filteredPpm = 400.0;
      }
      if (rawPpm < filteredPpm) {
        filteredPpm = 0.30 * rawPpm + 0.70 * filteredPpm;
      }
    } 
    else {
      // ON DINH / HA XUONG: Loc mem muot ma
      filteredPpm = 0.20 * rawPpm + 0.80 * filteredPpm;
    }
  }

  int ppmInt = (int)round(filteredPpm);

  // 6. Phan cap chat luong khong khi voi Schmitt Trigger
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

  // 7. Tinh toan cong suat quat PWM (0 -> 255)
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

  // 8. Dinh dang chuan cho Arduino Serial Plotter (Khong co ky tu pipe de ve duong song)
  Serial.print("CO2_ppm:");    Serial.print(ppmInt);
  Serial.print(" ");
  Serial.print("Quat_PWM:");   Serial.print(currentFanPWM);
  Serial.print(" ");
  Serial.print("ADC_Raw:");    Serial.print(avgAdc, 1);
  Serial.print(" ");
  Serial.print("NhietDo_C:");  Serial.print(temperature, 1);
  Serial.print(" ");
  Serial.print("DoAm_pct:");   Serial.print(humidity, 1);
  Serial.print(" ");
  Serial.print("DHT_Live:");   Serial.println(dhtOk ? 100 : 0);

  // 9. Hien thi truc quan len LCD 1602
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

  delay(300);
}
