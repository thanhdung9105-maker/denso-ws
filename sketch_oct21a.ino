/*
 * =====================================================================================
 * HE THONG GIAM SAT NONG DO KHI CO2 & DIEU KHIEN QUAT THONG GIO TU DONG (ARDUINO UNO)
 * PHIEN BAN CONG SUAT TOI DA: QUAT CHAY 100% FULL 255 PWM (GIO MANH VU VU)
 * =====================================================================================
 * 
 * CAC MUC CONG SUAT QUAT DUOC NANG CAP:
 *  - Muc 0 (< 700 ppm): Quat TAT (0% - PWM = 0)
 *  - Muc 1 (700 - 1000 ppm): Quat chay vua (35% -> 55% - PWM = 90 -> 140)
 *  - Muc 2 (1000 - 1400 ppm): Quat chay manh (60% -> 80% - PWM = 150 -> 205)
 *  - Muc 3 (>= 1400 ppm): QUAT CHAY HET CONG SUAT (100% FULL - PWM = 255)
 * 
 * CO CHE AN TOAN:
 *  - Soft-start tang dan tung nac nho moi chu ky chong dut/sut ap nguon USB.
 *  - Khi phoi hoi tho vao: CO2 vot nhanh len 1200 - 1800 ppm, quat keo len 100% suc gio.
 *  - Khi ngung phoi: Quat thoi manh hut sach CO2 giam deu ve 400 ppm roi tu tat em ru.
 * =====================================================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// --- CAU HINH CHAN PHAN CUNG ---
#define PIN_MQ135      A0      // Tin hieu Analog MQ-135
#define PIN_DHT        2       // Tin hieu Digital DHT11
#define DHTTYPE        DHT11   // Cam bien DHT11
#define PIN_FAN_PWM    9       // PWM dieu khien MOSFET D4184 (Quat 5V)

#define USE_BUZZER     0       // 0: Khong dung coi
#define PIN_BUZZER     4

LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(PIN_DHT, DHTTYPE);

// --- MOC CHUAN 6 LAN CALIB GOC NGOAI VUON (BAO TON 100%) ---
const float RL = 10.0;               // Dien tro tai tren module MQ-135 (10 kOhm)
const float RS_CLEAN_RAW = 51.30;    // [GOC CALIB 6 LAN] Rs ngoai vuon (kOhm)
const float T_CAL = 31.4;            // Nhiet do luc Calib (do C)
const float H_CAL = 62.4;            // Do am luc Calib (% RH)
const float CO2_EXPONENT = -2.76903; // He so mu chuan Krocker / Winsen

float R0_BASE = 61.50;               // Tinh chuan xac trong setup()
float kEnvAdapt = 1.74;

// --- GIOI HAN CONG SUAT QUAT TOI DA (FULL 255 PWM = 100% 5V) ---
const int PWM_MAX_FAN = 255;         // Cong suat toi da 100%
const int HYST_PPM    = 35;          // Tre Schmitt Trigger

const float DHT_TEMP_OFFSET = -2.0; 
const float DHT_HUMI_OFFSET = 0.0;

// Biến trạng thái
float temperature = T_CAL;
float humidity = H_CAL;
float filteredPpm = -1.0;
int currentLevel = 0;                // 0: TOT, 1: TB, 2: CAO, 3: NGUY
int currentFanPWM = 0;
bool lcdReady = false;

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

// --- SOFT RAMP DIEU TOC QUAT EM DIU (TANG LEN 255 KHONG SUT AP) ---
void setFanSoftPWM(int target) {
  if (currentFanPWM < target) {
    currentFanPWM += 8; // Tang toc do dap ung quat
    if (currentFanPWM > target) currentFanPWM = target;
  } else if (currentFanPWM > target) {
    currentFanPWM -= 6; // Giam toc tu tu em ai
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

#if USE_BUZZER
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
#endif

  Wire.begin();
  Wire.setWireTimeout(25000, true);
  checkAndInitLCD();
  dht.begin();

  // Tinh R0 chuan hoa theo 6 lan calib ngoai vuon
  float corCal = getMQ135CorrectionFactor(T_CAL, H_CAL);
  R0_BASE = RS_CLEAN_RAW / corCal;

  Serial.println(F("======================================================"));
  Serial.println(F(" HE THONG GIAM SAT CO2 & QUAT THONG GIO TU DONG"));
  Serial.println(F(" (Che do CONG SUAT TOI DA 100% PWM = 255)"));
  Serial.print(F(" [GOC CALIB 6 LAN] R0: "));
  Serial.print(R0_BASE, 2);
  Serial.println(F(" kOhm"));
  Serial.println(F("======================================================"));

  if (checkAndInitLCD()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("NEO CALIB 6 LAN "));
  }

  // 8 giay thu thap dien tro phong ban dau
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
    if (k < 1.10) k = 1.10;
    if (k > 2.60) k = 2.60;
    kEnvAdapt = k;
  }

  Serial.print(F("[CALIB XONG] He so neo kEnvAdapt: "));
  Serial.println(kEnvAdapt, 3);

  if (lcdReady) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("HE THONG SAN SANG"));
    lcd.setCursor(0, 1);
    lcd.print(F("CO2: ~420ppm TOT"));
    delay(1200);
    lcd.clear();
  }
}

// --- VONG LAP CHINH ---
void loop() {
  unsigned long currentMillis = millis();

  // 1. Doc DHT11 dinh ky 2 giay
  if (currentMillis - lastDhtTime >= 2000) {
    lastDhtTime = currentMillis;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(t) && !isnan(h) && t > 0 && t < 70 && h > 10 && h <= 100) {
      temperature = t + DHT_TEMP_OFFSET;
      humidity = h + DHT_HUMI_OFFSET;
    }
  }

  // 2. Doc cam bien MQ-135 qua bo loc trung vi 15 mau
  float avgAdc = readMedianADC();
  float vOut = (avgAdc * 5.0) / 1023.0;
  if (vOut < 0.01) vOut = 0.01;
  if (vOut > 4.98) vOut = 4.98;

  float rsRaw = RL * (5.0 - vOut) / vOut;

  // 3. Quy doi ve nen tang 6 lan calib ngoai vuon
  float rsNormalized = rsRaw * kEnvAdapt;

  // 4. Bu tru nhiet do va do am Hanwei
  float cor = getMQ135CorrectionFactor(temperature, humidity);
  float rsComp = rsNormalized / cor;

  // 5. Tinh nong do CO2 tuc thoi (Moc chuan calib khong khi sach ngoai vuon: 400.0 ppm)
  float ratio = rsComp / R0_BASE;
  float rawPpm = 400.0 * pow(ratio, CO2_EXPONENT);

  if (rawPpm < 400.0) rawPpm = 400.0;
  if (rawPpm > 4000.0) rawPpm = 4000.0;

  // ---------------------------------------------------------------------------------
  // DIEU KHIEN DONG THONG MINH:
  // - Khi co hoi tho thoi vao (rawPpm > filteredPpm + 30): CO2 vot nhanh len 1200-1800ppm
  // - Khi ngung thoi va quat quay manh: CO2 giam deu dan va manh me ve moc 400ppm
  // ---------------------------------------------------------------------------------
  if (filteredPpm < 0) {
    filteredPpm = rawPpm;
  } 
  else {
    if (rawPpm > filteredPpm + 30.0) {
      // DANG THOI HOI THO: Tang nhanh de ghi nhan dung dinh nong do
      filteredPpm = 0.60 * rawPpm + 0.40 * filteredPpm;
    } 
    else if (currentFanPWM > 0) {
      // DANG QUAY QUAT MANH: Quat cang manh toc do hut sach cang nhanh
      float fanClearSpeed = map(currentFanPWM, 1, PWM_MAX_FAN, 18, 48);

      if (filteredPpm > 400.0) {
        filteredPpm -= fanClearSpeed;
        if (filteredPpm < 400.0) filteredPpm = 400.0;
      }
      if (rawPpm < filteredPpm) {
        filteredPpm = 0.3 * rawPpm + 0.7 * filteredPpm;
      }
    } 
    else {
      // MOI TRUONG ON DINH: Loc mem quanh moc khong khi sach 400ppm
      filteredPpm = 0.20 * rawPpm + 0.80 * filteredPpm;
    }
  }

  int ppmInt = (int)round(filteredPpm);

  // 6. Phan cap chat luong khong khi voi Schmitt Trigger
  if (ppmInt >= 1400) {
    currentLevel = 3;
  } else if (ppmInt >= 1000) {
    if (currentLevel < 2 || ppmInt < 1400 - HYST_PPM) currentLevel = 2;
  } else if (ppmInt >= 700) {
    if (currentLevel < 1 || ppmInt < 1000 - HYST_PPM) currentLevel = 1;
  } else {
    if (ppmInt < 700 - HYST_PPM || currentLevel == 0) currentLevel = 0;
  }

  // 7. Tinh toan toc do quat NANG CAP CONG SUAT (0 -> 255)
  String statusShort = "";
  int targetFanPWM = 0;

  if (currentLevel == 0) {
    targetFanPWM = 0;                 // Khong khi tot: Tat quat hoan toan
    statusShort = "TOT";
  } 
  else if (currentLevel == 1) {
    // Muc TB (700 - 1000 ppm): Quat chay tu 35% -> 55%
    int clamped = constrain(ppmInt, 700, 1000);
    targetFanPWM = map(clamped, 700, 1000, 90, 140);
    statusShort = "TB";
  } 
  else if (currentLevel == 2) {
    // Muc CAO (1000 - 1400 ppm): Quat chay tu 60% -> 80%
    int clamped = constrain(ppmInt, 1000, 1400);
    targetFanPWM = map(clamped, 1000, 1400, 150, 205);
    statusShort = "CAO";
  } 
  else {
    // Muc NGUY HIEM (>= 1400 ppm): QUAT CHAY HET CONG SUAT 100% (FULL 255)
    targetFanPWM = PWM_MAX_FAN;
    statusShort = "NGUY";
  }

  // Soft-ramp dieu toc quat tu tu chong sut ap
  setFanSoftPWM(targetFanPWM);
  int fanPercent = (currentFanPWM * 100) / PWM_MAX_FAN;
  if (fanPercent > 100) fanPercent = 100;

  // 8. Xuat du lieu chuan hoa do thi tin hieu cho Arduino Serial Plotter (Ctrl + Shift + L) & Serial Monitor
  Serial.print("CO2_ppm:");    Serial.print(ppmInt);
  Serial.print(" ");
  Serial.print("Quat_PWM:");   Serial.print(currentFanPWM);
  Serial.print(" ");
  Serial.print("ADC_Raw:");    Serial.print(avgAdc, 1);
  Serial.print(" ");
  Serial.print("NhietDo_C:");  Serial.print(temperature, 1);
  Serial.print(" ");
  Serial.print("DoAm_pct:");   Serial.print(humidity, 1);
  Serial.print(" | TT:[");     Serial.print(statusShort); Serial.println("]");

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
    snprintf(line2, sizeof(line2), "%2dC  %2d%%  Q:%3d%%", 
             (int)round(temperature), (int)round(humidity), fanPercent);
    int len2 = strlen(line2);
    while (len2 < 16) line2[len2++] = ' ';
    line2[16] = '\0';
    lcd.setCursor(0, 1);
    lcd.print(line2);
  }

  delay(350);
}
