/*
 * =====================================================================================
 * HE THONG GIAM SAT NONG DO KHI CO2 & DIEU KHIEN QUAT THONG GIO TU DONG (ARDUINO UNO)
 * PHIEN BAN CHUAN HOA DO THI SERIAL PLOTTER & PHAN UNG NHANH VOI HOI THO (MOC 400 PPM)
 * =====================================================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// --- CAU HINH CHAN PHAN CUNG ---
#define PIN_MQ135      A0      // Tin hieu Analog MQ-135 (Cam vao chan AO)
#define PIN_DHT        2       // Tin hieu Digital DHT11
#define DHTTYPE        DHT11   // Cam bien DHT11
#define PIN_FAN_PWM    9       // PWM dieu khien MOSFET D4184 (Quat 5V)

LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(PIN_DHT, DHTTYPE);

// --- MOC CHUAN CALIB NGOAI VUON (400 PPM) ---
const float RL = 10.0;               // Dien tro tai tren module MQ-135 (10 kOhm)
const float RS_CLEAN_RAW = 51.30;    // Rs ngoai vuon tai moc chuan 400 ppm (kOhm)
const float T_CAL = 31.4;            // Nhiet do luc Calib (do C)
const float H_CAL = 62.4;            // Do am luc Calib (% RH)
const float CO2_EXPONENT = -2.76903; // He so mu chuan Winsen MQ-135

const int PWM_MAX_FAN = 255;         // Cong suat toi da 100%
const int HYST_PPM    = 25;          // Tre Schmitt Trigger

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

// --- DIEU TOC QUAT EM DIU ---
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

  if (lcdReady) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("HE THONG CO2"));
    lcd.setCursor(0, 1);
    lcd.print(F("SAN SANG 400PPM"));
    delay(1000);
    lcd.clear();
  }

  // Tu dong quay nhe quat 1.2 giay de test mach MOSFET va dong co
  analogWrite(PIN_FAN_PWM, 140);
  delay(1200);
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
      dhtHeartbeat = !dhtHeartbeat; // Dao nhip tim bieu thi DHT11 dang nhan tin hieu tot
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

  // 3. Bu tru nhiet do va do am Hanwei
  float corCal = getMQ135CorrectionFactor(T_CAL, H_CAL);
  float corNow = getMQ135CorrectionFactor(temperature, humidity);
  float rsComp = rsRaw * (corCal / corNow);

  // 4. Tinh nong do CO2 theo moc chuan ngoai vuon 400.0 ppm
  float ratio = rsComp / RS_CLEAN_RAW;
  float rawPpm = 400.0 * pow(ratio, CO2_EXPONENT);

  if (rawPpm < 400.0) rawPpm = 400.0;
  if (rawPpm > 5000.0) rawPpm = 5000.0;

  // 5. Bo loc thich nghi chong rung dao dong so (Anti-Jitter Adaptive Filter)
  if (filteredPpm < 0.0) {
    filteredPpm = rawPpm;
  } 
  else {
    float diff = rawPpm - filteredPpm;
    float alpha;
    if (diff > 40.0) {
      // Khi thoi hoi tho manh: Bat len tuc thi trong 0.2 giay
      alpha = 0.60;
    } 
    else if (diff < -50.0) {
      // Khi quat dang hut giam nhanh
      alpha = 0.25;
    } 
    else {
      // Khi khong khi on dinh: Loc muot ma on dinh tuyet doi, triet tieu nhap nhay so
      alpha = 0.08;
    }
    filteredPpm = alpha * rawPpm + (1.0 - alpha) * filteredPpm;

    // Khi quat quay manh: ho tro keo CO2 ve moc 400 ppm
    if (currentFanPWM > 0 && filteredPpm > 400.0 && rawPpm <= filteredPpm) {
      float fanClearSpeed = map(currentFanPWM, 1, PWM_MAX_FAN, 6, 20);
      filteredPpm -= fanClearSpeed;
      if (filteredPpm < 400.0) filteredPpm = 400.0;
    }
  }

  int ppmInt = (int)round(filteredPpm);

  // 6. Phan cap chat luong khong khi (Ha nguong de quat bat nhay va de hon)
  if (ppmInt >= 1300) {
    currentLevel = 3;
  } else if (ppmInt >= 900) {
    if (currentLevel < 2 || ppmInt < 1300 - HYST_PPM) currentLevel = 2;
  } else if (ppmInt >= 600) {
    if (currentLevel < 1 || ppmInt < 900 - HYST_PPM) currentLevel = 1;
  } else {
    if (ppmInt < 600 - HYST_PPM || currentLevel == 0) currentLevel = 0;
  }

  // 7. Tinh toan toc do quat NANG CAP CONG SUAT (0 -> 255)
  String statusShort = "";
  int targetFanPWM = 0;

  if (currentLevel == 0) {
    targetFanPWM = 0;                 // Duoi 600 ppm: Khong khi sach, tat quat
    statusShort = "TOT";
  } 
  else if (currentLevel == 1) {
    // 600 - 900 ppm: Quat quay em 40% -> 60%
    int clamped = constrain(ppmInt, 600, 900);
    targetFanPWM = map(clamped, 600, 900, 100, 155);
    statusShort = "TB";
  } 
  else if (currentLevel == 2) {
    // 900 - 1300 ppm: Quat quay manh 60% -> 85%
    int clamped = constrain(ppmInt, 900, 1300);
    targetFanPWM = map(clamped, 900, 1300, 155, 215);
    statusShort = "CAO";
  } 
  else {
    // Tren 1300 ppm: QUAT CHAY HET CONG SUAT 100% (FULL 255)
    targetFanPWM = PWM_MAX_FAN;
    statusShort = "NGUY";
  }

  setFanSoftPWM(targetFanPWM);
  int fanPercent = (currentFanPWM * 100) / PWM_MAX_FAN;
  if (fanPercent > 100) fanPercent = 100;

  // 8. Dinh dang chuan cho Arduino Serial Plotter (Khong chua dau gach dung de ve duong song)
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
