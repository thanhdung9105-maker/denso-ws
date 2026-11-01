/*
 * =====================================================================================
 * HE THONG GIAM SAT NONG DO KHI CO2 & DIEU KHIEN QUAT THONG GIO TU DONG (ARDUINO UNO)
 * =====================================================================================
 * Linh kien ket noi:
 *  - Arduino Uno R3 (ATmega328P, 16MHz)
 *  - Cam bien khi MQ-135 (Chan Analog A0)
 *  - Cam bien nhiet do - do am DHT11 (Chan Digital D2)
 *  - Man hinh LCD 1602 giao tiep I2C (SDA -> A4, SCL -> A5)
 *  - Module MOSFET D4184 PWM (Chan PWM/TRIG -> D9, GND -> GND)
 *  - Quat tan nhiet DC 5V khong choi than (Noi vao coc OUT+ va OUT- tren MOSFET)
 *  - Coi chip Buzzer (Chan D4) & He thong 3 LED canh bao (Xanh D5, Vang D6, Do D7)
 * =====================================================================================
 * CAI TIEN KY THUAT QUAN TRONG:
 *  1. Khu nhieu Ground-Bounce & Sut ap 5V khi doc ADC (Synchronous Blank-Out Sampling):
 *     Tam ngat xung PWM quat & coi trong 30ms luc lay mau A0 de tranh dong tai quat
 *     lam dang ap GND / sut ap Vcc khien ADC bi vọt ao va tu khoa cung (Latch-up) o muc NGUY.
 *  2. Hieu chuan 6 moc ngoai vuon (30 phut say nong on dinh): RS_CLEAN_RAW = 51.30 kOhm,
 *     cho ket qua ~410 - 460 ppm trong phong thoang co 1 nguoi ngoi xa.
 *  3. Gioi han cong suat PWM an toan cho cong USB Laptop (PWM_MAX_USB = 160) & coi keu ngat quang
 *     giup tong dong tieu thu luon < 380mA, khong bao gio lam sap cong USB hay chop LCD.
 * =====================================================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// --- CAU HINH CHIEU CHAN KET NOI (PIN CONFIGURATION) ---
#define PIN_MQ135      A0      // Tin hieu Analog cam bien MQ-135
#define PIN_DHT        2       // Tin hieu 1-Wire Digital cam bien DHT11
#define DHTTYPE        DHT11   // Loai cam bien DHT11

#define PIN_BUZZER     4       // Coi bao dong (Buzzer)
#define PIN_LED_GREEN  5       // LED Xanh (Khong khi TOT)
#define PIN_LED_YELLOW 6       // LED Vang  (Khong khi TRUNG BINH)
#define PIN_LED_RED    7       // LED Do    (Khong khi ONHIEM / NGUYHIEM)
#define PIN_FAN_PWM    9       // Xung PWM dieu khien Module MOSFET D4184 (Quat 5V)

// Khoi tao LCD 1602 I2C (Dia chi 0x27, 16 cot, 2 dong)
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(PIN_DHT, DHTTYPE);

bool lcdReady = false;
int i2cFailCount = 0;

// --- CAU HINH THONG SO CALIB CAM BIEN MQ-135 CHUAN DATASHEET ---
const float RL = 10.0;               // Dien tro tai tren module MQ-135 (10 kOhm)
const float RS_CLEAN_RAW = 51.30;    // Dien tro Rs_raw can bang sau 6 moc Calib ngoai vuon (400ppm, ADC~166.1, Vout~0.81V)
const float T_CAL = 31.4;            // Nhiet do tham chieu luc Calib (do C)
const float H_CAL = 62.4;            // Do am tham chieu luc Calib (% RH)
const float CO2_EXPONENT = -2.76903; // He so mu dac tuyen CO2 (Winsen / Krocker Datasheet)

// --- CAU HINH BO LOC EMA, TRE SCHMITT TRIGGER & BAO VE NGUON USB ---
const float EMA_ALPHA_RISE   = 0.45; // He so bam nhanh khi CO2 tang (co hoi tho)
const float EMA_ALPHA_FALL   = 0.35; // He so hoi phuc nhanh khi CO2 giam
const int   HYST_PPM         = 25;   // Do tre Schmitt Trigger chong nhay trang thai (ppm)
const int   PWM_MAX_USB      = 160;  // Gioi han PWM toi da khi dung nguon USB Laptop (<380mA, chong sap COM)

const unsigned long DHT_INTERVAL = 2000; 
const float DHT_TEMP_OFFSET = -3.0;  // +/- do C hieu chinh nhiet buc xa
const float DHT_HUMI_OFFSET = 0.0;   // +/- % RH

float temperature = T_CAL;
float humidity = H_CAL;
float filteredPpm = -1.0;
int currentLevel = 0;                // 0: TOT, 1: TB, 2: CAO, 3: NGUY
int currentFanPWM = 0;               // Xung PWM hien tai
bool buzzerToggle = false;           // Trang thai coi bip ngat quang (tiet kiem dong)
unsigned long lastDhtReadTime = 0;

// Ham tinh he so bu tru Nhiet do - Do am (COR) chuan Datasheet Hanwei / Krocker
float getMQ135CorrectionFactor(float t, float h) {
  float cor = (0.00035 * t * t - 0.02718 * t + 1.39538) - ((h - 33.0) * 0.0018);
  if (cor < 0.35) cor = 0.35;
  if (cor > 2.50) cor = 2.50;
  return cor;
}

bool checkAndInitLCD() {
  Wire.beginTransmission(0x27);
  if (Wire.endTransmission() == 0) {
    i2cFailCount = 0;
    if (!lcdReady) {
      lcd.init();
      lcd.backlight();
      lcdReady = true;
    }
    return true;
  }
  Wire.beginTransmission(0x3F);
  if (Wire.endTransmission() == 0) {
    i2cFailCount = 0;
    if (!lcdReady) {
      lcd.init();
      lcd.backlight();
      lcdReady = true;
    }
    return true;
  }
  i2cFailCount++;
  if (i2cFailCount >= 5) {
    lcdReady = false;
  }
  return lcdReady;
}

// Ham dieu toc quat khoi dong mem (Soft-Start) chong sut ap 5V USB
void setFanSoftPWM(int targetPWM) {
  if (targetPWM > PWM_MAX_USB) targetPWM = PWM_MAX_USB;
  if (targetPWM == 0) {
    currentFanPWM = 0;
    analogWrite(PIN_FAN_PWM, 0);
    return;
  }
  while (currentFanPWM < targetPWM) {
    currentFanPWM += 20;
    if (currentFanPWM > targetPWM) currentFanPWM = targetPWM;
    analogWrite(PIN_FAN_PWM, currentFanPWM);
    delay(20);
  }
  currentFanPWM = targetPWM;
  analogWrite(PIN_FAN_PWM, currentFanPWM);
}

/*
 * Doc ADC voi ky thuat Synchronous Blank-Out:
 * Tam ngat PWM quat & coi trong ~28ms de duong 5V va GND tren breadboard hoi phuc 100%,
 * loai bo hoan toan hien tuong Ground-Bounce lam vọt ADC ao khi quat dang quay!
 */
float readUnloadedTrimmedAdc(int pin) {
  if (currentFanPWM > 0) {
    analogWrite(PIN_FAN_PWM, 0);
    digitalWrite(PIN_BUZZER, LOW);
    delay(15); // Cho dien ap 5V/GND on dinh
  }
  // Doc bo 1 mau dau de xa tu S/H cua ADC
  analogRead(pin);

  long sum = 0;
  int minVal = 1024;
  int maxVal = -1;
  for (int i = 0; i < 12; i++) {
    int val = analogRead(pin);
    sum += val;
    if (val < minVal) minVal = val;
    if (val > maxVal) maxVal = val;
    delay(2);
  }

  // Khoi phuc lai xung PWM quat ngay lap tuc (quat van quay deu nho quan tinh co hoc)
  if (currentFanPWM > 0) {
    analogWrite(PIN_FAN_PWM, currentFanPWM);
  }

  sum -= (minVal + maxVal);
  return sum / 10.0;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setWireTimeout(25000, true); // 25ms timeout an toan cho LCD I2C

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_FAN_PWM, OUTPUT);
  analogWrite(PIN_FAN_PWM, 0);

  dht.begin();

  if (checkAndInitLCD()) {
    lcd.setCursor(0, 0);
    lcd.print(" HE THONG DO CO2");
    lcd.setCursor(0, 1);
    lcd.print(" KHOI DONG MEM..");
    delay(800);
  }

  // Test quat nhe nhang voi Soft-Start (khong gay sut ap 5V USB)
  digitalWrite(PIN_LED_GREEN, HIGH);
  setFanSoftPWM(110);
  delay(500);
  setFanSoftPWM(0);
  digitalWrite(PIN_LED_GREEN, LOW);
}

void loop() {
  bool lcdConnectedNow = checkAndInitLCD();

  // 1. Doc du lieu tu cam bien DHT11
  unsigned long currentMillis = millis();
  if (currentMillis - lastDhtReadTime >= DHT_INTERVAL || lastDhtReadTime == 0) {
    lastDhtReadTime = currentMillis;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      temperature = t + DHT_TEMP_OFFSET;
      humidity = h + DHT_HUMI_OFFSET;
    }
  }

  // 2. Doc tin hieu Analog tu MQ-135 (Synchronous Blank-Out + Trimmed Mean)
  float avgAdc = readUnloadedTrimmedAdc(PIN_MQ135);

  // 3. Tinh dien ap Vout va dien tro Rs_raw
  float vOut = (avgAdc / 1023.0) * 5.0;
  if (vOut <= 0.05) vOut = 0.05;
  if (vOut >= 4.95) vOut = 4.95;
  float rsRaw = ((1023.0 - avgAdc) / avgAdc) * RL;

  // 4. Bu tru nhiet do & do am tuong doi theo diem Calib (T_CAL, H_CAL)
  float corCal = getMQ135CorrectionFactor(T_CAL, H_CAL);
  float corNow = getMQ135CorrectionFactor(temperature, humidity);
  float rsComp = rsRaw * (corCal / corNow);

  // 5. Tinh nong do eCO2 ppm va loc muot EMA
  float ratio = rsComp / RS_CLEAN_RAW;
  float rawPpm = 400.0 * pow(ratio, CO2_EXPONENT);
  if (rawPpm < 400.0)  rawPpm = 400.0;
  if (rawPpm > 5000.0) rawPpm = 5000.0;

  if (filteredPpm < 0.0) {
    filteredPpm = rawPpm;
  } else {
    float alpha = (rawPpm > filteredPpm) ? EMA_ALPHA_RISE : EMA_ALPHA_FALL;
    filteredPpm = alpha * rawPpm + (1.0 - alpha) * filteredPpm;
  }
  int ppmInt = (int)round(filteredPpm);

  // 6. Phan cap chat luong khong khi voi tre Schmitt Trigger
  if (ppmInt >= 1500) {
    currentLevel = 3;
  } else if (ppmInt >= 1200) {
    if (currentLevel < 2 || ppmInt < 1500 - HYST_PPM) currentLevel = 2;
  } else if (ppmInt >= 800) {
    if (currentLevel < 1 || ppmInt < 1200 - HYST_PPM) currentLevel = 1;
  } else {
    if (ppmInt < 800 - HYST_PPM || currentLevel == 0) currentLevel = 0;
  }

  String statusShort = "";
  int targetFanPWM = 0;

  if (currentLevel == 0) {
    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_YELLOW, LOW);
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_BUZZER, LOW);
    targetFanPWM = 0;
    statusShort = "TOT";
  } 
  else if (currentLevel == 1) {
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_YELLOW, HIGH);
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_BUZZER, LOW);
    int clampedPpm = constrain(ppmInt, 800, 1200);
    targetFanPWM = map(clampedPpm, 800, 1200, 75, 115);
    statusShort = "TB";
  } 
  else if (currentLevel == 2) {
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_YELLOW, LOW);
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_BUZZER, LOW);
    int clampedPpm = constrain(ppmInt, 1200, 1500);
    targetFanPWM = map(clampedPpm, 1200, 1500, 120, 145);
    statusShort = "CAO";
  } 
  else {
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_YELLOW, LOW);
    digitalWrite(PIN_LED_RED, HIGH);
    // Coi keu bip-nghi luan phien de giam dong tieu thu va do chói tai
    buzzerToggle = !buzzerToggle;
    digitalWrite(PIN_BUZZER, buzzerToggle ? HIGH : LOW);
    targetFanPWM = PWM_MAX_USB;
    statusShort = "NGUY";
  }

  setFanSoftPWM(targetFanPWM);
  int fanPercent = (targetFanPWM * 100) / PWM_MAX_USB;
  if (fanPercent > 100) fanPercent = 100;

  // 7. Xuat du lieu giam sat len Serial Monitor (toc do 9600 baud)
  Serial.print("ADC: "); Serial.print(avgAdc, 1);
  Serial.print(" | Vout: "); Serial.print(vOut, 2);
  Serial.print("V | Rs: "); Serial.print(rsRaw, 2);
  Serial.print("k | RsComp: "); Serial.print(rsComp, 2);
  Serial.print("k | CO2: "); Serial.print(ppmInt);
  Serial.print("ppm | T: "); Serial.print(temperature, 1);
  Serial.print("C | H: "); Serial.print(humidity, 1);
  Serial.print("% | Quat: "); Serial.print(fanPercent);
  Serial.print("% ["); Serial.print(statusShort); Serial.println("]");

  // 8. Hien thi truc quan len man hinh LCD 1602 (khong dung lcd.clear() de tranh nhap nhay)
  if (lcdConnectedNow) {
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

  delay(300);
}
