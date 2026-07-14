#include <Motoron.h>
#include <Wire.h>
#include <math.h>
// ===================================================================
// SPRZĘT
// ===================================================================
MotoronI2C mc(0x10);
const uint8_t MPU_ADDR = 0x68;
// ===================================================================
// STROJENIE PID
// ===================================================================
float setpoint = 0.0f;  // Auto-kalibrowany w setup()! Nie musisz ustawiać ręcznie.
// ===================================================================
// PID — agresywne wartości, bo przy niskich robot tylko jedzie a nie balansuje!
//   Kp za małe  → robot jedzie w kierunku upadku zamiast się prostować
//   Kp za duże  → robot drży/oscyluje szybko
//   Kd za małe  → robot oscyluje (przeskakuje pion)
//   Kd za duże  → silniki szarpią, robot jest "nerwowy"
// ===================================================================
float Kp = 55.0f;       // DUŻO wyższe — nawet 1° musi dać silną reakcję
float Ki = 0.0f;        // ZOSTAW na 0 dopóki robot nie zacznie balansować
float Kd = 2.5f;        // Silne tłumienie — zapobiega przelatywaniu przez pion
// Pełna moc dostępna — silniki potrzebują gwałtownego przyspieszenia
// żeby OBRÓCIĆ ciało robota, a nie tylko je przesunąć!
const float MOC_MAX = 800.0f;
// ===================================================================
// KOMPENSACJA MARTWEJ STREFY SILNIKÓW
// Silniki DC potrzebują minimalnego napięcia żeby w ogóle ruszyć.
// Bez tego: mały błąd → mały PID → za mało napięcia → silnik stoi → robot pada
// Z tym: nawet mały błąd → silnik ZAWSZE się kręci → robot "drży" w pionie ✓
// ===================================================================
const int DEAD_ZONE = 80;  // Minimalna prędkość silnika (przetestuj 60-120)
// ===================================================================
// STAŁE FILTRA I CZUJNIKA
// ===================================================================
const float ALPHA      = 0.98f;
const float GYRO_SCALE = 131.0f;
// ===================================================================
// ZMIENNE STANU
// ===================================================================
bool  fazaAktywna   = false;
float katFilt       = 0.0f;
float sumaBladu     = 0.0f;
float poprzedniBlad = 0.0f;
float gyroOffsetX   = 0.0f;
float gyroOffsetY   = 0.0f;
float gyroOffsetZ   = 0.0f;
unsigned long poprzedniCzas = 0;
// ===================================================================
// Indeksy osi — ustawiane automatycznie na podstawie orientacji MPU
//   0 = oś X jest pionowa (tak jest u Ciebie!)
//   1 = oś Y jest pionowa
//   2 = oś Z jest pionowa (typowe gdy MPU jest pionowo)
// ===================================================================
int verticalAxis = -1;  // Wykrywany automatycznie
// ===================================================================
// ODCZYT SUROWYCH DANYCH z MPU-6050 (wszystkie 6 osi)
// ===================================================================
struct MPUData {
  int16_t accX, accY, accZ;
  int16_t gyroX, gyroY, gyroZ;
  bool ok;
};
MPUData czytajMPU() {
  MPUData d;
  d.ok = false;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);
  if (Wire.available() < 14) return d;
  d.accX  = (int16_t)(Wire.read() << 8 | Wire.read());
  d.accY  = (int16_t)(Wire.read() << 8 | Wire.read());
  d.accZ  = (int16_t)(Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read();  // temperatura — pomijamy
  d.gyroX = (int16_t)(Wire.read() << 8 | Wire.read());
  d.gyroY = (int16_t)(Wire.read() << 8 | Wire.read());
  d.gyroZ = (int16_t)(Wire.read() << 8 | Wire.read());
  d.ok = true;
  return d;
}
// ===================================================================
// KALIBRACJA — żyroskop (offset) + wykrywanie orientacji MPU
// Robot musi stać PIONOWO i NIERUCHOMO!
// ===================================================================
void kalibracja() {
  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F("  KALIBRACJA — trzymaj robota PIONOWO"));
  Serial.println(F("  i NIERUCHOMO przez 3 sekundy!"));
  Serial.println(F("========================================"));
  delay(500);  // Chwila na ustabilizowanie
  long sumGX = 0, sumGY = 0, sumGZ = 0;
  long sumAX = 0, sumAY = 0, sumAZ = 0;
  const int N = 500;
  for (int i = 0; i < N; i++) {
    MPUData d = czytajMPU();
    if (!d.ok) { i--; continue; }
    sumAX += d.accX;  sumAY += d.accY;  sumAZ += d.accZ;
    sumGX += d.gyroX; sumGY += d.gyroY; sumGZ += d.gyroZ;
    delay(4);
  }
  // Offset żyroskopu
  gyroOffsetX = (float)sumGX / N;
  gyroOffsetY = (float)sumGY / N;
  gyroOffsetZ = (float)sumGZ / N;
  // Średni wektor grawitacji
  float avgAX = (float)sumAX / N;
  float avgAY = (float)sumAY / N;
  float avgAZ = (float)sumAZ / N;
  Serial.println(F("\n--- Sredni wektor grawitacji ---"));
  Serial.print(F("  accX: ")); Serial.println(avgAX, 0);
  Serial.print(F("  accY: ")); Serial.println(avgAY, 0);
  Serial.print(F("  accZ: ")); Serial.println(avgAZ, 0);
  // Wykryj która oś jest PIONOWA (ma największą wartość bezwzględną)
  float absAX = fabsf(avgAX);
  float absAY = fabsf(avgAY);
  float absAZ = fabsf(avgAZ);
  if (absAX >= absAY && absAX >= absAZ) {
    verticalAxis = 0;
    Serial.println(F("  → Os PIONOWA: X"));
  } else if (absAY >= absAX && absAY >= absAZ) {
    verticalAxis = 1;
    Serial.println(F("  → Os PIONOWA: Y"));
  } else {
    verticalAxis = 2;
    Serial.println(F("  → Os PIONOWA: Z"));
  }
  // Oblicz kąt startowy (setpoint) — powinien być bliski 0°
  // Używamy atan2 z osiami prostopadłymi do osi pionowej
  float katStart = obliczKat(avgAX, avgAY, avgAZ);
  setpoint = katStart;
  Serial.print(F("\n  Setpoint (kat przy pionie): "));
  Serial.print(setpoint, 2);
  Serial.println(F(" stopni"));
  Serial.print(F("  Gyro offset X: ")); Serial.println(gyroOffsetX, 1);
  Serial.print(F("  Gyro offset Y: ")); Serial.println(gyroOffsetY, 1);
  Serial.print(F("  Gyro offset Z: ")); Serial.println(gyroOffsetZ, 1);
  Serial.println(F("========================================\n"));
}
// ===================================================================
// OBLICZ KĄT POCHYLENIA [°] — automatycznie na podstawie osi pionowej
// ===================================================================
float obliczKat(float ax, float ay, float az) {
  switch (verticalAxis) {
    case 0:  // X jest pionowa → kąt = odchylenie od X w płaszczyźnie X-Z
      return atan2f(az, ax) * RAD_TO_DEG;
    case 1:  // Y jest pionowa → kąt = odchylenie od Y w płaszczyźnie Y-Z
      return atan2f(az, ay) * RAD_TO_DEG;
    case 2:  // Z jest pionowa (klasyczny montaż) → kąt = odchylenie od Z w płaszczyźnie X-Z
    default:
      return atan2f(ax, az) * RAD_TO_DEG;
  }
}
// ===================================================================
// PRĘDKOŚĆ KĄTOWA POCHYLENIA [°/s] — wybiera właściwą oś żyroskopu
// ===================================================================
float obliczGyro(int16_t gx, int16_t gy, int16_t gz) {
  float raw;
  float offset;
  switch (verticalAxis) {
    case 0:  // X pionowa, pochylenie w płaszczyźnie X-Z → rotacja wokół Y
      raw = (float)gy;
      offset = gyroOffsetY;
      break;
    case 1:  // Y pionowa, pochylenie w płaszczyźnie Y-Z → rotacja wokół X
      raw = (float)gx;
      offset = gyroOffsetX;
      break;
    case 2:  // Z pionowa, pochylenie w płaszczyźnie X-Z → rotacja wokół Y
    default:
      raw = (float)gy;
      offset = gyroOffsetY;
      break;
  }
  return (raw - offset) / GYRO_SCALE;
}
// ===================================================================
// SETUP
// ===================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);       // 400 kHz I2C — 4x szybciej niż domyślne
  Wire.setWireTimeout(3000, true);
  delay(1000);
  // --- Motoron ---
  mc.reinitialize();
  mc.clearResetFlag();
  mc.setCommandTimeoutMilliseconds(200);
  // KRYTYCZNE: wyłącz limity przyspieszenia → natychmiastowa reakcja silników
  mc.setMaxAcceleration(1, 0);
  mc.setMaxDeceleration(1, 0);
  mc.setMaxAcceleration(2, 0);
  mc.setMaxDeceleration(2, 0);
  // --- MPU-6050: wybudzenie ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(150);
  // --- Sample rate: 500 Hz ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19);
  Wire.write(0x01);
  Wire.endTransmission();
  // --- DLPF: 94 Hz, ~3 ms delay ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x02);
  Wire.endTransmission();
  // --- Żyroskop: ±250°/s ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
  // --- Akcelerometr: ±2g ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
  // --- Kalibracja: żyroskop + wykrywanie orientacji + setpoint ---
  kalibracja();
  poprzedniCzas = micros();
  Serial.println(F("Gotowy! Postaw robota pionowo i pusc."));
  Serial.println(F(""));
  Serial.println(F("DIAGNOSTYKA — przechyl robota W PRZOD i sprawdz:"));
  Serial.println(F("  1. Kat — powinien ROSNAC (stawac sie bardziej dodatni)"));
  Serial.println(F("  2. Gyr — powinien byc DODATNI"));
  Serial.println(F("  3. Kola — powinny jechac DO PRZODU (pod robota)"));
  Serial.println(F("Jesli ktorykolwiek jest odwrotnie — powiedz mi!\n"));
}
// ===================================================================
// LOOP
// ===================================================================
void loop() {
  // ---- dt ----
  unsigned long teraz = micros();
  float dt = (teraz - poprzedniCzas) / 1000000.0f;
  poprzedniCzas = teraz;
  if (dt < 0.0002f || dt > 0.05f) return;
  // ---- Odczyt MPU ----
  MPUData d = czytajMPU();
  if (!d.ok) return;
  // ---- Kąt z akcelerometru [°] ----
  float katAccel = obliczKat((float)d.accX, (float)d.accY, (float)d.accZ);
  // ---- Prędkość kątowa [°/s] ----
  float predkoscKat = obliczGyro(d.gyroX, d.gyroY, d.gyroZ);
  // Gyro ma odwrócony znak względem kąta — korekcja:
  predkoscKat = -predkoscKat;
  // ---- Filtr komplementarny ----
  if (!fazaAktywna) {
    katFilt = katAccel;
  } else {
    katFilt = ALPHA * (katFilt + predkoscKat * dt) + (1.0f - ALPHA) * katAccel;
  }
  float blad = setpoint - katFilt;
  // ---- Maszyna stanów ----
  if (fabsf(blad) > 35.0f) {
    if (fazaAktywna) {
      Serial.println(F("UPADEK! Silniki OFF."));
      sumaBladu = 0.0f;
    }
    fazaAktywna = false;
    mc.setSpeed(1, 0);
    mc.setSpeed(2, 0);
    return;
  }
  if (!fazaAktywna && fabsf(blad) < 5.0f) {
    // Zwiększony próg z 3° do 5° — łatwiej uruchomić
    fazaAktywna = true;
    poprzedniBlad = blad;
    sumaBladu = 0.0f;
    katFilt = katAccel;  // Resetuj filtr do czystego odczytu
    Serial.println(F("PION! Start silnikow."));
  }
  // ---- PID ----
  if (fazaAktywna) {
    float P = Kp * blad;
    sumaBladu = constrain(sumaBladu + blad * dt, -100.0f, 100.0f);
    float I = Ki * sumaBladu;
    // D: wprost z żyroskopu (derivative on measurement)
    float D = -Kd * predkoscKat;
    int moc = (int)constrain(P + I + D, -MOC_MAX, MOC_MAX);
    // *** KOMPENSACJA MARTWEJ STREFY ***
    // Jeśli PID chce kręcić silnikiem ale wartość jest za mała
    // żeby pokonać tarcie → wymuś minimum. To daje "drżenie" w pionie.
    if (moc > 0 && moc < DEAD_ZONE) moc = DEAD_ZONE;
    if (moc < 0 && moc > -DEAD_ZONE) moc = -DEAD_ZONE;
    // *** JEŚLI KOŁA JADĄ W ZŁYM KIERUNKU → zamień znaki: ***
    // mc.setSpeed(1, moc);  mc.setSpeed(2, -moc);
    mc.setSpeed(1, -moc);
    mc.setSpeed(2,  moc);
  }
  // ---- Logi co 80 ms ----
  static unsigned long ostatniLog = 0;
  if (millis() - ostatniLog >= 80) {
    Serial.print(F("Kat:"));    Serial.print(katFilt, 1);
    Serial.print(F("\tBlad:"));  Serial.print(blad, 1);
    Serial.print(F("\tGyr:"));   Serial.print(predkoscKat, 1);
    Serial.print(F("\tP:"));     Serial.print(Kp * blad, 0);
    Serial.print(F("\tD:"));     Serial.print(-Kd * predkoscKat, 0);
    Serial.print(F("\tdt:"));    Serial.print(dt * 1000.0f, 1);
    Serial.print(F("\tAkt:"));   Serial.println(fazaAktywna ? F("TAK") : F("NIE"));
    ostatniLog = millis();
  }
}
