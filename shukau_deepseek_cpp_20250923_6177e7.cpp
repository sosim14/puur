#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Servo.h>

// Определяем пины для Speed Bee F405-Wing (используем стандартные пины для этой платы)
#define RC_CH1_PIN PA0  // Канал 1 приемника (Aileron)
#define RC_CH2_PIN PA1  // Канал 2 приемника (Elevator)
#define RC_CH3_PIN PA2  // Канал 3 приемника (Throttle)
#define RC_CH4_PIN PA3  // Канал 4 приемника (Rudder)
#define RC_CH5_PIN PA4  // Канал 5 приемника (Mode switch)

// Выходы для сервоприводов и мотора (используем пины, поддерживающие Hardware PWM)
#define SERVO_AIL_PIN PA8  // Сервопривод элеронов
#define SERVO_ELE_PIN PA9  // Сервопривод руля высоты
#define MOTOR_PIN PA10     // ESC мотор

// Serial порты
#define GPS_SERIAL Serial1  // UART1 для GPS
#define RC_SERIAL Serial2   // UART2 для RC приемника (SBUS)
#define DEBUG_SERIAL Serial // USB Serial для отладки

// Создаем объекты
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
Servo servoAil, servoEle, escMotor;

// Переменные для данных RC
uint16_t rcChannels[16];  // Для хранения значений всех каналов
int rcCh1 = 1500, rcCh2 = 1500, rcCh3 = 1000, rcCh4 = 1500, rcCh5 = 1000;

// Переменные для данных с датчиков
float temperature = 0;
float pressure = 0;
float altitude = 0;

// Переменные для управления
bool isAutoPilotEnabled = false;
bool gpsValid = false;
int motorSpeed = 1000;      // Минимальный газ (1000-2000)
int servoAilValue = 1500;   // Нейтральное положение (1500 мкс)
int servoEleValue = 1500;   // Нейтральное положение (1500 мкс)

// Координаты целевой точки (Автономный режим)
double targetLat = 55.7558;  // Широта Москвы
double targetLon = 37.6173;  // Долгота Москвы

// Таймеры
unsigned long lastRcReadTime = 0;
unsigned long lastSensorReadTime = 0;
unsigned long lastGpsReadTime = 0;

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL) delay(10);  // Ждем инициализации Serial
  
  DEBUG_SERIAL.println("Initializing Speed Bee F405-Wing...");

  // Инициализация RC приемника (SBUS)
  RC_SERIAL.begin(100000, SERIAL_8E2);  // SBUS протокол: 100000 бод, 8 бит, четность, 2 стоп-бита
  
  // Инициализация GPS
  GPS_SERIAL.begin(9600);
  
  // Инициализация BMP280
  if (!bmp.begin(0x76)) {
    DEBUG_SERIAL.println("Could not find BMP280 sensor!");
    while (1) delay(10);
  }
  
  // Настройка BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Режим работы
                  Adafruit_BMP280::SAMPLING_X2,     // Температура
                  Adafruit_BMP280::SAMPLING_X16,    // Давление
                  Adafruit_BMP280::FILTER_X16,      // Фильтр
                  Adafruit_BMP280::STANDBY_MS_500); // Время ожидания

  // Настройка пинов и сервоприводов
  servoAil.attach(SERVO_AIL_PIN);
  servoEle.attach(SERVO_ELE_PIN);
  escMotor.attach(MOTOR_PIN);
  
  // Установка минимального газа
  escMotor.writeMicroseconds(1000);
  
  DEBUG_SERIAL.println("System Initialized! Waiting for GPS signal...");
}

void loop() {
  // 1. Чтение данных с RC приемника
  readRCData();
  
  // 2. Проверка переключения режима (ручной/авто)
  checkModeSwitch();
  
  // 3. Чтение данных с датчиков (каждые 100 мс)
  if (millis() - lastSensorReadTime > 100) {
    readSensors();
    lastSensorReadTime = millis();
  }
  
  // 4. Чтение GPS данных
  readGPS();
  
  // 5. Основная логика управления
  if (isAutoPilotEnabled && gpsValid) {
    runAutopilot();
  } else {
    runManualControl();
  }
  
  // 6. Отправка управляющих сигналов
  writeOutputs();
  
  // 7. Логирование данных (каждые 500 мс)
  static unsigned long lastLogTime = 0;
  if (millis() - lastLogTime > 500) {
    logData();
    lastLogTime = millis();
  }
  
  // Задержка для стабильности
  delay(10);
}

// Чтение данных с RC приемника (SBUS)
void readRCData() {
  static uint8_t sbusData[25];
  static int sbusIndex = 0;
  
  while (RC_SERIAL.available()) {
    uint8_t byte = RC_SERIAL.read();
    
    // Начало SBUS пакета
    if (sbusIndex == 0 && byte != 0x0F) continue;
    
    sbusData[sbusIndex++] = byte;
    
    // Конец SBUS пакета
    if (sbusIndex == 25) {
      sbusIndex = 0;
      
      // Декодирование SBUS данных
      rcChannels[0]  = ((sbusData[1]    | sbusData[2]  << 8) & 0x07FF);
      rcChannels[1]  = ((sbusData[2]>>3 | sbusData[3]  << 5) & 0x07FF);
      rcChannels[2]  = ((sbusData[3]>>6 | sbusData[4]  << 2 | sbusData[5] << 10) & 0x07FF);
      rcChannels[3]  = ((sbusData[5]>>1 | sbusData[6]  << 7) & 0x07FF);
      rcChannels[4]  = ((sbusData[6]>>4 | sbusData[7]  << 4) & 0x07FF);
      rcChannels[5]  = ((sbusData[7]>>7 | sbusData[8]  << 1 | sbusData[9] << 9) & 0x07FF);
      
      // Преобразование значений в диапазон 1000-2000 мкс
      rcCh1 = map(rcChannels[0], 0, 2047, 1000, 2000);
      rcCh2 = map(rcChannels[1], 0, 2047, 1000, 2000);
      rcCh3 = map(rcChannels[2], 0, 2047, 1000, 2000);
      rcCh4 = map(rcChannels[3], 0, 2047, 1000, 2000);
      rcCh5 = map(rcChannels[4], 0, 2047, 1000, 2000);
      
      lastRcReadTime = millis();
    }
  }
  
  // Проверка потери сигнала RC (если нет данных более 500 мс)
  if (millis() - lastRcReadTime > 500) {
    // Аварийный режим - отключаем мотор, устанавливаем нейтральное положение серв
    motorSpeed = 1000;
    servoAilValue = 1500;
    servoEleValue = 1500;
    isAutoPilotEnabled = false;
  }
}

// Проверка переключения режима
void checkModeSwitch() {
  // Если канал 5 (переключатель) выше среднего - авторежим
  isAutoPilotEnabled = (rcCh5 > 1500);
}

// Чтение данных с датчиков
void readSensors() {
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure() / 100.0F; // в гПа
  altitude = bmp.readAltitude(1013.25); // Относительное давление
}

// Чтение GPS данных
void readGPS() {
  while (GPS_SERIAL.available()) {
    char c = GPS_SERIAL.read();
    gps.encode(c);
  }
  
  gpsValid = (gps.location.isValid() && gps.location.age() < 2000);
}

// Ручное управление
void runManualControl() {
  motorSpeed = rcCh3;        // Газ с канала 3
  servoAilValue = rcCh1;     // Элероны с канала 1
  servoEleValue = rcCh2;     // Руль высоты с канала 2
}

// Автономный режим
void runAutopilot() {
  if (!gpsValid) return;
  
  double currentLat = gps.location.lat();
  double currentLon = gps.location.lon();
  double currentAlt = altitude;
  double currentCourse = gps.course.deg();
  
  // Вычисление курса на цель
  double targetCourse = TinyGPSPlus::courseTo(
    currentLat, currentLon, targetLat, targetLon
  );
  
  // Вычисление расстояния до цели
  double distanceToTarget = TinyGPSPlus::distanceBetween(
    currentLat, currentLon, targetLat, targetLon
  );
  
  // Вычисление ошибки курса
  double courseError = targetCourse - currentCourse;
  
  // Нормализация ошибки (-180 до 180)
  if (courseError > 180) courseError -= 360;
  if (courseError < -180) courseError += 360;
  
  // ПИД-регулятор для курса (пропорциональная часть)
  double steerCommand = courseError * 0.5; // Коэффициент нужно настраивать
  
  // Преобразование команды поворота в сигналы для сервоприводов
  servoAilValue = 1500 + constrain(steerCommand, -400, 400);
  servoEleValue = 1500; // Пока держим горизонтальный полет
  
  // Управление тягой в зависимости от высоты (простейший алгоритм)
  motorSpeed = 1300; // Средняя тяга
  
  // Если близко к цели - уменьшаем тягу
  if (distanceToTarget < 50) {
    motorSpeed = 1200;
  }
}

// Отправка сигналов на исполнительные устройства
void writeOutputs() {
  escMotor.writeMicroseconds(motorSpeed);
  servoAil.writeMicroseconds(servoAilValue);
  servoEle.writeMicroseconds(servoEleValue);
}

// Логирование данных
void logData() {
  DEBUG_SERIAL.print("Mode: ");
  DEBUG_SERIAL.print(isAutoPilotEnabled ? "AUTO" : "MANUAL");
  DEBUG_SERIAL.print(" | RC: ");
  DEBUG_SERIAL.print(rcCh5);
  DEBUG_SERIAL.print(" | Temp: ");
  DEBUG_SERIAL.print(temperature);
  DEBUG_SERIAL.print("C | Press: ");
  DEBUG_SERIAL.print(pressure);
  DEBUG_SERIAL.print("hPa | Alt: ");
  DEBUG_SERIAL.print(altitude);
  DEBUG_SERIAL.print("m");
  
  if (gpsValid) {
    DEBUG_SERIAL.print(" | GPS: ");
    DEBUG_SERIAL.print(gps.location.lat(), 6);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(gps.location.lng(), 6);
    DEBUG_SERIAL.print(" | Sats: ");
    DEBUG_SERIAL.print(gps.satellites.value());
    DEBUG_SERIAL.print(" | Course: ");
    DEBUG_SERIAL.print(gps.course.deg());
  } else {
    DEBUG_SERIAL.print(" | GPS: No signal");
  }
  
  DEBUG_SERIAL.print(" | Motor: ");
  DEBUG_SERIAL.print(motorSpeed);
  DEBUG_SERIAL.print(" | Ail: ");
  DEBUG_SERIAL.print(servoAilValue);
  DEBUG_SERIAL.print(" | Ele: ");
  DEBUG_SERIAL.print(servoEleValue);
  
  DEBUG_SERIAL.println();
}