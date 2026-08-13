#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// ==========================================
// 1. НАСТРОЙКИ РАДИОМОДУЛЯ (RX)
// ==========================================
RF24 radio(6, 7);
const uint64_t address = 0xF0F0F0F0E1LL;

// ==========================================
// 2. НАСТРОЙКИ МОТОРА И ШИМ (L298N)
// ==========================================
#define IN1 4
#define IN2 3
#define ENA 9

// ==========================================
// 3. МЕХАНИЧЕСКИЕ ПАРАМЕТРЫ КОНВЕЙЕРА
// ==========================================
const int ENCODER_PIN = 2;       // Пин прерывания
const int SLOTS_PER_REV = 20;    // Число прорезей диска
const float ROLLER_DIAM_CM = 4;  // Диаметр приводного ролика в см
const float GEAR_RATIO = 1.0;    // Коэффициент редуктора

volatile unsigned long pulseCount = 0;

void countPulse() {
  pulseCount++;
}

void setHighPWM_Pin9() {
  TCCR1B = (TCCR1B & 0b11111000) | 0b00000001;
}

void motorBrake() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 255);   
  delay(100);              
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

void moveConveyor(float distance_cm) {
  float circumference = 3.1415926 * ROLLER_DIAM_CM;
  unsigned long targetPulses = (distance_cm / circumference) * (SLOTS_PER_REV * GEAR_RATIO);
  
  Serial.print("Задача: проехать ");
  Serial.print(distance_cm);
  Serial.print(" см. Импульсов: ");
  Serial.println(targetPulses);

  noInterrupts();
  pulseCount = 0;
  interrupts();

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 255);
  delay(50);
  analogWrite(ENA, 195); 

  while (true) {
    noInterrupts();
    unsigned long currentPulses = pulseCount;
    interrupts();

    if (currentPulses >= targetPulses) {
      break;
    }
  }

  motorBrake();
  Serial.println("--- Конвейер прибыл в целевую точку ---");
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  setHighPWM_Pin9();

  pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, RISING);
  motorBrake(); 

  // Инициализация радиомодуля
  if (!radio.begin()) {
    Serial.println("ОШИБКА: Радиомодуль не найден!");
    while (1) {} 
  }
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(115);
  radio.setPayloadSize(32);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, address);
  radio.startListening();

  Serial.println("Система позиционирования конвейера (SLAVE RX) готова.");
}

void loop() {
  // Проверяем эфир на наличие пакетов от мастера-манипулятора
  if (radio.available()) {
    char receivedMessage[32] = {0};
    radio.read(receivedMessage, sizeof(receivedMessage));
    Serial.print("Получена радиокоманда: ");
    Serial.println(receivedMessage);

    String msg = String(receivedMessage);
    
    // Если команда формата "MOVE:20" или "MOVE:15.5"
    if (msg.startsWith("MOVE:")) {
      float dist = msg.substring(5).toFloat();
      if (dist > 0) {
        moveConveyor(dist);
      }
    }
  }
}