#include <Servo.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>

// Создаем объекты сервоприводов
Servo servo1_base;
Servo servo2_shoulder;
Servo servo3_elbow;
Servo servo4_forearm;
Servo servo5_wrist;
Servo servo6_gripper;

// Создаем объект лазерного датчика и радиомодуля
VL53L0X sensor;

RF24 radio(10, 9); // CE = 10, CSN = 9
const uint64_t address = 0xF0F0F0F0E1LL;

void setup() {
  Serial.begin(115200); 
  Serial.setTimeout(10);
  
  servo1_base.attach(3);
  servo2_shoulder.attach(4);
  servo3_elbow.attach(5);
  servo4_forearm.attach(6);
  servo5_wrist.attach(7);
  servo6_gripper.attach(8);

  Wire.begin();
  sensor.init();
  sensor.startContinuous();

  servo1_base.write(90);
  servo2_shoulder.write(90);
  servo3_elbow.write(90);
  servo4_forearm.write(90);
  servo5_wrist.write(90);
  servo6_gripper.write(45); 

  // Инициализация радио-мастера
  if (!radio.begin()) {
    Serial.println("ОШИБКА: Радиомодуль TX не найден!");
    while (1) {} 
  }
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(115);
  radio.setPayloadSize(32);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.stopListening(); // Мастер работает на передачу
}

void loop() {
  if (Serial.available() > 0) {
    char firstChar = Serial.peek();

    // 1. ЕСЛИ ПРИШЛА РАДИОКОМАНДА (например, "RADIO:MOVE:20\n")
    if (firstChar == 'R') {
      String cmd = Serial.readStringUntil('\n');
      cmd.remove(0, 6); // Удаляем префикс "RADIO:" (6 символов)
      
      char msg[32] = {0};
      cmd.toCharArray(msg, 32);
      
      bool success = radio.write(msg, sizeof(msg));
      if (success) {
        Serial.println("RADIO_TX_OK");
      } else {
        Serial.println("RADIO_TX_FAIL");
      }
    } 
    // 2. ЕСЛИ ПРИШЛИ УГЛЫ ДЛЯ СЕРВОПРИВОДОВ (число или знак минуса)
    else if (isDigit(firstChar) || firstChar == '-') {
      int j1 = Serial.parseInt();
      int j2 = Serial.parseInt();
      int j3 = Serial.parseInt();
      int j4 = Serial.parseInt();
      int j5 = Serial.parseInt();
      int gripper = Serial.parseInt();

      if (Serial.read() == '\n') {
        servo1_base.write(constrain(j1, 0, 180));
        servo2_shoulder.write(constrain(j2, 0, 180));
        servo3_elbow.write(constrain(j3, 0, 180));
        servo4_forearm.write(constrain(j4, 0, 180));
        servo5_wrist.write(constrain(j5, 0, 180));
        servo6_gripper.write(constrain(gripper, 0, 90));
        
        int distance = sensor.readRangeContinuousMillimeters();
        Serial.println(distance);
      }
    } else {
      // Очистка буфера от случайного мусора
      Serial.read();
    }
  }
}