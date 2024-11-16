#include <Wire.h>
#include <SoftwareSerial.h>

const int MPU = 0x68; //MPU6050 I2C address
float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime, logTime;
float alertThreshold = 40;
int c = 0;

//Make SoftwareSerial object in pin GPIO17(TX) dan GPIO16 (RX)
SoftwareSerial telemetrySerial(16, 17); // RX, TX

void setup() {
  telemetrySerial.begin(57600);  
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);    
  Wire.write(0x00);    
  Wire.endTransmission(true);
  calculate_IMU_error();
  delay(20);
}

void loop() {
  // === Read accelerometer data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  AccX = (int16_t)(Wire.read() << 8 | Wire.read()); //Read as int16_t then cast to float
  AccY = (int16_t)(Wire.read() << 8 | Wire.read());
  AccZ = (int16_t)(Wire.read() << 8 | Wire.read());

  //Convert to g-force
  AccX = AccX / 16384.0;
  AccY = AccY / 16384.0;
  AccZ = AccZ / 16384.0;

  if ((AccX != 0) && (AccY != 0) && (AccZ != 0)) {
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + AccErrorY;
  } else {
    accAngleX = 0;
    accAngleY = 0;
  }

  // === Read gyroscope data === //
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  GyroX = (int16_t)(Wire.read() << 8 | Wire.read());
  GyroY = (int16_t)(Wire.read() << 8 | Wire.read());
  GyroZ = (int16_t)(Wire.read() << 8 | Wire.read());

  //Convert to deg/s
  GyroX = GyroX / 131.0;
  GyroY = GyroY / 131.0;
  GyroZ = GyroZ / 131.0;

  //Correct the readings with error values
  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;

  //Calculate the gyro angles
  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;

  //Complementary filter
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  //Reset roll, pitch, and yaw if they exceed 180 degrees
  if (roll > 180.0) {
    roll -= 360.0;
  } else if (roll < -180.0) {
    roll += 360.0;
  }

  if (pitch > 180.0) {
    pitch -= 360.0;
  } else if (pitch < -180.0) {
    pitch += 360.0;
  }

  if (yaw > 180.0) {
    yaw -= 360.0;
  } else if (yaw < -180.0) {
    yaw += 360.0;
  }

  //Showing logTime in second
  logTime = currentTime/1000;

  //Print values to Serial Monitor
  telemetrySerial.print(logTime);
  telemetrySerial.print("\tRoll: ");
  telemetrySerial.print(roll);
  telemetrySerial.print("\tPitch: ");
  telemetrySerial.print(pitch);
  telemetrySerial.print("\tYaw: ");
  telemetrySerial.print(yaw);
  telemetrySerial.print("\tGyroX: ");
  telemetrySerial.print(GyroX);
  telemetrySerial.print("\tGyroY: ");
  telemetrySerial.print(GyroY);
  telemetrySerial.print("\tGyroZ: ");

  //Sending alert if roll exceeds threshold
  if (abs(roll) > alertThreshold) {
    telemetrySerial.print(GyroZ);
    telemetrySerial.print("\tALERT: Roll angle exceeds ");
    telemetrySerial.print(alertThreshold);
    telemetrySerial.println(" degrees!");
  }

  else{
    telemetrySerial.println(GyroZ);
  }

  //Checking if there are input to change threshold
  if (telemetrySerial.available()) {
    char input = telemetrySerial.read();
    if (input == 'w') {
      alertThreshold += 5;
      if (alertThreshold > 90){ 
      alertThreshold = 90; //Maximal
      telemetrySerial.print("Threshold is at maximum: ");
      telemetrySerial.println(alertThreshold);
      }
      else{
      telemetrySerial.print("Threshold increased to: ");
      telemetrySerial.println(alertThreshold);
      }
    } 
    else if (input == 's') {
      alertThreshold -= 5;
      if (alertThreshold < 5){ 
      alertThreshold = 5; //Minimal
      telemetrySerial.print("Threshold is at minimum: ");
      telemetrySerial.println(alertThreshold);
      }
      else{
      telemetrySerial.print("Threshold decreased to: ");
      telemetrySerial.println(alertThreshold);
      }
    }
  }
}

void calculate_IMU_error() {
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    AccX = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;

    AccErrorX += (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
    AccErrorY += (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);
    c++;
  }
  AccErrorX /= 200;
  AccErrorY /= 200;
  c = 0;

  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    GyroX = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
    GyroY = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;

    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    c++;
  }
  GyroErrorX /= 200;
  GyroErrorY /= 200;
  GyroErrorZ /= 200;
}