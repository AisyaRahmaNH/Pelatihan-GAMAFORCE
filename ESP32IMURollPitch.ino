#include <Wire.h>

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime, logTime;
float alertThreshold = 40;
int c = 0;

void setup() {
  Serial.begin(19200);
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

  AccX = (int16_t)(Wire.read() << 8 | Wire.read()); // Read as int16_t then cast to float
  AccY = (int16_t)(Wire.read() << 8 | Wire.read());
  AccZ = (int16_t)(Wire.read() << 8 | Wire.read());

  // Convert to g-force (satuan gravitasi)
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

  // Convert to deg/s
  GyroX = GyroX / 131.0;
  GyroY = GyroY / 131.0;
  GyroZ = GyroZ / 131.0;

  // Correct the readings with error values
  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;

  // Calculate the gyro angles
  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;

  // Complementary filter
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  // Reset roll, pitch, and yaw if they exceed 180 degrees
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

    // Mengirim alert jika roll melebihi threshold
  if (abs(roll) > alertThreshold) {
    Serial.print(GyroZ);
    Serial.println("\tALERT: Roll angle exceeds 40 degrees!");
  }

  else{
    Serial.println(GyroZ);
  }

  //Definisi logtime dalam second untuk ditampilkan
  logTime = currentTime/1000;

  // Print values to Serial Monitor
  Serial.print(logTime);
  Serial.print("\tRoll: ");
  Serial.print(roll);
  Serial.print("\tPitch: ");
  Serial.print(pitch);
  Serial.print("\tYaw: ");
  Serial.print(yaw);
  Serial.print("\tGyroX: ");
  Serial.print(GyroX);
  Serial.print("\tGyroY: ");
  Serial.print(GyroY);
  Serial.print("\tGyroZ: ");
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