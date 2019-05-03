
  accelgyro.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

  // Convert accelerometer raw values to degrees
  double accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  double accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;

  // Convert raw gyro values to degrees per second
  double gyroXrate = (double)gyroX / 131.0;
  double gyroYrate = -((double)gyroY / 131.0);

  // Calculate angles using complementary filter
  compAngleX.myDouble = (0.93 * (compAngleX.myDouble + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.07 * accXangle);
  compAngleY.myDouble = (0.93 * (compAngleY.myDouble + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.07 * accYangle);

  timer = micros();

  // Write values to serial port
  for (int i=1; i <= 4; i++){
    Serial.write(255);
  }
  Serial.write(compAngleX.myChars, sizeof(double));
  Serial.write(compAngleY.myChars, sizeof(double));