/*  ================================================================================
          GY-521 IMU test code for flipping the etch a sketch - daftmike 2018
    ================================================================================      */

#include <Wire.h>
#include <Servo.h>

// Servo is attached to pin 5
Servo servo;
const int servoPin = 5;

// Variables used for the IMU readings
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
long gyroXCali = 0, gyroYCali = 0, gyroZCali = 0;
long gyroXPresent = 0, gyroYPresent = 0, gyroZPresent = 0;
long gyroXPast = 0, gyroYPast = 0, gyroZPast = 0;
float rotX, rotY, rotZ;
float angleX = 0, angleY = 0, angleZ = 0;
long timePast = 0;
long timePresent = 0;

// Variables used to keep track of the 'flip'
bool flipStarted = 0;
bool flipFinished = 0;
int flips = 0;
int servoDir = 90;

// String to hold the serial data from the Pi
String piMsg;

// Speed, angle and number of 'flips'
const int forwardSpeed = 60;
const int backwardSpeed = 180;
const int numFlips = 5;
const int lowAngle = -140;
const int midAngle = -120;


/*================================================================================*/

void setup() {
  Serial.begin(9600);       // start the serial port
  Wire.begin();             // start i2c
  setUpMPU();               // start IMU coms
  calibrateGyroValues();    
  timePresent = millis();   // time reference for IMU readings
}

/*================================================================================*/

void loop() {
  readAndProcessAccelData();
  readAndProcessGyroData();
  readPi();
}

/*================================================================================*/

void servoFlip() {   
  if (servoDir == 90) {
    servo.attach(servoPin);
    servoDir = forwardSpeed;
    servo.write(servoDir);
    Serial.println("flip started, servo moving forward...");
    flipStarted = 1;
  }

  if (flipStarted == 1) {

    if (servoDir == forwardSpeed && angleX < lowAngle && flips < numFlips) {
      servoDir = backwardSpeed;
      servo.write(servoDir);
      Serial.println("servo backward...");
      flips++;
      Serial.print("flip number: ");
      Serial.println(flips);
    }
    if (servoDir == backwardSpeed && angleX > midAngle && flips < numFlips) {
      servoDir = forwardSpeed;
      servo.write(servoDir);
      Serial.println("servo forward...");
      flips++;
      Serial.print("flip number: ");
      Serial.println(flips);
    }
    if (servoDir == backwardSpeed && -2 < angleX) {
      Serial.println("turning servo OFF...");
      servo.detach();
      servoDir = 90;
      flips = 0;
      flipStarted = 0;
      delay(2000);
      setUpMPU();
      calibrateGyroValues();
    }
  }
}

void readPi() {     // read data from the serial port into a string to check for messages
  
  if (Serial.available()) {
    piMsg = Serial.readString();

    if (piMsg == "flip") {
      servoFlip();
    }
    else if (piMsg == "cali") {
      Serial.println("calibrating gyro values...");
      setUpMPU();
      calibrateGyroValues();
    }
  }
}


void setUpMPU() {
  // power management
  Wire.beginTransmission(0b1101000);          // Start the communication by using address of MPU
  Wire.write(0x6B);                           // Access the power management register
  Wire.write(0b00000000);                     // Set sleep = 0
  Wire.endTransmission();                     // End the communication

  // configure gyro
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);                           // Access the gyro configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();

  // configure accelerometer
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);                           // Access the accelerometer configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();
}

void calibrateGyroValues() {
  for (int i = 0; i < 5000; i++) {
    getGyroValues();
    gyroXCali = gyroXCali + gyroXPresent;
    gyroYCali = gyroYCali + gyroYPresent;
    gyroZCali = gyroZCali + gyroZPresent;
  }
  gyroXCali = gyroXCali / 5000;
  gyroYCali = gyroYCali / 5000;
  gyroZCali = gyroZCali / 5000;
}

void readAndProcessAccelData() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);
  while (Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  processAccelData();
}

void processAccelData() {
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}

void readAndProcessGyroData() {
  gyroXPast = gyroXPresent;                                   // Assign Present gyro reaging to past gyro reading
  gyroYPast = gyroYPresent;                                   // Assign Present gyro reaging to past gyro reading
  gyroZPast = gyroZPresent;                                   // Assign Present gyro reaging to past gyro reading
  timePast = timePresent;                                     // Assign Present time to past time
  timePresent = millis();                                     // get the current time in milli seconds, it is the present time

  getGyroValues();                                            // get gyro readings
  getAngularVelocity();                                       // get angular velocity
  calculateAngle();                                           // calculate the angle
}

void getGyroValues() {
  Wire.beginTransmission(0b1101000);                          // Start the communication by using address of MPU
  Wire.write(0x43);                                           // Access the starting register of gyro readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);                             // Request for 6 bytes from gyro registers (43 - 48)
  while (Wire.available() < 6);                               // Wait untill all 6 bytes are available
  gyroXPresent = Wire.read() << 8 | Wire.read();              // Store first two bytes into gyroXPresent
  gyroYPresent = Wire.read() << 8 | Wire.read();              // Store next two bytes into gyroYPresent
  gyroZPresent = Wire.read() << 8 | Wire.read();              // Store last two bytes into gyroZPresent
}

void getAngularVelocity() {
  rotX = gyroXPresent / 131.0;
  rotY = gyroYPresent / 131.0;
  rotZ = gyroZPresent / 131.0;
}

void calculateAngle() {
  // same equation can be written as
  // angleZ = angleZ + ((timePresentZ - timePastZ)*(gyroZPresent + gyroZPast - 2*gyroZCalli)) / (2*1000*131);
  // 1/(1000*2*131) = 0.00000382
  // 1000 --> convert milli seconds into seconds
  // 2 --> comes when calculation area of trapezium
  // substacted the callibated result two times because there are two gyro readings
  angleX = angleX + ((timePresent - timePast) * (gyroXPresent + gyroXPast - 2 * gyroXCali)) * 0.00000382;
  angleY = angleY + ((timePresent - timePast) * (gyroYPresent + gyroYPast - 2 * gyroYCali)) * 0.00000382;
  angleZ = angleZ + ((timePresent - timePast) * (gyroZPresent + gyroZPast - 2 * gyroZCali)) * 0.00000382;
}

void printData() {
  Serial.print("X : ");
  Serial.print(angleX);
  Serial.print("° | Y : ");
  Serial.print(angleY);
  Serial.print("° | Z : ");
  Serial.print(angleZ); Serial.println("°");

}

