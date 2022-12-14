//Kp = 6.28 Kd = 106.76 voltage = 10.51 V pwm = 200 


const double Kp = 6.28;   // Kp value that you have to change
const double Kd = 106.76;   // Kd value that you have to change
const int setPoint = 35;    // Middle point of sensor array
const int baseSpeed = 212;    // Base speed for your motors
const int maxSpeed = 255;   // Maximum speed for your motors

const byte rx = 0;    // Defining pin 0 as Rx
const byte tx = 1;    // Defining pin 1 as Tx
const byte serialEn = 2;    // Connect UART output enable of LSA08 to pin 2
const byte junctionPulse = 3;   // Connect JPULSE of LSA08 to pin 9
//const byte dir1 = 13;   // Connect DIR1 of motor driver to pin 13
//const byte dir2 = 12;   // Connect DIR2 of motor driver to pin  12
//const byte r = 11;   // Connect r of motor driver to pin 11
//const byte l = 10;   // Connect l of motor driver to pin 10

int lmf = 4;
int lmb = 5;
int rmf = 8;
int rmb = 7;
int r = 6;
int l = 9;
void setup() {
  pinMode(serialEn,OUTPUT);   // Setting serialEn as digital output pin
  pinMode(junctionPulse,INPUT);   // Setting junctionPulse as digital input pin

  // Setting pin 10 - 13 as digital output pin
  for(int i=3;i<=8;i++) {
    pinMode(i,OUTPUT);
  }

  // Setting initial condition of serialEn pin to HIGH
  digitalWrite(serialEn,HIGH);

  // Setting the initial condition of motors
  // make sure both PWM pins are LOW
  digitalWrite(r,LOW);
  digitalWrite(l,LOW);

  // State of DIR pins are depending on your physical connection
  // if your robot behaves strangely, try changing thses two values
  digitalWrite(rmf,HIGH);
  digitalWrite(lmf,HIGH);
  digitalWrite(rmb,LOW);
  digitalWrite(lmb,LOW);

  // Begin serial communication with baudrate 9600
  Serial.begin(230400);

}

int lastError = 0;    // Declare a variable to store previous error

void loop() {
  digitalWrite(serialEn,LOW);   // Set serialEN to LOW to request UART data
  while(Serial.available() <= 0);   // Wait for data to be available
  int positionVal = Serial.read();    // Read incoming data and store in variable positionVal
  Serial.println(positionVal);
  digitalWrite(serialEn,HIGH);    // Stop requesting for UART data

  // If no line is detected, stay at the position
  if(positionVal == 255) {
    analogWrite(r,0);
    analogWrite(l,0);
  }

  // Else if line detected, calculate the motor speed and apply
  else {
    int error = positionVal - setPoint;   // Calculate the deviation from position to the set point 
    int motorSpeed = Kp * error +Kd * (error - lastError);   // Applying formula of PID
    lastError = error;    // Store current error as previous error for next iteration use

    // Adjust the motor speed based on calculated value
    // You might need to interchange the + and - sign if your robot move in opposite direction
    int rightMotorSpeed = baseSpeed - motorSpeed;
    int leftMotorSpeed = baseSpeed + motorSpeed;
   // Serial.println("R= ");
   // Serial.println(rightMotorSpeed);
   // Serial.println("L= ");
   // Serial.println(leftMotorSpeed);
    // If the speed of motor exceed max speed, set the speed to max speed
    if(rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
    if(leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;

    // If the speed of motor is negative, set it to 0
    if(rightMotorSpeed < 0) rightMotorSpeed = 0;
    if(leftMotorSpeed < 0) leftMotorSpeed = 0;

    // Writing the motor speed value as output to hardware motor
    analogWrite(r,rightMotorSpeed);
    analogWrite(l,leftMotorSpeed);
  }

}
