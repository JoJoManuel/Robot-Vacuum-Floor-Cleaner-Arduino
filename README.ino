//AUVC has mainly two ultrasonic sensors. One of the sensor performs

//The function of obstacle avoiding by comparing the left, right and forward distances, and avoids the closer obstacle path or 
//in other words select less obstacle path, if the robot is covered with full obstacles around, then the robot will turn around.
// The other sensor which avoid edges by measuring depth

//Robot-Vacuum-Floor-Cleaner-Arduino
//AUVC Automatic vacuum cleaning robot with UV germicidal irradiation
//Code developed by  AKHIL JOSEPH ,ADARSH MOHAN, BASIL T ABRAHAM and EDWIN JOHNY
//akhil.vaj@gmail.com
//https://youtu.be/JHIQBQzUffg 

#include <AFMotor.h>      //add Adafruit Motor Shield library
#include <Servo.h>        //add Servo Motor library            
#include <NewPing.h>      //add Ultrasonic sensor library
#define TRIG_PIN A0 // Pin A0 on the Motor Drive Shield soldered to the ultrasonic sensor
#define ECHO_PIN A1 // Pin A1 on the Motor Drive Shield soldered to the ultrasonic sensor
#define trigPin A3 // define the pins of your sensor
#define echoPin A2  // Pin A3 on the Motor Drive Shield soldered to the ultrasonic sensor fof depth
#define MAX_DISTANCE 300 // sets maximum useable sensor measuring distance to 300cm
#define MAX_SPEED 200 // sets speed of DC traction motors to 150/250 or about 70% of full speed - to get power drain down.
#define MAX_SPEED_OFFSET 40 // this sets offset to allow for differences between the two DC traction motors
#define COLL_DIST 50 // sets distance at which robot stops and reverses to 30cm
#define TURN_DIST COLL_DIST+20 // sets distance at which robot veers away from object
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);// sets up sensor library to use the correct pins to measure distance.
AF_DCMotor leftMotor1(1, MOTOR12_1KHZ); // create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
AF_DCMotor leftMotor2(2, MOTOR12_1KHZ); // create motor #2, using M2 output, set to 1kHz PWM frequency
AF_DCMotor rightMotor1(3, MOTOR34_1KHZ);// create motor #3, using M3 output, set to 1kHz PWM frequency
AF_DCMotor rightMotor2(4, MOTOR34_1KHZ);// create motor #4, using M4 output, set to 1kHz PWM frequency
Servo myservo;  // create servo object to control a servo 
int leftDistance, rightDistance; //distances on either side
int curDist = 0;
int depthDist = 0;
String motorSet = "";
int speedSet = 0;
//------------------ SETUP LOOP ---------------------------------------------------------------------
void setup() {
  myservo.attach(10);  // attaches the servo on pin 10 (SERVO_1 on the Motor Drive Shield to the servo object 
  myservo.write(90); // tells the servo to position at 90-degrees ie. facing forward.
  delay(1000); // delay for one seconds
  Serial.begin(9600); // begin serial communitication  
  Serial.println("Motor test!");
   pinMode(trigPin, OUTPUT);// set the trig pin to output (Send sound waves)
  pinMode(echoPin, INPUT);// set the echo pin to input (recieve sound waves)
 }
//------------------forward ultrasonic sensor---------------------------------------------------------
void loop() {
  myservo.write(90);  // move eyes forward
  delay(90);
  curDist = readPing();   // read distance
   long duration, distance; // start the scan
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2); // delays are required for a succesful sensor operation.
  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10); //this delay is required as well!
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;// convert the distance to centimeters.
  if (distance <12)/*if there's an obstacle 25 centimers, ahead, do the following: */ 
  {   
   Serial.println ("Close Obstacle detected!" );
Serial.println ("Obstacle Details:");
Serial.print ("Distance From Robot is " );
Serial.print ( distance);
Serial.print ( " CM!");// print out the distance in centimeters.
Serial.println (" The obstacle is declared a threat due to close distance. ");
Serial.println (" Turning !");
    //delay(800);
   if (curDist < COLL_DIST) {changePath();}  // if forward is blocked change direction
  moveForward();  // move forward
  delay(100); //if there's no obstacle ahead, Go Forward! 
}
else{
 
   turnAround(); //if there's no obstacle ahead, Go Forward! 
    
  }   
 }
//--------------------Servo motor-----------------------------------------------------------------------
void changePath() {
  moveStop();   // stop forward movement
  myservo.write(36);  // check distance to the right
    delay(500);
    rightDistance = readPing(); //set right distance
    delay(500);
    myservo.write(144);  // check distace to the left
    delay(700);
    leftDistance = readPing(); //set left distance
    delay(500);
    myservo.write(90); //return to center
    delay(100);
    compareDistance();
  }
void compareDistance()   // find the longest distance
{
  if (leftDistance>rightDistance) //if left is less obstructed 
  {
    turnLeft();
  }
  else if (rightDistance>leftDistance) //if right is less obstructed
  {
    turnRight();
  }
   else //if they are equally obstructed
  {
    turnAround();
  }
}

//------------Ultrasonic sensor call----------------------------------------------------------------
int readPing() { // read the ultrasonic sensor distance
  delay(70);   
  unsigned int uS = sonar.ping();
  int cm = uS/US_ROUNDTRIP_CM;
  return cm;
}
//----------------- stop motors--------------------------------------------------------------------
void moveStop() {
   leftMotor1.run(RELEASE); 
   leftMotor2.run(RELEASE);
   rightMotor1.run(RELEASE); 
   rightMotor2.run(RELEASE);}  // stop the motors.
//-------------------move Forward---------------------------------------------------------------------
void moveForward() {
    motorSet = "FORWARD";
    leftMotor1.run(FORWARD);      // turn it on going forward
    leftMotor2.run(FORWARD);      // turn it on going forward
    rightMotor1.run(FORWARD);     // turn it on going forward
    rightMotor2.run(FORWARD);     // turn it on going forward
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    leftMotor1.setSpeed(speedSet);
    leftMotor2.setSpeed(speedSet);
    rightMotor1.setSpeed(speedSet); 
    rightMotor2.setSpeed(speedSet);
    delay(5);
  }
}
//---------------move Right-----------------------------------------------------------------------------
void turnRight() {
  motorSet = "RIGHT";
  leftMotor1.run(FORWARD);      // turn motor 1 forward
  leftMotor2.run(FORWARD);      // turn motor 2 forward
  rightMotor1.run(BACKWARD);    // turn motor 3 backward
  rightMotor2.run(BACKWARD);    // turn motor 4 backward
  rightMotor1.setSpeed(speedSet+MAX_SPEED_OFFSET);      
  rightMotor2.setSpeed(speedSet+MAX_SPEED_OFFSET);     
  delay(1500); // run motors this way for 1500        
  motorSet = "FORWARD";
  leftMotor1.run(FORWARD);      // set both motors back to forward
  leftMotor2.run(FORWARD);
  rightMotor1.run(FORWARD);
  rightMotor2.run(FORWARD);      
}  
//-----------------move left------------------------------------------------------------------------------
void turnLeft() {
  motorSet = "LEFT";
  leftMotor1.run(BACKWARD);      // turn motor 1 backward
  leftMotor2.run(BACKWARD);      // turn motor 2 backward
  leftMotor1.setSpeed(speedSet+MAX_SPEED_OFFSET);     
  leftMotor2.setSpeed(speedSet+MAX_SPEED_OFFSET);    
  rightMotor1.run(FORWARD);     // turn motor 3 forward
  rightMotor2.run(FORWARD);     // turn motor 4 forward
  delay(1500); // run motors this way for 1500  
  motorSet = "FORWARD";
  leftMotor1.run(FORWARD);      // turn it on going forward
  leftMotor2.run(FORWARD);      // turn it on going forward
  rightMotor1.run(FORWARD);     // turn it on going forward
  rightMotor2.run(FORWARD);     // turn it on going forward
}  
//------------- Turn Around ------------------------------------------------------------------------------
void turnAround()
{
  motorSet = "RIGHT";
  leftMotor1.run(BACKWARD);      // turn motor 1 forward
  leftMotor2.run(BACKWARD);      // turn motor 2 forward
  rightMotor1.run(FORWARD);    // turn motor 3 backward
  rightMotor2.run(FORWARD);    // turn motor 4 backward
  rightMotor1.setSpeed(speedSet+MAX_SPEED_OFFSET);      
  rightMotor2.setSpeed(speedSet+MAX_SPEED_OFFSET);
  delay(5000); // run motors this way for 1700
  motorSet = "FORWARD";
  leftMotor1.run(FORWARD);      // set both motors back to forward
  leftMotor2.run(FORWARD);
  rightMotor1.run(FORWARD);
  rightMotor2.run(FORWARD);      
}  
