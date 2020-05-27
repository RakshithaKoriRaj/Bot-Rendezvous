
#include <Servo.h> 

#include "ESP8266WiFi.h"

char ssid[] = "network1";     //  your network SSID (name)
char pass[] = "01234567";  // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

//Class to setup motor
class Motor{
  public:
  //Pins are intilized here
  Motor(int enable,int input1,int input2){
    pinMode(enable, OUTPUT);
    pinMode(input1, OUTPUT);
    pinMode(input2, OUTPUT);
    _enable=enable;
    _input1=input1;
    _input2=input2; 
  }

  //Move forward at a given speed
  void forward(int speed){
    digitalWrite(_input1, HIGH);
    digitalWrite(_input2, LOW);
    analogWrite(_enable, speed);       
  }

  //Move reverse at a given speed
  void reverse(int speed){
    digitalWrite(_input1, LOW);
    digitalWrite(_input2, HIGH);
    analogWrite(_enable, speed);       
  }

  //Turn off the motor
  void off(){
    digitalWrite(_input1, LOW);
    digitalWrite(_input2, LOW);       
  }
  
  private:
    int _enable;
    int _input1;
    int _input2;
};


//Class to setup Ultrasonic sensor pins
class UltraSonicSensor{
  public:
  //Trig pin and Echo pin are intilized
  UltraSonicSensor(int trigPin,int echoPin){
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    _trigPin=trigPin;
    _echoPin=echoPin; 
  }

  //Gets obstacle distance in the direction in which Ultrasonic sensor is pointing
  int getDistance(){
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(5);
    // Trigger the sensor by setting the trigPin high for 10 microseconds:
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);
    // Read the echoPin, pulseIn() returns the duration (length of the pulse) in microseconds:
    long duration = pulseIn(_echoPin, HIGH);
    // Calculate the distance:
    int distance= duration*0.034/2;
    return distance; 
  }
  
  private:
    int _trigPin;
    int _echoPin;
};


////////////////////////////////////
///////Intialization section////////
////////////////////////////////////

//Left motor speed
int sl = 1000;
//Right motor speed
int sr = 1000;

//Time the motor must turn for one degree[Fine tuned]
double rightDelayConst = 650.0/90.0;
double leftDelayConst = 550.0/90.0;

//Counter to check obstacle at 45 and 135 degree
int counter = 0;
//Counter to check Wifi signal
int counterW = 0;

int signalStrength = 100;
//Use the for fine tuning 
int xFactor = 10;

//Setting pins for motor left and right
Motor ml(5,2,16);
Motor mr(4,13,15);

//Setting pins for Ultrasonic sensor
UltraSonicSensor us(12,14);
//Declaring servo motor[Used for controling UltraSonicSensor]
Servo Servo1;
int servoPin = 0;

//Bot Turns right at a given angle 
void turnRight(int angle){
  mr.off();
  ml.off();
  delay(250);
  //forward only left motor for rightDelayConst*angle
  ml.forward(sl);
  delay(rightDelayConst*angle);
  mr.off();
  ml.off();    
}


//Bot Turns left at a given angle
void turnLeft(int angle){
  mr.off();
  ml.off();
  delay(250);
  //forward only right motor for leftDelayConst*angle
  mr.forward(sr);
  delay(leftDelayConst*angle);
   mr.off();
   ml.off(); 
}

//Get distance at a angle
//Turns sensor at the mentioned angle and returns distance
int getDistanceAtAngle(int a){
  Servo1.write(a);
  delay(500);
  int distance = us.getDistance();
  delay(500);
  //Resets sensor straight
  Servo1.write(90);
  return distance;  
}

//Get distance Right of the bot
int getRightDistance(){
  return getDistanceAtAngle(0);  
}


//Get distance straight of the bot
int getStrightDistance(){
  return getDistanceAtAngle(90);  
}

//Get distance left of the bot
int getLeftDistance(){
  return getDistanceAtAngle(180);  
}


//Get distance 45 degree of the bot
int get45(){
  return getDistanceAtAngle(45);  
}


//Get distance 135 degree of the bot
int get135(){
  return getDistanceAtAngle(135);  
}

//Establish wifi signal
void wifiConnect(){
 
 if (WiFi.status() == WL_NO_SHIELD) {
   Serial.println("WiFi shield not present");
   while (true);
 }


 // attempt to connect to Wifi network:
 while (status != WL_CONNECTED && status != WL_DISCONNECTED ) {
   status = WiFi.begin(ssid, pass);
   delay(10000);
 }
}

//Move forward d mili seconds
void forward(int d){
  mr.off();
  ml.off();
  delay(100);
  mr.forward(sr);
  ml.forward(sl);
  delay(d);
  mr.off();
  ml.off();
  delay(1000);
}


//Move reverse d mili seconds
void reverse(int d){
  mr.off();
  ml.off();
  delay(100);
  mr.reverse(sr);
  ml.reverse(sl);
  delay(d);
  mr.off();
  ml.off();
  delay(1000);
  
}

//Take a u turn 
//Checks right and left distance
//Takes trun in the direction which has farthest obstacle
void uTurn(){
  mr.off();
  ml.off();
  delay(200);
  int rightDistance = getRightDistance();
  int leftDistance = getLeftDistance();
  Serial.print("ut RD");
  Serial.println(rightDistance);
  Serial.print("ut LD");
  Serial.println(leftDistance);
  reverse(500);
  if(rightDistance >leftDistance){
    turnRight(180);
   }else {
    turnLeft(180);
   }
}

//Returns Wifi Signal strength
//Also halts the motor when the Signal strenght is more than -37 db 
int getWiFiSignal(){
  int ss = WiFi.RSSI();
  if(ss>-37){
    Serial.println("You made it");
    //To Halt,speed of left and right motor is set to zero 
    sr = 0;
    sl = 0;
    //Servo is detached
    Servo1.detach();
  }
  return ss;

}

//This function is called when the wifi signal has degraded
//Decides weather to bot has to go right or left based on signal strength
void getDirection(){
  mr.off();
  ml.off();
  delay(250);
  //Current postion is considered left
  int ls = getWiFiSignal();
  Serial.print("Left Strength");
  Serial.println(ls);

  Serial.println("Turning right to get strength");
  delay(100);
  //Truning right to get strength
  turnRight(90);
  
  //obstacle checking when going right to get the signal strength
  for(int i = 0; i < xFactor ; i++){
    int ds = us.getDistance();
    //Called here to stop if signal ever becomes -37 db or better 
    getWiFiSignal();
    if (ds>20){
      // move the motors in forward direction
      mr.forward(sr);
      ml.forward(sl);
      delay(50);
    } else {
      //If obstacle is encountered take a U-turn
      Serial.println("Found obs in get direction");
      delay(100);
      uTurn();
      return;
    }
  }
  delay(250);
  int rs = getWiFiSignal();
  Serial.print("Right Strength:");
  Serial.println(rs);

  
  if(ls >= rs){
    //Left was better
    Serial.print("Left was better");
    delay(100);
    uTurn();
    return;
  }
  
  Serial.print("Right is better. Going straight");
  
}

void setup()
{
  
  //Begin Serial communication at a baudrate of 9600:
  Serial.println("");
  Serial.println("");  
  Serial.begin(9600);
  
  Servo1.attach(servoPin);
  //Sets ultrasonic sensor straight
  Servo1.write(90);  
  Serial.println("Setup Done");
  //Connect to wifi
  wifiConnect();
  //Setting intial Signal Strength
  signalStrength = WiFi.RSSI();
  Serial.print("Intial Signal Strength");
  Serial.println(signalStrength);
  
}


void loop()
{
  int yf = 15;
  //This delay is the frequency at which we check distance to obstacle
  //And multiples of this delay with counter is used to decide when we check wifi signal strength  
  delay(100);
  int distance = us.getDistance();

  //Factor are set at different signal strength levels 
  if(signalStrength < -60 ){
      xFactor = 10;
      yf = 15;
  } else if(signalStrength < -50) {
      xFactor = 8;
      yf = 12;
  } else if(signalStrength < -40) {
      xFactor = 6;
      yf = 10;
  } else {
     xFactor = 4;  
     yf = 8; 
  }

  //Check and stop
  getWiFiSignal();

  //If the obstacle is more the 20cm away
  if (distance>20){
      counter += 1;
      counterW += 1;
      //Check wifi signal every yf[yf is set based on signal strenght]
      if (counterW > yf){
        mr.off();
        ml.off();
        //Stop and check to get better values
        delay(750);
        //Current wifi strength
        int currentWifi = getWiFiSignal();
        Serial.print("Current: ");
        Serial.println(currentWifi);

        //if current signal strenght less than previous signal strength by 2db [Fine tuned after multiple tries]
        if (currentWifi < signalStrength - 1 ){
          Serial.println("Signal Less");
          //This function adjust the bot to move in right direction
          getDirection();
          currentWifi = getWiFiSignal();
        }
        counterW = 0;
        signalStrength = currentWifi;
      }
      //Move forward
      mr.forward(sr);
      ml.forward(sl);
      //Checks obstacle at 45 and 135 degree
      if (counter > 20){
        counter = 0;
        mr.off();
        ml.off();
        //Getting distance at 45 and 135
        int rDist = get45();
        int lDist = get135();
        
        if(rDist < 30 || lDist < 30){
          mr.reverse(sr);
          ml.reverse(sl);
          delay(200);
          if(rDist > lDist){
            Serial.println("Turn Right Obs");
            turnRight(60);
          }else{
            Serial.println("Turn Left Obs");
            turnLeft(60);
          }
        }
      }
      
  }else{
      //When an obstacle is identified at distance less than 20 cm
      Serial.println("Obstacle Encountered");
      mr.off();
      ml.off();
      mr.reverse(sr);
      ml.reverse(sl);
      delay(500);
      mr.off();
      ml.off();

      //Check right and left distance to decide the direction of turn
      int rightDistance = getRightDistance();
      int leftDistance = getLeftDistance();
      if(rightDistance>leftDistance){
        Serial.println("Turn Right");
        turnRight(90);
      }else{
        Serial.println("Turn left");
        turnLeft(90);
      }  
  }  
}
