#include<Servo.h>
int servoPin=6;
Servo Servo6;
Servo Servo5;
Servo Servo4;
Servo Servo3;
Servo Servo2;
Servo Servo1;

int incoming[16];
int data;



void getAngles();
void getSerial();

void setup() {
//  // put your setup code here, to run once:
  Serial.begin(9600);
  Servo6.attach(6); 
  Servo5.attach(5);
  Servo4.attach(4);
  Servo3.attach(3);
  Servo2.attach(2);
  Servo1.attach(7);

//  pinMode(11,OUTPUT);

}

void loop() {
//zero position //note:moved 3 before 6
  Servo4.write(180);
  delay(1000);
  Servo5.write(120);
  delay(1000);
  Servo3.write(90);
  delay(1000);
  Servo6.write(0);
  delay(1000);
  Servo1.writeMicroseconds(1500);
  delay(1000);
  Servo2.write(90);
  delay(2000);

//position one
//  getAngles();
//  Servo6.write(incoming[3]);
//  delay(1000);
//  Servo4.write(incoming[1]);
//  delay(1000);
//  Servo5.write(incoming[2]);
//  delay(1000);
//  Servo3.write(incoming[0]);
//  delay(1000);
//  Servo1.writeMicroseconds(1960);
//  delay(1000);
//  Servo2.write(90);
//  delay(3000);

//testing without incoming
  Servo6.write(79.39); //79.39
  delay(1000);
  Servo4.write(120); // to nt get knocked
  Servo5.write(150); //knocked pt2
  Servo3.write(137.85); //137.85
  delay(1000);
  Servo5.write(72.05); //72.05
  delay(1000);
  Servo4.write(178.53); //178.53
  delay(1000);
  Servo1.writeMicroseconds(1975);
 // delay(1000);
  //Servo2.write(90);
  delay(3000);

//position move back up
  Servo5.write(120);
  delay(1000);
  Servo4.write(180);
  delay(1000);
  Servo3.write(90);
  delay(1000);

  Servo6.write(180); 
  delay(2000);
//  Servo1.writeMicroseconds(1950);
//  delay(1000);
//  Servo2.write(90);
//  delay(3000);

//position "box"
  Servo4.write(180);
  delay(1000);
  Servo3.write(150);
  delay(1000);
  Servo5.write(97);
  delay(1000);
  Servo1.writeMicroseconds(1500);
  delay(1000);

  exit(0);
}

void loop_serial(){
  getSerial();
  delay(500);
}


void getSerial(){
  if(Serial.available() >= 0){
    data = Serial.read();
    Serial.write(data);
  }
}

void getAngles(){
  while(Serial.available() >= 0){
    //filling array with angles
    for(int i = 0; i < 4; i++){
      incoming[i] = Serial.read();
    }
  }
}
