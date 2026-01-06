//MurinX

///OPs:
// drive: D5,D6
// cage: D11

///IPs:
//D7 - Right side ground
//D12 - Side distance
//D13- Left side ground
//D8- Front Distances
//D2 - Object Sensor
//

//myservo1.write(DS1+10); //left back
//myservo2.write(DS2-10); //right back

#include <Servo.h> 

Servo myservo1;
Servo myservo2;
Servo myservo3;

int DS1 = 89;
int DS2 = 90;

int state = 0;
int sp;
int Gs;

int RG;
int LG;
int LD;
int FD;
int OD;

int i_state;
int End;
int tick;

unsigned long sV;
unsigned long time;

int read_QTR(int pin){
  pinMode(pin,OUTPUT);
  digitalWrite(pin,HIGH);
  delay(1);
  pinMode(pin,INPUT);
  sV = micros();
  time = millis();
  while((digitalRead(pin))&&(millis()-time<10)){}
  sV = micros() - sV;
  //Serial.println(sV/100);
  delay(1);
  if (sV >= 0){
    return (int)(sV/100);}
  else{
    return 1000;} 
  }
  
void turn_about(){
  i_state = 0;
  End = 1;
  tick = 0;
  while (End){
    RG = read_QTR(7);
    LG = read_QTR(13);

    if (i_state == 0){
      if (tick == 0){
        myservo1.write(DS1-10);
        myservo2.write(DS2-10);}
      if (RG > 20){
        i_state = 1;
        tick = 0;}}
        
    if (i_state == 1){
      if (tick == 0){
        myservo1.write(DS1-10);
        myservo2.write(DS2-10);}
      if (LG > 20){
        i_state = 2;
        tick = 0;
        myservo1.write(DS1);
        myservo2.write(DS2);}} 
    
    if (i_state == 2){
      if (tick == 0){
        myservo1.write(DS1-10);
        myservo2.write(DS2-10);}
      if ((RG < 8)&&(LG < 8)){
        i_state = 3;
        tick = 0;
        End = 0;
        myservo1.write(DS1);
        myservo2.write(DS2);}}
  }
}

void corner_turn(){
  i_state = 0;
  End = 1;
  tick = 0;
  while (End){
    RG = read_QTR(7);
    LG = read_QTR(13);
    
    if (i_state == 0){
      if (tick == 0){
        myservo1.write(DS1+2);
        myservo2.write(DS2+5);
        tick++;}
      if (RG < 5){
        tick = 0;
        i_state = 1;}}
        
    if (i_state == 1){
      if (tick == 0){
        myservo1.write(DS1+2);
        myservo2.write(DS2+5);
        tick++;}
      if (RG < 5){
        tick = 0;
        End = 0;
        i_state = 3;}}
 }
}

 
void line_follow(){
  if ((RG>5)&(LG>5)){
    //myservo1.write(DS1-3);
    //myservo2.write(DS2-3);
    corner_turn();
    //turn_about();
  }  
  else if (RG > 5){
    myservo1.write(DS1-3);
    myservo2.write(DS2-3);}
  else if (LG > 5){
    myservo1.write(DS1+3);
    myservo2.write(DS2+3);}
  else{
    myservo1.write(DS1-3);
    myservo2.write(DS2+3);}
}

void setup(){
  Serial.begin(9600);
  myservo1.attach(5);
  myservo2.attach(6);
  myservo3.attach(11);
  myservo1.write(DS1);
  myservo2.write(DS2);
  myservo3.write(10); //gate up
  //myservo3.write(100); //gate down
  //delay(10000);
  //turn_about(1);
}

void loop(){
  state++;
  RG = read_QTR(7);
  LG = read_QTR(13);
  LD = read_QTR(12);
  FD = read_QTR(8);
  OD = read_QTR(2);
  line_follow();
  Serial.print("tick");
  delay(100);
}


/*
  Serial.print(state);
  Serial.print(",");
  Serial.print(LG);
  Serial.print(",");
  Serial.print(RG);
  Serial.print(",");
  Serial.print(FD);
  Serial.print(",");
  Serial.print(OD);
  Serial.print(",");
  Serial.println(LD);
*/

