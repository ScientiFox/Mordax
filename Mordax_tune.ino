/*
MurinX

Firmware to tune the Mordax robot, particularly the settings for the
wheel servos, but also the distance sensor thresholds



OPs:
 drive: D5,D6
 cage: D11

IPs:
D7 - Right side ground
D12 - Side distance
D13- Left side ground
D8- Front Distances
D2 - Object Sensor

myservo1.write(DS1+10); //left back
myservo2.write(DS2-10); //right back

*/

//Interface libraries
#include <Servo.h> 

//Servo objects
Servo myservo1;
Servo myservo2;
Servo myservo3;

//If it looks like the battery might be dying or something
//weird is happening, check these.
//Drive servo zero points
int DS1 = 88; //left +back -fwd
int DS2 = 89; //right -back +fwd
int thresh = 20; //adjust for environments, once BK up, try finding an
                 //autoset based on some init readings?

//State variables
int state = 0;
int sp;
int Gs;

//Bins for sensor readings
int RG;
int LG;
int LD;
int FD;
int OD;

int i_state; //internal state to subroutines- always zeros at routine start
int End; //terminus tag for subroutines
int tick; //GP counter variable

unsigned long sV; //bin for use in read_QTR
unsigned long time; //bin for use in read_QTR

//Utility functions - diagnostic, indicative, setup etc.
//-------------------------------------------------------------

void beep_(){
  //Make a noise on the piezo
  digitalWrite(4,HIGH);
  delay(1000);
  digitalWrite(4,LOW);
}

void stop_all(){
  //turn the motors off
  myservo1.write(DS1);
  myservo2.write(DS2);
}

//Sensor Functions
//-------------------------------------------------------------

//Read the QTR sensor

int read_QTR(int pin){
  //take a reading off the selected QTR sensor
  pinMode(pin,OUTPUT); //Set pin to output
  digitalWrite(pin,HIGH); //Set high to charge
  delay(1); //Wait for charge
  pinMode(pin,INPUT); //Convert to input
  sV = micros(); //start microsecond timer
  time = millis(); //grab time for timeout
  while((digitalRead(pin))&&(millis()-time<10)){} //Read until toggle or timeout
  sV = micros() - sV; //Get dT
  //Serial.println(sV/100); //Optional diagnostic
  delay(1); //wait a bit
  if (sV >= 0){return (int)(sV/100);} //If actually read, return scaled value
  else{return 1000;} //Otherwise, return maxrange- 10ms
}

//Behavior subroutines
//-------------------------------------------------------------  

void turn_about(){
  //For doing a reverse on a line-following maze-
  //  turn around to face the other way
  //  used in maze drop-off operations
  i_state = 0; //Internal state 
  End = 1; //End flag
  blink_s = 0; //blink state
  blink_time = millis(); //blink timer
  tick = 0; //loop ticks

  //Until the end flag is tripped
  while (End){
    //Read both bottom QTR sensors
    RG = read_QTR(7);
    LG = read_QTR(12);

    //If 500ms have passed since last reset
    if (millis()-blink_time > 500){
      blink_s = 3 - blink_s; //cycle blink state (0,3,0,3...)
      write_color(blink_s); //write the toggled state color
      blink_time = millis();} //Reset timer

    //State 0, initial backup
    if (i_state == 0){
      if (tick == 0){ //On init tick
        myservo1.write(DS1-3); //write servos to reverse turn
        myservo2.write(DS2-4);}
      if (RG > thresh){ //If the right sensor is over the line
        i_state = 1; //Next state
        tick = 0;} //Reset tick counter
    }

    //State 1, watch for left sensor encountering line
    if (i_state == 1){
      if (tick == 0){ //On init tick
        myservo1.write(DS1-3); //Continue reverse turn
        myservo2.write(DS2-4);}
      if (LG > float(thresh)*0.5){ //On detecting edge of the line
        i_state = 2; //Next state
        tick = 0; //Reset tick
        myservo1.write(DS1); //Stop drive servos
        myservo2.write(DS2);}
    }

    //State 2, rotate until both sensors straddling line again
    if (i_state == 2){
      if (tick == 0){ //on init tick
        myservo1.write(DS1-3); //keep turning
        myservo2.write(DS2-4);}
      //If both sensors back off the line itself
      if ((RG < float(thresh)*1.2)&&(LG < float(thresh)*1.2)){
        i_state = 3; //'next' state, just for templating
        tick = 0; //clear ticks
        End = 0; //toddle end of loop 
        myservo1.write(DS1); //Stop both motors
        myservo2.write(DS2);}
    }
  }
}

//line maze- take a turn at an intersection
void corner_turn(int dir){
  i_state = 0; //Internal state
  End = 1; //Set end flag
  tick = 0; //Tick count
  blink_s = 1; //Blink state
  blink_time = millis(); //Blink timer

  //Until end flag tripped
  while (End){
    //Read both QTR sensors
    RG = read_QTR(7);
    LG = read_QTR(12);

    //Every 0.05 seconds
    if (millis()-blink_time > 50){
      blink_s = 4-blink_s; //Toggle blink state (color 4)
      write_color(blink_s); //Write the LED
      blink_time = millis();} //Reset clock
    if (dir == 0){ //if left turn
      myservo1.write(DS1+2); //Write servos
      myservo2.write(DS2+5);
      if (RG < float(thresh)*1.2){ //When right sensor  passing through the line fully
        End = 0; //Flag end of loop
        //myservo1.write(DS1); //Optional stop- makes the robot jerky
        //myservo2.write(DS2);
      }
    }
    if (dir == 1){//if right turn
      myservo1.write(DS1-5); //Write servos
      myservo2.write(DS2-2);
      if (LG < float(thresh)*1.2){ //When left sensor passing through the line fully
        End = 0; //Flag end of loop
        //myservo1.write(DS1); //Optional stop- makes the robot shaky
        //myservo2.write(DS2);
      }
    }
  delay(30); //Smoothness delay
  }
}

//Follow a line
void line_follow(int dir){

  //If both sensors over line- encountering an intersection
  if ((RG>thresh)&(LG>thresh)){
    cornerTimer = millis(); //Set cornering timer
    corner_turn(dir); //Turn assigned direction
  }
  else if (RG > thresh){ //If right side only
    myservo1.write(DS1-3); //veer right
    myservo2.write(DS2-3);
  }
  else if (LG > thresh){ //If left side only
    myservo1.write(DS1+3); //veer left
    myservo2.write(DS2+3);
  }
  else{ //Otherwise
    myservo1.write(DS1-6); //go straight
    myservo2.write(DS2+3);
  }
}

//Wall-following routine
void wall_follow(int sp){
  //Read the side distance QTR sensor
  LD = read_QTR(9);

  //Calculate error from the setpoint distance
  int e = LD - sp;//closer is negative
  if (e < -1){ //If negative e
    myservo1.write(DS1-8); //veer away
    myservo2.write(DS2+4);
  }
  else if (e > 1){ //If positive e
    myservo1.write(DS1-2); //veer towards
    myservo2.write(DS2+8);
  } 
  else{ //Otherwise
    myservo1.write(DS1-3); //Go straight
    myservo2.write(DS2+3);
  }
}

//Obstacle avoidance routine- turn to follow obstacle, and rejoin line at next chance
void avoid_obstacle(int sp,int dir){

  //Enters in a global state, 2 is wall following
  if(i_state==2){
    wall_follow(sp); //Follow the wall
    //When the right line sensor, side sensor are tripped and at least 3s have passed
    //  (encountering the next line intersecting the obstacle)
    if ((RG>thresh)&&(LD>thresh)&&(millis()-ct>3000)){i_state = 3;} //Next state
  }

  //when turning onto the new line
  if (i_state == 3){
    corner_turn(0); //execute a corner turn to the right (towards the obstacle)
    i_state = 4; //Next state
  }
  
  //Once turned to face the obstacle again
  if (i_state == 4){
    while ((RG > thresh)&&(LG > thresh)){ //until both ground sensors over line
      RG = read_QTR(7); //Read sensors
      LG = read_QTR(12);
      myservo1.write(DS1+3); //Rotate
      myservo2.write(DS2-3);}
    i_state = 5; //next state
  }
  
  //Once both sensors see the line
  if (i_state == 5){
    myservo1.write(DS1+3); //Pivot around right wheel
    myservo2.write(DS2);
    if (LG < thresh){ //When left sensor is on the line
      i_state = 6;} //Next state
  }
  
  //Once the right sensor is over the line and the left is
  if (i_state == 6){
    myservo1.write(DS1+3); //Keep rotating until the left sensor is on the other side
    myservo2.write(DS2);
    if (LG > thresh){ //Once there
      i_state = 7;} //Next state
  }

  //Once left sensor is 'behind' the line
  if (i_state == 7){
    myservo1.write(DS1); //pivot around left wheel
    myservo2.write(DS2+3);
    if (RG < thresh){ //when the right sensor is on the other side
      i_state = 8;} //Next state
  }
  
  //Once the right sensor is on the right side
  if (i_state == 8){
      myservo1.write(DS1+3); //Pivot on the right side again to make sure the left is in the clear
      myservo2.write(DS2);
      if (LG < thresh){ //Once cleared
        i_state = 9;} //Next state
  }
  
  //Once left sensor positioned, straighten up to clear right, also
  if (i_state == 9){
      myservo1.write(DS1); //Pivot on right again
      myservo2.write(DS2+3);
      if (RG < thresh){ //If right clear
        state = 0;} //Exit state
  }
}

//Setup function
void setup(){

  Serial.begin(9600); //Start serial port

  //Attach servos
  myservo1.attach(5);
  myservo2.attach(6);
  myservo3.attach(11);

  //Set servo to no motion
  myservo1.write(DS1);
  myservo2.write(DS2);

  myservo3.write(5); //gate up

  //keep this delay- it helps startup positioning
  delay(3000);

  //Set analog pin to output
  pinMode(A5,OUTPUT);

  beep_(); //Beep to note init done

}

//counter
long ct = 0;

//Main loop
void loop(){

  //Read all QRT sensors
  RG = read_QTR(7);
  LG = read_QTR(13);
  LD = read_QTR(12);
  FD = read_QTR(8);
  OD = read_QTR(2);

  //Starting state- follow a line until seeing an obstacle
  if (state == 0){
    line_follow();
    if (FD < 50){state = 1; ct = millis();} //On seeing an obstacle, next state
  }
  
  //State 1, turn left distance to face wall
  if (state == 1){
    myservo1.write(DS1-3); //Turning
    myservo2.write(DS2-3);
    Serial.println(LD); //Report distance sensor value

    //When seeing the obstacle on the side at 7cm, next state
    if ((LD < 70)||((millis()-ct)>2000)){state = 2; ct = millis();}
  }
  
  //State 2, follow the wall at 7cm distance until seeing the line again
  if(state==2){
    wall_follow(70);
    //Tick state until both ground sensors see the line
    if ((RG>25)&&(LD>25)&&(millis()-ct>3000)){state = 3;}
  }

  //State 3, do a corner turn
  if (state == 3){
    corner_turn();
    state = 4;
  }

  //Next, follow the line again
  if (state == 4){
    line_follow();
    //Next state when both ground sensors are off the line
    if ((RG < 5)&&(LG < 5)){state = 5;}
  }

  //State 5, pivot on right wheel
  if (state == 5){
    myservo1.write(DS1+3);
    myservo2.write(DS2);
    
    //Loop back to start state when the left sensor sees the line
    if (LG > 15){state = 0;}
  }

  //Terminal state,  stop everything
  if (state == 254){stop_all();}

  delay(100); //Smoothness delay
}

