/*
MurinXIV

14th Revision of the Mordax robot, implementing rigorous behavior-based
robotics routines, including:



Actuators:
 drive: D5,D6
 cage: D11

Sensors:
 D7 - Right side ground
 D12 - Side distance
 D13- Left side ground
 D8- Front Distances
 D10 - Object Sensor
*/

//Interface libraries
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

//Color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

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

//For tuning
//myservo1.write(DS1+10); //left back
//myservo2.write(DS2-10); //right back

//Bins for sensor readings
int RG;
int LG;
int LD;
int FD;
int OD;

//blink vars
int blink_s = 0;
long blink_time = millis();

int state = 0; //master state machine
int sp; //setpoint for wall follow
int i_state; //internal state to subroutines- always zeros at routine start
int End; //terminus tag for subroutines
int tick; //GP counter variable
long ct = 0; //counter for time checks
int o_check = 0; //object check status
long cornerTimer = 0;
int cornerCount = 0;
int color = 0;
int color_n = 0;
int color_p = 0;
int color_count = 0;
int cage_state = 0;
char s;

unsigned long sV; //bin for use in read_QTR
unsigned long time; //bin for use in read_QTR

String c_list[5] = {"None","Red","Blue","Yellow","Green"};
uint16_t r, g, b, c;

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

void write_color(int index){
  // Write a color code to the RGB LED
  digitalWrite(A0,HIGH);
  digitalWrite(A1,HIGH);
  digitalWrite(A2,HIGH);
  
  if (index == 1){
    digitalWrite(A0,LOW);}
  if (index == 2){
    digitalWrite(A1,LOW);}
  if (index == 3){
    digitalWrite(A2,LOW);}
}

//Main init function
void initialize(){

  //startup commands to run
  Serial.begin(9600); //Start serial port

  write_color(0); // Set to color 0

  myservo1.attach(5); //Activate servos
  myservo2.attach(6);
  myservo3.attach(11);

  myservo1.write(DS1); //Set wheel servos to zero velocity
  myservo2.write(DS2);

  //Raise the interaction cage
  raise_cage();
  	//myservo3.write(5); //gate up
  	//myservo3.write(100); //gate down

  //Set IO pins to default state
  pinMode(4,OUTPUT);
  pinMode(A0,OUTPUT);
  pinMode(A1,OUTPUT);
  pinMode(A2,OUTPUT);

  //Cycle through colors as sstartuo sequence signal
  write_color(1);
  delay(250);
  write_color(2);
  delay(250);
  write_color(3);
  delay(250);
  write_color(0);
  delay(250);

  //Check the color sensor and report
  if (tcs.begin()){
    Serial.println("Found sensor");}
  else{
    Serial.println("No TCS34725 found ... check your connections");}
}

//Classify a color read on the TCS sensor
int color_get(uint16_t r,uint16_t g,uint16_t b){
  uint16_t avg = r+g+b;
  float _r = (float)r/((float)avg);
  float _g = (float)g/((float)avg);
  float _b = (float)b/((float)avg);

  //Optional diagnostic print
  /*
  Serial.print(_r);
  Serial.print(",");
  Serial.print(_g);
  Serial.print(",");
  Serial.print(_b);
  Serial.println();
  */

  //Color output:
  //String c_list[5] = {"None","Red","Blue","Yellow","Green"};
  if ((_r >0.65)&(_g<0.2)&(_b<0.2)){
    return 1;}
  else if ((_r<0.3)&(_g>0.4)&(_b<0.35)){
    return 4;}
  else if ((_r<0.3)&(_g<0.35)&(_b>0.4)){
    return 2;}
  else if ((_r>0.45)&(_b<0.2)){
    return 3;}
  else{
    return 0;}
  }

//print the current master state and sensor values
void print_states(){

  //Serial.print(state);
  //Serial.print(",");
  Serial.print(LG);
  Serial.print(",");
  Serial.print(RG);
  /*
  Serial.print(",");
  Serial.print(FD);
  Serial.print(",");
  Serial.print(OD);
  Serial.print(",");
  Serial.print(LD);
  */
  Serial.println();
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
void drop_cage(){
  //lower the gate
  myservo3.write(130);
}

void raise_cage(){
  //raise the gate
  myservo3.write(50);
}

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

/*
Main execution setup and loops
*/

//Setup function
void setup(){

  //Run init function
  initialize();

  //keep this delay- it helps:
  // gives a chance to check servo 0 points
  // and lets battery stabilize
  Serial.begin(9600);
  delay(100);
  Serial.println("Online...");
  delay(3000);
  Serial.println("Begin");
  //beep_(); //Optional audible diagnostic
}

//Set canonical direction (0 left, 1 right)
int dir = 0;

//Main loop
void loop(){

  //Read all QRT sensors
  RG = read_QTR(7); //Right ground
  LG = read_QTR(12); //Left ground
  LD = read_QTR(9); //Left side distance sensor
  FD = read_QTR(8); //Forward distance sensor
  OD = read_QTR(10); //Object distance sensor

  //print_states(); //Optional diagnostic

  //Serial command, default 'n' for none
  s = 'n';
  while (Serial.available()>0){ //Read from serial port
    s = Serial.read();}

  //If 's' for supervisor mode
  if (s == 's'){ 
    Serial.println("Supervisory control mode"); //Note the change on port
    state = 100;} //Set supervisor lockout state

  //While in supervisor state
  while (state == 100){
    if (s == 'f'){ //Forward motion
      myservo1.write(DS1-8);
      myservo2.write(DS2+6);}
    if (s == 'b'){ //Reverse
      myservo1.write(DS1+3);
      myservo2.write(DS2-3);}
    if (s == 'r'){ //Right turn
      myservo1.write(DS1-3);
      myservo2.write(DS2-3);}
    if (s == 'l'){ //Left turn
      myservo1.write(DS1+3);
      myservo2.write(DS2+3);}
    if (s == 'p'){ //Pause, stop motion
      myservo1.write(DS1);
      myservo2.write(DS2);}
    if (s == 'd'){ //Drop cage
      drop_cage();
      cage_state = 1;} //Note state internalls
    if (s == 'u'){ //Raise cage
      raise_cage();
      cage_state = 0;} //Note state

    //Speed adjust commands
    if (s == 'k'){DS1 += 1;}
    if (s == 'z'){DS2 += 1;}
    if (s == 'x'){DS1 -= 1;}
    if (s == 'y'){DS2 -= 1;}

    //distance read command
    if (s == 'w'){
      LD = analogRead(A3); //Read the left distance sensor
      Serial.print(LD); //Report value
      Serial.println();}

    s = Serial.read(); //read character again
    if (s == 'c'){ //If c, that's switch back to autonomous
      Serial.println("autonomous mode");
      state = 0;} //Reset state
  }

  //Get TCS reading
  tcs.getRawData(&r, &g, &b, &c);
  color_n = color_get(r,g,b); //Get read color

  //count the number of times this color has been seen
  color_count = (color_count + 1)*(color_p == color_n);
  color = color_n*(color_count > 0) + color*(color_count < 1  ); //set active color to current
  color_p = color_n; //Reset 'previous' color
  
  //Output state variables
  Serial.print(c_list[color]);
  Serial.print(",");
  Serial.print(state);
  Serial.print(",");
  Serial.print(LD);
  Serial.print(",");
  Serial.print(cornerCount);
  Serial.println();

  //Optional english color names in order
  //String c_list[5] = {"None","Red","Blue","Yellow","Green"};

  //Use color flag to set primary state
  if (state < 1){ //If not in an active state already

    if (color == 0){state = 0;} //No color- clear state

    if (color == 1){ //Red, left turn mode
      dir = 0;
      state = 0;}

    if (color == 4){ //Green, right turn mode
      dir = 1;
      state = 0;}

    if (color == 2){ //Blue, reverse direction
      turn_about();
      color = 0; //Make sure to reset so it doesn't keep reversing!
      state = 0;}

    if (color == 3){ //Yellow, stop execution
      state = -1; //Block state flag for noe
      stop_all(); //Stop everything 
      write_color(0); //LED off
      raise_cage(); //Raise cage
      cage_state = 0; //Set cage state
      write_color(color_count+1); //Set current color count to LED
      color_count = (color_count + 1)%3; //Lock
      delay(300);} //Short wait

    //Write direction mode to LED
    if (dir == 0){
      write_color(1);} //Red for left
    if (dir == 1){
      write_color(2);} //Blue for right

    //If longer than 1s in a corner, and the timer isn't cleared
    if ((millis() - cornerTimer > 1000)&(cornerTimer != 0)){
      cornerCount++; //Increment the corner turn counter
      cornerTimer = 0;} //Reset the timer
  
    //If object distance under threshold and the cage is up, and not in the terminal state
    if ((OD < 20)&(cage_state == 0)&(color != 3)){
      drop_cage(); //Drop the cage to get the object
      delay(300); //Wait a bit
      cage_state = 1;} //Note cage state change
    
    //If forward distance sensor less than 7cm
    if ((FD <= 70)&(0)){ //CURRENTLY DISABLED wuth &(0)
      myservo1.write(DS1+2); //Set to reverse
      myservo2.write(DS2-5);
      delay(500); //Wait a half second
      myservo1.write(DS1-5); //set to tuen
      myservo2.write(DS2-5);
      delay(700); //Another wait
      state = 2; //state 2 for obstacle avoid
      ct = millis(); //set timer
      i_state = 2;}//state 2 for obstacle avoid
  }

  //print_states(); //Optional state output

  //If in 0 state, follow a line
  if (state == 0){
    line_follow(dir);}

  //If in 2 state. go to obstacle avoid
  if (state == 2){
    //wall_follow(60);
    avoid_obstacle(60,dir);
  }

  //Kill state
  if (state == 254){stop_all();}

  delay(30); //Smoothness delay
}

