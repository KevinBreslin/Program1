#include <UnoWiFiDevEd.h>

# include<avr/sleep.h>
#include <EEPROM.h>
#include <HX711.h>


#include <LiquidCrystal.h>



// MENU COMMANDS
int timer;
int timer2;
int timer3;
int timer4;
int timer5;
int check1=0;
float minvalue=0.0;
float maxvalue=0.0;
int pullvalue=0;
String pullString;
String weightString;
bool permission = false;

// PIN IDENTITIES

const  int potPin = A0;
const int VoltagePin = 9;    // PWM signal connected to digital pin 9

const int sensorPin = A1;    // pin that the sensor is attached to
const int ledPin = 20;        // pin that the LED is attached to

// variables:
int sensorValue = 0;         // the sensor value
int sensorMin = 1023;        // minimum sensor value
int sensorMax = 0;           // maximum sensor value
 
 // LCD pins <--> Arduino pins
 const int RS = 11, EN = 12, D4 = 5, D5 = 6, D6 = 7, D7 = 8;
 LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
 

//Constants and Coversion Factor Metrics
float pi=3.14159265;//35897932384626433832795;
 float radius=20;
 float rad2deg=180/pi;
 float deg2rad=pi/180;
 float radpms2rpm=4549;

//WEIGHT MEASUREMENTS
// Need a calibration curve to assign a PWM value to a particular weight;
//Each weight should have a corresponding PWM value, assumed to be linear at this stage.
int W1 = 51;
int W2 = 102;
int W3 = 153;
int W4 = 204;
int W5 = 254;
int signalvalue2=0;
int signalvalue3=0;
int i=0;

int weightPWM[]= {0, W1, W2, W3, W4, W5};

//Variables

 int angle = 0;
 int rep=0;
 float xmin=.2;
 float xmax=.4;
 float repfunct=0;
 float turnfunct=0;
 int check=0;
 float radang =0;
 float xang = 0;
 float distance = 0;
 float disp = 0;
 float dist = 0;
 float newangle, rotation, oldangle, AngDiff, AngVel, AveRepTime = 0;
 unsigned long newtime = 0;
 unsigned long oldtime = 0;
 unsigned long LastRepTime=0;
 unsigned long NewRepTime=0;
 unsigned long RepIntTime=0;
 unsigned long RepIntTime1=0;
 unsigned long FStartTime, FFinishTime, FInterval = 0;
 unsigned long BStartTime, BFinishTime, BInterval, RInterval = 0;
 unsigned long BIT, FIT, BST, FST, FFT, BFT=0;
 float BID, FID, BSD, FSD, FFD, BFD=0;
 float FStartDist, FFinishDist, FIncrement = 0;
 float BStartDist, BFinishDist, BIncrement = 0;
 float RepIntDist, NewRepDist, RepIntDist1, RIncrement = 0; 
 unsigned long FIntTime1, BIntTime1, NewBTime, NewFTime =0;
  unsigned long HoldTime, RestTime =0;
  float HoldDist, RestDist = 0;
 float FIntDist1, BIntDist1, NewBDist, NewFDist =0;

  
 float olddist;
 float olddisp;
 float vel=0;
 float oldvel=0;
 float absement=0;
 float interval = 0;
 float angVel =0;
 float LinVel =0;
 float oldspeed =0;
 float newspeed =0;
 float oldacc=0;
 float acc =0;
 float RepsPerMin =0;
 float s=0;
 float newa=0;
 float deltarep=0;
 float oldrep=0;
 
 int disp2=0;
 //float sigmadeltarep=0;
 int oldstate[]= {0,0,0,0};
 int state[]= {0,0,0,0};
 int oldturns[]= {0,0};
 int turns[]= {0,0};
 int oldfixstate[] = {0,0,0,0};

 int forwards[]={0,0};
 int backwards[]={0,0};
 int oldforwards[]={0,0};
 int oldbackwards[]={0,0};
 //int trial[]= {
 int reps=0;
 int preps=0;
 int breps=0;
 int bpreps=0;
 int freps=0;
 int fpreps=0;
 int pcrep=0;
 int turncount=0;
 int errcount=0;
 int fixstate[]= {0,0,0,0};
 int stopvar[]={0,0,0,0};
// int repfunct=0;
 int statefunct=0;
 int cleanrep=0;
 int repmarker=0;
 int fscheck=0;
 int continue1=0;
  
volatile long temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder

volatile long counter2, counter1 = 0; //This variable will increase or decrease depending on the rotation of encoder

int readValue = 0;
int sensorValue = 0;
int signalvalue=0;
int weightvalue=0;
int rotations=0;

char S1[16];
char S2[16];
char S3[16];
char S4[16];
char Intro1[16];
char Intro2[16];
char Introstring[100];
char str[2];


void setup() {
//SLEEP MODE SETUP
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // LCD SETUP
  pinMode(A0, INPUT);
  lcd.clear();
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

//SERIAL SETUP & INTRO

Serial.begin(9600);

pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  
  pinMode(3, INPUT_PULLUP); // internal pullup input pin 3
   //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
   
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);

}
void loop() {
//if (menuchoice=1){
//  Serial.println("Do you want to use Serial Mode or Potentionmeter mode?") 
//}

if (stopvar[0]==1){
  Serial.println("Program Finished due to Count Limit          ");
  sleep_mode();
  }
  else if(stopvar[1]==1){
      Serial.println("Program Finished due to Long Hold Time          ");
      sleep_mode();
    }
   else if(stopvar[2]==1){
      Serial.println("Program Finished due to Long Rest Time          ");
      sleep_mode();
    }
   else if(stopvar[3]==1){
      Serial.println("Program Finished due to Long Exercise Distance          ");
      sleep_mode();
   }
   else{}
  
if (pullvalue==0 & continue1==0){
DetermineLimits();
ValidValues();
continue1=1*(pullvalue>0);
}
else if (continue1==1){
xmin = minvalue/100;
xmax = maxvalue/100;
pullvalue;
continue1=2;
Serial.println("           ");
Serial.println("New Program Run          ");
Serial.println("           ");

sprintf(Introstring,"    Lower Limit is: %i", int(minvalue),"cm");
Serial.println(Introstring);
sprintf(Introstring,"    Upper Max is: %i", int(maxvalue),"cm");
Serial.println(Introstring);
sprintf(Introstring,"    Pull Limit is: %i", int(pullvalue)," pulls");
Serial.println(Introstring);
Serial.println("    PULL THREAD TO BEGIN EXERCISE");
Serial.println("");

sprintf(Intro1,"Set Length: %i", int(xmax*100)-int(xmin*100));
sprintf(Intro2,"Pull: %i", pullvalue);
  lcd.setCursor(0, 0);
  // print out the value at LCD Display:
  lcd.print(Intro2);
  lcd.setCursor(2, 1);
  // print out the value at LCD Display:
  lcd.print(Intro1);

  
}
else if (pullvalue=!0 & continue1==2){
// read the input on analog pin 0:
  sensorValue = analogRead(A0);
  signalvalue = sensorValue/4;
  weightvalue = signalvalue/51;
  signalvalue2 = weightvalue*51;
  signalvalue3 = weightPWM[weightvalue];



  distance = counter;
  //Angular Velocity Calculation
  rotation=(counter/800);
  olddisp=disp;
  disp=rotation*2*pi*radius/1000;//meters - Displacement in meters
  dist=dist+abs(disp-olddisp);

setoldstate(state);
setoldfixstate(fixstate);
setoldturns(turns);
 
preps = getMin(oldstate, 5);
fpreps = min(oldstate[0],oldstate[1]);
bpreps = min(oldstate[2],oldstate[3]);

repfunction(olddisp, disp, xmax, xmin);
fscheck = fixstate[2];
fixedstatecount(olddisp, disp, xmax, xmin, fscheck);
statecount(olddisp, disp, xmax, xmin);
pcrep = cleanrep;
cleanrepcount(fixstate);
repmarker=cleanrep-pcrep;
setforwards(state);
setbackwards(state);  

  
  int reps = getMin(state, 5);
  int check = getMax(state, 5);
  int freps = min(state[0],state[1]);
  //getMin(forwards, 3);
  int breps = min(state[2],state[3]);
  
  

// FORWARD AND BACKWARD MEASUREMENTS

FST=max((state[0]-oldstate[0])*newtime,FST);// Forward Start Time
FFT=max((state[1]-oldstate[1])*newtime,FFT);//Forward Finish Time
FIT=max((state[1]-oldstate[1])*(FFT-FST),(state[0]==oldstate[0])*(state[1]==oldstate[1])*FIT);//Forward Interval Time
HoldTime=max((state[2]-oldstate[2])*(FFT-BST),(state[2]==oldstate[2])*(state[1]==oldstate[1])*HoldTime);//Hold Time
BST=max((state[2]-oldstate[2])*newtime,BST);//Backward Start Time
BFT=max((state[3]-oldstate[3])*newtime, BFT);//Backward Finish Time
BIT=max((state[3]-oldstate[3])*(BFT-BST),(state[2]==oldstate[2])*(state[3]==oldstate[3])*BIT);//Backward Interval Time
RestTime=max((state[0]-oldstate[0])*(BST-FFT),(state[0]==oldstate[0])*(state[3]==oldstate[3])*RestTime);//Rest Time
FSD=max((state[0]-oldstate[0])*dist,FSD);
FFD=max((state[1]-oldstate[1])*dist,FFD);
FID=max((state[1]-oldstate[1])*(FFD-FSD),(state[0]==oldstate[0])*(state[1]==oldstate[1])*FID);
HoldDist=max((state[2]-oldstate[2])*(FFD-BSD),(state[2]==oldstate[2])*(state[1]==oldstate[1])*HoldDist);//Hold Distance
BSD=max((state[2]-oldstate[2])*dist, BSD);
BFD=max((state[3]-oldstate[3])*dist, BFD);
BID=max((state[3]-oldstate[3])*abs(BFD-BSD),(state[2]==oldstate[2])*(state[3]==oldstate[3])*BID);
RestDist=max((state[0]-oldstate[0])*(BFD-FSD),(state[0]==oldstate[0])*(state[3]==oldstate[3])*RestDist);//Rest Time

  RepIntDist=(reps-preps)*(max(dist*(reps-preps),NewRepDist)-min(dist*(reps-preps),NewRepDist));
  RepIntDist1=max(RepIntDist,RepIntDist1);
  NewRepDist=max(dist*(reps-preps),NewRepDist);
  RIncrement=NewRepDist;

   
  RepIntTime=(reps-preps)*(max(newtime*(reps-preps),NewRepTime)-min(newtime*(reps-preps),NewRepTime));
  RepIntTime1=max(RepIntTime,RepIntTime1);
  NewRepTime=max(newtime*(reps-preps),NewRepTime);
  LastRepTime=NewRepTime;

  RInterval=RepIntTime;
  AveRepTime=(newtime)/(1000*reps);
  RepsPerMin=60/AveRepTime;

  oldangle=newangle;
  newa = ((counter % 800)*360/800);
  newangle = ((abs(newa)==newa)*(newa))+((abs(newa)==-newa)*(360+newa));
  
  AngDiff=newangle-oldangle;
  oldtime = newtime;
  newtime = millis();
  interval = (newtime-oldtime);
  oldvel=vel;
  vel=1000*(disp-olddisp)/interval;
//  oldacc=acc;
//  acc=1000*(vel-oldvel)/interval;
//  absement=absement+(interval*(disp-olddisp))/1000;


  if( counter != temp ){

  temp = counter;  
  rotations=abs(counter/800);
  disp2=(int)disp;
  analogWrite(VoltagePin, signalvalue2);
  sprintf(S1,"Weight: %i", weightvalue);
  sprintf(S2,"Disp: %i", disp2);
  sprintf(S3,"Rep Count: %i", int(reps));
  sprintf(S4,"Interval: %i", int(AveRepTime));

  Serial.print("    ");
  Serial.print(S1);
  Serial.print("    Time: ");
  Serial.print(newtime/1000);
  //sprintf(S3,);
  Serial.print("    Disp:");
  Serial.print(disp);
  Serial.print("    Dist:");
  Serial.print(dist);
  Serial.print("    Clean Reps:");
  Serial.print(cleanrep);

      Serial.print("    FSA:");
     Serial.print(fixstate[0]);
     Serial.print("-");  
     Serial.print(fixstate[1]);
     Serial.print("-");
     Serial.print(fixstate[2]);
     Serial.print("-");
     Serial.print(fixstate[3]);
  
  
  Serial.print("    Repititions:");
  Serial.print(reps);
//    Serial.print("    Prepititions:");
//  Serial.print(preps);
  
    Serial.print("    Rep Time: ");
  Serial.print(newtime*(reps-preps)/1000);

  Serial.print("    Clean Rep Time: ");
  Serial.print(newtime*(repmarker)/1000);
//  Serial.print("  Last Rep Time: ");
//  Serial.print(LastRepTime/1000);
//    Serial.print("  New Rep Time: ");
//  Serial.print(NewRepTime/1000);
  ///Serial.print("  Rep Interval Time: ");
  //Serial.print(RepIntTime/1000);
  //Serial.print("  Average Time: ");
  //Serial.print(AveRepTime);
  //Serial.print("  RepsPerMin: ");
  //Serial.print(RepsPerMin);
  //Serial.print("  Times: ");
/*  Serial.print(FST/1000);
  Serial.print("-");
  Serial.print(FFT/1000);
  Serial.print("-");
  */
 
  Serial.print("  Forward Time: ");
  Serial.print(FIT/1000);
  /*
  Serial.print("-");
  Serial.print(BST/1000);
  Serial.print("-");
  Serial.print(BFT/1000);
  Serial.print("-");
  */
   
  Serial.print("  Backward Time: ");
  Serial.print(BIT/1000);

  Serial.print("  Hold Time: ");
  Serial.print(HoldTime/1000);
  Serial.print("  Rest Time: ");
  Serial.print(RestTime/1000);
  /*

  Serial.print(FSD/1000);
  Serial.print("-");
  Serial.print(FFD/1000);
  Serial.print("-");
  */
  Serial.print("  Forward  Increment: ");
  Serial.print(FID);
  /*
  Serial.print("-");
  Serial.print(BSD/1000);
  Serial.print("-");
  Serial.print(BFD/1000);
  Serial.print("-");
*/  
  Serial.print("  Backward  Increment: ");

  Serial.print(BID);
/*
  

  Serial.print("  Intervals: ");
  Serial.print(NewFTime/1000);
  Serial.print("-");
  Serial.print(NewBTime/1000);
  Serial.print("-");
  Serial.print(NewRepTime/1000);
  Serial.print("  Increments: ");
  Serial.print(NewFDist/1000);
  Serial.print("-");
  Serial.print(NewFDist/1000);
  Serial.print("-");
  Serial.print(NewRepDist/1000);
  
//  Serial.print("    Delta Rep function:");
//  Serial.print(deltarep);
//  Serial.print("    SigmaDelta Rep function:");
//  Serial.print(sigmadeltarep);

// Serial.print("    Velocity:");
// Serial.print(vel);
// Serial.print("    Acceleration:");
// Serial.print(acc);
  //Serial.print("    Interval:");
  //Serial.print(interval/1000);
  
*\ 
*
 
 Serial.print("    Velocity:");
 Serial.print(vel);
 Serial.print("    Acceleration:");
 Serial.print(acc);
 Serial.print("   Absement:");
 Serial.print(absement);

Serial.print("    Interval:");
Serial.print(interval/1000);

 */ 
//  Serial.print("    ");
  Serial.println("    ");

  
  //lcd.setCursor(0, 0);
  // print out the value at LCD Display:
  //lcd.print(S1);
 // lcd.setCursor(2, 1);
  // print out the value at LCD Display:
 // lcd.print(S2);
  //delay(50);
  lcd.setCursor(0, 0);
  // print out the value at LCD Display:
  lcd.print(S1);
  lcd.setCursor(2, 1);
  // print out the value at LCD Display:
  lcd.print(S3);
  }  
  //stopvariable (int stopvar[], int cleanrep, int pullvalue, float dist, unsigned long HoldTime, unsigned long RestTime);
}
}
void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
  counter++;
  //counter1=1;
  }else{
  counter--;
  //counter1=0;
  }
}
   
void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
  counter--;
  //counter2=0;
  }else{
  counter++;
  //counter2=1;
  }
}

int getMin(int* array, int size)
{
  int minimum = array[0];
  for (int i = 0; i < 4; i++)
  {
    if (array[i] < minimum) minimum = array[i];
  }
  return minimum;
}

int getMax(int* array, int size)
{
  int maximum = array[0];
  for (int i = 0; i < 4; i++)
  {
    if (array[i] > maximum) maximum = array[i];
  }
  return maximum;
}


//pcrep = cleanrep;

int setoldstate(int state [])
{
 oldstate[0]=state[0];
 oldstate[1]=state[1];
 oldstate[2]=state[2];
 oldstate[3]=state[3];
 
 return oldstate;
}

int setoldfixstate(int fixstate [])
{
 oldfixstate[0]=fixstate[0];
 oldfixstate[1]=fixstate[1];
 oldfixstate[2]=fixstate[2];
 oldfixstate[3]=fixstate[3];
 
 return oldfixstate;
}


int cleanrepcount(int fixstate[])
{ 
 if (fixstate[0]==1 & fixstate[1]==1 & fixstate[2]==1 & fixstate[3]==1) {
  cleanrep++;
  fixstate[0]=0;
  fixstate[1]=0;
  fixstate[2]=0;
  fixstate[3]=0;
  }
 else
 {}

return cleanrep;
}


int repfunction(float olddisp, float disp, float xmax, float xmin)
{
repfunct=4*((olddisp>xmin)*(disp<=xmin))+3*((olddisp>xmax)*(disp<=xmax))+2*((olddisp<xmax)*(disp>=xmax))+1*((olddisp<xmin)*(disp>=xmin));
return  repfunct;
  
}

int fixedstatecount(float olddisp, float disp, float xmax, float xmin, int fscheck)
{
  if (disp<=0){
  fixstate[0]=0;
  fixstate[1]=0;
  fixstate[2]=0;
  fixstate[3]=0;
    }
  else if((olddisp<xmin)*(disp>=xmin)) {
  fixstate[0]=1;
  fixstate[1]=0;
  fixstate[2]=0;
  fixstate[3]=0;
  }
  else if((olddisp<xmax)*(disp>=xmax)){
  fixstate[1]=1;
  fixstate[2]=0;
  fixstate[3]=0;
  }
  else if((olddisp>xmax)*(disp<=xmax)){
  fixstate[2]=1;
  fixstate[3]=0;
  }
  else if(((olddisp>xmin)*(disp<=xmin)) & fscheck==1){
  fixstate[3]=1;
  }
  else
  {}

 return  fixstate;
}

int statecount(float olddisp, float disp, float xmax, float xmin)
{
  if(repfunct==1) {
  state[0]++;
  }
  else if(repfunct==2){
  state[1]++;
  }
  else if(repfunct==3){
  state[2]++;
  }
  else if(repfunct==4 & fixstate[2]==1){
  state[3]++;
  }
  else
  {}
  return  state;
}

int setoldturns(int turns[])
{
 oldturns[0]=turns[0];
 oldturns[1]=turns[1];
 return turns;
}

int setforwards(int state[])
{
  forwards[0]=state[0];
  forwards[1]=state[1];
  return forwards;
  }

int setbackwards (int state[])
{
  backwards[0]=state[2];
  backwards[1]=state[3];
  return backwards;
}

int stopvariable (int stopvar[], int cleanrep, int pullvalue, float dist, unsigned long HoldTime, unsigned long RestTime)
{
  stopvar[1]=(cleanrep>=pullvalue);//(cleanrep>=10);
  stopvar[2]=HoldTime>=300;
  stopvar[3]=RestTime>=300;
  stopvar[4]=dist>=1000000;
  return stopvar;
}

void InsertMinimumDistance() 
  {
    char minString = '0' ;
    while( minString != '\n' && Serial.available() > 0 ) 
    { 
        String minString = Serial.readStringUntil('\n');
        minString.trim();
        minvalue=minString.toFloat();
        minvalue=minvalue*(minvalue>0);
        if (minvalue==0){
          Serial.println("Try Again");
          }
        else{
        Serial.println("min value is:");
        Serial.println(minvalue);}
        //Serial.readBytes( &ch, 1 ) ; 
        //order += ch ;
    }

    if( minString == '\n' ) 
    {  
        Serial.println(minvalue) ; 
        minString = "" ;
        timer = 0 ;
    }
}

void InsertMaximumDistance() 
  {
    char maxString = '0' ;
    while( maxString != '\n' && Serial.available() > 0 ) 
    { 
        String maxString = Serial.readStringUntil('\n');
        maxString.trim();
        maxvalue=maxString.toFloat();
        //maxve=maxvalue;
        maxvalue=maxvalue*(maxvalue>0)*(maxvalue>minvalue);
        if (maxvalue==0){
          Serial.print("Value entered ");
          Serial.print(maxString);
          Serial.println(" was incorrect");
          Serial.println("Try Again");
          }
        else{Serial.println("max value is:");
        Serial.println(maxvalue);}
        //Serial.readBytes( &ch, 1 ) ; 
        //order += ch ;
    }

    if( maxString == '\n' ) 
    {  
        Serial.println(maxvalue) ; 
        maxString = "" ;
        timer2 = 0 ;
    }
}

void CheckMaxAndMin()
{ 
  while (minvalue>=maxvalue)
  { 
  maxvalue=0;
  timer2=1;
  }
  if (maxvalue>minvalue)
  minvalue=minvalue;
  maxvalue=maxvalue;
  }
  
void InsertMaximumPull() 
  {
    char pullString = '0' ;
    while( pullString != '\n' && Serial.available() > 0 ) 
    { 
        String pullString = Serial.readStringUntil('\n');
        pullString.trim();
        pullvalue=pullString.toInt();
        pullvalue=pullvalue*(pullvalue>0);
        if (pullvalue==0){
          Serial.println("Try Again");
          }
          else{
            Serial.println("pull limit is:");
            Serial.println(pullvalue);
          }
    }

    if( pullString == '\n' ) 
    {  
        Serial.println(pullvalue) ; 
        pullString = "" ;
        timer3 = 0 ;
    }
}

void SelectResistanceLevel() 
  {
    char weightString = '0' ;
    while( weightString != '\n' && Serial.available() > 0 ) 
    { 
        String weightString = Serial.readStringUntil('\n');
        weightString.trim();
        weightvalue=weightString.toInt();
        weightvalue=weightvalue*(weightvalue>0)*(weightvalue<5000);
        if (weightvalue==0){
          Serial.println("Try Again");
          }
          else{
            Serial.println("weight limit (grams) is:");
            Serial.println(weightvalue);
          }
    }

    if( weightString == '\n' ) 
    {  
        Serial.println(weightvalue) ; 
        pullString = "" ;
        timer5 = 0 ;
    }
}

void ValidValues()
{
      if (minvalue<maxvalue & minvalue >0 & maxvalue >0 & pullvalue >0 & check1==0){
        Serial.println("Valid values");
        Serial.print("Min value is:");
        Serial.println(minvalue);
        Serial.print("Max value is:");
        Serial.println(maxvalue);
        Serial.print("Pull limit is:");
        Serial.println(pullvalue);
        check1=1;
    }
    else{}
}

void PermissionRequest()
{
    char permitString = '0' ;
    while( permitString != '\n' && Serial.available() > 0 ) 
    { 
        String pullString = Serial.readStringUntil('\n');
        pullString.trim();
        pullvalue=pullString.toInt();
        pullvalue=pullvalue*(pullvalue>0);
        if (pullvalue==0){
          Serial.println("Try Again");
          }
          else{
            Serial.println("pull limit is:");
            Serial.println(pullvalue);
          }
    }

    if( pullString == '\n' ) 
    {  
        Serial.println(pullvalue) ; 
        pullString = "" ;
        timer3 = 0 ;
    }
}

void DetermineLimits()
{
    //MINIMUMVALUE ENTRY
  
  if (minvalue==0 & timer == 1)
  {
    InsertMinimumDistance();
  }
  else if(minvalue==0)
  {
    delay(1000);
    Serial.println("How much is your minimum length limit in cm?");
    timer++;
  }
  else{}
  //MAXIMUM VALUE ENTRY  
  if (minvalue>0 & maxvalue==0){
  if (timer2 == 1 & maxvalue==0){
    InsertMaximumDistance();
    }
  else if(maxvalue==0){
    delay(1000);
    Serial.println("How much is your maximum length limit in cm?");
    timer2++;
  }
  else{}
  }
if (minvalue>0 & maxvalue>0 & pullvalue==0){  
if (timer3 == 1 & pullvalue==0){
    InsertMaximumPull();
    }
  else if(pullvalue==0){
    delay(1000);
    Serial.println("How much is your pull limit?");
    timer3++;
  }
  else{}
}

void CalibrateSensorSU(int sensorPin, int LEDoutPin)
{
  // turn on LED to signal the start of the calibration period:
  pinMode(LEDoutPin, OUTPUT);
  digitalWrite(LEDoutPin, HIGH);

  // calibrate during the first five seconds
  while (millis() < 5000) {
    sensorValue = analogRead(sensorPin);

    // record the maximum sensor value
    if (sensorValue > sensorMax) {
      sensorMax = sensorValue;
    }

    // record the minimum sensor value
    if (sensorValue < sensorMin) {
      sensorMin = sensorValue;
    }
  }

  // signal the end of the calibration period
  digitalWrite(LEDoutPin, LOW);
}

void CalibrateSensorLP(int sensorPin, int LEDoutPin)
{
  sensorValue = analogRead(sensorPin);

  // in case the sensor value is outside the range seen during calibration
  sensorValue = constrain(sensorValue, sensorMin, sensorMax);

  // apply the calibration to the sensor reading
  sensorValue = map(sensorValue, sensorMin, sensorMax, 0, 255);

  // fade the LED using the calibrated value:
  analogWrite(ledPin, sensorValue);
}
