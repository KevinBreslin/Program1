String order = "";
int timer;
int timer2;
int timer3;
int timer4;
int check1=0;
int serialordial =0;
float minvalue=0.0;
float maxvalue=0.0;
int pullvalue=0;
String pullString;
String SoDString;

bool permission = false;
//bool validation;

char choicearray[3][10] = {
                         "Serial",
                         "Dial",
                         "Invalid"
                     };
int SoD=0;
int timer0 =0;

void setup() {
  Serial.begin(9600);
  timer = 0;
  timer0 =0;
  timer2 = 0;
  timer3 = 0;
  timer4 = 0;
}

void loop() {
  SerialorDial();
  if (SoD=! 0){
  Serial.println(SoD);
  }
//SerialorDial();
//DetermineLimits();
//ValidValues();

}

void SerialorDial()

{
   //SERIAL OR DIAL CHOICE
  
  if (SoD==0 & timer0 == 1)
  {
    char SoDString = '0' ;
    while( SoDString != '\n' && Serial.available() > 0 ) 
    { 
        String SoDString = Serial.readStringUntil('\n');
        SoDString.trim();
        SoD=SoDString.toInt();
        SoD=SoD*(SoD>0)*(SoD<3);
        if (SoD==0){
          Serial.println("Try Again");
          }
        else{
        Serial.println("Serial/Dial Choice:");
        Serial.println(SoD);}
        if(SoD==1){
          Serial.println("Serial chosen");
          }
          else if (SoD==2){
          Serial.println("Dial chosen");
          }
        //Serial.readBytes( &ch, 1 ) ; 
        //order += ch ;
    }

    if( SoDString == '\n' ) 
    {  
        Serial.println(SoD) ; 
        SoDString = "" ;
        timer0 = 0 ;
    }
  }
  else if(SoD==0)
  {
    delay(1000);
    Serial.println("Do you want to use the Serial (1) or the Dial (2)?");
    timer0++;
  }
 
}


void SerialorDial2()
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
    else{}
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
        maxvalue=maxvalue*(maxvalue>0)*(maxvalue>minvalue);
        if (maxvalue==0){
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
}
