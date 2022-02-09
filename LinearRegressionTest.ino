# include <math.h>;
# include <stdio.h>;

float x[10]= { 0 };
float y[10]= { 0 };
float xs[10]= { 0 };
float xy[10]= { 0 };
float ys[10]= { 0 };
int rows=10;
int n=0;
int i;
float Sx=0;
float Sy=0;
float Sxs=0;
float Sxy=0;
float Sys=0;
float alph=0;
float bet=0;
float gamm=0;
float delt=0;
float r = 0;
float sepss=0;
float sbets=0;
float salphs=0;
String s1; 
String s2; 

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600); 

}

void loop() {

float x[10]={1,2,3,4,5,6,7,8,9,10};
float y[10]={0.01,0.08,0.11,0.41,0.53,0.62,0.63,0.79,0.88,0.91};
int rows=10;
int n=rows;

//char buffer[10];
//sizeof(x)/sizeof(int);
  // put your main code here, to run repeatedly:
//Serial.println(rows);
for ( int i = 0; i < rows; ++i ) {
Sx +=x[i];
Sy +=y[i];  
Sxs +=(x[i]*x[i]);
Sxy +=(x[i]*y[i]);
Sys +=(y[i]*y[i]);
}
//Serial.println(Sx);
//Serial.println(Sy);
//Serial.println(Sxs);
//Serial.println(Sxy);
//Serial.println(Sys);
bet=(((n*Sxy)-(Sx*Sy))/(n*Sxs-Sx*Sx));
alph=(1/n)*Sy-(bet*1/n)*Sx;
r=((n*Sxy)-(Sx*Sy))/(sqrt((n*Sxs-Sx*Sx)*((n*Sys-(Sy*Sy)))));
sepss=(1/(n*(n-2)))*(n*Sys-(Sy*Sy)-(bet*bet)*(n*Sxs-(Sx*Sx)));
sbets=(n*sepss)/(n*Sxs-(Sx*Sx));
salphs=(sbets/n)*Sxs;
gamm=1/bet;
delt=-alph/bet;

s1 = "  Formula y = " + String(bet, 5) + "x + " + String(alph,5); 
Serial.println(s1);
s2 = "  Formula x = " + String(gamm, 5)+ "y + " + String(delt,5); 
Serial.println(s2);
Serial.println("");

Serial.print("  alpha coefficent: "); 
Serial.println(alph);
Serial.print("  standard error (alpha): "); 
Serial.println(String(salphs, 6));
Serial.println("  ");

Serial.print("  beta coefficent: ");
Serial.println(bet);
Serial.print("  standard error (beta): "); 
Serial.println(String(sbets, 6));
Serial.println("  ");
Serial.print("  product-moment correlation coefficient: "); 
Serial.println(r);
Serial.println("  ");
Serial.print("  standard error (epsilon): "); 
Serial.println(sepss);
Serial.println("  ");
Serial.println("  ");

delay(100000);

;
/*
for ( int i = 0; i < rows; ++i ) {
  Sx = Sx + x[i];
  Sy = Sy + y[i];
  Sxs = Sxs + x[i]*x[i];
  Sxy = Sxy + x[i]*y[i];
  Sys = Sys + y[i]*y[i];
  // loop through columns of current row
  //return Sx, Sy, Sxs, Sxy, Sys;
}


Serial.println(Sx);
int VoltagePin = 9;    // PWM signal connected to digital pin 9

// Need a calibration curve to assign a PWM value to a particular weight;
//Each weight should have a corresponding PWM value, assumed to be linear at this stage.
int Weight1 = 51;
int Weight2 = 102;
int Weight3 = 153;
int Weight4 = 204;
int Weight5 = 255;

//String command;

void setup() {
    Serial.begin(9600);
    pinMode(VoltagePin, OUTPUT);
    Serial.println("How much weight do you wish to use (5, 4, 3, 2, 1, off)"); 

}
 
void loop() {
  
    if(Serial.available()){
        
        String command = Serial.readStringUntil('\n');
        command.trim();
        Serial.println("You have selected Setting " + command + "!");
        
        if(command == "5"){
          Serial.println("Setting 5 selected");
         digitalWrite(VoltagePin, Weight5); // Set Weight to Weight 5
        }  
        else if(command == "4"){
          Serial.println("Setting 4 selected");
         digitalWrite(VoltagePin, Weight4); // Set Weight to Weight 4
        }
        else if(command == "3"){
          Serial.println("Setting 3 selected");
         digitalWrite(VoltagePin, Weight3); // Set Weight to Weight 3
        }
        else if(command == "2"){
          Serial.println("Setting 2 selected");
         digitalWrite(VoltagePin, Weight2); // Set Weight to Weight 2
        }
        else if(command == "1"){
          Serial.println("Setting 1 selected");
         digitalWrite(VoltagePin, Weight1); // Set Weight to Weight 1
        }
        else if(command == "off"){
          Serial.println("No weight selected");
         digitalWrite(VoltagePin, 0); // Set Weight to Weight 0
        }
        else{
          Serial.println("No weight selected");
          digitalWrite(VoltagePin, 0); // Set Weight to Weight 0
        }
    }
}


*/
}
