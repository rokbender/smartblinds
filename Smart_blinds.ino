#include <SoftwareSerial.h>
#include <avr/sleep.h> //бібліотека сну
#include <avr/power.h>
SoftwareSerial BTserial(9, 11); //tx  rx HM-10
String btBuffer;

#include <EEPROM.h>
int address = 0; //0-комірка памяті це положення а 1-це кут а 5-це Before mode

int MotorPins[4] = {4, 5, 7, 8};
const int OneTurnPhasesCount = 8;
int TurnPhasesDelay = 2;
int CurrentPhase = 7;
unsigned long lasttiming=0;
unsigned long stepper;
bool MotorTurnPhases[8][4] = {
  { 1, 1, 0, 0},
  { 0, 1, 0, 0},
  { 0, 1, 1, 0},
  { 0, 0, 1, 0},
  { 0, 0, 1, 1},
  { 0, 0, 0, 1},
  { 1, 0, 0, 1},
  { 1, 0, 0, 0} };

float vout = 0.0;           //
float vin = 0.0;            //
float R1 = 9890.0;          // сопротивление R1 (10K)
float R2 = 9840.0;         // сопротивление R2 (1,5K)
float max_v = 4.25;        // максимальний заряд аккумулятора
float min_v = 3.00;        // минимальный заряд аккумулятора
int val = 0;
int v=0;
float s,num;

unsigned long timing=0;
int Mode=1; //1-Auto,2-Time,3-Degree
int Hour;int Min;
 int State,beforeMode;
boolean polozhenia;//true це жалюзі відкриті,false закриті
int kut=282;
int kutNow;
int dopov=15;//доповнення до закривання 15 градусів;
int Deg;// приходять значення кута від 0 до 360;
int ValueSvitla;
#define DatchykSvitla 3 //до якого піна підключений
#define PowerBLE 2
#define analogvolt A0
void wakeUp()
{  

}
void EnterSleep()
{
  Serial.println("SON");
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  attachInterrupt(0, wakeUp, RISING);
  if(Mode==1)
  {attachInterrupt(1, wakeUp, CHANGE);}//Если на 0-вом прерываниии - ноль, то просыпаемся.
  delay(500);

  sleep_enable(); //Разрешаем спящий режим
  sleep_mode();
  detachInterrupt(0);
  if(Mode==1)
  {detachInterrupt(1);}//Отключаем прерывания
  sleep_disable();//Запрещаем спящий режим//Спим (Прерывания продолжают работать.) Программа останавливается.
  loop();
}
void engine(int pov,bool Run,unsigned long Grad)
{  int Step=Grad*(11.38);
  while(Run){
  for (int i = 0; i < 4; i++)

    {
      digitalWrite(MotorPins[i], ( (MotorTurnPhases[CurrentPhase][i] == 1) ? HIGH : LOW) );
    }
    CurrentPhase += pov;
    stepper++;
    delayMicroseconds(1700);
   if (CurrentPhase>7){CurrentPhase=0;}
   if (CurrentPhase<0){CurrentPhase=7;}

     if (stepper==Step)
     {
       
      stepper=0;
        for (int i = 0; i < 4; i++)
        {
          digitalWrite(MotorPins[i],LOW );
        }
        Run=false;
     }

}
  }

void Time()
{
  Serial.println(Hour);
  Serial.println(Min);
  
}

void setup() {
 
Serial.begin(9600);
while (!Serial) { /* Wait until serial is ready */ }
Serial.println("begin");// put your setup code here, to run once:
BTserial.begin(9600);
//EEPROM.write(0,0);
polozhenia = EEPROM.read(address); 

//якщо перший раз записується ардуіно то в комірку потрібно записати значення EEPROM.write(0,0);це 0 закриті жалюзі
EEPROM.get(1,kutNow); 
beforeMode = EEPROM.read(5); 
pinMode(DatchykSvitla,INPUT);
pinMode(PowerBLE,INPUT);
pinMode(analogvolt,INPUT);
  for (int i = 0; i < 4; i++)
  {
    pinMode(MotorPins[i], OUTPUT);
    digitalWrite(MotorPins[i],LOW);
  }
  Serial.print("BeforeMode=");
  Serial.println(beforeMode);
  Serial.print("Kutnow=");
  Serial.println(kutNow);
}
void SendData(){
  BTserial.print("*");
  BTserial.print("Mode=");
  BTserial.print(Mode);
  BTserial.println("/");
  BTserial.print("Position=");
  BTserial.print(polozhenia);
  BTserial.println("/");
  BTserial.print("Angle_now=");
  BTserial.print(kutNow);
  BTserial.println("/");

   
}
void Povorot()
{ Serial.println("tyt");
  Serial.println(kutNow);
  
  if(beforeMode == 3) {
    
    if(State == 0) kut=kutNow; //BeforeMode має дорівнювтиа 3
    
   else if(State == 1){    //відкривання
    
     if((282 - kutNow) < 0){
      engine(1,true,kutNow-210);//190 це просто підлаштовано шоб воно вернулоя в знач open
      polozhenia=true; //BeforeMode немає дорівнювтиа 3
      Serial.println("tut54");
      beforeMode=Mode;
      kutNow=282;
      }
      else if((282-kutNow)>0){ kut=(300-kutNow); Serial.println("1235t"); }  //BeforeMode має дорівнювтиа 3
      else {polozhenia=true; beforeMode=Mode; }  //BeforeMode немає дорівнювтиа 3
    }
   
   }
   else {kut=282;dopov=15;}
  
  if(State==0 and (polozhenia==true or beforeMode==3))
   {
    Serial.println("1"); // +15 градусів для закривання можна добавити закривання
    engine(1,true,kut+dopov);
    polozhenia=false;
    kutNow=-15;
   }
 if(State==1 and (polozhenia==false or beforeMode==3))
   {
    Serial.println("2"); // відкривання
    engine(-1,true,kut); 
    polozhenia=true;
    kutNow=282;
   }
   
   beforeMode=Mode;
   EEPROM.update(address,polozhenia);
   EEPROM.update(5, beforeMode);
   EEPROM.put(1, kutNow);
   //SendData();
   if(State==digitalRead(DatchykSvitla) and Mode==1) //перевірка якшо під час роботи мотора помінялося значення датчика то виконуєть знов фція Auto
     {Auto();}
if (digitalRead(PowerBLE)==LOW)   
   EnterSleep();
 
}
void Auto()
{
  State=!digitalRead(DatchykSvitla);
  delay(2000);
  
  
Povorot();    
}

void Degree(){
  
  int kut;
  int StateDeg;
  bool flag=true;
  bool flagRun=true;
  if (Deg > 450 or Deg < 0 ){
    BTserial.println("Not right angle.Enter again");
    flagRun=false;
  }
  if(flagRun){
   
  //if(beforeMode !=3) kutNow = (!polozhenia)? -15 : 282;
  
 /* if (polozhenia == false and beforeMode !=3 )  //жалюзі закриті;
  {
    kutNow=0; 
    }
    if (polozhenia == true and beforeMode !=3 )  //жалюзі відкриті;
  {
    kutNow=282; 
    }*/

    
    kut=abs(Deg-kutNow); // 360-282=78
    
    if     (Deg>kutNow) StateDeg =-1;
    else if(Deg<kutNow) StateDeg = 1;
    else if(Deg==kutNow) flag=false;
    
   /* if(Deg>kutNow)  // значить потрібно відкривати жалюзі
    {
      StateDeg=-1;
      }
      if(Deg<kutNow)  // значить потрібно відкривати жалюзі
    {
      StateDeg=1;
      }
      if(Deg==kutNow)
      {
        flag=false;
        } */

    if (flag){
    engine(StateDeg,true,kut);
    kutNow = Deg;
    beforeMode = Mode;
    EEPROM.put(1, kutNow); //кут
    EEPROM.update(5, beforeMode); //Попередній режим
    //SendData();
    } 

  }
    if (digitalRead(PowerBLE)==LOW)   EnterSleep();
    else loop();
  
}

  

void refreshMode(){
  switch (Mode) {
    case 0:
      Povorot();
      break;
    case 1:
      Auto();
      break;
    case 2:
      Time();
      break;
    case 3:
      Degree();
      break;      
    default:   
      break; 
  }
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
void processCommand(){

  if (btBuffer.startsWith("Open")) {
    //long R = getValue(btBuffer, ',', 1).toInt();
    Serial.println("Open");
    Mode=0;
    State=1;
    //Serial.println(R);
  } else if (btBuffer.startsWith("Close")) {
    Serial.println("Close");
    Mode=0;
    State=0;
    }
  else if (btBuffer.startsWith("Auto")) {
    Serial.println("Auto");
    Mode=1;
    }
    else if (btBuffer.startsWith("Degree")) {
    Serial.println("Degree");
    Mode=3;
    Deg=getValue(btBuffer, ',', 1).toInt();
    }
    else if (btBuffer.startsWith("Volt")) {
    Serial.println("Volt");
    volt();
    }
  else if (btBuffer.startsWith("Time")) {
    Serial.println("Time");
    Hour = getValue(btBuffer, ',', 1).toInt();
    Min = getValue(btBuffer, ':', 1).toInt();
    Mode=2;
    }      
    else
    {Serial.println(btBuffer);
      Serial.println("Error");
      BTserial.println("Error");
    btBuffer = "";
    }

    refreshMode();
}
void volt()
{ s = 0;
  num = 0;
  for (int i = 0; i < 5; i++)
  {
    val = analogRead(analogvolt);
    vout = (val * 5.14) / 1024.0;
    vin = vout / (R2 / (R1 + R2));
    if (vin < 0.09)
    {
      vin = 0.0; // обнуляем нежелательное значение
    }
    s += vin;
  }
  num = s / 5.0;
  // num+=0.09;
  if (vin <= 6) {
    int proc = ((vin - min_v) / (max_v - min_v)) * 100;
    Serial.println(num);
    BTserial.print("Voltage=");
    BTserial.print(num);
    BTserial.println("/");
    
  }

  v = 0;
  timing=millis();
  Serial.println(millis() - timing);
}


void loop() {
  delay(10);
 if( digitalRead(PowerBLE)==HIGH)
 { 
  if (BTserial.available())
  {
    char received = BTserial.read();
    btBuffer += received; 
   // Serial.println(btBuffer);
    if (received == '|')
    {
        processCommand();
        btBuffer = "";
     }
 }

    if (millis() - timing > 5000 and BTserial.available()==false)
      {
        SendData();
        volt();
        BTserial.print("Brigtness=");
        BTserial.print(analogRead(A3));
        BTserial.print("/");
        BTserial.println("#");
      }
      
       
       
      
 }
 else{
      refreshMode();
 }  
}
