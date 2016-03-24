#include <PID_v1.h>
#include <Servo.h>
#include <SharpIR.h>
#include <Wire.h> //I2C Arduino Library
#include "arduinoFFT.h"

#define ir A0
#define model 20150

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


#define HMC5883L_ADDR 0x1E //0011110b, I2C 7bit address of HMC5883

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

Servo myservo;
Servo myservo_gripper;
Servo myservo_hook;

bool haveHMC5883L = false;

  // create servo object to control a servo
  // variable to read the value from the analog pin
int state,freq_stop,freq_start,start;
const int mz80=9;
const int mz80_right=12;
const int mz80_left=11;
const int IN1 = 5;
const int IN2 = 6;
const int IN3 = 7;
const int IN4 = 8;

const int PWM_LIMIT = 255;
double Input, Output,Setpoint;
double kP=10, kI=0.01, kD=4;
PID myPID(&Input, &Output, &Setpoint, kP, kI, kD, REVERSE);

SharpIR SharpIR(ir, model);

      int forward(){
      Serial.println("I'am in forward");
      myservo.write(90);
      analogWrite(IN2, 245);
      analogWrite(IN1, 0);
      analogWrite(IN4, 255);
      analogWrite(IN3, 0);
      }
      int backward(){
      while(true){
      Serial.println("I'am in backward");
      analogWrite(IN2, 0);
      analogWrite(IN1, 245);
      analogWrite(IN4, 0);
      analogWrite(IN3, 255);
      }
      }
      int left(int motor){
      Serial.println("I'am in motor");
      analogWrite(IN2, 0);
      analogWrite(IN1, motor);
      analogWrite(IN4, motor);
      analogWrite(IN3, 0);
      }
      int right(int motor2){
      Serial.println("I'am in motor2");
      analogWrite(IN2, motor2);
      analogWrite(IN1, 0);
      analogWrite(IN4, 0);
      analogWrite(IN3, motor2);
      }
         int Stop(){
      Serial.println("I'am in stop");
      myservo.write(90);
      myservo_gripper.write(90);
      analogWrite(IN2, 0);
      analogWrite(IN1, 0);
      analogWrite(IN4, 0);
      analogWrite(IN3, 0);
      }
int fft(){
  Serial.println("I'am in fft");
  const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
  const double samplingFrequency = 47000;

  double vReal[samples]; 
  double vImag[samples];

   for (uint8_t i = 0; i < samples; i++) 
  {
    int mic= analogRead(A1);
    vReal[i] = mic;
  }
 
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */ 
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  memset(vReal, 0, sizeof(vReal));
  memset(vImag, 0, sizeof(vImag));
  return x;
  
  }

  
int getMin(int* array, int size)
{
  int minimum = array[0];
  int min_i;
  for (int i = 0; i < size; i++)
  {
    if (array[i] < minimum) {
      minimum = array[i];
      min_i = i;
    }
  }
  return min_i;
}
int motor(int degree_handle, int distance_handle)
{
  Serial.println("I'am in motor");
  Serial.println(degree_handle);
  int distance=SharpIR.distance();
  delay(50);
  if (degree_handle < 0 && degree_handle > 180) {
    analogWrite(IN2, 0);
    analogWrite(IN1, 0);
    analogWrite(IN4, 0);
    analogWrite(IN3, 0);
  }
  if (degree_handle >85 && degree_handle<95 )
  {
          analogWrite(IN2, 255);
          analogWrite(IN1, 0);
          analogWrite(IN4, 255);
          analogWrite(IN3, 0);
        
  }
  else if (degree_handle >= 95 && degree_handle <= 180) {
    while(distance>(distance_handle+30))
    {
      distance=SharpIR.distance();
      delay(50);
      
      analogWrite(IN2, 0);
      analogWrite(IN1, 50);
      analogWrite(IN4, 50);
      analogWrite(IN3, 0);
    }
    analogWrite(IN2, 0);
    analogWrite(IN1, 0);
    analogWrite(IN4, 0);
    analogWrite(IN3, 0);
  }
  else if (degree_handle <= 85 && degree_handle >= 0) {
   while(distance>(distance_handle+30))
    {
      distance=SharpIR.distance();
      delay(50);
     
      analogWrite(IN2, 50);
      analogWrite(IN1, 0);
      analogWrite(IN4, 0);
      analogWrite(IN3, 50);
    }
    analogWrite(IN2, 0);
    analogWrite(IN1, 0);
    analogWrite(IN4, 0);
    analogWrite(IN3, 0);
    }
}


bool detectHMC5883L ()
{
  // read identification registers
  Wire.beginTransmission(HMC5883L_ADDR); //open communication with HMC5883
  Wire.write(10); //select Identification register A
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 3);
  if(3 == Wire.available()) {
    char a = Wire.read();
    char b = Wire.read();
    char c = Wire.read();
    if(a == 'H' && b == '4' && c == '3')
      return true;
  }

  return false;
}

  int compass(){
    
    bool detect = detectHMC5883L();
  float heading;
  float headingDegrees;
  if(!haveHMC5883L) 
  {
    if(detect) 
    {
      haveHMC5883L = true;
      Serial.println("We have HMC5883L, moving on");
      // Put the HMC5883 IC into the correct operating mode
      Wire.beginTransmission(HMC5883L_ADDR); //open communication with HMC5883
      Wire.write(0x02); //select mode register
      Wire.write(0x00); //continuous measurement mode
      Wire.endTransmission();
    }
    else
    {  
      Serial.println("No HMC5883L detected!");
      delay(2000);
      return 0;
    }
  }
  else
  {
    if(!detect) {
      haveHMC5883L = false;
      Serial.println("Lost connection to HMC5883L!");
      delay(2000);
      return 0;
    }
  }
  
  int x,y,z; //triple axis data

  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
  Wire.requestFrom(HMC5883L_ADDR, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb

    heading = atan2(y, x);
  if(heading < 0)
    heading += 2*PI;

  if(heading > 2*PI)
    heading -= 2*PI;

  headingDegrees = heading * 180/M_PI; 
  }
  
  return headingDegrees;
  delay(250);
    }


    
  int backtohome(int pos_home){
    Serial.println("I'am in backtohome");
    while(true){
      double pos_dif_motor,pos_dif,motor_pwm;
      double pos_current=compass();

      if(pos_home>pos_current){
        pos_dif=pos_home-pos_current;
        if(pos_dif>180){
          pos_dif_motor=360-pos_dif;
          if(pos_dif_motor>10){
            left(motor_pwm);
            }
          else if(pos_dif_motor<=10){
            forward();
            }
          }
        else if(pos_dif<180){
          pos_dif_motor=pos_dif;
          if(pos_dif_motor>10){
            right(motor_pwm);
            }
            else if(pos_dif_motor<=10){
            forward();
            }
          }
         else{
            forward();
          }
        }
      else if(pos_home<pos_current){
        pos_dif=pos_current-pos_home;
        if(pos_dif>180){
          pos_dif_motor=360-pos_dif;
           if(pos_dif_motor>10){
            right(motor_pwm);
            }
            else if(pos_dif_motor<=10){
            forward();
            }
          }
        else if(pos_dif<180){
          pos_dif_motor=pos_dif;
          if(pos_dif_motor>10){
            left(motor_pwm);
            }
          else if(pos_dif_motor<=10){
            forward();
            }
          }
        else{
            forward();
          }
        }
        
        else{
            forward();
          }

      Setpoint=pos_dif_motor;
      Input=pos_dif_motor;
      myPID.Compute();
      motor_pwm=Output;
      /*
      Serial.println(pos_dif_motor);
      Serial.println("motor_pwm");
      Serial.println(motor_pwm);
     */
      }
    }

    
  int state_0(){
      Serial.println("I'am in state_0");
      int inc[72] = {
        0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105,
        110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180
        , 175, 170, 165, 160, 155, 150, 145, 140, 135, 130, 125, 120, 115, 110, 105, 100, 95, 90,
        85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5
      };
      int radar[72] = {};
      int dis_handle,handle, degree;
      int i=0;
      int val;
      while(i<=72){
      int dis = SharpIR.distance();
       val = inc[i];
       radar[i] = dis;
       myservo.write(val);//100-178 aralık// sets the servo position according to the scaled value
       //Serial.println(val);
       delay(50);
      // waits for the servo to get there
      i = i + 1;
      if (i == 73)
      {
        handle = getMin(radar, 72);
        degree = inc[handle];
        dis_handle = radar[handle];
        myservo.write(90);
        delay(50);
        Serial.println(degree);
        motor(degree,dis_handle);
       }
     }
     return dis_handle;
    }
    
    int go(int distance_handle){
          Serial.println("I'am in go");
          int mid=digitalRead(mz80);
          int distance=SharpIR.distance();
          unsigned long previousMillis;       
          const long interval = 1000;
           unsigned long currentMillis = millis();
           previousMillis=currentMillis;
          while(mid==1 && (currentMillis - previousMillis < interval)){
            currentMillis = millis();
            Serial.println(currentMillis - previousMillis);
            forward();
            }
          while(mid==1 && (currentMillis - previousMillis >= interval)){
          currentMillis = millis();
          int left=digitalRead(mz80_left);
          int right=digitalRead(mz80_right);
          mid=digitalRead(mz80);
          distance=SharpIR.distance();
          if(left==0 && right==1){
            //şimdi sağ gördü hafif sola dön taki sharp en azı görene kadar(motordan al)
                analogWrite(IN2, 75);
                analogWrite(IN1, 0);
                analogWrite(IN4, 0);
                analogWrite(IN3, 75);
             /*while(distance>(distance_handle+20))
              {
                distance=SharpIR.distance();
                delay(50);
                analogWrite(IN2, 75);
                analogWrite(IN1, 0);
                analogWrite(IN4, 0);
                analogWrite(IN3, 75);
              }
              forward();
              distance_handle=SharpIR.distance();*/
            }
          else if(left==1 && right==0){
            //şimdi sol gördü hafif sağa döne ama devam et sharp en azı görene kadar(motordan al)
                analogWrite(IN2, 0);
                analogWrite(IN1, 75);
                analogWrite(IN4, 75);
                analogWrite(IN3, 0);
                 /*while(distance>(distance_handle+20))
              {
                distance=SharpIR.distance();
                delay(50);
                analogWrite(IN2, 0);
                analogWrite(IN1, 75);
                analogWrite(IN4, 75);
                analogWrite(IN3, 0);
              }
              forward();
              distance_handle=SharpIR.distance();*/
            }
          else if(left==0 && right==0 ){
            forward();
            //distance_handle=SharpIR.distance();;
            }
          else if(left==1 && right==1){
            forward();
            //distance_handle=SharpIR.distance();;
            }
          }
      }

      
      int gripper(int Open){
        if(Open==1){
        myservo.write(90);
        myservo_gripper.write(178);
        delay(50);
        }
        else if(Open==0){
        myservo.write(90);
        myservo_gripper.write(80);
        delay(50);
        }
        else{
        myservo.write(90);
        myservo_gripper.write(80);
        delay(50);
        }
        }
void setup() {
  myservo_gripper.attach(10);
  myservo.attach(4);// attaches the servo on pin 9 to the servo objec
  myservo_hook.attach(2);
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, PWM_LIMIT);
  myPID.SetSampleTime(50);
  Wire.begin();
  // lower I2C clock http://www.gammon.com.au/forum/?id=10896
  TWBR = 78;  // 25 kHz 
  TWSR |= _BV (TWPS0);  // change prescaler  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(mz80, INPUT);
  pinMode(mz80_left, INPUT);
  pinMode(mz80_right, INPUT);
  pinMode(A1, INPUT);
  sbi(ADCSRA, 1);
  cbi(ADCSRA, 1);
  cbi(ADCSRA, 0);
  
  delay(100);
  Serial.begin(9600);
  state=0; // fft çalışırken 10 yap
  freq_stop=0;
  freq_start=0;
  start=0;
  myservo_hook.write(0);
}


void loop() {
  // scale it to use it with the servo (value between 0 and 180)
    int pos_home,dist_handle,pos_handle;
    int freq=fft();
    Serial.println(freq);
    Serial.println(freq_start);
    /*
    if(freq<=6000 && freq>=4000){
    freq_start=freq_start+1;
    }
    if(freq_start==10){
      freq_start=0;
      state=0;
      }
     if(freq<=11000 && freq>=9000){
      freq_stop=freq_stop+1;
    }
    if(freq_stop==10){
      freq_stop=0;
      state=3;
     }
    */
    
    if(state==0){
    pos_home=90;
    myservo_hook.write(0);
    state_0();
    dist_handle=SharpIR.distance();
    state=1;
    }
    if(state==1){
    if(digitalRead(mz80)==HIGH){
      go(dist_handle);
      gripper(0);
      }
     else{
      Stop();
      gripper(1);
      myservo_hook.write(90);
      state=2;
      }
    }
  if(state==2){
    backtohome(pos_home);
  }
  if(state==3){
    Stop();
    }
}
