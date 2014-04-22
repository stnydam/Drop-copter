/*
 HC-SR04 Ping distance sensor]
 VCC to arduino 5v GND to arduino GND
 Echo to Arduino pin 13 Trig to Arduino pin 12
 Red POS to Arduino pin 11
 Green POS to Arduino pin 10
 560 ohm resistor to both LED NEG and GRD power rail
 More info at: http://goo.gl/kJ8Gl
 Original code improvements to the Ping sketch sourced from Trollmaker.com
 Some code and wiring inspired by http://en.wikiversity.org/wiki/User:Dstaub/robotcar
 */
 
#include <Servo.h>

#define trigPin 8
#define echoPin 9
#define trig 10
#define echo 11
#define ledb 12
#define ledr 22
#define ledg 13
#define roll_pin 2
#define pitch_pin 3
#define throttle_pin 20
#define yaw_pin 21
#define servo_pin 18
#define servo2_pin 19
#define rolli_pin 0
#define pitchi_pin 1
#define throttlei_pin 3
#define yawi_pin 2
#define servoi_pin 5
#define servo2i_pin 4
#define rolls 45
#define pitchs 5
#define thrs 6
#define yaws 7
#define auxs 44
#define pay 46
volatile unsigned long timer_startr;
volatile unsigned long timer_startp;
volatile unsigned long timer_startt;
volatile unsigned long timer_starty;
volatile unsigned long timer_starts;
volatile unsigned long timer_starts2;

volatile int pulse_timer;
volatile int pulse_timep; 
volatile int pulse_timet; 
volatile int pulse_timey; 
volatile int pulse_times; 
volatile int pulse_times2; 

volatile int last_interrupt_timer;
volatile int last_interrupt_timep;
volatile int last_interrupt_timey;
volatile int last_interrupt_timet;
volatile int last_interrupt_times;
volatile int last_interrupt_times2;

int n=0,pn=0;
int ROLL, PITCH, THR, YAW, AUX1, AUX2, AUX3, AUX4, desiredT, throttle, altitude, not_started, plip, event, event1, time, d, altit;
int ave[5]={0};
int rc_signals[8] = { 1234 };

Servo servor;
Servo servop;
Servo servot;
Servo servoy;
Servo servoa;
Servo servo_pay;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(ledr, OUTPUT);
  pinMode(ledb, OUTPUT);
  pinMode(ledr, OUTPUT);
  pinMode(ledg, OUTPUT);
  timer_startr = 0;
  timer_startp = 0;
  timer_startt = 0;
  timer_starty = 0;
  timer_starts = 0;
  timer_starts2 = 0;
  attachInterrupt(rolli_pin, calcSignalr, CHANGE);
  attachInterrupt(pitchi_pin, calcSignalp, CHANGE);
  attachInterrupt(throttlei_pin, calcSignalt, CHANGE);
  attachInterrupt(yawi_pin, calcSignaly, CHANGE);
  attachInterrupt(servoi_pin, calcSignals, CHANGE);
  attachInterrupt(servo2i_pin, calcSignals2, CHANGE);
  Serial.begin(115200);
  not_started = 1;
  throttle = 1101;
  plip = 0;
  d=1;
  servor.attach(rolls);
  servop.attach(pitchs);
  servoy.attach(yaws);
  servot.attach(thrs);
  servoa.attach(auxs);
  servo_pay.attach(pay);
}

void loop(){
  servo_pay.write(0);
  while(pulse_times > 1500){
    rc_signals[1] = pulse_timer;
    rc_signals[2] = pulse_timep;
    rc_signals[3] = pulse_timet;
    rc_signals[4] = pulse_timey;
    rc_signals[5] = 1800;
    set_rc(rc_signals);
    throttle = rc_signals[3];
  }
  while(pulse_times2 > 1500){
    altitude = alt();
    throttle = land(throttle, altitude);
    rc_signals[1] = 1500;
    rc_signals[2] = 1500;
    rc_signals[3] = throttle;
    rc_signals[4] = 1500;
    rc_signals[5] = 2000;
    set_rc(rc_signals);
    throttle = rc_signals[3];
  }
  while(avoidance){
    rc_signals[1] = 1500;
    rc_signals[2] = 1400;
    rc_signals[3] = throttle;
    rc_signals[4] = 1500;
    rc_signals[5] = 2000;
    set_rc(rc_signals);
    throttle = rc_signals[3];
  }
  /*demo();*/
}

int alt(){
  if(d>10){
    d=1;
  }
  int dur, dist;
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  dur = pulseIn(echo, HIGH);
  ave[d] = (dur/2) / 29.1;
  dist = (ave[1]+ave[2]+ave[3]+ave[4]+ave[5])/5;
  d++;
  return dist;
}

int land(int throttle, int altitude){
  int prev_alt;
  prev_alt = altitude;
  delay(200);
  altitude = alt();
  if (throttle > 1100 && altitude > 100){
    if(prev_alt - altitude < 4){
      throttle = throttle - 4;}
    else{
      throttle = throttle + 6;}
  }
  else if(throttle > 1100 && altitude < 100){
    if(prev_alt - altitude < 2){
      throttle = throttle - 2;}
    else{
      throttle = throttle + 3;}
  }
  return throttle;
  }  

int avoidance() {
  if(n=pn){
    n=0;}
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  pn = n;
  if(distance<100){
    n++;
    if(n<5){
      return 0;
    }
    else{
      return 1;}
  }
  else{
  return 0;
  }
}

int set_throttle(int throttle, int desiredT){
  int dist;
  dist = alt();
  if(dist > desiredT + 25){
    throttle = throttle - 3;
  }
  else if(dist<desiredT-25){
    throttle = throttle + 2;
}
else{
  throttle = throttle;
}
return throttle;
}


void calcSignalr()
{
//record the interrupt time so that we can tell if the receiver has a signal from the transmitter
 last_interrupt_timer = micros();

//if the pin has gone HIGH, record the microseconds since the Arduino started up
 if(digitalRead(roll_pin) == HIGH)
    {
        timer_startr = micros();
    }
//otherwise, the pin has gone LOW
    else
    {
        //only worry about this if the timer has actually started
        if(timer_startr > 0)
        {
            //record the pulse time
            pulse_timer = ((volatile int)micros() - timer_startr);
            //restart the timer
            timer_startr = 0;
        }
    }
}

void calcSignalp()
{
//record the interrupt time so that we can tell if the receiver has a signal from the transmitter
 last_interrupt_timep = micros();

//if the pin has gone HIGH, record the microseconds since the Arduino started up
 if(digitalRead(pitch_pin) == HIGH)
    {
        timer_startp = micros();
    }
//otherwise, the pin has gone LOW
    else
    {
        //only worry about this if the timer has actually started
        if(timer_startp > 0)
        {
            //record the pulse time
            pulse_timep = ((volatile int)micros() - timer_startp);
            //restart the timer
            timer_startp = 0;
        }
    }
}

void calcSignalt()
{
//record the interrupt time so that we can tell if the receiver has a signal from the transmitter
 last_interrupt_timet = micros();

//if the pin has gone HIGH, record the microseconds since the Arduino started up
 if(digitalRead(throttle_pin) == HIGH)
    {
        timer_startt = micros();
    }
//otherwise, the pin has gone LOW
    else
    {
        //only worry about this if the timer has actually started
        if(timer_startt > 0)
        {
            //record the pulse time
            pulse_timet = ((volatile int)micros() - timer_startt);
            //restart the timer
            timer_startt = 0;
        }
    }
}

void calcSignaly()
{
//record the interrupt time so that we can tell if the receiver has a signal from the transmitter
 last_interrupt_timey = micros();

//if the pin has gone HIGH, record the microseconds since the Arduino started up
 if(digitalRead(yaw_pin) == HIGH)
    {
        timer_starty = micros();
    }
//otherwise, the pin has gone LOW
    else
    {
        //only worry about this if the timer has actually started
        if(timer_starty > 0)
        {
            //record the pulse time
            pulse_timey = ((volatile int)micros() - timer_starty);
            //restart the timer
            timer_starty = 0;
        }
    }
}

void calcSignals()
{
//record the interrupt time so that we can tell if the receiver has a signal from the transmitter
 last_interrupt_times = micros();

//if the pin has gone HIGH, record the microseconds since the Arduino started up
 if(digitalRead(servo_pin) == HIGH)
    {
        timer_starts = micros();
    }
//otherwise, the pin has gone LOW
    else
    {
        //only worry about this if the timer has actually started
        if(timer_starts > 0)
        {
            //record the pulse time
            pulse_times = ((volatile int)micros() - timer_starts);
            //restart the timer
            timer_starts = 0;
        }
    }
}

void calcSignals2(){
//record the interrupt time so that we can tell if the receiver has a signal from the transmitter
 last_interrupt_times2 = micros();

//if the pin has gone HIGH, record the microseconds since the Arduino started up
 if(digitalRead(servo2_pin) == HIGH)
    {
        timer_starts2 = micros();
    }
//otherwise, the pin has gone LOW
    else
    {
        //only worry about this if the timer has actually started
        if(timer_starts2 > 0)
        {
            //record the pulse time
            pulse_times2 = ((volatile int)micros() - timer_starts2);
            //restart the timer
            timer_starts2 = 0;
        }
    }
}

void start(){
  if(Serial.available()){
  int inByte = Serial.read();
    if(inByte == byte('$')){
      not_started = 0;
      Serial.write(inByte);
    }
    else{
      Serial.write('#');
      not_started = 1;
  }
  }
  else{
    not_started = 1;
  }
}

void drop(){
  while(plip<3){
  int prev_alt;
  prev_alt = altitude;
  altitude = alt();
  if(plip == 0 || plip == 1){
    rc_signals[1] = 1500;
    rc_signals[2] = 1600;
    rc_signals[3] = throttle;
    rc_signals[4] = 1500;
    rc_signals[5] = 2000;
    set_rc(rc_signals);
    if(prev_alt - altitude > 45 || prev_alt - altitude < -45){
      if(plip = 0){
      event = micros();
      }
      else{
        time = (micros() - event)/2;
        event = micros();
      }
      plip = plip + 1;
    }
  }
  if(plip == 2){
    rc_signals[1] = 1500;
    rc_signals[2] = 1400;
    rc_signals[3] = throttle;
    rc_signals[4] = 1500;
    rc_signals[5] = 2000;
    set_rc(rc_signals);
    if(micros()-event >= time){
      plip = plip + 1;}
  }}
  if(plip == 3){
    servo_pay.write(180);
  }
}
  
void set_rc(int rc_signal[5]){
    servor.writeMicroseconds(rc_signal[1]);
    servop.writeMicroseconds(rc_signal[2]);
    servot.writeMicroseconds(rc_signal[3]);
    servoy.writeMicroseconds(rc_signal[4]);
    servoa.writeMicroseconds(rc_signal[5]);
} 

void demo(){
  int m;
  char thro[4], ya[4], alti[3];
  char data[15];
  if (Serial.available() > 0){
     Serial.readBytes(data, 15);
  for(m=0; m<15; m++){
    if(data[m] == byte('9')){
      if(m+4<=15){
        thro[1]=data[m+1];
        thro[2]=data[m+2];
        thro[3]=data[m+3];
        thro[4]=data[m+4];
      } if(m+3==15){
        thro[1]=data[m+1];
        thro[2]=data[m+2];
        thro[3]=data[m+3];
        thro[4]=data[1];
      }else if(m+2==15){
        thro[1]=data[m+1];
        thro[2]=data[m+2];
        thro[3]=data[1];
        thro[4]=data[2];
      } else{
        thro[1]='1';
        thro[2]='5';
        thro[3]='0';
        thro[4]='0';
      }}else if(data[m]==byte('8')){
        if(m+4<=15){
        ya[1]=data[m+1];
        ya[2]=data[m+2];
        ya[3]=data[m+3];
        ya[4]=data[m+4];
      }else if(m+3==15){
        ya[1]=data[m+1];
        ya[2]=data[m+2];
        ya[3]=data[m+3];
        ya[4]=data[1];
      }else if(m+2==15){
        ya[1]=data[m+1];
        ya[2]=data[m+2];
        ya[3]=data[1];
        ya[4]=data[2];
      } else{
        ya[1]='1';
        ya[2]='5';
        ya[3]='0';
        ya[4]='0';
      }}else if(data[m]==byte('8')){
        if(m+3<=15){
        alti[1]=data[m+1];
        alti[2]=data[m+2];
        alti[3]=data[m+3];
      }else if(m+2==15){
        alti[1]=data[m+1];
        alti[2]=data[m+2];
        alti[3]=data[1];
      }else if(m+1==15){
        alti[1]=data[m+1];
        alti[2]=data[1];
        alti[3]=data[2];
      } else{
        alti[1]='1';
        alti[2]='6';
        alti[3]='5';
        
      }}else if(data[m]==byte('@')){
        drop();
      }}
    PITCH = atoi(thro);
    YAW = atoi(ya);
    altit = atoi(alti);
    rc_signals[1] = 1500;
    rc_signals[2] = PITCH;
    rc_signals[3] = set_throttle(throttle, altit);
    rc_signals[4] = YAW;
    rc_signals[5] = 1800;
    set_rc(rc_signals);
    
}}
      
     
    
  
