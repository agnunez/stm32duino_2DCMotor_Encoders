// Control of 2 DC Motors with Encoders using STM32F103C8 and arduino enviroment
// Control positions of both motors independently using PID, configured from a Serial UART
// 2018 https://github.com/agnunez/stm32duino_2DCMotor_Encoders.git

#include <PID_v1.h>
#define pinLED PC13

int enc[]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  // Quadrature Encoder machine state array
int pos0=0;
int pos1=0;
int state0=0;
int state1=0;
double In0,In1,Out0,Out1,Set0=30,Set1=30;   // PID Input Signal, Output command and Setting speed for each wheel 
double kp0=0.5,ki0=10,kd0=0.0;              // Left/right wheel PID constants. Can be modiffied while running with:
double kp1=0.5,ki1=10,kd1=0.0;              // s nnn (setting point), p nnn (kp), i nnn (ki), d nnn (kd). nnn is divided by 10 to get decimals nnn -> nn.n
int period=50;                              // PID sample timer period in ms
double kt=1000/period;                      // number of periods/sec
int t0,t1=0;
bool debug=false;

//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID lPID(&In0, &Out0, &Set0, kp0, ki0, kd0, DIRECT);
PID rPID(&In1, &Out1, &Set1, kp1, ki1, kd1, DIRECT);

static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}


void setup() {
  Serial1.begin(115200);
  Serial1.println("Serial1 START");
  disableDebugPorts(); // To access to PB3 that is used as SWO in JTAG
  pinMode(pinLED, OUTPUT);
  pinMode(PB0, INPUT);  // Enc A Motor 1  
  pinMode(PB1, INPUT);  // Enc B Motor 1
  pinMode(PB3, INPUT);  // Enc A Motor 2
  pinMode(PB4, INPUT);  // Enc B Motor 1
  pinMode(PA0, PWM);
  pinMode(PA1, PWM);
  pinMode(PA2, PWM);
  pinMode(PA3, PWM);
  attachInterrupt(PB0,encoder1,CHANGE);
  attachInterrupt(PB1,encoder1,CHANGE);
  attachInterrupt(PB3,encoder2,CHANGE);
  attachInterrupt(PB4,encoder2,CHANGE);
  state0 = GPIOB->regs->IDR & B11;          //save gpio PB1,PB0 in binary 
  state1 = (GPIOB->regs->IDR & B11000)>>3;  //save gpio PB3,PB4 in binary 

  Timer4.setChannel1Mode(TIMER_OUTPUTCOMPARE);
  Timer4.setPeriod(period*1000); // in microseconds
  Timer4.setCompare1(1);         // overflow might be small
  Timer4.attachCompare1Interrupt(tick);
  motion(0,0);
  motion(1,0);
 
}
void tick(){
  int d0=t0-pos0;
  int d1=t1-pos1;
  if(abs(d0)>10){
    motion(0,-200*sgn(d0));
  }else{
    motion(0,0);
  }
  if(abs(d1)>10){
    motion(1,200*sgn(d1));
  }else{
    motion(1,0);
  }
}
void encoder1(){
  state0 = (state0<<2) | (GPIOB->regs->IDR & B11);
  pos0 += enc[state0];
  state0 &= B11;
} 
void encoder2(){
  state1 = (state1<<2) | ((GPIOB->regs->IDR & B11000)>>3);
  pos1 += enc[state1];
  state1 &= B11;
} 

void motion(int motor, int vel){
  if(motor==1){
    if(vel>0){
      analogWrite(PA0,0);
      analogWrite(PA1,vel);
    }
    if(vel<0){
      analogWrite(PA0,-vel);
      analogWrite(PA1,0);
    }
    if(vel==0){
      analogWrite(PA0,0);
      analogWrite(PA1,0);
    }
  }
  if(motor==0){
    if(vel>0){
      analogWrite(PA2,0);
      analogWrite(PA3,vel);
    }
    if(vel<0){
      analogWrite(PA2,-vel);
      analogWrite(PA3,0);
    }
    if(vel==0){
      analogWrite(PA2,0);
      analogWrite(PA3,0);
    }
  }
}


void log(){
  Serial1.print("t0:");
  Serial1.print(t0);
  Serial1.print(" t1:");
  Serial1.print(t1);
  Serial1.print(" pos0:");
  Serial1.print(pos0);
  Serial1.print(" pos1:");
  Serial1.println(pos1);
  if(debug)Serial1.print(GPIOB->regs->IDR,BIN);
}

void loop() {
    digitalWrite(pinLED, HIGH);
    t0=40000;
    log();
    delay(1000);
    digitalWrite(pinLED, LOW);
    t1=30000;
    delay(1000);    
}
