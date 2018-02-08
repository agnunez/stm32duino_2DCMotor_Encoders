// Control of 2 DC Motors with Encoders using STM32F103C8 and arduino enviroment
// Control positions of both motors independently using PID, configured from a Serial UART
// 2018 https://github.com/agnunez/stm32duino_2DCMotor_Encoders.git

#include <PID_v1.h>
#define pinLED PC13

int enc[]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  // Quadrature Encoder machine state array
int state0=0;
int state1=0;
double pos0,pos1,t0,t1,Out0,Out1;         // PID Input Signal, Output command and Setting speed for each wheel 
double kp0=40,ki0=10,kd0=1.0;             // motors PID constants. Can be modiffied while running with:
double kp1=40,ki1=10,kd1=1.0;             // s nnn (setting point), p nnn (kp), i nnn (ki), d nnn (kd). nnn is divided by 10 to get decimals nnn -> nn.n
int period=1;                             // PID sample timer period in ms
int debug=0;
int toggle=0;

//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
//PID PID0(&In0, &Out0, &Set0, kp0, ki0, kd0, DIRECT);
//PID PID1(&In1, &Out1, &Set1, kp1, ki1, kd1, DIRECT);
PID PID0(&pos0, &Out0, &t0, kp0, ki0, kd0, DIRECT);
PID PID1(&pos1, &Out1, &t1, kp1, ki1, kd1, DIRECT);

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
  PID0.SetSampleTime(period);
  PID1.SetSampleTime(period);
  PID0.SetOutputLimits(-255,255);
  PID1.SetOutputLimits(-255,255);
  PID0.SetMode(AUTOMATIC);
  PID1.SetMode(AUTOMATIC); 
}
void tick(){
  PID0.Compute();
  PID1.Compute();
  motion(0,Out0);
  motion(1,Out1);
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

void motion(int motor, float vel){
  if(motor==1){
    if(vel>0){
      analogWrite(PA0,vel);
      analogWrite(PA1,0);
    }
    if(vel<0){
      analogWrite(PA0,0);
      analogWrite(PA1,-vel);
    }
    if(vel==0){
      analogWrite(PA0,0);
      analogWrite(PA1,0);
    }
  }
  if(motor==0){
    if(vel>0){
      analogWrite(PA2,vel);
      analogWrite(PA3,0);
    }
    if(vel<0){
      analogWrite(PA2,0);
      analogWrite(PA3,-vel);
    }
    if(vel==0){
      analogWrite(PA2,0);
      analogWrite(PA3,0);
    }
  }
}

void dump(){
  Serial1.print("kp0:");
  Serial1.print(kp0);
  Serial1.print(" ki0:");
  Serial1.print(ki0);
  Serial1.print(" kd0:");
  Serial1.print(kd0);
  Serial1.print(" t0:");
  Serial1.print(t0);
  Serial1.print(" t1:");
  Serial1.print(t1);
  Serial1.print(" pos:");
  Serial1.print(pos0);
  Serial1.print(" pos1:");
  Serial1.print(pos1);    
  Serial1.print(" Out0:");
  Serial1.print(Out0);    
  Serial1.print(" Out1:");
  Serial1.println(Out1);    
}

char c=0;             // input char from keys
void loop(){
  // watch for input
  if(c==0){
    if (Serial1.available() != 0) {
      c = Serial1.read();
      Serial1.println(c);
    }
  } else {
    if (Serial1.available() != 0) {
      int gv = Serial1.parseInt();
      if(c=='0'){
        t0=gv/10.;
      } else if(c=='1'){
        t1=gv/10.;
      } else if(c=='p'){
        kp0=gv/10.;
        kp1=gv/10.;
        PID0.SetTunings(kp0, ki0, kd0);
        PID1.SetTunings(kp1, ki1, kd1);
      } else if(c=='i'){
        ki0=gv/10.;
        ki1=gv/10.;
        PID0.SetTunings(kp0, ki0, kd0);
        PID1.SetTunings(kp1, ki1, kd1);
      } else if(c=='d'){
        kd0=gv/10.;
        kd1=gv/10.;
        PID0.SetTunings(kp0, ki0, kd0);
        PID1.SetTunings(kp1, ki1, kd1);
      } else if(c=='?'){
        Serial1.print(int(pos0));
        Serial1.print(":");
        Serial1.println(int(pos1));
      } else if(c=='s'){
        dump();
      } else if(c=='b'){
        debug^=1;
      } else {
        Serial1.println("Command not recognized. Use:\n '0 NNN' or '1 NNN', for target position,\n '?' for current position.\n 'p' NNN, 'i' NNN, 'd' NNN with NNN=n*10,\n 's' to dump variables, 'b' to toggle debug");
      }
      while (Serial1.available()) Serial1.read();
      c=0;
    }
  }
  if(debug){
    dump();
  }
  toggle^=1;
  digitalWrite(pinLED, toggle);
  //delay(1000);
}

