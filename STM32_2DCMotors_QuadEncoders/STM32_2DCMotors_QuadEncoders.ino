// Control of 2 DC Motors with Encoders using STM32F103C8 and arduino enviroment
// Control positions of both motors independently using PID, configured from a Serial UART
// 2018 https://github.com/agnunez/stm32duino_2DCMotor_Encoders.git

#define pinLED PC13

int enc[]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  // Quadrature Encoder machine state array
int pos0=0;
int pos1=0;
int state0=0;
int state1=0;
bool debug=false;

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
      digitalWrite(PA0,1);
      digitalWrite(PA1,1);
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
      digitalWrite(PA2,1);
      digitalWrite(PA3,1);
    }
  }
}

void move(int motor, int target){
  while((abs(target-pos0))>10){
    if(target>pos0){
      motion(0,-150);
    }else{
      motion(0,150);
    }
    delay(100);
    log();
  }
}

void log(){
  Serial1.print(" pos0:");
  Serial1.print(pos0);
  Serial1.print(" pos1:");
  Serial1.println(pos1);

}
void loop() {
    digitalWrite(pinLED, HIGH);
    move(0,10000);
    log();
    delay(1000);
    digitalWrite(pinLED, LOW);
    move(0,0);
    delay(1000);
  if(debug)Serial1.print(GPIOB->regs->IDR,BIN);
}
