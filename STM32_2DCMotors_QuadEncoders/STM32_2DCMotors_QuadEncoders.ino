// Control of 2 DC Motors with Encoders using STM32F103C8 and arduino enviroment
// Control positions of both motors independently using PID, configured from a Serial UART
// 2018 https://github.com/agnunez/stm32duino_2DCMotor_Encoders.git

#define pinLED PC13

int enc[]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  // Quadrature Encoder machine state array
int pos1=0;
int pos2=0;
int state1=0;
int state2=0;

void setup() {
  Serial1.begin(115200);
  Serial1.println("Serial1 START");
  disableDebugPorts(); // To access to PB3 that is used as SWO in JTAG
  pinMode(pinLED, OUTPUT);
  pinMode(PB0, INPUT_PULLUP);  // Enc A Motor 1  
  pinMode(PB1, INPUT_PULLUP);  // Enc B Motor 1
  pinMode(PB3, INPUT_PULLUP);  // Enc A Motor 2
  pinMode(PB4, INPUT_PULLUP);  // Enc B Motor 1
  attachInterrupt(PB0,encoder1,CHANGE);
  attachInterrupt(PB1,encoder1,CHANGE);
  attachInterrupt(PB3,encoder2,CHANGE);
  attachInterrupt(PB4,encoder2,CHANGE);
  state1 = GPIOB->regs->IDR & B11;  //save gpio PB1,PB0 in binary 
  state2 = (GPIOB->regs->IDR & B11000)>>3;  //save gpio PB1,PB0 in binary 

}
void encoder1(){
  state1 = (state1<<2) | (GPIOB->regs->IDR & B11);
  pos1 += enc[state1];
  state1 &= B11;
} 
void encoder2(){
  state2 = (state2<<2) | ((GPIOB->regs->IDR & B11000)>>3);
  pos2 += enc[state2];
  state2 &= B11;
} 

void loop() {
  digitalWrite(pinLED, HIGH);
  delay(1000);
  digitalWrite(pinLED, LOW);
  delay(1000);
  Serial1.print(GPIOB->regs->IDR,BIN);
  Serial1.print(" pos1:");
  Serial1.print(pos1);
  Serial1.print(" pos2:");
  Serial1.println(pos2);
}
