#define pinLED PC13

int enc[]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
int pos=0;
int state=0;

void setup() {
  Serial1.begin(115200);
  Serial1.println("Serial1 START");
  pinMode(pinLED, OUTPUT);
  pinMode(PB0, INPUT);  
  pinMode(PB1, INPUT);  
  attachInterrupt(PB0,encoder,CHANGE);
  attachInterrupt(PB1,encoder,CHANGE);
  state = GPIOB->regs->IDR & B11;  //save gpio PB1,PB0 in binary 

}
void encoder(){
  state = (state<<2) | (GPIOB->regs->IDR & B11);
  pos += enc[state];
  state &= B11;
} 

void loop() {
  digitalWrite(pinLED, HIGH);
  delay(1000);
  digitalWrite(pinLED, LOW);
  delay(1000);
  Serial1.print("pos:");
  Serial1.println(pos);
}
