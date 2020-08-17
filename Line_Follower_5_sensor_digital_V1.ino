/* CNY70 Sensor Array Test Code
As Digital Signal

By DrakerDG (c)

https://www.youtube.com/user/DrakerDG
*/

const byte PnX[5] = {3, 4, 5, 6, 7};
byte SnX = 0;
unsigned long Tm0 = 0;

void EstSnX(void);

void setup(){
  Serial.begin(9600);
  delay(50);
  DDRC = 0b00000000;
  for (byte i=0; i<5; i++){
    pinMode(PnX[i], INPUT);
  }
}

void loop(){
  unsigned long Tm1 = millis();
  if ((Tm1-Tm0)>50){
    EstSnX();
    SnX = SnX + 0b10000000;

    Serial.println(SnX, BIN);
    Tm0 = Tm1;
  }
}

void EstSnX(){
  SnX = PINC & 0b00011111;

}