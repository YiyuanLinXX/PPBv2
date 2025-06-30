int dischargePin = 11;
int chargePin = 13;
int cam = 10;

// ------ strobe light cable settting ------
// green-yellow: GND
// ONE: 72V
// TWO: discharge signal
// THREE: charge

// GREEN-YELLOW -> GND
// ONE -> 72V
// TWO -> 11
// CHARGE -> 13


// ------ camera GPIO cable settting ------
// brown: GND, camera power ground
// blue: Opto GND, Opto-isolated ground
// green: power input, auxiliary input voltage (DC)
// black: Opto IN, Opto-isolated input

// BROWN, BLUE -> GND
// GREEN -> Power
// BLACK -> 10

int flag=0;

void setup() {
  Serial.begin(9600);
  pinMode(dischargePin, OUTPUT);
  pinMode(chargePin, OUTPUT);
  pinMode(cam, OUTPUT);

  // Pins Initialization
  digitalWrite(chargePin,LOW); //stop charging
  digitalWrite(chargePin,LOW); //stop charging
  digitalWrite(cam, LOW); // trigger camera
  delay(50);
}

void loop() {
  if(flag==1){
    digitalWrite(cam, HIGH);
    delayMicroseconds(5);

    digitalWrite(dischargePin, HIGH); //start discharging
    
    // Serial.println("trigger camera");
    digitalWrite(cam, LOW); // trigger camera
    delayMicroseconds(500);
    digitalWrite(dischargePin,LOW); //stop discharging

    delayMicroseconds(1000); //1ms

    digitalWrite(chargePin,HIGH); //start charging
    // digitalWrite(cam,HIGH); // set camera signal to HIGH, wait for next trigger signal
    delay(500); //frame rate
    digitalWrite(chargePin,LOW); //stop charging
  }

  char r = Serial.read();

  if(r=='s'){
    flag=1;
  }
  else if(r=='e'){
    flag=0;
  }
  else{
    flag=flag;
  }

  delay(5);
 
}
