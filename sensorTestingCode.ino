#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


int sensorValue[8], sensorArray[8];

void setup() {

  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(9600);
}

void readLine() {

  for (int i = 0; i < 8; i++) {
    sensorValue[i] = analogRead(i);
    sensorArray[i] = sensorValue[i] > 500;

  }
}

void loop() {

  while (1) {
    readLine();
    for(int i=0;i<8;i++){
      Serial.print(sensorValue[i]);
      Serial.print(",");
    }
    Serial.println("");
    for(int i=0;i<8;i++){
      Serial.print(sensorArray[i]);
      Serial.print(",");
    }
    Serial.println("");
  }
}
