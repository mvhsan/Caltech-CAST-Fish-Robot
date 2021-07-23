float reading;
long t=0;
int timeStep;
int nPoints;
int counter=0;

int servoRelay1 = 13;
int servoRelay2 = 12;
int servoRelay3 = 11;
int servoRelay4 = 10;

int FORCE_THRESHOLD = 300;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  pinMode(servoRelay1, OUTPUT);
  pinMode(servoRelay2, OUTPUT);
  pinMode(servoRelay3, OUTPUT);
  pinMode(servoRelay4, OUTPUT);
}

void loop() {
  if(Serial.available() > 0){
    nPoints = Serial.parseInt();
    timeStep = Serial.parseInt();
    t = millis();
    while(counter<nPoints){            
      if(millis() > t+timeStep-1){
        reading = analogRead(0);
        t = millis();    
        Serial.println(t);
        Serial.println(reading);
        counter = counter + 1;

        if (reading >= FORCE_THRESHOLD) {
          digitalWrite(servoRelay1, HIGH);
          digitalWrite(servoRelay2, HIGH);
          digitalWrite(servoRelay3, HIGH);
          digitalWrite(servoRelay4, HIGH);  
        }
        else {
          digitalWrite(servoRelay1, LOW);
          digitalWrite(servoRelay2, LOW);
          digitalWrite(servoRelay3, LOW);
          digitalWrite(servoRelay4, LOW);  
        }
      }
    }            
    t = 0;
    counter = 0;
  }
}
