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

}

void loop() {
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
    }      
  t = 0;
  counter = 0;
  
}
