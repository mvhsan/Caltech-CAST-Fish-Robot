float reading;
long t=0;
int timeStep; 
int nPoints;
int counter=0; 

void setup() {
 Serial.begin(9600);
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
        }
    }            
    t = 0;
    counter = 0;
  }
}
