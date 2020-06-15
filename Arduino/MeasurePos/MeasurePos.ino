#define encoder0PinA  2 // Bown wire
#define encoder0PinB  4 //Orange wire


volatile long encoder0Pos = 0;
long t=0;
int nPoints;
int timeStep;
int counter=0;


void setup()
{
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder, RISING);  // encoDER ON PIN 2
  Serial.begin (9600);
 }

void loop()
{

if(Serial.available() > 0){
    nPoints = Serial.parseInt();
    timeStep = Serial.parseInt();
    t = millis();
    while(counter<nPoints){            
      if(millis() > t+timeStep-1){
        t = millis();     
        Serial.println(t);
        Serial.println(encoder0Pos);
        counter = counter + 1;              
        }
    }            
    t = 0;
    counter = 0;
  }
}

void doEncoder()
{
  if (digitalRead(encoder0PinB) == digitalRead(encoder0PinA)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }  
}
