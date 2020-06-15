 #include <EEPROM.h>

long EEPROMAddr = 0; //Current EEPROM Address
int curVector = 1;
int numVectors = 0;
boolean running;

float f = 0.00f; //Stores data read from EEPROM

void setup() {
  Serial.begin(9600);
  running = true;

  numVectors = readInt(); // Retrieve the total number of vectors stored in EEPROM
  EEPROMAddr += 2;
  
  Serial.print("Number of vectors: ");
  Serial.println(numVectors);
  delay(4000);
}

int readInt(){
  long second = EEPROM.read(EEPROMAddr);
  long first = EEPROM.read(EEPROMAddr + 1);

    return ((second <<0) & 0xFFFFFF) + ((first <<8) & 0xFFFFFF);
}

void loop() {  
    if(curVector > numVectors || EEPROMAddr > 1000){
      running = false;
      Serial.print("END OF MEMORY");
      Serial.end();
      return;
    }
    
    Serial.print(curVector);
    Serial.print("\t");

    EEPROM.get(EEPROMAddr, f); //Timestep
    Serial.print(f);
    Serial.print(" ");
    EEPROMAddr += sizeof(float);

    EEPROM.get(EEPROMAddr, f); //phi
    Serial.print(f);
    Serial.print(" ");
    EEPROMAddr += sizeof(float);

    EEPROM.get(EEPROMAddr, f); //theta
    Serial.print(f);
    Serial.print(" ");
    EEPROMAddr += sizeof(float);
    
    EEPROM.get(EEPROMAddr, f); //psi
    Serial.println(f);
    EEPROMAddr += sizeof(float);

    curVector++;

    if(curVector > numVectors){
      Serial.end();
    }
}
