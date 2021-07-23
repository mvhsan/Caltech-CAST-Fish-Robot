#include <SPI.h>
#include <SD.h>

File sdFile;
int cur = 0;

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(100);
    
    if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("initialization failed!");
      while (1);
    }
  
    //Open file on SD for writing
    sdFile = SD.open("test.txt", FILE_WRITE);
  
    //If file cannot be opened:
     if (!sdFile){
      Serial.println("error opening file");
      while (1);
     }

     Serial.println("Writing...");
     sdFile.print("Test 123");
     sdFile.print("test 321");

  sdFile.close();
}

void loop() {
    //sdFile.print(Serial.read());
}
