#include <SPI.h>
#include <SD.h>

File sdFile;
int cur = 0;

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(100);

    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    
    //Initialize SD
    pinMode(53, OUTPUT);

    Serial.print("Initializing SD card...");
    
    if (!SD.begin(10)) {
      Serial.println("initialization failed!");
      while (1);
    }
    Serial.println("initialization done.");

    //Open file on SD for reading
    sdFile = SD.open("test.txt");
  
    //If file cannot be opened:
     if (!sdFile){
      Serial.println("error opening file");
      while (1);
     }

     Serial.println("Reading...");

   while (sdFile.available()){
    Serial.write(sdFile.read());
   }

  Serial.println("Done!");

  sdFile.close();
}

void loop() {

}
