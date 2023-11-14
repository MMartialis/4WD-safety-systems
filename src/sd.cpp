// sd.cpp

#include "sd.hpp"
#include <SPI.h>
#include <SD.h>
#include <string>

// CS    = 5;
// MOSI  = 23;
// MISO  = 19;
// SCK   = 18;

String dataString =""; // holds the data to be written to the SD card

// array
float escData[12]; // duty, currentM, erpm
float sdLoggingFloat[32]; // duty, currentM, erpm, tFET, tMot, tacho, Vbatt, Ibatt
std::string sdLoggingString[32];

void FillLogWithZeros(){
    for (int i=0; i < 32; i++){
        sdLoggingFloat[i]=0.00;
    }
}

void LogFloatToString(){
    for (int i=0; i<32; i++){
        sdLoggingString[i] = std::to_string(sdLoggingFloat[i]);
    }
}



File sensorData;


void saveData(){
    if(SD.exists("data.csv")){ // check the card is still there
        // now append new data file
        sensorData = SD.open("data.csv", FILE_WRITE);
        if (sensorData){
            sensorData.println(dataString);
            sensorData.close(); // close the file
        }
    }
    else{
        Serial.println("Error writing to file !");
    }
}


// void setup()
// {
//     pinMode(SDCSpin, OUTPUT);

//     // Open serial communications
//     Serial.begin(9600);
//     Serial.print("Initializing SD card...");

//     // see if the card is present and can be initialized:
//     if (!SD.begin(SDCSpin)){
//         Serial.println("Card failed, or not present");
//         return;
//     }

//     Serial.println("card initialized");

// }

// void loop(){
//     // build the data string
//     dataString = String(sensorReading1) + "," + String(sensorReading2) + "," + String(sensorReading3); // convert to CSV
//     saveData(); // save to SD card
//     delay(60000); // delay before next write to SD Card, adjust as required
// }
