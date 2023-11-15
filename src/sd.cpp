// sd.cpp
// inspiration from https://rydepier.wordpress.com/2015/08/07/using-an-sd-card-reader-
//to-store-and-retrieve-data-with-arduino/

#include "sd.hpp"
#include <SPI.h>
#include <SD.h>
#include <string>

// CS    = 5;
// MOSI  = 23;
// MISO  = 19;
// SCK   = 18;

String dataString =""; // holds the data to be written to the SD card
std::string myDataString = "";
// array
//float escData[12]; // duty, currentM, erpm
float sdLoggingFloat[LOG_LENGTH]; // duty, currentM, erpm, tFET, tMot, tacho, Vbatt, Ibatt

void FillLogWithZeros(){
    for (int i=0; i < LOG_LENGTH; i++){
        sdLoggingFloat[i]=0.00;
    }
}

void LogAppendValues(){
    for(int i=0; i<LOG_LENGTH-1; i++){
        dataString += String(sdLoggingFloat[i]) + ",";
    }
    dataString += String(sdLoggingFloat[LOG_LENGTH-1]);
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
