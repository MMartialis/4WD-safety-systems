// sd.cpp

// inspiration from https://rydepier.wordpress.com/2015/08/07/using-an-sd-card-reader-
// to-store-and-retrieve-data-with-arduino/

#include "sd.hpp"
#include <SPI.h>
#include <SD.h>
#include <string>

// CS    = 5;
// MOSI  = 23;
// MISO  = 19;
// SCK   = 18;

bool SD_ACTIVE = 1;
String dataString = ""; // holds the data to be written to the SD card
String aiString = "";
// std::string myDataString = "";
char dataLogFileName[13] = {'/','d', 'a', 't', 'a', '0', '0', '0', '0', '.', 'c', 's', 'v'};

// array
// float escData[12]; // duty, currentM, erpm
float sdLoggingFloat[SD_LOG_ENTRY_SIZE]; // duty, currentM, erpm, tFET, tMot, tacho, Vbatt, Ibatt

void FillLogWithZeros()
{
    for (int i = 0; i < SD_LOG_ENTRY_SIZE; i++)
    {
        sdLoggingFloat[i] = 0.00;
    }
}

void LogAppendValues()
{
    for (int i = 0; i < SD_LOG_ENTRY_SIZE - 1; i++)
    {
        dataString += String(sdLoggingFloat[i]) + ",";
    }
    dataString += String(sdLoggingFloat[SD_LOG_ENTRY_SIZE - 1]);
}

char *findDataLogFileName()
{
    for (int i = 0; i < 10000; i++)
    {
        dataLogFileName[8] = '0' + i % 10;
        dataLogFileName[7] = '0' + (i % 100 - i % 10) / 10;
        dataLogFileName[6] = '0' + (i % 1000 - i % 100) / 100;
        dataLogFileName[5] = '0' + (i % 10000 - i % 1000) / 1000;

        if (!SD.exists(dataLogFileName))
        {
            return dataLogFileName;
        }
    }
    return dataLogFileName;
}

File sensorDataLog;

void saveDataLog()
{
    extern bool SD_ACTIVE;
    SD_ACTIVE = 1;
    digitalWrite(SD_CS_PIN, LOW);
    
    if (SD.exists(dataLogFileName))
    { // check the card is still there
        // now append new data file
        sensorDataLog = SD.open(dataLogFileName, FILE_APPEND);
        if (sensorDataLog)
        {
            sensorDataLog.println(dataString);
            sensorDataLog.close(); // close the file
        }
    }
    digitalWrite(SD_CS_PIN, HIGH);
    SD_ACTIVE = 0;
    // else{4
    //     Serial.println("Error writing to file !");
    // }
}

File aiParameter;

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

// void readData(){
//     // read from the SD card and display on the serial monitor
//     sensorDataLog = SD.open(dataLogFileName);
//     if (sensorDataLog){
//         Serial.println("Reading from file:");
//         while (sensorDataLog.available()){
//             Serial.write(sensorDataLog.read());
//         }
//         sensorDataLog.close(); // close the file
//     }
//     else{
//         Serial.println("Error opening file !");
//     }
// }
