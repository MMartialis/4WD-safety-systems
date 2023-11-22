// sd.cpp
// inspiration from https://rydepier.wordpress.com/2015/08/07/using-an-sd-card-reader-
// to-store-and-retrieve-data-with-arduino/

#include "sd.hpp"
#include <SPI.h>
#include <SD.h>
#include <string>

// CS    = 17;
// MOSI  = 23;
// MISO  = 19;
// SCK   = 18;

String dataString = ""; // holds the data to be written to the SD card
char dataLogFileName[13] = {'/','d', 'a', 't', 'a', '0', '0', '0', '0', '.', 'c', 's', 'v'};
uint32_t sdLoggingMessage[SD_LOG_ENTRY_SIZE]; // duty, currentM, erpm, tFET, tMot, tacho, Vbatt, Ibatt

//extern uint8_t rxBuf[8];

void FillLogWithZeros()
{
    for (int i = 0; i < SD_LOG_ENTRY_SIZE; i++)
    {
        sdLoggingMessage[i] = 0.00;
    }
}

void LogAppendValues()
{
    for (int i = 0; i < SD_LOG_ENTRY_SIZE - 1; i++)
    {
        dataString += String(sdLoggingMessage[i]) + ",";
    }
    dataString += String(sdLoggingMessage[SD_LOG_ENTRY_SIZE - 1]);
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
    // else{
    //     Serial.println("Error writing to file !");
    // }
}