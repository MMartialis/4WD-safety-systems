// sd.hpp

#include <string>
#include <algorithm>
#include <SD.h>


#define LOG_LENGTH 32

extern char dataLogFileName[12];
extern float sdLoggingFloat[LOG_LENGTH]; // duty, currentM, erpm, tFET, tMot, tacho, Vbatt, Ibatt
extern String dataString;
extern String aiString;
//extern float escData[12]; // duty, currentM, erpm



void FillLogWithZeros();
void LogAppendValues();
char* findDataLogFileName();
void saveDataLog();
void saveAiParameter();
std::string readAiParameter();
