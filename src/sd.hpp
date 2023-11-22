// sd.hpp

#include <string>
#include <algorithm>
#include <SD.h>

#include "defs.hpp"

extern char dataLogFileName[13];
extern float sdLoggingFloat[SD_LOG_ENTRY_SIZE]; // duty, currentM, erpm, tFET, tMot, tacho, Vbatt, Ibatt
extern String dataString;
extern String aiString;
//extern float escData[12]; // duty, currentM, erpm



void FillLogWithZeros();
void LogAppendValues();
char* findDataLogFileName();
void saveDataLog();
void saveAiParameter();
std::string readAiParameter();
