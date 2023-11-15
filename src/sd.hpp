// sd.hpp

#include <string>
#include <algorithm>


#define LOG_LENGTH 32

extern char dataLogFileName[12];
extern float sdLoggingFloat[LOG_LENGTH]; // duty, currentM, erpm, tFET, tMot, tacho, Vbatt, Ibatt
extern String dataString;
//extern float escData[12]; // duty, currentM, erpm



void FillLogWithZeros();
void LogAppendValues();
char* findDataLogFileName();
void saveDataLog();
void saveAiParameter();
