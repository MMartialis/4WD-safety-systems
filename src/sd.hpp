// sd.hpp

#include <string>
#include <algorithm>

extern float escData[12]; // duty, currentM, erpm
extern float sdLoggingFloat[32]; // duty, currentM, erpm, tFET, tMot, tacho, Vbatt, Ibatt
extern std::string sdLoggingString[32];

void FillLogWithZeros();
void LogFloatToString();
void saveData();