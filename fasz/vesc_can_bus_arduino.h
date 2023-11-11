#include <Arduino.h> //required for PD# definitions

class CAN
{

public:

// can IDs
// első 53, 77
// hátsó 40, 13
// can baud rate 250

extern MCP_CAN CAN0;

#define CAN0_INT 21                              // Set INT to pin 21

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];// Array to store serial string


float inpVoltage, dutyCycleNow, avgInputCurrent, avgMotorCurrent, tempFET, tempMotor;
long erpm, WattHours;


void initialize();
void spin();
void get_frame(); // populates rxId and rxBuf with latest can frame 
void can_send(byte data[8]); //transmits the send commands to the sensor
void print_raw_can_data(); //output raw can data to serial (debug)


void vesc_set_duty(float duty);
void vesc_set_current(float current);
void vesc_set_erpm(float erpm);
float process_data_frame_vesc(char datatype, unsigned char byte1, unsigned char byte2);
int hex2int(char buf[]);

};
