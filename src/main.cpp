#include <Arduino.h>
#include <RFTransmitter.h>
#include <Sim800l.h>
#include <Gpsneo.h>
#include <MPU9250.h>

#define SLEEPING        0
#define IDLE            1
#define SIM_TRANSMIT    2
#define RF_TRASNMIT     3
#define ALERT_MODE      4

#define VBAT_PIN        A4
#define HALL_PIN        4
#define ENABLE_HALL_PIN PB0
#define ENABLE_NEO_PIN  PB1
#define ENABLE_GSM_PIN  PB2
#define UART_SIM_PIN    PB3
#define UART_NEO_PIN    PD5
#define BUZZER_PIN      11
#define LED_PIN         13
#define RF_PIN          PF1
#define SIM_INT_PIN     9
#define IMU_INT_PIN     8

Sim800l SIM800;
Gpsneo GPS(0, 1);
RFTransmitter RF(RF_PIN, 1);
MPU9250 IMU(Wire,0x68);


char latitud[11];
char latitudHemisphere[3];
char longitud[11];
char longitudMeridiano[3];
char* text;
char* number;
bool error; //to catch the response of sendSms

char time[10];
char status[3];
char speedKnots[10];
char trackAngle[8];
char date[10];
char magneticVariation[10];
char magneticVariationOrientation[3];

uint16_t mG_threshold = 200;
byte main_state = 0;


void IMU_wake_ISR()
{

}

bool GPS_init()
{
  pinMode(ENABLE_NEO_PIN, OUTPUT);
  digitalWrite(ENABLE_NEO_PIN, HIGH);
   GPS.getDataGPRMC(time, status, latitud, latitudHemisphere, longitud, longitudMeridiano,
                    speedKnots, trackAngle,date, magneticVariation, magneticVariationOrientation);
  return true;
}

bool SIM_init()
{
  pinMode(ENABLE_GSM_PIN, OUTPUT);
  digitalWrite(ENABLE_GSM_PIN, HIGH);
  SIM800.begin(); // initializate the library. 
	// text="Testing Sms";  //text for the message. 
	// number="2926451386"; //change to a valid number.
	// error=SIM.sendSms(number,text);
  return true;
}

bool IMU_init()
{
  if (IMU.begin() < 0) {
    return false;
  }
  else
  {
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    IMU.setSrd(19);
    IMU.enableWakeOnMotion(mG_threshold, MPU9250::LP_ACCEL_ODR_125HZ);
    attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), IMU_wake_ISR, FALLING);
    return true;
  }
}

void alert_buzz()
{

}

void state_machine()
{
  switch(main_state)
  {
    case SLEEPING:

                  break;
    case IDLE:

                  break;
    case SIM_TRANSMIT:

                  break;
    case RF_TRASNMIT:

                  break;
    case ALERT_MODE:

                  break;
                      
    default:      break;
  }
}

void setup() {
  Serial1.begin(9600);
  Wire.begin();
  pinMode(ENABLE_HALL_PIN, OUTPUT);
  digitalWrite(ENABLE_HALL_PIN, LOW);
  pinMode(UART_SIM_PIN, OUTPUT);
  digitalWrite(UART_SIM_PIN, LOW);
  pinMode(UART_NEO_PIN, OUTPUT);
  digitalWrite(UART_NEO_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  GPS_init();
  SIM_init();
  IMU_init();
}
    

void loop() {
  
}