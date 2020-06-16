#include "sps30.h"

#define SP30_COMMS I2C_COMMS  // UART OR I2C

// function prototypes (sometimes the pre-processor does not create prototypes themself on ESPxx)
void ErrtoMess(char *mess, uint8_t r);
void Errorloop(char *mess, uint8_t r);
bool read_all();

// create constructor
SPS30 sps30;

int PM25 = 0;

void setup() {

  Serial.begin(115200);

  Serial.println(F("Trying to connect"));

  // Begin communication channel;
  if (sps30.begin(SP30_COMMS) == false) {
    Errorloop((char *) "could not initialize communication channel.", 0);
  }

  // check for SPS30 connection
  if (sps30.probe() == false) {
    Errorloop((char *) "could not probe / connect with SPS30.", 0);
  }
  else
    Serial.println(F("Detected SPS30."));

  // reset SPS30 connection
  if (sps30.reset() == false) {
    Errorloop((char *) "could not reset.", 0);
  }

  // start measurement
  if (sps30.start() == true)
    Serial.println(F("Measurement started"));
  else
    Errorloop((char *) "Could NOT start measurement", 0);

  if (SP30_COMMS == I2C_COMMS) {
    if (sps30.I2C_expect() == 4)
      Serial.println(F(" !!! Due to I2C buffersize only the SPS30 MASS concentration is available !!! \n"));
  }
}

void loop() {
  read_all();
  delay(1000);
}

/**
 * @brief : read and display all values
 */
bool read_all()
{
  static bool header = true;
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do {

    ret = sps30.GetValues(&val);

    // data might not have been ready
    if (ret == ERR_DATALENGTH){

        if (error_cnt++ > 3) {
          ErrtoMess((char *) "Error during reading values: ",ret);
          return(false);
        }
        delay(1000);
    }

    // if other error
    else if(ret != ERR_OK) {
      ErrtoMess((char *) "Error during reading values: ",ret);
      return(false);
    }

  } while (ret != ERR_OK);

  Serial.print("PM2.5: ");
  Serial.print(val.MassPM2);
  Serial.print(" ug/m3");

  PM25 = round (val.MassPM2);
  Serial.print("  ");
  Serial.print(PM25);
  Serial.println(" ug/m3");
  
  return(true);
}

/**
 *  @brief : continued loop after fatal error
 *  @param mess : message to display
 *  @param r : error code
 *
 *  if r is zero, it will only display the message
 */

void Errorloop(char *mess, uint8_t r)
{
  if (r) ErrtoMess(mess, r);
  else Serial.println(mess);
  Serial.println(F("Program on hold"));
  for(;;) delay(100000);
}

/**
 *  @brief : display error message
 *  @param mess : message to display
 *  @param r : error code
 *
 */
void ErrtoMess(char *mess, uint8_t r)
{
  char buf[80];

  Serial.print(mess);

  sps30.GetErrDescription(r, buf, 80);
  Serial.println(buf);
}