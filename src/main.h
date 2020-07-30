
using namespace std;

HardwareSerial hpmaSerial(1);
HPMA115S0 hpma115S0(hpmaSerial);

//HardwareSerial hpmaSerial2(2);
//HPMA115S0 hpma115S0_2(hpmaSerial2);

vector<unsigned int> v25; // for average PMS7003
vector<unsigned int> v10; // for average PMS7003
unsigned int apm25 = 0;   // last PM2.5 average
unsigned int apm10 = 0;   // last PM10 average
unsigned int pm25 = 0;
unsigned int pm10 = 0;
float pm25f = 0.0;
#define SENSOR_RETRY 1000 // Sensor read retry

vector<unsigned int> v250; // for average Honeywell
vector<unsigned int> v251; // for average Panasonic
vector<unsigned int> v252; // for average PMS7003
vector<unsigned int> v253; // for average PMSA003
vector<unsigned int> v254; // for average SPS30
vector<float> v254f; // for average SPS30

unsigned int apm250 = 0;
unsigned int apm251 = 0;
unsigned int apm252 = 0;
unsigned int apm253 = 0;
unsigned int apm254 = 0;
float apm254f = 0;

int numsensor = 0;

// Sensirion SPS30 sensor
#define SP30_COMMS SERIALPORT2 // UART OR I2C
uint8_t ret, error_cnt = 0;
struct sps_values val;

// Humidity sensor
Adafruit_AM2320 am2320 = Adafruit_AM2320();
float humi = 0.0; // % Relative humidity
float temp = 0.0; // Temperature (°C)

// Battery level
unsigned int chargeLevel = 0;
#ifdef TTGO_TQ
const int IP5306_2 = 27; // PIN2 IP5306
const int IP5306_3 = 14; // PIN3 IP5306
unsigned int Rdelay = 0;
#endif

// WiFi fields
#define WIFI_RETRY_CONNECTION 20
bool dataSendToggle;
bool wifiOn;
int rssi = 0;

// Bluetooth fields
BLEServer* pServer = NULL;
BLECharacteristic* pCharactData = NULL;
BLECharacteristic* pCharactConfig = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
#define SERVICE_UUID        "c8d1d262-861f-4082-947e-f383a259aaf3"
#define CHARAC_DATA_UUID    "b0f332a8-a5aa-4f3f-bb43-f99e7791ae01"
#define CHARAC_CONFIG_UUID  "b0f332a8-a5aa-4f3f-bb43-f99e7791ae02"

// CanAirIO API fields
CanAirIoApi api(false);

// InfluxDB fields
#define IFX_RETRY_CONNECTION 2
InfluxArduino influx;

// Config and settings handler
ConfigApp cfg;
int triggerSaveIcon = 0;

// GUI fields
#define LED 2
GUIUtils gui;

// Watchdog timer
hw_timer_t *timer = NULL;
unsigned int resetvar = 0;

// some prototypes
bool wifiCheck();
void wifiConnect(const char* ssid, const char* pass);
void wifiInit();
void wifiStop();
void wifiRestart();
void wifiLoop();
void wifiRSSI();