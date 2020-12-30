using namespace std;

HardwareSerial hpmaSerial(1);
HPMA115S0 hpma115S0(hpmaSerial);

vector<unsigned int> v25;      // for average
vector<unsigned int> v10;      // for average
vector<float> v25f;            // for average
unsigned int apm25 = 0;        // last PM2.5 average
unsigned int apm10 = 0;        // last PM10 average
unsigned int pm25 = 0;
unsigned int pm10 = 0;
bool WrongSerialData = false;
#define SENSOR_RETRY  1000     // Sensor read retry

// Sensirion SPS30 sensor
#ifdef SENSIRION
 #define SP30_COMMS SERIALPORT2 // UART OR I2C
 #define SP30_COMMS1 Serial1 // UART OR I2C
 SPS30 sps30;
#endif
uint8_t ret, error_cnt = 0;
struct sps_values val;

// Humidity sensor
float humi = 0.0;              // % Relative humidity 
float temp = 0.0;              // Temperature (°C)
#ifdef BME280S
 #define SEALEVELPRESSURE_HPA (1013.25)
 Adafruit_BME280 bme;           // BME280 I2C
#elif DHT22S
 #define DHTTYPE DHT22         // DHT22 & DHT21
 #define DHTPIN 13              // Pin13 Lima Peru
 DHT dht(DHTPIN, DHTTYPE);
#elif AHT10
 TwoWire I2CBME = TwoWire(0);
 Adafruit_AHTX0 aht;
 //Adafruit_Sensor *aht_humidity, *aht_temp;
#else
 Adafruit_AM2320 am2320 = Adafruit_AM2320();
#endif

// Battery level
unsigned int chargeLevel = 0;
#ifdef TTGO_TQ
const int IP5306_2 = 27;     // PIN2 IP5306
const int IP5306_3 = 14;     // PIN3 IP5306
unsigned int Rdelay = 0;
#endif

// WiFi fields
#define WIFI_RETRY_CONNECTION    20
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
#define IFX_RETRY_CONNECTION   2
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

// OK para ESP32S 30 pines
#define OLEDdata 4    // D8
#define OLEDclock 16  // D7
#define PINRXSEN 17   // D6
#define PINTXSEN 5    // D5
#define HUMSDA 21     // D4
#define HUMSCL 3      // D3

// OK para ESP32S grande de 38 pines y ESP32DevKitD y U de 38 pines
#define OLEDdatad 2   // D8
#define OLEDclockd 0  // D7
#define PINRXSENd 4   // D6
#define PINTXSENd 16  // D5
//#define HUMSDAd 18    // D4
//#define HUMSCLd 19    // D3
#define HUMSDAd 21    // D4
#define HUMSCLd 3    // D3


// some prototypes
bool wifiCheck();
void wifiConnect(const char* ssid, const char* pass);
void wifiInit();
void wifiStop();
void wifiRestart();
void wifiLoop();
void wifiRSSI();
