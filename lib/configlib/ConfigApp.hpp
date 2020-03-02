#ifndef CanfigApp_hpp
#define CanfigApp_hpp

#include <ArduinoJson.h>
#include <Preferences.h>
#include <ArduinoJson.h>

class ConfigApp
{
  private:
  // Config Settings
  Preferences preferences;
  char* _app_name;

  public:
  uint64_t chipid;
  char* deviceId;
  //String dname;
  String dname = "PM2.5_pruebasultimas";
  //bool wifiEnable;
  bool wifiEnable = true;
  bool ifxEnable;
//  bool apiEnable;
  bool apiEnable = true;
//  String ssid;   
  String ssid = "TPred";
//  String pass;
  String pass = "apt413sago16";
  String ifxdb;  
  String ifxip;  
  uint16_t ifxpt;
  String ifusr;
  String ifpss;
//  String apiusr;
  String apiusr = "danielbernalb";
//  String apipss;
String apipss = "danielchangeme";
//  String apisrv;
  String apisrv = "api.canair.io";
  String apiuri;
  int apiprt;
//  int stime;
  int stime = 60;
  double lat;
  double lon;
  float alt;
  float spd;
  bool isNewIfxdbConfig;
  bool isNewAPIConfig;
  bool isNewWifi;

  void init(const char app_name[]);

  void reload();

  bool save(const char *json);
  
  String getCurrentConfig();

  bool isWifiEnable();
  
  bool isIfxEnable();
  
  bool isApiEnable();

  void reboot ();

};

#endif