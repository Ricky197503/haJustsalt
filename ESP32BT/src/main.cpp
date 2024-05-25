#define ARDUINOHA_DEBUG
#include <Arduino.h>


/** Electolyseur JustSalt & co
 *
 *  Passerelle Bt Justsalt to Mqtt
 *
 *  Created: Mai 2025
 *      Author: Ricky
 *
*/


#include <NimBLEDevice.h>
#include <ArduinoHA.h>
#include <WiFi.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <TaskScheduler.h>

//#include <FS.h>         
//#include <DNSServer.h>
//#include <WiFiManager.h>   
//#include <ArduinoJson.h>   

const char* version = "0.1.2";
// Update these with values suitable for your wifi network.
const char* ssid = "Durand-wifi";
const char* password = "1975198020132017";

//Home Assistant integration
// configure here your HA params for connection to MQTT server
#define BROKER_ADDR IPAddress(192,168,50,11)
#define BROKER_USERNAME     "" // replace with your credentials
#define BROKER_PASSWORD     ""

//Unique device for HA integration
const byte deviceUniqID[] = { 0x94, 0xDE, 0xB8, 0xA1, 0x1A, 0xAC };

//justsalt service and char UUID
// The remote service we wish to connect to.
static  BLEUUID serviceUUID("09912756-7b32-4629-aeb1-b309d9a338ae");
// The characteristic of the remote service we are interested in.
static  BLEUUID    charUUID("ef785c24-22bb-463d-b651-0b7445ba091c");
static  BLEUUID    charUUIDwrite("4d32c5e5-2bb5-45c6-8c89-6f59bb3930d2");

static NimBLEAdvertisedDevice* advDevice;
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static int countertrameold = 0;
static int countertrame = 5;
static int countertry = 0;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static NimBLERemoteCharacteristic* pRemoteCharWrite;

static NimBLEClient* pClient;
static NimBLERemoteService* pSvc ;
static NimBLERemoteCharacteristic* pChr ;
static NimBLERemoteDescriptor* pDsc ;

WiFiClient wifiMQTT;
WiFiClient telnet;
WiFiServer telnetServer(23);

Scheduler timeScheduler;
AsyncWebServer webServer(80);
//WebServer webServer(80);

unsigned long previousMillis = 0;
unsigned long interval = 30000;

void scanEndedCB(NimBLEScanResults results);
void setup_wifi();
void setup_glogal();
void reconnect_wifi();
void setupHaIntegration();
void setup_telnet();
void cb_handleTelnet();
void cb_loopHaIntegration();
void cb_loopElegantOTA();
void cb_loopAvaibilityMQTT();
void cb_setupAndScan_ble();
void cb_connectBleServer();
void onValueConsignePhChanged( HANumeric number, HANumber* sender);
void onValueConsigneorpChanged( HANumeric number, HANumber* sender);
void onValueConsigneorpalarmeChanged( HANumeric number, HANumber* sender);
void onValueConsignevolChanged( HANumeric number, HANumber* sender);
void onValueConsigneacideChanged( HANumeric number, HANumber* sender);
void onValueConsigneprodChanged( HANumeric number, HANumber* sender);
void onValueConsigneinversionChanged( HANumeric number, HANumber* sender);
void onwriteble (int ValueID ,float numberset,int ValueSizea);
void handleRoot();
void reboot_Bluetooth();
void reboot_ESP();

bool connectToServer();


Task taskRebootBluetooth(interval*200,TASK_FOREVER,&reboot_Bluetooth);
Task taskRebootESP(interval*1000,TASK_FOREVER,&reboot_ESP);

Task taskSetup(5000,TASK_FOREVER,&setup_wifi);
Task taskReconnectWifi(interval,TASK_FOREVER,&reconnect_wifi);
Task taskTelnet(5000,TASK_FOREVER,&cb_handleTelnet);
Task taskloopElegantOTA(5000,TASK_FOREVER,&cb_loopElegantOTA);
Task taskBleSetupAndScan(30000, TASK_FOREVER, &cb_setupAndScan_ble);
Task taskConnectBleServer(50000, TASK_FOREVER, &cb_connectBleServer);
Task taskloopHaIntegration(5000, TASK_FOREVER,&cb_loopHaIntegration);
Task taskloopAvaibilityMQTT(3000, TASK_FOREVER, &cb_loopAvaibilityMQTT);

//static BLEAdvertisedDevice* advDevice;

static uint32_t scanTime = 0; /** 0 = scan forever */


struct detailvalue {
  int ID;
  int taille;
  String valeurdetrame;
  String valeurconverted;
  int status; // 0: pas decodé ; 1: decodé ; 2 partielement decodé  ; 3: inconnu (non exporter dans Mqtt) 
  String Description;
};


struct detailvalue Electrovaluefull[255];


HADevice deviceHA;
// dernier paramètre pour le nombre de sensorMQTT à lister
HAMqtt  mqtt(wifiMQTT, deviceHA, 90);

// Command ESP
HASwitch rebootesp("rebootESP");
HASwitch bluetoothesp("bluetoothESP");

//List of sensor for HA
HABinarySensor bluetoothConnected("pool_bluetooth_connected");
HASensorNumber wifiStrength("pool_wifi_strength", HASensorNumber::PrecisionP2);
HASensor justsaltIp("pool_ip");
HASensorNumber tempeau("pool_temp",HASensorNumber::PrecisionP1) ; 
HASensorNumber ph("pool_ph",HASensorNumber::PrecisionP1);
HASensorNumber orp("pool_orp",HASensorNumber::PrecisionP0);
HASensorNumber sel("pool_sel",HASensorNumber::PrecisionP1);
HANumber vol("pool_vol",HASensorNumber::PrecisionP0);
HANumber phconsigne("pool_ph_consigne",HASensorNumber::PrecisionP1);
HANumber acide("pool_acide",HASensorNumber::PrecisionP1);
HANumber prod("pool_prod",HASensorNumber::PrecisionP0);
HANumber orpconsigne("pool_orp_consigne",HASensorNumber::PrecisionP0);
HANumber orpalarme("pool_orp_alarme",HASensorNumber::PrecisionP0);
HANumber inversion("pool_inversion",HASensorNumber::PrecisionP0);
HASensorNumber temp02("pool_temp02",HASensorNumber::PrecisionP0);
HASensorNumber temp03("pool_temp03",HASensorNumber::PrecisionP0);
HASensorNumber temp08("pool_temp08",HASensorNumber::PrecisionP0);
HASensorNumber temp0B("pool_temp0B",HASensorNumber::PrecisionP0);
HASensorNumber temp0C("pool_temp0C",HASensorNumber::PrecisionP0);
HASensorNumber temp0D("pool_temp0D",HASensorNumber::PrecisionP0);
HASensorNumber temp0E("pool_temp0E",HASensorNumber::PrecisionP0);
HASensorNumber temp0F("pool_temp0F",HASensorNumber::PrecisionP0);
HASensorNumber temp10("pool_temp10",HASensorNumber::PrecisionP0);
HASensor temp12("pool_temp12");
HASensor temp13("pool_temp13");
HASensorNumber temp1F("pool_temp1F",HASensorNumber::PrecisionP0);
HASensorNumber temp28("pool_temp28",HASensorNumber::PrecisionP0);
HASensorNumber temp29("pool_temp29",HASensorNumber::PrecisionP0);
HASensorNumber temp2A("pool_temp2A",HASensorNumber::PrecisionP0);
HASensorNumber temp31("pool_temp31",HASensorNumber::PrecisionP0);
HASensorNumber temp50("pool_temp50",HASensorNumber::PrecisionP0);
HASensorNumber temp51("pool_temp51",HASensorNumber::PrecisionP0);
HASensorNumber temp5F("pool_temp5F",HASensorNumber::PrecisionP0);
HASensorNumber temp69("pool_temp69",HASensorNumber::PrecisionP0);
HASensorNumber temp6A("pool_temp6A",HASensorNumber::PrecisionP0);
HASensorNumber temp8F("pool_temp8F",HASensorNumber::PrecisionP0);
HASensor temp90("pool_temp90");
HASensor temp91("pool_temp91");
HASensorNumber temp92("pool_temp92",HASensorNumber::PrecisionP0);
HASensorNumber temp93("pool_temp93",HASensorNumber::PrecisionP0);
HABinarySensor temp93B00("false93B00");
HABinarySensor temp93B01("false93B01");
HABinarySensor temp93B02("false93B02");
HABinarySensor temp93B03("false93B03");
HABinarySensor temp93B04("false93B04");
HABinarySensor temp93B05("false93B05");
HABinarySensor temp93B06("false93B06");
HABinarySensor temp93B07("false93B07");
HABinarySensor temp93B10("false93B10");
HABinarySensor temp93B11("false93B11");
HABinarySensor temp93B12("false93B12");
HABinarySensor temp93B13("false93B13");
HABinarySensor temp93B14("false93B14");
HABinarySensor temp93B15("false93B15");
HABinarySensor temp93B16("false93B16");
HABinarySensor temp93B17("false93B17");
HASensorNumber temp94("pool_temp94",HASensorNumber::PrecisionP0);
HASensorNumber temp95("pool_temp95",HASensorNumber::PrecisionP0);
HASensor temp96("pool_temp96");
HASensor temp97("pool_temp97");
HASensor temp99("pool_temp99");
HASensor temp9A("pool_temp9A");
HASensorNumber temp9B("pool_temp9B",HASensorNumber::PrecisionP0);
HASensorNumber temp9C("pool_temp9C",HASensorNumber::PrecisionP0);
HASensor temp9D("pool_temp9D");
HASensor tempA3("pool_tempA3");
HASensorNumber tempB0("pool_tempB0",HASensorNumber::PrecisionP0);
HASensor tempB1("pool_tempB1");
HASensor tempD0("pool_tempD0");
HASensorNumber tempD1("pool_tempD1",HASensorNumber::PrecisionP0);
HASensor tempE1("pool_tempE1");
HASensorNumber tempE1O8("pool_tempE1O8",HASensorNumber::PrecisionP0);
HASensorNumber tempE1O9("pool_tempE1O9",HASensorNumber::PrecisionP0);
HASensorNumber tempE1O10("pool_tempE1O10",HASensorNumber::PrecisionP0);
HASensorNumber tempE1O11("pool_tempE1O11",HASensorNumber::PrecisionP0);
HASensorNumber tempE1O12("pool_tempE1O12",HASensorNumber::PrecisionP0);
HASensorNumber tempE1O13("pool_tempE1O13",HASensorNumber::PrecisionP0);

HASensor tempE2("pool_tempE2");
HASensor tempE4("pool_tempE4");
HASensorNumber tempFE("pool_tempFE",HASensorNumber::PrecisionP0);

HABinarySensor injfPH("injph");


static String htmltabdecode = "";
static String htmltabpasdecode = "";
static String htmltabpartieldecode = "";
static String htmltabinconnu = "";
static String page = "" ;

void handleRoot(){   // Début de la page HTML
    page = "<!DOCTYPE html>";
    page += "<html lang='fr'>";
    
    page += "<head>";
    page += "    <title>Passerel ESP32 / Electolyseur</title>";
    page += "    <meta http-equiv='refresh' content='60' name='viewport' content='width=device-width, initial-scale=1' charset='UTF-8'/>";
    page += "    <link rel='stylesheet' href='https://www.w3schools.com/w3css/4/w3.css'>";  // Utilisation du css 
    page += "</head>";

    page += "<body>";
    page += "    <div class='w3-panel w3-padding-small w3-blue w3-center'>";
    page += "        <p>Décodé</p>";
    page += "    </div>";
    page += "   <table class='w3-table-all'>";
    page += "   <thead><tr class='w3-green'><th scope='col'>id hex</th><th scope='col'>id</th><th scope='col'>Taille</th><th scope='col'>Valeur</th><th scope='col'>Valeur exploité</th><th scope='col'>Description</th><th scope='col'>Status</th></tr>";
    page += "   </thead><tbody>";
    page +=  htmltabdecode;
    page += "   </table>";

    page += "    <div class='w3-panel w3-padding-small w3-blue w3-center'>";
    page += "        <p>Pas décodé</p>";
    page += "    </div>";
    page += "   <table class='w3-table-all'>";
    page += "   <thead><tr class='w3-orange'><th scope='col'>id hex</th><th scope='col'>id</th><th scope='col'>Taille</th><th scope='col'>Valeur</th><th scope='col'>Valeur exploité</th><th scope='col'>Description</th><th scope='col'>Status</th></tr>";
    page += "   </thead><tbody>";
    page +=  htmltabpasdecode;
    page += "   </table>";

    page += "    <div class='w3-panel w3-padding-small w3-blue w3-center'>";
    page += "        <p>Partiellement décodé</p>";
    page += "    </div>";
    page += "   <table class='w3-table-all'>";
    page += "   <thead><tr class='w3-orange'><th scope='col'>id hex</th><th scope='col'>id</th><th scope='col'>Taille</th><th scope='col'>Valeur</th><th scope='col'>Valeur exploité</th><th scope='col'>Description</th><th scope='col'>Status</th></tr>";
    page += "   </thead><tbody>";
    page +=  htmltabpartieldecode;
    page += "   </table>";

    page += "    <div class='w3-panel w3-padding-small w3-blue w3-center'>";
    page += "        <p>Value inconnu</p>";
    page += "    </div>";
    page += "   <table class='w3-table-all'>";
    page += "   <thead><tr class='w3-red'><th scope='col'>id hex</th><th scope='col'>id</th><th scope='col'>Taille</th><th scope='col'>Valeur</th><th scope='col'>Valeur exploité</th><th scope='col'>Description</th><th scope='col'>Status</th></tr>";
    page += "   </thead><tbody>";
    page +=  htmltabinconnu;
    page += "   </table>";


    page += "    <div class='w3-center w3-padding-16'>";
    page += "        <p>Ce serveur est hébergé sur un ESP32</p>";
    page += "        <i>Créé par Richard Durand</i>";
    page += "    </div>";

    page += "</body>";
    page += "</html>";  // Fin de la page HTML

    //webServer.setContentLength(page.length());  // Permet l'affichage plus rapide après chaque clic sur les boutons
    //webServer.send(200, "text/html", page);
    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", page);
    });
}

void cb_handleTelnet() {
  if (telnetServer.hasClient()) {
    if (!telnet || !telnet.connected()) {
      if (telnet) telnet.stop();
      telnet = telnetServer.available();
    } else {
      telnetServer.available().stop();
    }
  }
}
void reboot_Bluetooth(){
    Serial.println("Reboot Ble by timer");
    btStop();
    delay(500);
    btStart();
}

void reboot_ESP(){
    Serial.println("Reboot ESP by timer");
    esp_restart();
}

void cb_loopElegantOTA(){
  ElegantOTA.loop();
}


void cb_loopAvaibilityMQTT(){
  mqtt.loop();
  //remove setAvaibility to use native check of Ha integration Shared availability
  //deviceHA.setAvailability(true);

  //savoir si la connexion bluetooth est OK ou si le justsalt n'est pas sous tension.
    bluetoothConnected.setState(connected);
  

}

void cb_loopHaIntegration(){
    //Serial.println("cb_loopHaIntegration");
    taskloopHaIntegration.disable();
    mqtt.loop();
    //remove setAvaibility to use native check of Ha integration Shared availability
    deviceHA.setAvailability(true);


    wifiStrength.setValue(WiFi.RSSI());
    justsaltIp.setValue(WiFi.localIP().toString().c_str());
    Serial.println("----------------------------- debut");
    telnet.println("----------------------------- debut");

    htmltabdecode = "";
    htmltabpasdecode = "";
    htmltabpartieldecode = "";
    htmltabinconnu = "";

  for (size_t i = 0; i < 254; i++)
  {
    if(Electrovaluefull[i].valeurdetrame != NULL ){
     
        std::string str = "Valeur ";
        str += ": ID = " + std::to_string(Electrovaluefull[i].ID);
        str += ": taille = " + std::to_string(Electrovaluefull[i].taille);
        str += ": Value = " + std::string(Electrovaluefull[i].valeurdetrame.c_str());

        Serial.println(str.c_str());
        telnet.println(str.c_str());

        String convertid = String(Electrovaluefull[i].ID,HEX) ;
        if (convertid.length() == 1 ){ 
            convertid = "0" + convertid ;
            }
        convertid.toUpperCase();


// 0: pas decodé ; 1: decodé ; 2 partielement decodé  ; 3: inconnu (non exporter dans Mqtt) 

        switch (Electrovaluefull[i].status){
            case 0:
                htmltabpasdecode += "<tr><th scope=''row''>" + convertid + "</th><th scope=''row''>" + String(Electrovaluefull[i].ID) + "</th><td>" + String(Electrovaluefull[i].taille) +"</td><td>" + String(Electrovaluefull[i].valeurdetrame.c_str()) +"</td><td>" + String(Electrovaluefull[i].valeurconverted.c_str()) +"</td><td>" + String(Electrovaluefull[i].Description) +"</td><td>" + String(Electrovaluefull[i].status) +"</td></tr>";
                break;
            case 1:
                htmltabdecode += "<tr><th scope=''row''>" + convertid + "</th><th scope=''row''>" + String(Electrovaluefull[i].ID) + "</th><td>" + String(Electrovaluefull[i].taille) +"</td><td>" + String(Electrovaluefull[i].valeurdetrame.c_str()) +"</td><td>" + String(Electrovaluefull[i].valeurconverted.c_str()) +"</td><td>" + String(Electrovaluefull[i].Description) +"</td><td>" + String(Electrovaluefull[i].status) +"</td></tr>";
                break;
            case 2:
                htmltabpartieldecode += "<tr><th scope=''row''>" + convertid + "</th><th scope=''row''>" + String(Electrovaluefull[i].ID) + "</th><td>" + String(Electrovaluefull[i].taille) +"</td><td>" + String(Electrovaluefull[i].valeurdetrame.c_str()) +"</td><td>" + String(Electrovaluefull[i].valeurconverted.c_str()) +"</td><td>" + String(Electrovaluefull[i].Description) +"</td><td>" + String(Electrovaluefull[i].status) +"</td></tr>";
                break;
            case 3:
                htmltabinconnu += "<tr><th scope=''row''>" + convertid + "</th><th scope=''row''>" + String(Electrovaluefull[i].ID) + "</th><td>" + String(Electrovaluefull[i].taille) +"</td><td>" + String(Electrovaluefull[i].valeurdetrame.c_str()) +"</td><td>" + String(Electrovaluefull[i].valeurconverted.c_str()) +"</td><td>" + String(Electrovaluefull[i].Description) +"</td><td>" + String(Electrovaluefull[i].status) +"</td></tr>";
                break;
        }
    } 
  }
  htmltabdecode += "</table>";
  htmltabinconnu += "</table>";
  htmltabpartieldecode += "</table>";
  htmltabpasdecode += "</table>";

  Serial.println("----------------------------- Fin");
  telnet.println("----------------------------- Fin");
  handleRoot();

}

void onStateChangedrebootesp (bool state, HASwitch* s){
    if (state == true){
        //lancer le reboot
        state =false;
        esp_restart();       
    }
}


void onStateChangedbluetoothesp (bool state, HASwitch* s){
    if (state == true){
        Serial.println("BT start");
        btStart();    
    } else {
        Serial.println("BT stop");
        btStop();
    }
}

void setupHaIntegration(){
    //HA integration
    //deviceUniqID =WiFi.macAddress();
    deviceHA.setUniqueId(deviceUniqID, sizeof(deviceUniqID));
    
    //deviceHA.setUniqueId(WiFi.macAddress(), sizeof(WiFi.macAddress()));
    
    deviceHA.setName("justsalt");
    deviceHA.setSoftwareVersion(version);
    deviceHA.setModel("Electolyseur");
    deviceHA.setManufacturer("ricky");

    // This method enables availability for all device types registered on the device.
    // For example, if you have 5 sensors on the same device, you can enable
    // shared availability and change availability state of all sensors using
    // single method call "device.setAvailability(false|true)"
    deviceHA.enableSharedAvailability();

    // Optionally, you can enable MQTT LWT feature. If device will lose connection
    // to the broker, all device types related to it will be marked as offline in
    // the Home Assistant Panel.
    deviceHA.enableLastWill();

    rebootesp.setName("Reboot ESP");
    rebootesp.setIcon("mdi:restart");
    rebootesp.onCommand(onStateChangedrebootesp);

    bluetoothesp.setName("Bluetooth ESP");
    bluetoothesp.setIcon("mdi:bluetooth");
    bluetoothesp.onCommand(onStateChangedbluetoothesp);

    wifiStrength.setName("Pool wifi Strength");
    wifiStrength.setDeviceClass("signal_strength");
    wifiStrength.setUnitOfMeasurement("dB");

    justsaltIp.setName("ESP justsalt IP");
    justsaltIp.setIcon("mdi:ip-network");

        // HA integration List of Sensor
    tempeau.setName("Water temp");
    tempeau.setUnitOfMeasurement("°C");
    tempeau.setDeviceClass("temperature");
    tempeau.setIcon("mdi:thermometer");
    
    orp.setName("orp");
    orp.setUnitOfMeasurement("mV");
    orp.setIcon("mdi:flash-triangle-outline");

    ph.setName("PH");
    ph.setIcon("mdi:ph");
    ph.setUnitOfMeasurement("ph");

    sel.setName("Sel");
    sel.setUnitOfMeasurement("g/L");
    sel.setIcon("mdi:water-opacity");

    phconsigne.setName("PH Consigne");
    phconsigne.setIcon("mdi:ph");
    phconsigne.setUnitOfMeasurement("ph");
    phconsigne.setStep(0.1);
    phconsigne.setMin(6.8);
    phconsigne.setMax(7.6);
    phconsigne.onCommand(onValueConsignePhChanged);

    orpconsigne.setName("orp Consigne");
    orpconsigne.setUnitOfMeasurement("mV");
    orpconsigne.setIcon("mdi:flash-triangle-outline");
    orpconsigne.setStep(10);
    orpconsigne.setMin(200);
    orpconsigne.setMax(900);
    orpconsigne.onCommand(onValueConsigneorpChanged);
    
    orpalarme.setName("orp Alarme");
    orpalarme.setUnitOfMeasurement("h");
    orpalarme.setIcon("mdi:alarm");
    orpalarme.setStep(6);
    orpalarme.setMin(12);
    orpalarme.setMax(96);
    orpalarme.onCommand(onValueConsigneorpalarmeChanged);
  
    vol.setName("volume piscine");
    vol.setUnitOfMeasurement("m3");
    vol.setIcon("mdi:cup-water");
    vol.setStep(10);
    vol.setMin(10);
    vol.setMax(200);
    vol.onCommand(onValueConsignevolChanged);

    acide.setName("taux Acide");
    acide.setUnitOfMeasurement("%");
    acide.setIcon("mdi:skull-crossbones-outline");
    acide.setStep(1);
    acide.setMin(5);
    acide.setMax(55);
    acide.onCommand(onValueConsigneacideChanged);
    
    prod.setName("production");
    prod.setUnitOfMeasurement("%");
    prod.setIcon("mdi:cog-outline");
    prod.setStep(1);
    prod.setMin(10);
    prod.setMax(100);
    prod.onCommand(onValueConsigneprodChanged);

    inversion.setName("inversion");
    inversion.setUnitOfMeasurement("h");
    inversion.setIcon("mdi:alarm");
    inversion.setStep(1);
    inversion.setMin(2);
    inversion.setMax(24);
    inversion.onCommand(onValueConsigneinversionChanged);
    
    temp02.setName("temp02");
    temp03.setName("temp03");
    temp08.setName("temp08");
    temp0B.setName("temp0B");
    temp0C.setName("temp0C");
    temp0D.setName("temp0D");
    temp0E.setName("durée total electrolyseur");
    temp0E.setUnitOfMeasurement("h");
    temp0E.setIcon("mdi:timer-sand");

    temp0F.setName("minutes de fonctionement");
    temp0F.setUnitOfMeasurement("Min");
    temp0F.setIcon("mdi:counter");

    temp10.setName("temp10");
    temp12.setName("date etalonage PH?");
    temp12.setIcon("mdi:calendar-star-four-points");

    temp13.setName("date etalonage ORP?");
    temp13.setIcon("mdi:calendar-star-four-points");
    
    temp1F.setName("temp1F");
    temp28.setName("temp28");
    temp29.setName("temp29");
    temp2A.setName("temp2A");
    temp31.setName("temp31");
    temp50.setName("temp50");
    temp51.setName("temp51");
    temp5F.setName("temp5F");
    temp69.setName("temp69");
    temp6A.setName("temp6A");
    temp8F.setName("temp8F");
    temp90.setName("temp90");
    temp91.setName("temp91");
    temp92.setName("temp92");
    temp93.setName("temp93");
    temp93B00.setName("temp93B00");
    temp93B01.setName("temp93B01");
    temp93B02.setName("temp93B02");
    temp93B03.setName("temp93B03");
    temp93B04.setName("temp93B04");
    temp93B05.setName("temp93B05");
    temp93B06.setName("temp93B06");
    temp93B07.setName("temp93B07");
    temp93B10.setName("temp93B10");
    temp93B11.setName("temp93B11");
    temp93B12.setName("temp93B12");
    temp93B13.setName("temp93B13");
    temp93B14.setName("temp93B14");
    temp93B15.setName("temp93B15");
    temp93B16.setName("temp93B16");
    temp93B17.setName("temp93B17");
    temp94.setName("temp94");
    temp95.setName("ID code");
    temp95.setIcon("mdi:barcode");

    temp96.setName("version");
    temp96.setIcon("mdi:qrcode");

    temp97.setName("slave");
    temp97.setIcon("mdi:qrcode-edit");

    temp99.setName("nom");
    temp99.setIcon("mdi:card-account-details-outline");

    temp9A.setName("SN");
    temp9A.setIcon("mdi:barcode");

    temp9B.setName("temp9B");
    temp9C.setName("temp9C");
    temp9D.setName("temp9D");
    tempA3.setName("tempA3");
    tempB0.setName("tempB0");
    tempB1.setName("Mac adr");
    tempB1.setIcon("mdi:network-outline");

    tempD0.setName("tempD0");
    tempD1.setName("TLV");
    tempE1.setName("tempE1");

    tempE1O8.setName("tempE1O8");
    tempE1O9.setName("tempE1O9");
    tempE1O10.setName("tempE1O10");
    tempE1O11.setName("tempE1O11");
    tempE1O12.setName("tempE1O12");
    tempE1O13.setName("tempE1O13");
   
    tempE2.setName("tempE2");
    tempE4.setName("tempE4");
    tempFE.setName("tempFE");

    injfPH.setName("Injection PH-");
    injfPH.setIcon("mdi:ph");

    bluetoothConnected.setName("Bluetooth Status");

  mqtt.begin( BROKER_ADDR, BROKER_USERNAME, BROKER_PASSWORD );

}

void setup_telnet(){
  telnetServer.begin();
  telnetServer.setNoDelay(true); 

  Serial.print("Ready! Use 'telnet ");
  Serial.print(WiFi.localIP());
  Serial.println(" 23' to connect");
  
  timeScheduler.addTask(taskTelnet);
  taskTelnet.enable();
  Serial.println("Add Task telnet handle");
}

void reconnect_wifi(){
   unsigned long currentMillis = millis();
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    #ifdef SYSLOG_SERVER
      syslog.log(LOG_INFO, "WIFI lost and reconnect automatically");
    #endif
    previousMillis = currentMillis;
  }
}

void setup_wifi() {
    taskSetup.disable();

    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");

    Serial.println(ssid);
    Serial.println(password);
    

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    randomSeed(micros());
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    //Reconnect wifi Task
    timeScheduler.addTask(taskReconnectWifi);
    taskReconnectWifi.enable();
    Serial.print("Add task to monitor and reconnect wifi");
    
    //Elegant OTA
    // webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    // request->send(200, "text/plain", "Hi! I am ESP32 justsalt to update use http://[yourIP]/update.");
    //  });
    handleRoot();
    // webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    //     request->send_P(200, "text/html", index_html);
    // });
    ElegantOTA.begin(&webServer);    // Start ElegantOTA
    webServer.begin();
    Serial.println("HTTP server started");

    //elegant OTA loop
    timeScheduler.addTask(taskloopElegantOTA);
    taskloopElegantOTA.enable();
    Serial.println("Add task for loop of Elegant OTA");

    //telnet setup
    setup_telnet();
    //launch handle telnet
    timeScheduler.addTask(taskTelnet);
    taskTelnet.enable();
    Serial.print("Add task to Handle telnet");

    //setup mqtt
    setupHaIntegration();
    //loop avaibility for mqtt
    timeScheduler.addTask(taskloopAvaibilityMQTT);
    taskloopAvaibilityMQTT.enable();
    telnet.print("Add task for loop of MQTT");
    
    //cb_setupAndScan_ble();
    doScan = true;
    timeScheduler.addTask(taskConnectBleServer);
    taskConnectBleServer.enable();
    
    Serial.print("Add task to Connec Ble server!!!!!!");



}


/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        Serial.println("Connected");
        /** After connection we should change the parameters if we don't need fast response times.
         *  These settings are 150ms interval, 0 latency, 450ms timout.
         *  Timeout should be a multiple of the interval, minimum is 100ms.
         *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
         *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
         */
        pClient->updateConnParams(120,120,0,60);
    };

    void onDisconnect(NimBLEClient* pClient) {
        Serial.print(pClient->getPeerAddress().toString().c_str());
        Serial.println(" Disconnected - Starting scan");
        NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
    };

    /** Called when the peripheral requests a change to the connection parameters.
     *  Return true to accept and apply them or false to reject and keep
     *  the currently used parameters. Default will return true.
     */
    bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params) {
        if(params->itvl_min < 24) { /** 1.25ms units */
            return false;
        } else if(params->itvl_max > 40) { /** 1.25ms units */
            return false;
        } else if(params->latency > 2) { /** Number of intervals allowed to skip */
            return false;
        } else if(params->supervision_timeout > 100) { /** 10ms units */
            return false;
        }

        return true;
    };

    /********************* Security handled here **********************
    ****** Note: these are the same return values as defaults ********/
    uint32_t onPassKeyRequest(){
        Serial.println("Client Passkey Request");
        /** return the passkey to send to the server */
        return 123456;
    };

    bool onConfirmPIN(uint32_t pass_key){
        Serial.print("The passkey YES/NO number: ");
        Serial.println(pass_key);
    /** Return false if passkeys don't match. */
        return true;
    };

    /** Pairing process complete, we can check the results in ble_gap_conn_desc */
    void onAuthenticationComplete(ble_gap_conn_desc* desc){
        if(!desc->sec_state.encrypted) {
            Serial.println("Encrypt connection failed - disconnecting");
            /** Find the client with the connection handle provided in desc */
            NimBLEDevice::getClientByID(desc->conn_handle)->disconnect();
            return;
        }
    };
};

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    /**
        Called for each advertising BLE server.
    */
    void onResult(BLEAdvertisedDevice* advertisedDevice) {
        std::string str = "------------------------------- \r\n" ;
        str += "BLE Advertised Device found: \r\n";
        str += ": Name = " + std::string(advertisedDevice->getName().c_str());
        str += ", adr = " + std::string(advertisedDevice->getAddress().toString().c_str());
        
        Serial.println(str.c_str());
        telnet.println(str.c_str());

        char* manufacturerdata = BLEUtils::buildHexData(NULL, (uint8_t*)advertisedDevice->getManufacturerData().data(), advertisedDevice->getManufacturerData().length());
        Serial.println(manufacturerdata);
        telnet.println(manufacturerdata);

        if ((strcmp(manufacturerdata , "ffff00202020202020202020202020202020202020") == 0) or (strcmp(manufacturerdata , "ffff01202020202020202020202020202020202020") == 0))  {
        // if (advertisedDevice->haveServiceUUID() && advertisedDevice->getServiceUUID().equals(serviceUUID)) {

            BLEDevice::getScan()->stop();
            advDevice = advertisedDevice; /** Just save the reference now, no need to copy the object */
            
            Serial.print("Found our device!  address: ");

            doConnect = true;
            doScan = false;

            // timeScheduler.addTask(taskConnectBleServer);
            taskConnectBleServer.forceNextIteration();
            Serial.println("add Task Connect Ble server");

        } // Found our server
    } // onResult
}; // MyAdvertisedDeviceCallbacks



/** Notification / Indication receiving handler callback */
void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify){
    std::string str = (isNotify == true) ? "Notification" : "Indication";
    str += " from ";
    /** NimBLEAddress and NimBLEUUID have std::string operators */
    str += std::string(pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress());
    str += ": Service = " + std::string(pRemoteCharacteristic->getRemoteService()->getUUID());
    str += ", Characteristic = " + std::string(pRemoteCharacteristic->getUUID());
    str += ", Countertrame = " + std::to_string(countertrame);
    str += ", Value = " + std::string((char*)pData, length);
    countertrame++;
    if (countertrame > 65000){
        countertrame=5;
        countertrameold=0;
    } 
    Serial.println(str.c_str());
    telnet.println(str.c_str());

    Serial.print("recep chaine taille =");
    Serial.println(length);

    telnet.println("recep chaine");
    //telnet.println(std::to_string(length));
    if (length > 5){
        int index =3;
        while (index +2 < length) {
            uint8_t idvaleur = pData[index];
            int taillevaleur = pData[index + 1 ];
            String Valuetrame = ""; 
            for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                String converttrame = String(pData[i],HEX) ;
                if (converttrame.length() == 1 ){ 
                    converttrame = "0" + converttrame ;
                    }
                Valuetrame += converttrame;
                if (i != index + 1 + taillevaleur){
                    Valuetrame += "." ; 
                }
            }
            Valuetrame.toUpperCase();

            // Serial.printf( "Values ID : %ld - taille : %ld \r\n", idvaleur, taillevaleur );
            // telnet.printf( "Values ID : %ld - taille : %ld \r\n", idvaleur, taillevaleur );
            std::string str = "Trame " + std::to_string(countertrame);
            str += ": Values ID  = " + std::to_string(idvaleur);
            str += ", taille = " + std::to_string(taillevaleur);
            str += ", Value = " + std::string(Valuetrame.c_str());

            Serial.println(str.c_str());
            telnet.println(str.c_str());

            Electrovaluefull[idvaleur].ID=idvaleur;
            Electrovaluefull[idvaleur].taille=taillevaleur;
            Electrovaluefull[idvaleur].valeurdetrame=Valuetrame.c_str();
            Electrovaluefull[idvaleur].Description="";
            Electrovaluefull[idvaleur].status=0;

            switch (idvaleur) {
            case 0x00: { 

                }break;
        
            case 0x01: {
                uint8_t phhex = pData[index + 2];
                float phf = static_cast<float>(phhex)/10;
                ph.setValue(phf);
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(phf)).c_str();
                Electrovaluefull[idvaleur].Description="PH"; 
                Electrovaluefull[idvaleur].status=1;

                }break;

            case 0x02: {
                String info ="";
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                temp02.setValue(temp);
                if ((bool)((temphex & 0x000A) )) {
                    info += "<BR> &nbsp;&nbsp; - XXXA - Injection PH-: true";
                    injfPH.setCurrentState(true);
                } else {
                    info += "<BR> &nbsp;&nbsp; - XXX0 - Injection PH-: false";
                    injfPH.setCurrentState(false);
                }
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(temp)).c_str();
                //info += "</span>";
                Electrovaluefull[idvaleur].Description="Injection PH + ...." + info ; 
                Electrovaluefull[idvaleur].status=2;
                }break;

            case 0x03: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                temp03.setValue(temp);
                }break;

            case 0x06: {
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index + 3]));
                //float temp = static_cast<float>(temphex);
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(temphex)).c_str();
                Electrovaluefull[idvaleur].Description="ORP"; 
                orp.setValue(temphex);;
                }break;

            case 0x08: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                temp08.setValue(temp);
                }break;

            case 0x09: {
                uint16_t tempeauhex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float tempeauf = static_cast<float>(tempeauhex)/10;
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(tempeauf)).c_str();
                Electrovaluefull[idvaleur].Description="temperature eau"; 
                tempeau.setValue(tempeauf);
                }break;
            
            case 0x0A: {
                uint8_t selhex = pData[index + 2] ;
                float self = static_cast<float>(selhex)/10;
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(self)).c_str();
                Electrovaluefull[idvaleur].Description="taux de sel"; 
                sel.setValue(self);
                }break;

            case 0x0B: {
                if(countertry == 20) {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                temp0B.setValue(temp);
                }
                }break;
            
            case 0x0C: {
                if(countertry == 20) {
                    uint8_t temphex = pData[index + 2];
                    float temp = static_cast<float>(temphex);
                    temp0C.setValue(temp);
                }
                }break;

            case 0x0D: {
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                temp0D.setValue(temp);
                }break;

            case 0x0E: {
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(temp)).c_str();
                Electrovaluefull[idvaleur].Description="Compteur Heure total de fonctionnement"; 
                temp0E.setValue(temp);
                }break;

            case 0x0F: {
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(temp)).c_str();
                Electrovaluefull[idvaleur].Description="Minutes de fonctionnement"; 
                temp0F.setValue(temp);
                }break;
            
            case 0x10: {
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                temp10.setValue(temp);
                }break;

            case 0x11: {
                uint16_t volhex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float volf = static_cast<float>(volhex);
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(volf)).c_str();
                Electrovaluefull[idvaleur].Description="Volume de la piscine"; 
                vol.setState(volf);
                }break;
            
            case 0x12: {
                uint8_t dayphex = (pData[index + 3]);
                uint8_t monthphex = (pData[index + 4]);
                uint8_t yearphex = (pData[index +5]);
                int dayf = static_cast<int>(dayphex);
                int monthf = static_cast<int>(monthphex);
                int yearf = static_cast<int>(yearphex);

                std::string datestr =  std::to_string(dayf) + "/" + std::to_string(monthf) + "/20" + std::to_string(yearf);
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (datestr.c_str());
                Electrovaluefull[idvaleur].Description="Date etalonage PH"; 
                temp12.setValue(datestr.c_str()) ;
                }break;

            case 0x13: {
                uint8_t dayphex = (pData[index + 3]);
                uint8_t monthphex = (pData[index + 4]);
                uint8_t yearphex = (pData[index +5]);
                int dayf = static_cast<float>(dayphex);
                int monthf = static_cast<float>(monthphex);
                int yearf = static_cast<float>(yearphex);
                std::string datestr =  std::to_string(dayf) + "/" + std::to_string(monthf) + "/20" + std::to_string(yearf);
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (datestr.c_str());
                Electrovaluefull[idvaleur].Description="Date etalonage ORP"; 
                temp13.setValue(datestr.c_str()) ;
                }break;

            case 0x1F: {
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                temp1F.setValue(temp);
                }break;
            
            case 0x28: {
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                temp28.setValue(temp);
                
                }break;
            
            case 0x29: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                temp29.setValue(temp);
                }break;
            
            case 0x2A: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                temp2A.setValue(temp);
                }break;
            
            case 0x30: {
                uint8_t phchex = pData[index + 2] ;
                float phcf = static_cast<float>(phchex)/10;
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(phcf)).c_str();
                Electrovaluefull[idvaleur].Description="Consigne PH"; 
                phconsigne.setState(phcf);
                }break;
            
            case 0x31: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                temp31.setValue(temp);
                }break;         

            case 0x32: {
                uint8_t acidehex = pData[index + 2] ;
                float acidef = static_cast<float>(acidehex);
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(acidef)).c_str();
                Electrovaluefull[idvaleur].Description="taux acide"; 
                acide.setState(acidef);
                }break;
            
            case 0x33: {
                uint8_t prodhex = pData[index + 2] ;
                float prodf = static_cast<float>(prodhex);
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(prodf)).c_str();
                Electrovaluefull[idvaleur].Description="production"; 
                prod.setState(prodf);
                }break;

            case 0x35: {
                uint8_t orpchex = pData[index + 2] ;
                float orpcf = static_cast<float>(orpchex)*10;
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(orpcf)).c_str();
                Electrovaluefull[idvaleur].Description="Consigne ORP"; 
                orpconsigne.setState(orpcf);
                }break;
            
            case 0x37: {
                uint8_t alarmeorphex = pData[index + 2] ;
                float alarmeorpf = static_cast<float>(alarmeorphex);
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(alarmeorpf)).c_str();
                Electrovaluefull[idvaleur].Description="seuil Alarme ORP"; 
                orpalarme.setState(alarmeorpf);
                }break;
    
            case 0x39: {
                uint8_t inversionhex = pData[index + 2] ;
                float inversionvalf = static_cast<float>(inversionhex);
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(inversionvalf)).c_str();
                Electrovaluefull[idvaleur].Description="Inversion"; 
                inversion.setState(inversionvalf);
                }break;

            case 0x50: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                temp50.setValue(temp);
                }break;

            case 0x51: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                temp51.setValue(temp);
                }break;

            case 0x5F: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                temp5F.setValue(temp);
                }break;
            
            case 0x69: {
                uint32_t temphex = (pData[index + 2]<< 24) + (pData[index + 3]<< 16)+ (pData[index + 4]<< 8) + (pData[index +5]);
                float temp = static_cast<float>(temphex);
                temp69.setValue(temp);
                }break;

            case 0x6A: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                temp6A.setValue(temp);
                }break;

            case 0x8F: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                temp8F.setValue(temp);
                }break;
                
            case 0x90: {
                temp90.setValue(Valuetrame.c_str());
                }break;

            case 0x91: {
                temp91.setValue(Valuetrame.c_str());
                }break;
            
            case 0x92: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                temp92.setValue(temp);
                }break;

            case 0x93: {
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                temp93.setValue(temp);
                temp93B00.setCurrentState((bool)((temphex & 0x0001) ));
                temp93B01.setCurrentState((bool)((temphex & 0x0002) ));
                temp93B02.setCurrentState((bool)((temphex & 0x0004) ));
                temp93B03.setCurrentState((bool)((temphex & 0x0008) ));
                temp93B04.setCurrentState((bool)((temphex & 0x0010) ));
                temp93B05.setCurrentState((bool)((temphex & 0x0020) ));
                temp93B06.setCurrentState((bool)((temphex & 0x0040) ));
                temp93B07.setCurrentState((bool)((temphex & 0x0080) ));
                temp93B10.setCurrentState((bool)((temphex & 0x0100) ));
                temp93B11.setCurrentState((bool)((temphex & 0x0200) ));
                temp93B12.setCurrentState((bool)((temphex & 0x0400) ));
                temp93B13.setCurrentState((bool)((temphex & 0x0800) ));
                temp93B14.setCurrentState((bool)((temphex & 0x1000) ));
                temp93B15.setCurrentState((bool)((temphex & 0x2000) ));
                temp93B16.setCurrentState((bool)((temphex & 0x4000) ));
                temp93B17.setCurrentState((bool)((temphex & 0x8000) ));
                }break;

            case 0x94: {
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                temp94.setValue(temp);
                }break;

            case 0x95: {
                uint32_t idcodehex = (pData[index + 2]<< 24) + (pData[index + 3]<< 16)+ (pData[index + 4]<< 8) + (pData[index +5]);
                float idcodefloat = static_cast<float>(idcodehex);
                Electrovaluefull[idvaleur].status=2;
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(idcodefloat)).c_str();
                Electrovaluefull[idvaleur].Description="ID Code"; 
                temp95.setValue(idcodefloat);
                
                }break;
            
            case 0x96: {
                temp96.setValue(Valuetrame.c_str());
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = Valuetrame.c_str();
                Electrovaluefull[idvaleur].Description="version"; 
                }break;
            
            case 0x97: {
                temp97.setValue(Valuetrame.c_str());
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = Valuetrame.c_str();
                Electrovaluefull[idvaleur].Description="Slave (version hardware)";
                }break;

            case 0x99: {
                std::string tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    tempstring += pData[i];
                }
                temp99.setValue(tempstring.c_str());
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = tempstring.c_str();
                Electrovaluefull[idvaleur].Description="Nom";
                
                }break;

            case 0x9A: {
                std::string tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    tempstring += pData[i];
                }
                temp9A.setValue(tempstring.c_str());
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = tempstring.c_str();
                Electrovaluefull[idvaleur].Description="SN";
                }break;

            case 0x9B: {
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                temp9B.setValue(temp);
                }break;

            case 0x9C: {
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                temp9C.setValue(temp);
                }break;

            case 0x9D: {
                temp9D.setValue(Valuetrame.c_str());
                }break;
            
            case 0xA3: {
                tempA3.setValue(Valuetrame.c_str());
                }break;
            
            case 0xB0: {
                if(countertry == 20) {

                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                tempB0.setValue(temp);
                }
                
                }break;
                
    
            case 0xB1: {
                String tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    tempstring += String(pData[i],HEX) ;
                    if (i != index + 1 + taillevaleur){
                        tempstring += ":" ; 
                    }
                }
                tempstring.toUpperCase();
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = tempstring.c_str();
                Electrovaluefull[idvaleur].Description="Mac adress";
                tempB1.setValue(tempstring.c_str());
                }break;
            
            case 0xD0: {
            //     //D0.06.30.00.00.00.00.00
                String tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    String convert = String(pData[i],HEX) ;
                    if (convert.length() == 1 ){ 
                        convert = "0" + convert ;
                        }
                    tempstring += convert;
                    if (i != index + 1 + taillevaleur){
                        tempstring += "." ; 
                    }
                }
                tempstring.toUpperCase();
                tempD0.setValue(tempstring.c_str());
                }break;

            case 0xD1: {
                uint8_t temphex = (pData[index + 2]) ;
                float temp = static_cast<float>(temphex);
                tempD1.setValue(temp);
                Electrovaluefull[idvaleur].status=1;
                Electrovaluefull[idvaleur].valeurconverted = (std::to_string(temp)).c_str();
                Electrovaluefull[idvaleur].Description="TLV";
                }break;
            
            

            case 0xE1: {
                //  E1.0F.18.04.11.11.00.33.00.00.88.00.F8.00.00.00.00
                tempE1.setValue(Valuetrame.c_str());
                tempE1O8.setValue(static_cast<int>((pData[index +2 +8])) );
                tempE1O9.setValue(static_cast<int>((pData[index +2 +9])) );
                tempE1O10.setValue(static_cast<int>((pData[index +2 +10])) );
                tempE1O11.setValue(static_cast<int>((pData[index +2 +11])) );
                tempE1O12.setValue(static_cast<int>((pData[index +2 +12]) ));
                tempE1O13.setValue(static_cast<int>((pData[index +2 +13]) ));
                }break;
                
            case 0xE2: {
                // E2.0F.18.04.16.0A.22.10.00.00.00.41.3E.DC.00.00.00
                tempE2.setValue(Valuetrame.c_str());
                }break;

            case 0xE4: {
                // E4.0F.18.04.12.15.00.00.46.00.0B.13.00.00.00.00.00
                tempE4.setValue(Valuetrame.c_str());
                }break;
    
            case 0xFE: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                tempFE.setValue(temp);
                }break;
    
            default: {
                Serial.print( "idvaleur Non implementee " );
                Serial.println(idvaleur);
                Electrovaluefull[idvaleur].status=3;
                };
            }
            countertry ++;
            if (countertry == 21) {
                countertry = 0;
            }
            index = index + 2 + taillevaleur ;

        }
        timeScheduler.addTask(taskloopHaIntegration);
        taskloopHaIntegration.enable();
        //Serial.println("fin traitement trame");
       
    }
}

/** Callback to process the results of the last scan or restart it */
void scanEndedCB(NimBLEScanResults results){
    Serial.println("Scan Ended");
}


/** Create a single global instance of the callback class to be used by all clients */
static ClientCallbacks clientCB;

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(advDevice->toString().c_str());

  BLEClient*  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");

  // Connect to the remove BLE Server.
  pClient->connect(advDevice); 
  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");


  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  if(pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  //il faut s'abonner la la charactisitic "indication" afin de recevoir les notification de publication
  if(pRemoteCharacteristic->canIndicate())
    pRemoteCharacteristic->subscribe(false, notifyCB);

    pRemoteCharWrite = pRemoteService->getCharacteristic(charUUIDwrite);
    if (pRemoteCharWrite == nullptr) {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(charUUIDwrite.toString().c_str());
        pClient->disconnect();
        return false;
    } else {
        Serial.println("caracteristic Write - Found our characteristic");
    }


  connected = true;
  return true;
  
}

void cb_setupAndScan_ble() {
    taskBleSetupAndScan.disable();
    Serial.println("Starting NimBLE Client");
    /** Initialize NimBLE, no device name spcified as we are not advertising */
    NimBLEDevice::init("");
    /** Set the IO capabilities of the device, each option will trigger a different pairing method.
     *  BLE_HS_IO_KEYBOARD_ONLY    - Passkey pairing
     *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
     *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
     */
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY); // use passkey
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

    /** 2 different ways to set security - both calls achieve the same result.
     *  no bonding, no man in the middle protection, secure connections.
     *
     *  These are the default values, only shown here for demonstration.
     */
    NimBLEDevice::setSecurityAuth(true, true, true);
    NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM | BLE_SM_PAIR_AUTHREQ_SC);

    /** Optional: set the transmit power, default is 3db */
    #ifdef ESP_PLATFORM
        NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
    #else
        NimBLEDevice::setPower(9); /** +9db */
    #endif

    /** Optional: set any devices you don't want to get advertisments from */
    // NimBLEDevice::addIgnored(NimBLEAddress ("aa:bb:cc:dd:ee:ff"));

    /** create new scan */
    NimBLEScan* pScan = NimBLEDevice::getScan();
     Serial.println("avant callback");
    /** create a callback that gets called when advertisers are found */
    pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());

    /** Set scan interval (how often) and window (how long) in milliseconds */
    pScan->setInterval(45);
    pScan->setWindow(15);

    /** Active scan will gather scan response data from advertisers
     *  but will use more energy from both devices
     */
    Serial.println("avant set active scan");
    pScan->setActiveScan(true);
    /** Start scanning for advertisers for the scan time specified (in seconds) 0 = forever
     *  Optional callback for when scanning stops.
     */
    //pScan->start(scanTime, scanEndedCB);
    pScan->start(10, false);
    Serial.println("Fin de cb_setupAndScan_ble");
    
}

void onValueConsignePhChanged( HANumeric number, HANumber* sender){
  if (!number.isSet()) {
        // the reset command was send by Home Assistant
    } else {
        float numberFloat = number.toFloat();
        std::string str = "Value of PH Consigne changed: ";
        str += ": New valeur = " + std::to_string(numberFloat);

        Serial.println(str.c_str());
        telnet.println(str.c_str()); 
        onwriteble(48,numberFloat * 10,1);
    }
    sender->setState(number); // report the selected option back to the HA panel
  
}
void onValueConsigneorpChanged( HANumeric number, HANumber* sender){
  if (!number.isSet()) {
        // the reset command was send by Home Assistant
    } else {
        float numberFloat = number.toFloat();
        std::string str = "Value of ORP Consigne changed: ";
        str += ": New valeur = " + std::to_string(numberFloat);

        Serial.println(str.c_str());
        telnet.println(str.c_str()); 
        onwriteble(53,numberFloat / 10,1);
    }
    sender->setState(number); // report the selected option back to the HA panel
  
}

void onValueConsigneorpalarmeChanged( HANumeric number, HANumber* sender){
  if (!number.isSet()) {
        // the reset command was send by Home Assistant
    } else {
        float numberFloat = number.toFloat();
        std::string str = "Value of Alarme ORP Consigne changed: ";
        str += ": New valeur = " + std::to_string(numberFloat);

        Serial.println(str.c_str());
        telnet.println(str.c_str()); 
        onwriteble(0x37,numberFloat ,1);
    }
    sender->setState(number); // report the selected option back to the HA panel
}

void onValueConsignevolChanged( HANumeric number, HANumber* sender){
  if (!number.isSet()) {
        // the reset command was send by Home Assistant
    } else {
        float numberFloat = number.toFloat();
        std::string str = "Value of Volume Consigne changed: ";
        str += ": New valeur = " + std::to_string(numberFloat);

        Serial.println(str.c_str());
        telnet.println(str.c_str()); 
        onwriteble(0x11,numberFloat ,2);
    }
    sender->setState(number); // report the selected option back to the HA panel
}

void onValueConsigneacideChanged( HANumeric number, HANumber* sender){
  if (!number.isSet()) {
        // the reset command was send by Home Assistant
    } else {
        float numberFloat = number.toFloat();
        std::string str = "Value of Acide Consigne changed: ";
        str += ": New valeur = " + std::to_string(numberFloat);

        Serial.println(str.c_str());
        telnet.println(str.c_str()); 
        onwriteble(0x32,numberFloat ,1);
    }
    sender->setState(number); // report the selected option back to the HA panel
}

void onValueConsigneprodChanged( HANumeric number, HANumber* sender){
  if (!number.isSet()) {
        // the reset command was send by Home Assistant
    } else {
        float numberFloat = number.toFloat();
        std::string str = "Value of Production Consigne changed: ";
        str += ": New valeur = " + std::to_string(numberFloat);

        Serial.println(str.c_str());
        telnet.println(str.c_str()); ;
        onwriteble(0x33,numberFloat ,1);
    }
    sender->setState(number); // report the selected option back to the HA panel
}

void onValueConsigneinversionChanged( HANumeric number, HANumber* sender){
  if (!number.isSet()) {
        // the reset command was send by Home Assistant
    } else {
        float numberFloat = number.toFloat();
        std::string str = "Value of Version Consigne changed: ";
        str += ": New valeur = " + std::to_string(numberFloat);

        Serial.println(str.c_str());
        telnet.println(str.c_str());        
        onwriteble(0x39,numberFloat ,1);
    }
    sender->setState(number); // report the selected option back to the HA panel
}
void onwriteble (int ValueID ,float numberset,int ValueSizea){
    Serial.println("debut onwriteble : ");
    static uint8_t trame[80] = {0x00, 0x06, 0x03, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    //union float2bytes { float f; char b[sizeof(float)]; } ;
    typedef union _data {
        int f;
        char  s[2];
    } myData;

    //float2bytes myvalue;
    myData myvalue;
    myvalue.f =  static_cast<int>(numberset);
    trame[3]= ValueID ;
    trame[4]= ValueSizea;
    switch (ValueSizea){
    case 1:
        //uint8_t hexnumberset = (numberset) & 0xff;
        trame[5]= myvalue.s[0];
        break;

    case 2:
        trame[5]= 0x00;
        trame[6]= myvalue.s[0];
        break;
    
    default:
        break;
    }
    //Serial.println(sizeof(trame));
    std::string str = "send des valeur ";
    str += ": Number a set = " + std::to_string(numberset);
    str += ": Myvalue f = " + std::to_string(myvalue.f);
    str += ": Myvalue s 0 = " + std::to_string(myvalue.s[0]);
    str += ": Myvalue s 1 = " + std::to_string(myvalue.s[1]);
    str += ": value ID = " + std::to_string(trame[3]);
    str += ", taille = " + std::to_string(trame[4]);
    str += ", Value ch 5= " + std::to_string(trame[5]);
    str += ", Value ch 6= " + std::to_string(trame[6]);

    Serial.println(str.c_str());
    telnet.println(str.c_str());
    
    bool writeResponse;
    writeResponse = pRemoteCharWrite->writeValue(trame, 80, true);
    Serial.println(writeResponse);
    str = "onwriteble  : ";
    str += "send trame result = " + std::to_string(writeResponse);
    if (writeResponse == 1) {
        str += " OK ";
    }else {
        str += " KO ";
    }
    Serial.println(str.c_str());
    telnet.println(str.c_str());
}



void setup (){
    Serial.begin(115200);
    // Wifi Manager & wait?

    timeScheduler.init();
    timeScheduler.addTask(taskSetup);
    taskSetup.enable();
    Serial.println("add Task to setup justsalt");
    //bluetoothesp.setState(true);
}


void loop (){
    timeScheduler.execute();
    
}


void cb_connectBleServer(){
  //connection au serveur Ble
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
    Serial.println("debut cb_connectBleServer");
   
    if (connected == true ){
        Serial.println("connected = true");

        // check si on est connecté BLE (evolution des trames notifié ) 
        if (countertrame != countertrameold){
            //Serial.println("recepion notification ok donc conecté ");
            std::string str = "recepion notification ok donc conecté ";
            str += ": countertrame  = " + std::to_string(countertrame);
            str += ", countertrameold = " + std::to_string(countertrameold);
            
            Serial.println(str.c_str());
            telnet.println(str.c_str());

            countertrameold=countertrame;
            delay(50);
        } else {
            Serial.println("recepion notification KO donc liaison BT HS ");
            connected = false;
            doConnect = true;
        } 
        Serial.println("apres check connection");
        
    }else{
        if (doConnect == true) {
            Serial.println("doConnect is true");
            if (connectToServer()) {
                Serial.println("We are now connected to the BLE Server.");
                //write sur la prochaine itération de la task
                taskConnectBleServer.forceNextIteration();
            } else {
                Serial.println("We have failed to connect to the server; there is nothin more we will do.");
                doScan = true;
            }
            doConnect = false;
            Serial.println("doConnect ********************* to false ");
            //on relance un scan
        }
        if (doScan == true){
            Serial.println( "add Task to Scan Ble devices");
            timeScheduler.addTask(taskBleSetupAndScan);
            taskBleSetupAndScan.enable();
        }
        Serial.println("connected = false");
    } 
}