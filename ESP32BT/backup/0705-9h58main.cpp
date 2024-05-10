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


const char* version = "0.0.1";
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

WiFiClient wifiMQTT;
WiFiClient telnet;
WiFiServer telnetServer(23);

Scheduler timeScheduler;
AsyncWebServer webServer(80);


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


Task taskSetup(5000,TASK_FOREVER,&setup_wifi);
Task taskReconnectWifi(interval,TASK_FOREVER,&reconnect_wifi);
Task taskTelnet(5000,TASK_FOREVER,&cb_handleTelnet);
Task taskloopElegantOTA(5000,TASK_FOREVER,&cb_loopElegantOTA);
Task taskBleSetupAndScan(30000, TASK_FOREVER, &cb_setupAndScan_ble);
Task taskConnectBleServer(50000, TASK_FOREVER, &cb_connectBleServer);
Task taskloopHaIntegration(5000, TASK_FOREVER,&cb_loopHaIntegration);
Task taskloopAvaibilityMQTT(3000, TASK_FOREVER, &cb_loopAvaibilityMQTT);



static NimBLEAdvertisedDevice* advDevice;

static bool doConnect = false;
static uint32_t scanTime = 0; /** 0 = scan forever */

struct trame_value {
    uint8_t     V_id_00;
    float       V_id_01;    // PH
    float       V_id_02;
    float       V_id_03;
    uint16_t    V_id_06;    // ORP
    float       V_id_08;
    float       V_id_09;    // temp Eau
    float       V_id_0A;    // Sel
    float       V_id_0B;
    float       V_id_0C;
    float       V_id_0D;
    float       V_id_0E;
    float       V_id_0F;    // minutes de fonctionement
    float       V_id_10;
    float       V_id_11;    // Volume eau
    float       V_id_12;
    float       V_id_13;
    float       V_id_1F;
    float       V_id_28;
    float       V_id_29;
    float       V_id_2A;
    float       V_id_30;    // Consigne PH
    float       V_id_31;
    float       V_id_32;    // % Acide
    float       V_id_33;    // Production ?
    float       V_id_35;    // Consigne ORP
    float       V_id_37;    // Seuil alarme ORP
    float       V_id_39;    // Inversion
    float       V_id_50;
    float       V_id_51;
    float       V_id_5F;
    float       V_id_69;
    float       V_id_6A;
    float       V_id_8F;
    uint64_t    V_id_90;
    uint64_t    V_id_91;
    float       V_id_92;
    uint16_t     V_id_93;
    bool        V_id_93_b00;
    bool        V_id_93_b01;
    bool        V_id_93_b02;
    bool        V_id_93_b03;
    bool        V_id_93_b04;
    bool        V_id_93_b05;
    bool        V_id_93_b06;
    bool        V_id_93_b07;
    bool        V_id_93_b10;
    bool        V_id_93_b11;
    bool        V_id_93_b12;
    bool        V_id_93_b13;
    bool        V_id_93_b14;
    bool        V_id_93_b15;
    bool        V_id_93_b16;
    bool        V_id_93_b17;
    float       V_id_94;
    uint32_t    V_id_95;    // ID Code
    String      V_id_96;    // Version
    String      V_id_97;    // Slave
    String      V_id_99;    // Nom
    String      V_id_9A;    // SN
    float       V_id_9B;
    float       V_id_9C;
    String      V_id_9D;
    String      V_id_A3;
    float       V_id_B0;
    String      V_id_B1;    // Mac adresse
    String      V_id_D0;
    float       V_id_D1;
    String      V_id_E1;
    String      V_id_E2;
    String      V_id_E4;
    float       V_id_FE;
};
struct trame_value Electrovalue;
HADevice deviceHA;
// dernier paramètre pour le nombre de sensorMQTT à lister
HAMqtt  mqtt(wifiMQTT, deviceHA, 70);

// Command ESP
HASwitch rebootesp("rebootESP");
//List of sensor for HA
HASensorNumber wifiStrength("pool_wifi_strength", HASensorNumber::PrecisionP2);
HASensor justsaltIp("pool_ip");
HASensorNumber temp("pool_temp",HASensorNumber::PrecisionP1) ; 
HASensorNumber ph("pool_ph",HASensorNumber::PrecisionP1);
HASensorNumber orp("pool_orp",HASensorNumber::PrecisionP0);
HASensorNumber sel("pool_sel",HASensorNumber::PrecisionP1);
HASensorNumber vol("pool_vol",HASensorNumber::PrecisionP0);
HANumber phconsigne("pool_ph_consigne",HASensorNumber::PrecisionP1);
HASensorNumber acide("pool_acide",HASensorNumber::PrecisionP1);
HASensorNumber prod("pool_prod",HASensorNumber::PrecisionP0);
HASensorNumber orpconsigne("pool_orp_consigne",HASensorNumber::PrecisionP0);
HASensorNumber orpalarme("pool_orp_alarme",HASensorNumber::PrecisionP0);
HASensorNumber inversion("pool_inversion",HASensorNumber::PrecisionP0);
HASensorNumber temp02("pool_temp02",HASensorNumber::PrecisionP0);
HASensorNumber temp03("pool_temp03",HASensorNumber::PrecisionP0);
HASensorNumber temp08("pool_temp08",HASensorNumber::PrecisionP0);
HASensorNumber temp0B("pool_temp0B",HASensorNumber::PrecisionP0);
HASensorNumber temp0C("pool_temp0C",HASensorNumber::PrecisionP0);
HASensorNumber temp0D("pool_temp0D",HASensorNumber::PrecisionP0);
HASensorNumber temp0E("pool_temp0E",HASensorNumber::PrecisionP0);
HASensorNumber temp0F("pool_temp0F",HASensorNumber::PrecisionP0);
HASensorNumber temp10("pool_temp10",HASensorNumber::PrecisionP0);
HASensorNumber temp12("pool_temp12",HASensorNumber::PrecisionP0);
HASensorNumber temp13("pool_temp13",HASensorNumber::PrecisionP0);
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
HASensorNumber temp90("pool_temp90",HASensorNumber::PrecisionP0);
HASensorNumber temp91("pool_temp91",HASensorNumber::PrecisionP0);
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
HASensor tempE2("pool_tempE2");
HASensor tempE4("pool_tempE4");
HASensorNumber tempFE("pool_tempFE",HASensorNumber::PrecisionP0);

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

void cb_loopElegantOTA(){
  ElegantOTA.loop();
}


void cb_loopAvaibilityMQTT(){
  mqtt.loop();
  //remove setAvaibility to use native check of Ha integration Shared availability
  deviceHA.setAvailability(true);

  //savoir si la connexion bluetooth est OK ou si le justsalt n'est pas sous tension.
  //bluetoothConnected.setState(connected);

}

void cb_loopHaIntegration(){
    taskloopHaIntegration.disable();
    mqtt.loop();
    //remove setAvaibility to use native check of Ha integration Shared availability
    //deviceHA.setAvailability(true);
    wifiStrength.setValue(WiFi.RSSI());
    justsaltIp.setValue(WiFi.localIP().toString().c_str());

    ph.setValue(Electrovalue.V_id_01);
    temp02.setValue(Electrovalue.V_id_02);
    temp03.setValue(Electrovalue.V_id_03);
    orp.setValue(Electrovalue.V_id_06);
    temp08.setValue(Electrovalue.V_id_08);
    temp.setValue(Electrovalue.V_id_09);
    sel.setValue(Electrovalue.V_id_0A);
    temp0B.setValue(Electrovalue.V_id_0B);
    temp0C.setValue(Electrovalue.V_id_0C);
    temp0D.setValue(Electrovalue.V_id_0D);
    temp0E.setValue(Electrovalue.V_id_0E);
    temp0F.setValue(Electrovalue.V_id_0F);
    temp10.setValue(Electrovalue.V_id_10);
    vol.setValue(Electrovalue.V_id_11);
    temp12.setValue(Electrovalue.V_id_12);
    temp13.setValue(Electrovalue.V_id_13);
    temp1F.setValue(Electrovalue.V_id_1F);
    temp28.setValue(Electrovalue.V_id_28);
    temp29.setValue(Electrovalue.V_id_29);
    phconsigne.setState(Electrovalue.V_id_30);
    temp31.setValue(Electrovalue.V_id_31);
    acide.setValue(Electrovalue.V_id_32);
    prod.setValue(Electrovalue.V_id_33);
    orpconsigne.setValue(Electrovalue.V_id_35);
    orpalarme.setValue(Electrovalue.V_id_37);
    inversion.setValue(Electrovalue.V_id_39);
    temp50.setValue(Electrovalue.V_id_50);
    temp51.setValue(Electrovalue.V_id_51);
    temp5F.setValue(Electrovalue.V_id_5F);
    temp69.setValue(Electrovalue.V_id_69);
    temp6A.setValue(Electrovalue.V_id_6A);
    temp8F.setValue(Electrovalue.V_id_8F);
    temp92.setValue(Electrovalue.V_id_92);
    temp93.setValue(Electrovalue.V_id_93);
    temp93B00.setCurrentState(Electrovalue.V_id_93_b00);
    temp93B01.setCurrentState(Electrovalue.V_id_93_b01);
    temp93B02.setCurrentState(Electrovalue.V_id_93_b02);
    temp93B03.setCurrentState(Electrovalue.V_id_93_b03);
    temp93B04.setCurrentState(Electrovalue.V_id_93_b04);
    temp93B05.setCurrentState(Electrovalue.V_id_93_b05);
    temp93B06.setCurrentState(Electrovalue.V_id_93_b06);
    temp93B07.setCurrentState(Electrovalue.V_id_93_b07);
    temp93B10.setCurrentState(Electrovalue.V_id_93_b10);
    temp93B11.setCurrentState(Electrovalue.V_id_93_b11);
    temp93B12.setCurrentState(Electrovalue.V_id_93_b12);
    temp93B13.setCurrentState(Electrovalue.V_id_93_b13);
    temp93B14.setCurrentState(Electrovalue.V_id_93_b14);
    temp93B15.setCurrentState(Electrovalue.V_id_93_b15);
    temp93B16.setCurrentState(Electrovalue.V_id_93_b16);
    temp93B17.setCurrentState(Electrovalue.V_id_93_b17);
    temp94.setValue(Electrovalue.V_id_94);
    temp95.setValue(Electrovalue.V_id_95);
    temp9B.setValue(Electrovalue.V_id_9B);
    temp9C.setValue(Electrovalue.V_id_9C);
    tempFE.setValue(Electrovalue.V_id_FE);

//    tempB1.setValue(Electrovalue.V_id_B1);
//    tempD0.setValue(Electrovalue.V_id_D0);
    // Valeur des selecteurs



}

void onStateChangedrebootesp (bool state, HASwitch* s){
    if (state == true){
        //lancer le reboot
        state =false;
        //esp_restart();
        //timeScheduler.disable() ;
        // esp_restart();
     
        
    }
}


void setupHaIntegration(){
    //HA integration
    deviceHA.setUniqueId(deviceUniqID, sizeof(deviceUniqID));
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
    rebootesp.setIcon("mdi:alpha-r-box-outline");
    rebootesp.onCommand(onStateChangedrebootesp);

    wifiStrength.setName("Pool wifi Strength");
    wifiStrength.setDeviceClass("signal_strength");
    wifiStrength.setUnitOfMeasurement("dB");

    justsaltIp.setName("ESP justsalt IP");
    justsaltIp.setIcon("mdi:ip-network");

        // HA integration List of Sensor
    temp.setName("Water temp");
    temp.setUnitOfMeasurement("°C");
    temp.setDeviceClass("temperature");
    temp.setIcon("mdi:thermometer");
    
    orp.setName("orp");
    orp.setUnitOfMeasurement("mV");
    orp.setIcon("mdi:alpha-r-box-outline");

    ph.setName("PH");
    ph.setIcon("mdi:ph");
    ph.setUnitOfMeasurement("ph");

    sel.setName("Sel");
    sel.setUnitOfMeasurement("g/L");
    sel.setIcon("mdi:alpha-s-box-outline");

    phconsigne.setName("PH Consigne");
    phconsigne.setIcon("mdi:ph");
    phconsigne.setUnitOfMeasurement("ph");
    phconsigne.setStep(0.1);
    phconsigne.setMin(6.5);
    phconsigne.setMax(7.8);
    phconsigne.onCommand(onValueConsignePhChanged);

    orpconsigne.setName("orp Consigne");
    orpconsigne.setUnitOfMeasurement("mV");
    orpconsigne.setIcon("mdi:alpha-r-box-outline");
    
    orpalarme.setName("orp Alarme");
    orpalarme.setUnitOfMeasurement("h");
    orpalarme.setIcon("mdi:alpha-r-box-outline");
  
    vol.setName("volume piscine");
    vol.setUnitOfMeasurement("m3");
    vol.setIcon("mdi:alpha-r-box-outline");

    acide.setName("taux Acide");
    acide.setUnitOfMeasurement("%");
    acide.setStateClass("measurement");
    acide.setIcon("mdi:Skull-Crossbones");
    
    prod.setName("production");
    prod.setUnitOfMeasurement("%");
    prod.setStateClass("measurement");
    prod.setIcon("mdi:Cog-Outline");

    inversion.setName("inversion");
    inversion.setUnitOfMeasurement("h");
    

    temp02.setName("temp02");
    temp03.setName("temp03");
    temp08.setName("temp08");
    temp0B.setName("temp0B");
    temp0C.setName("temp0C");
    temp0D.setName("temp0D");
    temp0E.setName("temp0E");
    temp0F.setName("minutes de fonctionement");
    temp0F.setUnitOfMeasurement("Min");
    temp10.setName("temp10");
    temp12.setName("temp12");
    temp13.setName("temp13");
    temp1F.setName("temp1F");
    temp28.setName("temp28");
    temp29.setName("temp29");
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
    temp96.setName("version");
    temp97.setName("slave");
    temp99.setName("nom");
    temp9A.setName("SN");
    temp9B.setName("temp9B");
    temp9C.setName("temp9C");
    temp9D.setName("temp9D");
    tempA3.setName("tempA3");
    tempB0.setName("tempB0");
    tempB1.setName("Mac adr");
    tempD0.setName("tempD0");
    tempD1.setName("tempD1");
    tempE1.setName("tempE1");
    tempE2.setName("tempE2");
    tempE4.setName("tempE4");
    tempFE.setName("tempFE");

//   bluetoothConnected.setName("Bluetooth Status");

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

    WiFi.mode(WIFI_STA);
    Serial.println(ssid);
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
    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32 justsalt to update use http://[yourIP]/update.");
     });
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
    cb_setupAndScan_ble();
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


/** Define a class to handle the callbacks when advertisments are received */
class AdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {

    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        Serial.print("Advertised Device found: ");
        Serial.println(advertisedDevice->toString().c_str());
        telnet.println("BLE Advertised Device found: ");
        telnet.println(advertisedDevice->toString().c_str());

        char* manufacturerdata = BLEUtils::buildHexData(NULL, (uint8_t*)advertisedDevice->getManufacturerData().data(), advertisedDevice->getManufacturerData().length());
        Serial.println(manufacturerdata);
        if ((strcmp(manufacturerdata , "ffff00202020202020202020202020202020202020") == 0) or (strcmp(manufacturerdata , "ffff01202020202020202020202020202020202020") == 0))  {
        //if(advertisedDevice->isAdvertisingService(NimBLEUUID("DEAD")))
        //{
            Serial.println("Found Our device");
            /** stop scan before connecting */
            NimBLEDevice::getScan()->stop();
            /** Save the device reference in a global for the client to use*/
            advDevice = advertisedDevice;
            /** Ready to connect now */
            doConnect = true;
        }
    };
};


/** Notification / Indication receiving handler callback */
void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify){
    std::string str = (isNotify == true) ? "Notification" : "Indication";
    str += " from ";
    /** NimBLEAddress and NimBLEUUID have std::string operators */
    str += std::string(pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress());
    str += ": Service = " + std::string(pRemoteCharacteristic->getRemoteService()->getUUID());
    str += ", Characteristic = " + std::string(pRemoteCharacteristic->getUUID());
    str += ", Value = " + std::string((char*)pData, length);
    Serial.println(str.c_str());
    Serial.print("recep chaine taille =");
    Serial.println(length);
    telnet.print("recep chaine taille = ");
    telnet.println(length);
    if (length > 5){
        int index =3;
        while (index +2 < length) {
            uint8_t idvaleur = pData[index];
            int taillevaleur = pData[index + 1 ];
            //Serial.print("ID Val = ");
            //Serial.println(idvaleur);
            Serial.printf( "Values ID : %ld - taille : %ld \r\n", idvaleur, taillevaleur );

            //telnet.print("ID Val = ");
            //telnet.println(idvaleur);
            telnet.printf( "Values ID : %ld - taille : %ld \r\n", idvaleur, taillevaleur );
            
            std::list<uint8_t> listvalu8 = { 0x00, 0x93 };
            std::list<uint8_t> listvalstring = { 0x99, 0x9A, 0x9D, 0xA3, 0xB1, 0xD0 };
            
            if (find(listvalu8.rbegin(), listvalu8.rend(), idvaleur) != listvalu8.rend()){
                telnet.println("type string");
                Serial.println("type string");

            } 
            if (find(listvalstring.rbegin(), listvalstring.rend(), idvaleur) != listvalstring.rend()){
                telnet.println("type string");
                Serial.println("type string");

            }



            switch (idvaleur) {
            case 0x00: { 
                Electrovalue.V_id_00 = 0;
                }break;
        
            case 0x01: {
                uint8_t phhex = pData[index + 2];
                float ph = static_cast<float>(phhex)/10;
                Electrovalue.V_id_01 = ph;
                Serial.print("PH : " );
                Serial.println(ph);
                }break;

            case 0x02: {
                uint8_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_02 = temp;
                Serial.print("temp02 : " );
                Serial.println(temp);
                }break;

            case 0x03: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_03 = temp; 
                }break;

            case 0x06: {
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index + 3]));
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_06 = temphex;
                Serial.print("ORP : " );
                Serial.println(temp);
                telnet.print( "data: " );
                telnet.println( temphex );
                }break;

            case 0x08: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_08 = temp;
                }break;

            case 0x09: {
                uint16_t tempeauhex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float tempeau = static_cast<float>(tempeauhex)/10;
                Electrovalue.V_id_09 = tempeau;
                }break;
            
            case 0x0A: {
                uint8_t selhex = pData[index + 2] ;
                float sel = static_cast<float>(selhex)/10;
                Electrovalue.V_id_0A = sel;
                }break;

            case 0x0B: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_0B = temp;
                }break;
            
            case 0x0C: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_0C = temp;
                }break;

            case 0x0D: {
                uint8_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_0D = temp;
                }break;

            case 0x0E: {
                uint8_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_0E = temp;
                }break;

            case 0x0F: {
                uint8_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_0F = temp;
                }break;
            
            case 0x10: {
                uint8_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_10 = temp;
                }break;

            case 0x11: {
                uint16_t volhex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float vol = static_cast<float>(volhex);
                Electrovalue.V_id_11 = vol;
                }break;
            
            case 0x12: {
                uint32_t temphex = (pData[index + 2]<< 24) + (pData[index + 3]<< 16)+ (pData[index + 4]<< 8) + (pData[index +5]);
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_12 = temp;
                }break;

            case 0x13: {
                uint32_t temphex = (pData[index + 2]<< 24) + (pData[index + 3]<< 16)+ (pData[index + 4]<< 8) + (pData[index +5]);
                float temp = static_cast<float>(temphex);
                //std::string tempstr = value_accuracy_to_string (temp,0);
                Electrovalue.V_id_13 = temp;
                }break;

            case 0x1F: {
                uint8_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_1F = temp;
                }break;
            
            case 0x28: {
                uint8_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_28 = temp;
                }break;
            
            case 0x29: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_29 = temp;
                }break;
            
            case 0x2A: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_2A = temp;
                }break;
            
            case 0x30: {
                uint8_t phchex = pData[index + 2] ;
                float phc = static_cast<float>(phchex)/10;
                Electrovalue.V_id_30 = phc;
                }break;
            
            case 0x31: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_31 = temp;
                }break;         

            case 0x32: {
                uint8_t acidehex = pData[index + 2] ;
                float acide = static_cast<float>(acidehex);
                Electrovalue.V_id_32 = acide;
                }break;
            
            case 0x33: {
                uint8_t prodhex = pData[index + 2] ;
                float prod = static_cast<float>(prodhex);
                Electrovalue.V_id_33 = prod;
                }break;

            case 0x35: {
                uint8_t orpchex = pData[index + 2] ;
                float orpc = static_cast<float>(orpchex)*10;
                Electrovalue.V_id_35 = orpc;
                }break;
            
            case 0x37: {
                uint8_t alarmeorphex = pData[index + 2] ;
                float alarmeorp = static_cast<float>(alarmeorphex);
                Electrovalue.V_id_37 = alarmeorp;
                }break;
    
            case 0x39: {
                uint8_t inversionhex = pData[index + 2] ;
                float inversionval = static_cast<float>(inversionhex);
                Electrovalue.V_id_39 = inversionval;
                }break;

            case 0x50: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_50 = temp;
                }break;

            case 0x51: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_51 = temp;
                }break;

            case 0x5F: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_5F = temp;
                }break;
            
            case 0x69: {
                uint32_t temphex = (pData[index + 2]<< 24) + (pData[index + 3]<< 16)+ (pData[index + 4]<< 8) + (pData[index +5]);
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_69 = temp;
                }break;

            case 0x6A: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_6A = temp;
                }break;

            case 0x8F: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_8F = temp;
                }break;
                
            case 0x90: {
                // 90.08.00.00.00.00.00.00.00.14.
                // std::string tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "data: " );
                    Serial.println( pData[i], HEX );
                    telnet.print( "data: " );
                    telnet.println( pData[i], HEX );
                    //tempstring += pData[i];
                }
                // temp90.setValue(tempstring.c_str());
                uint64_t temphex =  (pData[index + 6]<< 24) + (pData[index + 7]<< 16)+ (pData[index + 8]<< 8) + pData[index + 9];
                float temp = static_cast<float>(temphex);
                telnet.printf( "Temp : %ld - tempex : %ld \r\n", temp, temphex );
                temp90.setValue(temp);
                //Electrovalue.V_id_90 = temp;
                }break;

            case 0x91: {
                //  91.08.00.00.00.00.00.06.6F.C3.
                //std::string tempstring = ""; 
                // for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                //     Serial.print( "data: " );
                //     Serial.println( pData[i], HEX );
                //     telnet.print( "data: " );
                //     telnet.println( pData[i], HEX );
                //     tempstring += pData[i];
                // }
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "data: " );
                    Serial.println( pData[i], HEX );
                    telnet.print( "data: " );
                    telnet.println( pData[i], HEX );
                    //tempstring += pData[i];
                }
                // temp90.setValue(tempstring.c_str());
                uint64_t temphex =  (pData[index + 6]<< 24) + (pData[index + 7]<< 16)+ (pData[index + 8]<< 8) + pData[index + 9];
                float temp = static_cast<float>(temphex);
                telnet.printf( "Temp : %ld - tempex : %ld \r\n", temp, temphex );
                temp91.setValue(temp);
                //temp91.setValue(tempstring.c_str());
                }break;
            
            case 0x92: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_92 = temp;
                }break;

            case 0x93: {
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_93 = temp;
                // Electrovalue.V_id_93_b00 = (bool)((temphex & 0x0001) );
                // Electrovalue.V_id_93_b01 = (bool)((temphex & 0x0002) );
                // Electrovalue.V_id_93_b02 = (bool)((temphex & 0x0004) );
                // Electrovalue.V_id_93_b03 = (bool)((temphex & 0x0008) );
                // Electrovalue.V_id_93_b04 = (bool)((temphex & 0x0010) );
                // Electrovalue.V_id_93_b05 = (bool)((temphex & 0x0020) );
                // Electrovalue.V_id_93_b06 = (bool)((temphex & 0x0040) );
                // Electrovalue.V_id_93_b07 = (bool)((temphex & 0x0080) );
                // Electrovalue.V_id_93_b10 = (bool)((temphex & 0x0100) );
                // Electrovalue.V_id_93_b11 = (bool)((temphex & 0x0200) );
                // Electrovalue.V_id_93_b12 = (bool)((temphex & 0x0400) );
                // Electrovalue.V_id_93_b13 = (bool)((temphex & 0x0800) );
                // Electrovalue.V_id_93_b14 = (bool)((temphex & 0x1000) );
                // Electrovalue.V_id_93_b15 = (bool)((temphex & 0x2000) );
                // Electrovalue.V_id_93_b16 = (bool)((temphex & 0x4000) );
                // Electrovalue.V_id_93_b17 = (bool)((temphex & 0x8000) );
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
                uint8_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_94 = temp;
                }break;

            case 0x95: {
                uint32_t idcodehex = (pData[index + 2]<< 24) + (pData[index + 3]<< 16)+ (pData[index + 4]<< 8) + (pData[index +5]);
                float idcodefloat = static_cast<float>(idcodehex);
                Electrovalue.V_id_95 = idcodefloat;
                
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "95 data: " );
                    Serial.println( pData[i], HEX );
                    telnet.print( "95 data: " );
                    telnet.println( pData[i], HEX );
                }
                
                }break;
            
            case 0x96: {
                String tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "data: " );
                    Serial.println( pData[i], HEX );
                    telnet.print( "data: " );
                    telnet.println( pData[i], HEX );
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
                telnet.println( tempstring.c_str());
                temp96.setValue(tempstring.c_str());
                }break;
            
            case 0x97: {
                String tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "97 data: " );
                    Serial.println( pData[i], HEX );
                    telnet.print( "97 data: " );
                    telnet.println( pData[i], HEX );
                    
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
                telnet.println( tempstring.c_str());
                temp97.setValue(tempstring.c_str());
                }break;

            case 0x99: {
                //99.16.47.45.4E.5F.30.35.32.32.2D.30.34.33.39.35.33.2D.30.30.33.00.00.00.
                // std::string tempstring = value2.substr((index + 2 ), taillevaleur);
                // id(JustSalt_nombt_textsensor).publish_state(tempstring.c_str());
                std::string tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "data: " );
                    Serial.println( pData[i], HEX );
                    tempstring += pData[i];
                }
                temp99.setValue(tempstring.c_str());
                }break;

            case 0x9A: {
                //9A.10.30.35.32.32.2D.30.34.33.39.35.33.2D.30.30.33.00.
                // std::string tempstring = value2.substr((index + 2 ), taillevaleur);
                // id(JustSalt_sn_textsensor).publish_state(tempstring.c_str());
                std::string tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "data: " );
                    Serial.println( pData[i], HEX );
                    tempstring += pData[i];
                }
                temp9A.setValue(tempstring.c_str());
                }break;

            case 0x9B: {
                uint8_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_9B = temp;
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "9B data: " );
                    Serial.println( pData[i], HEX );
                    telnet.print( "9B data: " );
                    telnet.println( pData[i], HEX );
                }
                }break;

            case 0x9C: {
                uint8_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_9C = temp;
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "9C data: " );
                    Serial.println( pData[i], HEX );
                    telnet.print( "9C data: " );
                    telnet.println( pData[i], HEX );
                }
                }break;

            case 0x9D: {
                //  9D.08.00.00.00.00.00.00.01.FF
    String tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "9D data: " );
                    Serial.println( pData[i], HEX );
                    telnet.print( "9D data: " );
                    telnet.println( pData[i], HEX );
                    
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
                telnet.println( tempstring.c_str());
                temp9D.setValue(tempstring.c_str());
                }break;
            
            case 0xA3: {
                //  E1.0F.18.04.11.11.00.33.00.00.88.00.F8.00.00.00.00
                String tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "A3 data: " );
                    Serial.println( pData[i], HEX );
                    telnet.print( "A3 data: " );
                    telnet.println( pData[i], HEX );
                    
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
                telnet.println( tempstring.c_str());
                tempA3.setValue(tempstring.c_str());
                }break;
            
            case 0xB0: {
            //     uint8_t temphex = pData[index + 2];
            //     float temp = static_cast<float>(temphex);
            //     id(JustSalt_tempb0_sensor).publish_state(temp);
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_B0 = temp;
                }break;
                
    
            case 0xB1: {
            //     std::string tempstring = rawhex.substr((index + 2 )* 2, 12);
            //     id(JustSalt_macbt_textsensor).publish_state(tempstring);
                String tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "data: " );
                    Serial.println( pData[i], HEX );
                    telnet.print( "data: " );
                    telnet.println( pData[i], HEX );

                    tempstring += String(pData[i],HEX) ;
                    if (i != index + 1 + taillevaleur){
                        tempstring += ":" ; 
                    }
                }
                tempstring.toUpperCase();
                telnet.println( tempstring.c_str());
                tempB1.setValue(tempstring.c_str());
                }break;
            
            case 0xD0: {
            //     //D0.06.30.00.00.00.00.00
                String tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "D0 data: " );
                    Serial.println( pData[i], HEX );
                    telnet.print( "D0 data: " );
                    telnet.println( pData[i], HEX );
                    
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
                telnet.println( tempstring.c_str());
                tempD0.setValue(tempstring.c_str());
                }break;

            case 0xD1: {
            //     uint8_t temphex = pData[index + 2];
            //     float temp = static_cast<float>(temphex);
            //     id(JustSalt_tempd1_sensor).publish_state(temp);
                uint16_t temphex = ((pData[index + 2]<< 8) + (pData[index +3]));
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_D1 = temp;
                }break;
            
            

            case 0xE1: {
                //  E1.0F.18.04.11.11.00.33.00.00.88.00.F8.00.00.00.00
                String tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "E1 data: " );
                    Serial.println( pData[i], HEX );
                    telnet.print( "E1 data: " );
                    telnet.println( pData[i], HEX );
                    
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
                telnet.println( tempstring.c_str());
                tempE1.setValue(tempstring.c_str());
                }break;
                
            case 0xE2: {
                // E2.0F.18.04.16.0A.22.10.00.00.00.41.3E.DC.00.00.00
                String tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "E2 data: " );
                    Serial.println( pData[i], HEX );
                    telnet.print( "E2 data: " );
                    telnet.println( pData[i], HEX );
                    
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
                telnet.println( tempstring.c_str());
                tempE2.setValue(tempstring.c_str());
                }break;

            case 0xE4: {
                // E4.0F.18.04.12.15.00.00.46.00.0B.13.00.00.00.00.00
                String tempstring = ""; 
                for ( int i = index + 2; i < index + 2 + taillevaleur ; i++ ) {
                    Serial.print( "E4 data: " );
                    Serial.println( pData[i], HEX );
                    telnet.print( "E4 data: " );
                    telnet.println( pData[i], HEX );
                    
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
                telnet.println( tempstring.c_str());
                tempE4.setValue(tempstring.c_str());
                }break;
    
            case 0xFE: {
                uint8_t temphex = pData[index + 2];
                float temp = static_cast<float>(temphex);
                Electrovalue.V_id_FE = temp;
                }break;
    
            default: {
                Serial.print( "idvaleur Non implementee " );
                Serial.println(idvaleur);
                };
            }
            index = index + 2 + taillevaleur ;

        }
        Serial.println("fin traitement trame");
        cb_loopHaIntegration();
        Serial.println("fin cb_loopHaIntegration");
        
    }
}

/** Callback to process the results of the last scan or restart it */
void scanEndedCB(NimBLEScanResults results){
    Serial.println("Scan Ended");
}


/** Create a single global instance of the callback class to be used by all clients */
static ClientCallbacks clientCB;



/** Handles the provisioning of clients and connects / interfaces with the server */
bool connectToServer() {
    NimBLEClient* pClient = nullptr;

    /** Check if we have a client we should reuse first **/
    if(NimBLEDevice::getClientListSize()) {
        /** Special case when we already know this device, we send false as the
         *  second argument in connect() to prevent refreshing the service database.
         *  This saves considerable time and power.
         */
        pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
        if(pClient){
            if(!pClient->connect(advDevice, false)) {
                Serial.println("Reconnect failed");
                return false;
            }
            Serial.println("Reconnected client");

        }
        /** We don't already have a client that knows this device,
         *  we will check for a client that is disconnected that we can use.
         */
        else {
            pClient = NimBLEDevice::getDisconnectedClient();
        }
    }

    /** No client to reuse? Create a new one. */
    if(!pClient) {
        if(NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS) {
            Serial.println("Max clients reached - no more connections available");
            return false;
        }

        pClient = NimBLEDevice::createClient();

        Serial.println("New client created");

        pClient->setClientCallbacks(&clientCB, false);
        /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
         *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
         *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
         *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
         */
        pClient->setConnectionParams(12,12,0,51);
        /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
        pClient->setConnectTimeout(5);


        if (!pClient->connect(advDevice)) {
            /** Created a client but failed to connect, don't need to keep it as it has no data */
            NimBLEDevice::deleteClient(pClient);
            Serial.println("Failed to connect, deleted client");
            return false;
        }
    }

    if(!pClient->isConnected()) {
        if (!pClient->connect(advDevice)) {
            Serial.println("Failed to connect");
            return false;
        }
    }

    Serial.print("Connected to: ");
    Serial.println(pClient->getPeerAddress().toString().c_str());
    Serial.print("RSSI: ");
    Serial.println(pClient->getRssi());
    timeScheduler.addTask(taskBleSetupAndScan);
    taskBleSetupAndScan.enable();

    /** Now we can read/write/subscribe the charateristics of the services we are interested in */
    NimBLERemoteService* pSvc = nullptr;
    NimBLERemoteCharacteristic* pChr = nullptr;
    NimBLERemoteDescriptor* pDsc = nullptr;

    //pSvc = pClient->getService("DEAD");
   
    pSvc = pClient->getService(serviceUUID);
    if(pSvc) {     /** make sure it's not null */
        //pChr = pSvc->getCharacteristic("BEEF");
        pChr = pSvc->getCharacteristic(charUUID);
        
        if(pChr) {     /** make sure it's not null */
            if(pChr->canRead()) {
                std::string value = pChr->readValue();
                Serial.print(pChr->getUUID().toString().c_str());
                Serial.print(" Value: ");
                Serial.println(value.c_str());
            }

            // if(pChr->canWrite()) {
            //     if(pChr->writeValue("Tasty")) {
            //         Serial.print("Wrote new value to: ");
            //         Serial.println(pChr->getUUID().toString().c_str());
            //     }
            //     else {
            //         /** Disconnect if write failed */
            //         pClient->disconnect();
            //         return false;
            //     }

            //     if(pChr->canRead()) {
            //         Serial.print("The value of: ");
            //         Serial.print(pChr->getUUID().toString().c_str());
            //         Serial.print(" is now: ");
            //         Serial.println(pChr->readValue().c_str());
            //     }
            // }

            /** registerForNotify() has been deprecated and replaced with subscribe() / unsubscribe().
             *  Subscribe parameter defaults are: notifications=true, notifyCallback=nullptr, response=false.
             *  Unsubscribe parameter defaults are: response=false.
             */
            if(pChr->canNotify()) {
                //if(!pChr->registerForNotify(notifyCB)) {
                if(!pChr->subscribe(true, notifyCB)) {
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            }
            else if(pChr->canIndicate()) {
                /** Send false as first argument to subscribe to indications instead of notifications */
                //if(!pChr->registerForNotify(notifyCB, false)) {
                if(!pChr->subscribe(false, notifyCB)) {
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            }
        }

    } else {
        Serial.println("DEAD service not found.");
    }

    // pSvc = pClient->getService("BAAD");
    // if(pSvc) {     /** make sure it's not null */
    //     pChr = pSvc->getCharacteristic("F00D");

    //     if(pChr) {     /** make sure it's not null */
    //         if(pChr->canRead()) {
    //             Serial.print(pChr->getUUID().toString().c_str());
    //             Serial.print(" Value: ");
    //             Serial.println(pChr->readValue().c_str());
    //         }

    //         pDsc = pChr->getDescriptor(NimBLEUUID("C01D"));
    //         if(pDsc) {   /** make sure it's not null */
    //             Serial.print("Descriptor: ");
    //             Serial.print(pDsc->getUUID().toString().c_str());
    //             Serial.print(" Value: ");
    //             Serial.println(pDsc->readValue().c_str());
    //         }

    //         if(pChr->canWrite()) {
    //             if(pChr->writeValue("No tip!")) {
    //                 Serial.print("Wrote new value to: ");
    //                 Serial.println(pChr->getUUID().toString().c_str());
    //             }
    //             else {
    //                 /** Disconnect if write failed */
    //                 pClient->disconnect();
    //                 return false;
    //             }

    //             if(pChr->canRead()) {
    //                 Serial.print("The value of: ");
    //                 Serial.print(pChr->getUUID().toString().c_str());
    //                 Serial.print(" is now: ");
    //                 Serial.println(pChr->readValue().c_str());
    //             }
    //         }

    //         /** registerForNotify() has been deprecated and replaced with subscribe() / unsubscribe().
    //          *  Subscribe parameter defaults are: notifications=true, notifyCallback=nullptr, response=false.
    //          *  Unsubscribe parameter defaults are: response=false.
    //          */
    //         if(pChr->canNotify()) {
    //             //if(!pChr->registerForNotify(notifyCB)) {
    //             if(!pChr->subscribe(true, notifyCB)) {
    //                 /** Disconnect if subscribe failed */
    //                 pClient->disconnect();
    //                 return false;
    //             }
    //         }
    //         else if(pChr->canIndicate()) {
    //             /** Send false as first argument to subscribe to indications instead of notifications */
    //             //if(!pChr->registerForNotify(notifyCB, false)) {
    //             if(!pChr->subscribe(false, notifyCB)) {
    //                 /** Disconnect if subscribe failed */
    //                 pClient->disconnect();
    //                 return false;
    //             }
    //         }
    //     }

    // } else {
    //     Serial.println("BAAD service not found.");
    // }

    Serial.println("Done with this device!");
    timeScheduler.addTask(taskConnectBleServer);
    taskConnectBleServer.forceNextIteration();
    Serial.println("add Task Connect Ble server");

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

    /** create a callback that gets called when advertisers are found */
    pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());

    /** Set scan interval (how often) and window (how long) in milliseconds */
    pScan->setInterval(45);
    pScan->setWindow(15);

    /** Active scan will gather scan response data from advertisers
     *  but will use more energy from both devices
     */
    pScan->setActiveScan(true);
    /** Start scanning for advertisers for the scan time specified (in seconds) 0 = forever
     *  Optional callback for when scanning stops.
     */
    pScan->start(scanTime, scanEndedCB);
    
}

void onValueConsignePhChanged( HANumeric number, HANumber* sender){
  if (!number.isSet()) {
        // the reset command was send by Home Assistant
    } else {
        float numberFloat = number.toFloat();
        telnet.println("Value of ORP Consigne changed: ");
        telnet.print(numberFloat);
        telnet.println("");

        uint8_t statehex = numberFloat * 10;
        telnet.print(statehex);
        telnet.println("");
        //static uint8_t * trame = {0x00, 0x06, 0x03, 0x30, 0x01, statehex, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        //telnet.print(trame);
        telnet.println("");
        bool writeResponse;
        
        //writeResponse = pRemoteCharacteristic->writeValue(trame, size, true);
         //Changer la conf sur le justsalt->bleWrite
        // commandePh(numberFloat);

    }
    sender->setState(number); // report the selected option back to the HA panel
  
}


void setup (){
    Serial.begin(115200);
    timeScheduler.init();

    timeScheduler.addTask(taskSetup);
    taskSetup.enable();
    Serial.println("add Task to setup justsalt");
}


void loop (){
    timeScheduler.execute();
}

void cb_connectBleServer(){
    /** Loop here until we find a device we want to connect to */
    while(!doConnect){
        delay(1);
    }
    mqtt.loop();
    doConnect = false;

    /** Found a device we want to connect to, do it now */
    if(connectToServer()) {
        Serial.println("Success! we should now be getting notifications, scanning for more!");
    } else {
        Serial.println("Failed to connect, starting scan");
    }

    NimBLEDevice::getScan()->start(scanTime,scanEndedCB);
}