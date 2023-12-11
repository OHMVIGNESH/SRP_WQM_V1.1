float ntu_value;
float ntu_con;
float ntu_out;
int trub_sensor_pin = 32;
float trub_analog_out;
float trub_sensor_voltage;
float tss;
float ts;
float ts_c , tds_c, tss_c , ntu_c;
//delay
int period = 0, configed;
unsigned long time_now = 0;
char eenew_bro[50];
String Cal_s1, Cal_s2, Cal_s3, Cal_s4, Th_s1, Th_s2, Th_s3, Th_s4, Relay1, Relay2, ssid_n, pass_n, dela_y, essid, epaswd, esid, epass = "", epssid, configed_s, pub_top, sub_top, alert_top, res_top, bro_id, port_id;
#include <EEPROM.h>
#define TdsSensorPin 33
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point
String broc,portc;
int Re_1 = 2;
int Re_2 = 4;


int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;       // current temperature for compensation

#include <WiFi.h> // https://github.com/tzapu/WiFiManager
#include <PubSubClient.h> //https://www.arduino.cc/reference/en/libraries/pubsubclient/
#include <WiFi.h>
#include <ArduinoJson.h>
// Time
#include <NTPClient.h>
#include <WiFiUdp.h>
int timeSinceLastRead = 0;
//const char* ssid     = "max";
//const char* password = "MaxPa$$1";
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

String textin;
String topicin, topicout, topicout_con, macadd, res_topic, topical;
byte mac[6];                     // the MAC address of your Wifi shield
char topic_in[50], topic_out[50], topic_out_con[50], cid[50], res_t[50], topic_al[50], newbroker_id[50];;
int pushButton = 12;

const char* mqtt_server = "broker.emqx.io";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50000];
int value = 0;
int Resid;
void callback(char* topic, byte* message, unsigned int length) {
  textin = "";
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");

  for (int i = 0; i < length; i++) {
    //Serial.print((char)message[i]);
    textin += (char)message[i];
  }
  Serial.println();
  textin = textin;
  Serial.println("*****************************************************************************");
  Serial.println(textin);
  StaticJsonDocument <1000> datain;
  deserializeJson(datain, message);
  String Macid = datain["Macid"];
  String Time = datain["Time"];
  int Typeid  = datain["Typeid"];
  int Resid = datain["Resid"];
  float cal_1 = datain["cal_1"];
  float cal_2 = datain["cal_2"];
  float cal_3 = datain["cal_3"];
  float cal_4 = datain["cal_4"];
  float Th_1 = datain["Th_1"];
  float  Th_2 = datain["Th_2"];
  float  Th_3 = datain["Th_3"];
  float  Th_4 = datain["Th_4"];
  int  dura = datain["delay"];
  String  Relay1 = datain["Relay_1"];
  String  Relay2 = datain["Relay_2"];
  String  ssid = datain["ssid"];
  String  pass = datain["pass"];
  String  publish_topic = datain["Livedata_topic"];
  String  subcribe_topic = datain["Config_topic"];
  String alert_topic = datain["Alert_topic"];
  String Res_topic = datain["Res_topic"];
  String Broker_id = datain["Broker_id"];
  String Port_id  = datain["Port_id"];
  Serial.println("*****************************************************************************");
  Serial.println("Macid :" + Macid);
  Serial.println("Time :" + Time);
  Serial.println("Resid :" + String(Resid));
  Serial.println("Typeid :" + String(Typeid ));
  Serial.println("cal_1 :" + String(cal_1));
  Serial.println("cal_2 :" + String(cal_2));
  Serial.println("cal_3 :" + String(cal_3));
  Serial.println("cal_4 :" + String(cal_4));
  Serial.println("Th_1  :" + String(Th_1));
  Serial.println("Th_2  :" + String(Th_2));
  Serial.println("Th_3 :" + String(Th_3));
  Serial.println("Th_4  :" + String(Th_4));
  Serial.println("Relay1  :" + String(Relay1));
  Serial.println("Relay2  :" + String(Relay2));
  Serial.println("ssid  :" + String(ssid));
  Serial.println("pass  :" + String(pass));
  Serial.println("delay  :" + String(dura));
  Serial.println(" publish_topic  :" + String(publish_topic));
  Serial.println("subcribe_topic  :" + String(subcribe_topic));
  Serial.println("alert_topic  :" + String(alert_topic));
  Serial.println("Res_topic  :" + String(Res_topic));
  Serial.println("Broker_id  :" + String(Broker_id));
  Serial.println("Port_id  :" + String(Port_id));

  if (Macid == cid) {
    Serial.println("mac passed");
    formattedDate = timeClient.getFormattedDate();
    StaticJsonDocument<200> doc_res;
    doc_res["Macid"] = cid,
                       doc_res["Time"] = formattedDate;
    doc_res["Resid"] = Resid;
    doc_res["Response"] = 1;
    char out_res[128];
    serializeJson(doc_res, out_res);
    boolean rc = client.publish(res_t, out_res);
    Serial.println(out_res);
    if (rc == true) {
      Serial.println("Msg Successfully sended");
    }
    if (rc == false) {
      Serial.println("Msg failed to be sended");
    }


    if (Typeid == 0)
    {
      Serial.println("...Type id : 0 Recived...");
      Serial.println("Storing Data in EEPROM...");

      Serial.println("Writing EEPROM cal_1...");
      String c1 = String(cal_1);
      for (int i = 0; i < c1.length() ; ++i) {
        EEPROM.write(0 + i, c1[i]);
        Serial.println("wrote");
        Serial.println(c1[i]);
      }
      Serial.println("Writing EEPROM cal_2...");
      String c2 = String(cal_2);
      for (int i = 0; i < c2.length() ; ++i) {
        EEPROM.write(4 + i, c2[i]);
        Serial.println("wrote");
        Serial.println(c2[i]);
      }
      Serial.println("Writing EEPROM cal_3...");
      String c3 = String(cal_3);
      for (int i = 0; i < c3.length() ; ++i) {
        EEPROM.write(9 + i, c3[i]);
        Serial.println("wrote");
        Serial.println(c3[i]);
      }
      Serial.println("Writing EEPROM cal_4...");
      String c4 = String(cal_4);
      for (int i = 0; i < c4.length() ; ++i) {
        EEPROM.write(13 + i, c4[i]);
        Serial.println("wrote");
        Serial.println(c4[i]);
      }
      Serial.println("Writing EEPROM TH_1...");
      String th1 = String(Th_1);
      for (int i = 0; i < th1.length() ; ++i) {
        EEPROM.write(17 + i, th1[i]);
        Serial.println("wrote");
        Serial.println(th1[i]);
      }
      Serial.println("Writing EEPROM TH_2...");
      String th2 = String(Th_2);
      for (int i = 0; i < th2.length() ; ++i) {
        EEPROM.write(21 + i, th2[i]);
        Serial.println("wrote");
        Serial.println(th2[i]);
      }
      Serial.println("Writing EEPROM TH_3...");
      String th3 = String(Th_3);
      for (int i = 0; i < th3.length() ; ++i) {
        EEPROM.write(25 + i, th3[i]);
        Serial.println("wrote");
        Serial.println(th3[i]);
      }
      Serial.println("Writing EEPROM TH_4...");
      String th4 = String(Th_4);
      for (int i = 0; i < th4.length() ; ++i) {
        EEPROM.write(29 + i, th4[i]);
        Serial.println("wrote");
        Serial.println(th4[i]);
      }
      Serial.println("Writing EEPROM Delay...");
      String dela = String(dura);
      for (int i = 0; i < dela.length() ; ++i) {
        EEPROM.write(80 + i, dela[i]);
        Serial.println("wrote");
        Serial.println(dela[i]);

      }

      Serial.println("Writing EEPROM ssid...");
      String ssid_s = String(ssid);
      for (int i = 0; i < ssid_s.length() ; ++i) {
        EEPROM.write(35 + i, ssid_s[i]);
        Serial.println("wrote");
        Serial.println(ssid_s[i]);
      }
      Serial.println("Writing EEPROM pass...");
      String pass_s = String(pass);
      for (int i = 0; i < pass_s.length() ; ++i) {
        EEPROM.write(56 + i, pass_s[i]);
        Serial.println("wrote");
        Serial.println(pass_s[i]);
      }

      Serial.println("Writing EEPROM pub_top...");
      String pub = String(publish_topic);
      for (int i = 0; i < pub.length() ; ++i) {
        EEPROM.write(102 + i, pub[i]);
        Serial.println("wrote");
        Serial.println(pub[i]);
      }

      Serial.println("Writing EEPROM sub_top...");
      String sub = String(subcribe_topic);
      for (int i = 0; i < sub.length() ; ++i) {
        EEPROM.write(124 + i, sub[i]);
        Serial.println("wrote");
        Serial.println(sub[i]);
      }
      Serial.println("Writing EEPROM alert_top...");
      String alert = String(alert_topic);
      for (int i = 0; i < alert.length() ; ++i) {
        EEPROM.write(146 + i,  alert[i]);
        Serial.println("wrote");
        Serial.println(alert[i]);
      }
      Serial.println("Writing EEPROM Res_top...");
      String resp = String(Res_topic);
      for (int i = 0; i < resp.length() ; ++i) {
        EEPROM.write(166 + i,  resp[i]);
        Serial.println("wrote");
        Serial.println(resp[i]);
      }

      Serial.println("Writing EEPROM Broker_id...");
      String bro = String(Broker_id);
      for (int i = 0; i < bro.length() ; ++i) {
        EEPROM.write(188 + i,  bro[i]);
        Serial.println("wrote");
        Serial.println(bro[i]);
      }
      Serial.println("Writing EEPROM Port_id...");
      String portid = String(Port_id);
      for (int i = 0; i < portid.length() ; ++i) {
        EEPROM.write(210 + i,  portid[i]);
        Serial.println("wrote");
        Serial.println(portid[i]);
      }
      Serial.println("Writing EEPROM SET CONFIG...");
      EEPROM.write(100 , 1);
      int setbit = EEPROM.read(100);
      Serial.println("START_SET BIT:  " + String(setbit));
      EEPROM.commit();
      delay(1000);
      ESP.restart();

    }



    if (Typeid == 1)
    {
      Serial.println("...Type id : 1 Recived...");

      Serial.println("*****************************************************************************");
      Serial.println("Storing Data in EEPROM...");

      Serial.println("Writing EEPROM cal_1...");
      String c1 = String(cal_1);
      for (int i = 0; i < c1.length() ; ++i) {
        EEPROM.write(0 + i, c1[i]);
        Serial.println("wrote");
        Serial.println(c1[i]);
      }
      Serial.println("Writing EEPROM cal_2...");
      String c2 = String(cal_2);
      for (int i = 0; i < c2.length() ; ++i) {
        EEPROM.write(4 + i, c2[i]);
        Serial.println("wrote");
        Serial.println(c2[i]);
      }
      Serial.println("Writing EEPROM cal_3...");
      String c3 = String(cal_3);
      for (int i = 0; i < c3.length() ; ++i) {
        EEPROM.write(9 + i, c3[i]);
        Serial.println("wrote");
        Serial.println(c3[i]);
      }
      Serial.println("Writing EEPROM cal_4...");
      String c4 = String(cal_4);
      for (int i = 0; i < c4.length() ; ++i) {
        EEPROM.write(13 + i, c4[i]);
        Serial.println("wrote");
        Serial.println(c4[i]);
      }
      Serial.println("Writing EEPROM TH_1...");
      String th1 = String(Th_1);
      for (int i = 0; i < th1.length() ; ++i) {
        EEPROM.write(17 + i, th1[i]);
        Serial.println("wrote");
        Serial.println(th1[i]);
      }
      Serial.println("Writing EEPROM TH_2...");
      String th2 = String(Th_2);
      for (int i = 0; i < th2.length() ; ++i) {
        EEPROM.write(21 + i, th2[i]);
        Serial.println("wrote");
        Serial.println(th2[i]);
      }
      Serial.println("Writing EEPROM TH_3...");
      String th3 = String(Th_3);
      for (int i = 0; i < th3.length() ; ++i) {
        EEPROM.write(25 + i, th3[i]);
        Serial.println("wrote");
        Serial.println(th3[i]);
      }
      Serial.println("Writing EEPROM TH_4...");
      String th4 = String(Th_4);
      for (int i = 0; i < th4.length() ; ++i) {
        EEPROM.write(29 + i, th4[i]);
        Serial.println("wrote");
        Serial.println(th4[i]);
      }
      Serial.println("Writing EEPROM Delay...");
      String dela = String(dura);
      for (int i = 0; i < dela.length() ; ++i) {
        EEPROM.write(80 + i, dela[i]);
        Serial.println("wrote");
        Serial.println(dela[i]);
      }



      EEPROM.commit();
      delay(1000);
      ESP.restart();

    }
    if (Typeid == 2)
    {
      Serial.println("...Type id : 2 Recived...");
      if (Relay1 == "on")
      {
        digitalWrite(Re_1, HIGH);
        Serial.println("Relay 1 ON");
      }
      if (Relay1 == "off")
      {
        digitalWrite(Re_1, LOW);
        Serial.println("Relay 1 OFF");
      }
      if (Relay2 == "on")
      {
        digitalWrite(Re_2, HIGH);
        Serial.println("Relay 2 ON");
      }
      if (Relay2 == "off")
      {
        digitalWrite(Re_2, LOW);
        Serial.println("Relay 2 OFF");
      }
    }

    if (Typeid == 3)
    {
      Serial.println("...Type id : 3 Recived...");
      Serial.println("Storing Data in EEPROM...");

      Serial.println("Writing EEPROM ssid...");
      String ssid_s = String(ssid);
      for (int i = 0; i < ssid_s.length() ; ++i) {
        EEPROM.write(35 + i, ssid_s[i]);
        Serial.println("wrote");
        Serial.println(ssid_s[i]);
      }
      Serial.println("Writing EEPROM pass...");
      String pass_s = String(pass);
      for (int i = 0; i < pass_s.length() ; ++i) {
        EEPROM.write(56 + i, pass_s[i]);
        Serial.println("wrote");
        Serial.println(pass_s[i]);
      }
      EEPROM.commit();
      delay(1000);
      ESP.restart();
    }
    if (Typeid == 4)
    {
      Serial.println("...Type id : 4 Recived...");
      rebooteeprom();
    }
  }
}

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}
void rebooteeprom()
{
  for (int i = 0; i < 512 ; ++i) {
    EEPROM.write( i, 0);
    Serial.println("wrote");
  }
  EEPROM.commit();
  delay(1000);
  ESP.restart();

}

void eepromread() {
  Serial.println("***************************************READING EEPROM DATAS******************************************************");

  for (int i = 0 ; i < 4; ++i)
  {
    Cal_s1 += char(EEPROM.read(i));
  }
  Serial.print("Cal_s1: ");
  Serial.println(Cal_s1);

  for (int i = 4 ; i < 9; ++i)
  {
    Cal_s2 += char(EEPROM.read(i));
  }
  Serial.print("Cal_s2: ");
  Serial.println(Cal_s2);

  for (int i = 9 ; i < 13; ++i)
  {
    Cal_s3 += char(EEPROM.read(i));
  }
  Serial.print("Cal_s3: ");
  Serial.println(Cal_s3);
  for (int i = 13; i < 17; ++i)
  {
    Cal_s4 += char(EEPROM.read(i));
  }
  Serial.print("Cal_s4: ");
  Serial.println(Cal_s4);



  for (int i = 17; i < 21; ++i)
  {
    Th_s1 += char(EEPROM.read(i));
  }
  Serial.print("Th_s1: ");
  Serial.println(Th_s1);

  for (int i = 21; i < 25; ++i)
  {
    Th_s2 += char(EEPROM.read(i));
  }
  Serial.print("Th_s2: ");
  Serial.println(Th_s2);


  for (int i = 25; i < 29; ++i)
  {
    Th_s3 += char(EEPROM.read(i));
  }
  Serial.print("Th_s3: ");
  Serial.println(Th_s3);

  for (int i = 29; i < 33; ++i)
  {
    Th_s4 += char(EEPROM.read(i));
  }
  Serial.print("Th_s4: ");
  Serial.println(Th_s4);

  for (int i = 80; i < 85; ++i)
  {
    dela_y += char(EEPROM.read(i));
  }
  Serial.print("dela_y: ");
  Serial.println(dela_y);

  for (int i = 35; i < 56; ++i)
  {
    ssid_n += char(EEPROM.read(i));
  }
  Serial.print("SSID: ");
  Serial.println(ssid_n);
  for (int i = 56; i < 77; ++i)
  {
    pass_n += char(EEPROM.read(i));
  }
  Serial.print("PASS: ");
  Serial.println(pass_n);
  for (int i = 102; i < 123; ++i)
  {
    pub_top += char(EEPROM.read(i));
  }
  Serial.print("PUB_TOP: ");
  Serial.println(pub_top);
  for (int i = 124; i < 145; ++i)
  {
    sub_top += char(EEPROM.read(i));
  }
  Serial.print("SUB_TOP: ");
  Serial.println(sub_top);
  for (int i = 146; i < 165; ++i)
  {
    alert_top += char(EEPROM.read(i));
  }
  Serial.print("alert_TOP: ");
  Serial.println(alert_top);


  for (int i = 166; i < 187; ++i)
  {
    res_top += char(EEPROM.read(i));
  }
  Serial.print("alert_TOP: ");
  Serial.println(res_top);


  for (int i = 188; i < 209 ; ++i)
  {
    bro_id += char(EEPROM.read(i));
  }
  Serial.print("Broker_id: ");
  Serial.println(bro_id);

  for (int i = 210; i < 216 ; ++i)
  {
    port_id += char(EEPROM.read(i));
  }
  Serial.print("Port_id: ");
  Serial.println(port_id);


}

void setup() {
  pinMode(TdsSensorPin, INPUT);
  pinMode(Re_1, OUTPUT);
  pinMode(Re_2, OUTPUT);
  pinMode(pushButton, INPUT_PULLUP);
  Serial.begin(115200);
  EEPROM.begin(512);
  Serial.println("Device Started");
  Serial.println("-------------------------------------");
  Serial.println("SRP_WM");
  Serial.println("-------------------------------------");

  for (int i = 35; i < 56; ++i)
  {
    esid += char(EEPROM.read(i));
  }
  Serial.println();
  Serial.print("SSID: ");
  Serial.println(esid);
  String sl = esid.c_str();
  Serial.println(sl.length());
  Serial.print("SSID: ");
  epssid = sl.c_str();
  Serial.println(epssid);
  delay(1000);


  for (int i = 56; i < 77; ++i)
  {
    epass += char(EEPROM.read(i));
  }
  Serial.print("PASS: ");
  Serial.println(epass);


  Serial.print("esid.c_str(): ");
  Serial.println(essid.length());
  essid = epssid.c_str();
  epaswd = epass.c_str();


  if (essid.length() > 0) {
    Serial.println("Reading EEPROM pass.......................");
    WiFi.begin(epssid.c_str(), epass.c_str());

  }

  if (essid.length() < 1) {

    Serial.println("SET HOTSPOT : (SSID : max , PASWORD: MaxPa$$1) to connect the device .............");
    WiFi.begin("max", "MaxPa$$1");

  }


  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    int buttonState = digitalRead(pushButton);
    if (buttonState == 0) {
      rebooteeprom();
    }
    if (millis() > time_now + 30000) {
      time_now = millis();
      ESP.restart();

    }
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  timeClient.begin();
  timeClient.setTimeOffset(19800);// GMT +1 = 3600 // GMT +8 = 28800 // GMT -1 = -3600 // GMT 0 = 0

  //MAC ID

  WiFi.macAddress(mac);
  String id1 = String(mac[0], HEX);
  String id2 = String(mac[1], HEX);
  String id3 = String(mac[2], HEX);
  String id4 = String(mac[3], HEX);
  String id5 = String(mac[4], HEX);
  String id6 = String(mac[5], HEX);
  macadd = id1 + id2 + id3 + id4 + id5 + id6;
  Serial.println(macadd);
  /*============================================================
    topicin = "srp/wm/p1";
    topicout = "srp/wm/p1/sub";
    topicout_con = "srp/wm/p1/sub_con";
    res_topic = "srp/wm/p1/res";
    topical = "srp/wm/p1/al";
    ================================================================*/
  /*
    topicin = pub_top;
    topicout = sub_top;
    res_topic = res_top;
    topical = alert_top;
  */

  /*
    macadd.toCharArray(cid, 50);
    topicin.toCharArray(topic_in, 50);
    topicout.toCharArray(topic_out, 50);
    res_topic.toCharArray(res_t, 50);
    topical.toCharArray(topic_al, 50);*/
  macadd.toCharArray(cid, 50);
  pub_top.toCharArray(topic_in, 50);
  sub_top.toCharArray(topic_out, 50);
  res_top.toCharArray(res_t, 50);
  alert_top.toCharArray(topic_al, 50);


  /******************************************************************/


  int configed = EEPROM.read(100);
  Serial.println ("START_BIT :" + String(configed));

  if (configed == 1)
  {
    Serial.println("...................DONE ALREADY..................");
    //bro_id.toCharArray(newbroker_id,50);
    //int portadd = port.toInt();

    for (int i = 188; i < 209 ; ++i)
    {
      broc += char(EEPROM.read(i));
    }
    

    for (int i = 210; i < 216 ; ++i)
    {
      portc += char(EEPROM.read(i));
    }
    broc.toCharArray(newbroker_id, 50);
    int portid = portc.toInt();
    client.setServer(newbroker_id, portid);
    client.setCallback(callback);
      Serial.println("************************************************************** ");
    Serial.print("Broker_id in: ");
    Serial.println(newbroker_id);
    Serial.print("Port_id in: ");
    Serial.println( portid);
    Serial.println("************************************************************** ");
  }



  if (configed == 0)
  {
    Serial.print("...................SEND WAKE UP CMD..................");
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    while (1) {
      while (!client.connected()) {
        while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.print(".");
        }
        Serial.print("Attempting MQTT connection...");
        if (client.connect("IOTWM")) {
          client.subscribe("new/device_con");
        } else {
          Serial.print("failed, rc=");
          Serial.print(client.state());
          Serial.println(" try again in 5 seconds");
          delay(5);
        }
      }
      client.loop();
      while (!timeClient.update()) {
        timeClient.forceUpdate();
      }
      formattedDate = timeClient.getFormattedDate();
      StaticJsonDocument<250> doc_out;
      doc_out["Macid"] = cid,
      doc_out["Time"] = formattedDate;
      doc_out["Location"] = "SRP";
      doc_out["Category"] = "wm";
      char out_s[128];
      serializeJson(doc_out, out_s);
      //Serial.println(out_s);
      if (millis() > time_now + 10000) {
        time_now = millis();
        boolean rc = client.publish("max/newdevice", out_s);
        Serial.println(out_s);
        if (rc == true) {
          Serial.println("Msg Successfully sended");
        }
        if (rc == false) {
          Serial.println("Msg failed to be sended");
        }
      }

    }

  }
  eepromread();
  macadd.toCharArray(cid, 50);
  pub_top.toCharArray(topic_in, 50);
  sub_top.toCharArray(topic_out, 50);
  res_top.toCharArray(res_t, 50);
  alert_top.toCharArray(topic_al, 50);
}



void reconnect() {
  while (!client.connected()) {
    while (WiFi.status() != WL_CONNECTED) {
      Serial.println("***********************************************");
      Serial.println("***********************************************");
      Serial.println("***********************************************");
      delay(500);
      Serial.print(".");
    }
    Serial.print("Attempting MQTT connection...");
    if (client.connect("IOTWM")) {
      client.subscribe(topic_out);
      Serial.println("***********************************************");
      Serial.println("***********************************************");
      Serial.println("***********************************************");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5);
    }
  }
}



void tds() {
  static unsigned long analogSampleTimepoint = millis();

  if (millis() - analogSampleTimepoint > 40U) { //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer

    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;

      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
      //temperature compensation
      float compensationVoltage = averageVoltage / compensationCoefficient;

      //convert voltage value to tds value
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
    }
  }
}

void senddata() {

  trub();
  tds();
  tss = ntu_out / 3;
  ts = tss + tdsValue;
  Serial.println(" Total Dissolved Solids (TDS) : " + String(tdsValue) + "PPM |" + " Nephelometric Turbidity unit(NTU) : " + String(ntu_out) + " |" + " Total Suspended Solids: " + String(tss) + "mg/l |" + " Total solids:" + String(ts) + " |");
  delay(1000);
  formattedDate = timeClient.getFormattedDate();
  float cl1 =  Cal_s1.toFloat();
  float cl2 =  Cal_s2.toFloat();
  float cl3 =  Cal_s3.toFloat();
  float cl4 =  Cal_s4.toFloat();

  ts_c   = ts * cl1;
  tds_c  = tdsValue * cl3;
  tss_c  = tss * cl2;
  ntu_c  = ntu_out * cl4;
  Serial.println("***************************************************************************");
  Serial.println("cl4 : " + String(cl4));
  Serial.println(" ntu_c  :" + String(ntu_c) );
  Serial.println("cl3 : " + String(cl3));
  Serial.println(" tds_c  :" + String(tds_c) );
  Serial.println("cl2 : " + String(cl2));
  Serial.println(" tss_c  :" + String(tss_c) );
  Serial.println("***************************************************************************");

  StaticJsonDocument<250> doc_out;
  doc_out["Macid"] = cid,
                     doc_out["Time"] = formattedDate;
  doc_out["parameter1"] = ts_c;
  doc_out["parameter2"] = tss_c;
  doc_out["parameter3"] = tds_c;
  doc_out["parameter4"] = ntu_c;
  //doc_out["parameter5"]=0;
  char out_s[128];
  serializeJson(doc_out, out_s);
  Serial.println(out_s);

  int sleepsec = dela_y.toInt();
  int sleepms = sleepsec * 1000;
  period = sleepms;
  if (millis() > time_now + period) {
    time_now = millis();
    Serial.print("Topic_in :");
    Serial.println(topic_in);
    boolean rc = client.publish(topic_in, out_s);
    Serial.println(out_s);
    if (rc == true) {
      Serial.println("Msg Successfully sended");
    }
    if (rc == false) {
      Serial.println("Msg failed to be sended");
    }
  }
}

void trub() {

  trub_analog_out = analogRead(trub_sensor_pin);
  //trub_sensor_voltage = 2.73;        //check ONE
  trub_sensor_voltage = trub_analog_out * (3.3 / 4095.0);
  // ntu_value = -2572.20 * (trub_sensor_voltage*trub_sensor_voltage) + (8700.5*trub_sensor_voltage)  - 4352.9;
  float OldMin = 2.65;
  float OldMax = 0;
  float OldRange = (OldMax - OldMin);
  float NewRange = (3000 - 0);
  ntu_out = (( trub_sensor_voltage -  OldMin) / (OldMax - OldMin) ) * (3000 - 0) + 0;
  if (ntu_out < 0) {
    ntu_out = 0;
  }

}

void loop() {
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }
  formattedDate = timeClient.getFormattedDate();
  int buttonState = digitalRead(pushButton);
  if (buttonState == 0) {
    rebooteeprom();
  }
  if (!client.connected()) {
    reconnect();


  }
  client.loop();

  senddata();

   Serial.println("*****************/////////*********************");
  Serial.println("Cal_s1: " + String(Cal_s1));
  Serial.println("Cal_s2: " + String(Cal_s2));
  Serial.println("Cal_s3: " + String(Cal_s3));
  Serial.println("Cal_s4: " + String(Cal_s4));
  Serial.println("Th_1: " + String(Th_s1));
  Serial.println("Th_2: " + String(Th_s2));
  Serial.println("Th_3: " + String(Th_s3));
  Serial.println("Th_4: " + String(Th_s4));
  Serial.println("delay  :" + String(dela_y));
  Serial.println("Relay1  :" + String(Relay1));
  Serial.println("Relay2  :" + String(Relay2));
  Serial.println("SSID  :" + String(ssid_n));
  Serial.println("PASS  :" + String(pass_n));
  Serial.println("TSS_c :" + String(tss_c));
  Serial.println("TDS_c  :" + String(tds_c));
  Serial.println("TS_c  :" + String(ts_c));
  Serial.println("NTU_c  :" + String(ntu_c));
  Serial.println("broker  :" + String(bro_id));
  Serial.println("Port  :" + String(port_id));
  Serial.println("pub_top  :" + String(pub_top));
  Serial.println("Sub_top  :" + String(sub_top));
  Serial.println("alert_top :" + String(alert_top));
  Serial.println("res_top :" + String(res_top));
  Serial.println("*****************/////////*********************");



  float th1 = Th_s1.toFloat();
  float th2 = Th_s2.toFloat();
  float th3 = Th_s3.toFloat();
  float th4 = Th_s4.toFloat();

  if (ts_c < th1)
  {
    Serial.println("*************TS ALERT*********");
    StaticJsonDocument<200> doc;
    doc["Macid"] = cid;
    doc["Time"] = formattedDate;
    doc["Alert"] = 1;
    doc["Value"] = ts_c;
    doc["Alert_t"] = 0;

    char out[128];
    serializeJson(doc, out);
    boolean rc = client.publish(topic_al, out);
    Serial.println(out);
    if (rc == true) {
      Serial.println("Msg Successfully sended");
    }
    if (rc == false) {
      Serial.println("Msg failed to be sended");
    }

  }

  if (tss_c < th2)
  {
    Serial.println("*************TSS ALERT*********");
    StaticJsonDocument<200> doc;
    doc["Macid"] = cid;
    doc["Time"] = formattedDate;
    doc["Alert"] = 2;
    doc["Value"] = tss_c;
    doc["Alert_t"] = 0;

    char out[128];
    serializeJson(doc, out);
    boolean rc = client.publish(topic_al, out);
    Serial.println(out);
    if (rc == true) {
      Serial.println("Msg Successfully sended");
    }
    if (rc == false) {
      Serial.println("Msg failed to be sended");
    }

  }

  if (tds_c < th3)
  {
    Serial.println("*************TDS ALERT*********");
    StaticJsonDocument<200> doc;
    doc["Macid"] = cid;
    doc["Time"] = formattedDate;
    doc["Alert"] = 3;
    doc["Value"] = tds_c;
    doc["Alert_t"] = 0;

    char out[128];
    serializeJson(doc, out);
    boolean rc = client.publish(topic_al, out);
    Serial.println(out);
    if (rc == true) {
      Serial.println("Msg Successfully sended");
    }
    if (rc == false) {
      Serial.println("Msg failed to be sended");
    }

  }

  if (ntu_c < th4)
  {
    Serial.println("*************NTU ALERT*********");
    StaticJsonDocument<200> doc;
    doc["Macid"] = cid;
    doc["Time"] = formattedDate;
    doc["Alert"] = 4;
    doc["Value"] = ntu_c;
    doc["Alert_t"] = 0;

    char out[128];
    serializeJson(doc, out);
    boolean rc = client.publish(topic_al, out);
    Serial.println(out);
    if (rc == true) {
      Serial.println("Msg Successfully sended");
    }
    if (rc == false) {
      Serial.println("Msg failed to be sended");
    }

  }

}

