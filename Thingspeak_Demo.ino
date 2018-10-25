/** 
 *  TaskScheduler Test
 *
 *  Initially only tasks 1 and 2 are enabled
 *  Task1 runs every 2 seconds 10 times and then stops
 *  Task2 runs every 3 seconds indefinitely
 *  Task1 enables Task3 at its first run
 *  Task3 run every 5 seconds
 *  Task1 disables Task3 on its last iteration and changed Task2 to run every 1/2 seconds
 *  At the end Task2 is the only task running every 1/2 seconds
 */
 
 
#include <TaskScheduler.h>
#include <dht11.h>
#include <WiFiEspClient.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include <dht11.h>
#include "SoftwareSerial.h"
#include<LiquidCrystal.h> //include LCD library to work with LCD display

#define LED A3
#define DEBUG 1           // change value to 1 to enable debuging using serial monitor  
#define ON    1
#define OFF   0
#define CLOSE 1
#define OPEN  0
#define YES   1
#define NO    0


// RX, TX
SoftwareSerial esp8266Module(3, 4);

// Use WiFiEspClient class to create TCP connections
// Initialize the Wifi client library.
WiFiEspClient client;  
const int httpPort = 80;
const char* host = "184.106.153.149";
String GET = "GET /update?key=";    
String APIkey = "YMBJKNG5ZQ3US7K3"; // replace with your channel key
unsigned long lastConnectionTime = 0;         // last time you connected to the server, in milliseconds
unsigned long lastConnectionTime2 = 0;
const unsigned long postingInterval = 10000L; // delay between updates, in milliseconds
const unsigned long postingInterval2 = 1000UL; //Delay between updates, in milliseconds
//LCD connected to pins RS-En, E-Rs, 10-D4, 11-D5, 12-D6, 13-D7
LiquidCrystal lcd(8, 9, 10, 11, 12, 13); 

int status = WL_IDLE_STATUS;
int R = 5;  // REED SWITCH
int P = 6;  // POWER
int X = 7;  // PIR
int T = A2; // TEMPERATURE - A2 has already been defined as Analog Pin 2 in
int H = A5; // HUMIDITY
int L = A4; // LIGHT

float CELSIUS, HUM, LIGHT;
int   PWRsw,REEDsw,PIRsw;
dht11 DHT11;

#define WIFI_AP "MLINK_4"
#define WIFI_PASSWORD "network@micro"
//#define WIFI_AP "embedded"
//#define WIFI_PASSWORD "asdfqaz1@*"


int Toggle;

//Callback methods prototypes
void SensorReadCallback();
void SensorPrintCallback();
void UpdateDataCallback();
void ReadDataCallback();

//Tasks
Task ReadData(1,TASK_FOREVER,&ReadDataCallback);
Task SensorRead(300, TASK_FOREVER, &SensorReadCallback);
Task SensorPrint(3000, TASK_FOREVER, &SensorPrintCallback);
Task UpdateData(1, TASK_FOREVER, &UpdateDataCallback);

Scheduler runner;
int DisSensor;

enum SensorDis {
  DisTemp,
  DisHum,
  DisLDR,
  DisPwr,
  DisReed,
  DisPIR
};
void SensorReadCallback() {
  CELSIUS = TEMPERATURE();
  HUM = HUMIDITY();
  LIGHT = LIG();
  PWRsw = POWER();
  REEDsw = REED();
  PIRsw = PIR();
}

void SensorPrintCallback() {

  switch(DisSensor){
    case DisTemp:
    lcd.setCursor(0,0);
    lcd.print("  CURRENT TEMP  ");
    lcd.setCursor(0,1);
    lcd.print(CELSIUS);
    lcd.setCursor(7,1);
    lcd.print("CELSIUS");
    DisSensor = DisHum;
    break;
    case DisHum:
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("CURRENT HUMIDITY");
    lcd.setCursor(0,1);
    lcd.print(HUM);
    //lcd.setCursor(7,1);
    //lcd.print("CELSIUS");
    DisSensor = DisLDR;
    break;
    case DisLDR:
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("CURRENT LIGHT");
    lcd.setCursor(0,1);
    lcd.print(LIGHT);
    lcd.setCursor(7,1);
    lcd.print("LUMINUS");
    DisSensor = DisPwr;
    break;
    case DisPwr:
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("POWER IS        ");
    lcd.setCursor(0,1);
    if(PWRsw)lcd.print("ON              ");
    else lcd.print("OFF             ");
    DisSensor = DisReed;
    break;
    case DisReed:
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("REED SWITCH IS  ");
    lcd.setCursor(0,1);
    if(REEDsw)lcd.print("OPEN            ");
    else lcd.print("CLOSED            ");
    DisSensor = DisPIR;
    break;
    case DisPIR:
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("PIR DETECTS     ");
    lcd.setCursor(0,1);
    if(PIRsw)lcd.print("YES             ");
    else lcd.print("NO                ");
    DisSensor = DisTemp;
    break;
    default:
    break;
    //Error
  }
}

void UpdateDataCallback() {
  
  // if 10 seconds have passed since your last connection,
  // then connect again and send data
  if (millis() - lastConnectionTime > postingInterval) {
    httpRequest();
  }
}

void ReadDataCallback(void){
   while (client.available()) {
    char c = client.read();
    Serial.print("RECEIVED: ");
    Serial.println(c);
    if(c == '1')
    {
      digitalWrite(LED,HIGH);
      break;
    }else if(c=='2'){
      digitalWrite(LED,LOW);
      break;
    }
    else
    Serial.println("\r\n\tERROR\r\n");
    //Serial.println(c);
  }

  // if 10 seconds have passed since your last connection,
  // then connect again and get data
  if (millis() - lastConnectionTime > postingInterval2) {
    ReadhttpRequest();
  }
}

void ReadhttpRequest(void)
{
  Serial.print("connecting to ");
  Serial.println(host);
  client.stop();
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }
  Serial.print("Requesting URL: ");
  String cmd = "GET /channels/560057/fields/7/last.txt?api_key=J1K0BOM01ULNVSGF";
  Serial.println(cmd);
  client.println(cmd);
  // note the time that the connection was made
  lastConnectionTime2 = millis();
}

void httpRequest(void)
{
  Serial.print("connecting to ");
  Serial.println(host);
  client.stop();
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }
  Serial.print("Requesting URL: ");
  String  cmd = GET + APIkey + "&field1=" + String(CELSIUS) + 
                                "&field2=" + String(HUM) + 
                                "&field3=" + String(LIGHT) + 
                                "&field4=" + String(PIRsw) + 
                                "&field5=" + String(PWRsw) + 
                                "&field6=" + String(REEDsw);
  Serial.println(cmd);
  client.println(cmd);
  // note the time that the connection was made
    lastConnectionTime = millis();
}

void setup () {
  Serial.begin(9600);
  Toggle = 1;
  pinMode(LED, OUTPUT);
  if(DEBUG){
//    Serial.begin(9600);                             // Setting hardware serial baud rate to 9600
    Serial.println("ESP8266 Thingspeak Test");
  }  
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("     ARDUINO    ");
  lcd.setCursor(0,1);
  lcd.print("Thingspeak Demo ");
  // ESP8266 Init
  InitWiFi();
  runner.init();
  runner.addTask(SensorRead);
  runner.addTask(SensorPrint);
  runner.addTask(UpdateData);
  runner.addTask(ReadData);
  SensorRead.enable();
  SensorPrint.enable();
  UpdateData.enable();
  ReadData.enable();
  delay(1000);
  digitalWrite(LED,HIGH);
}


void loop () {
  runner.execute();
}


void InitWiFi()
{
  // initialize serial for ESP module
  esp8266Module.begin(9600);
  // initialize ESP module
  WiFi.init(&esp8266Module);
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_AP);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(WIFI_AP, WIFI_PASSWORD);
    delay(500);
  }
  Serial.println("Connected to AP");
}

/*==========================================================================================*/
/*                     FUNCTION TO GET TEMPERATURE SENSOR OUTPUT                            */
/*==========================================================================================*/

float TEMPERATURE()
{
  int value_temp = analogRead(T); //Read analog value of temperature sensor output from pin A2
  delay(10);
  value_temp = analogRead(T);
  delay(10);
  float millivolts_temp = (value_temp / 1023.0) * 5000; //convert it to milli volts output ([actual temperature output from sensor] * [Input voltage (5V = 5000mV)] / [Resolution of ADC 2^10 = 1024]) 

  return millivolts_temp / 10;
}

/*==========================================================================================*/
/*                     FUNCTION TO GET HUMIDITY SENSOR OUTPUT                               */
/*==========================================================================================*/

float HUMIDITY()
{

  int chk = DHT11.read(H);

  return DHT11.humidity;
}

/*==========================================================================================*/
/*                     FUNCTION TO GET LDR SENSOR OUTPUT                                    */
/*==========================================================================================*/

float LIG()
{
  int value_lig = analogRead(L);
  delay(10);
  value_lig = analogRead(L);
  float volts_lig = (value_lig / 1023.0) * 5;

  return 500/(4*((5-volts_lig)/volts_lig));
}

/*==========================================================================================*/
/*                     FUNCTION TO GET POWER SENSOR OUTPUT                                  */
/*==========================================================================================*/

int POWER()
{
  return digitalRead(P);
}

/*==========================================================================================*/
/*                     FUNCTION TO GET REED SWITCH SENSOR OUTPUT                            */
/*==========================================================================================*/

int REED()
{
  return digitalRead(R);
}

/*==========================================================================================*/
/*                           FUNCTION TO GET PIR SENSOR OUTPUT                              */
/*==========================================================================================*/

int PIR()
{
  return digitalRead(X);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

