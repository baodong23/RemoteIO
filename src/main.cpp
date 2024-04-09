#include <Arduino.h>

//#include "EthernetENC.h"
#include "SPI.h"
#include "ModbusServerEthernet.h"
#include "WiFi.h"
#include "WiFiAP.h"
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"
#include "EEPROM.h"
#include "CRC16.h"
// put function declarations here:

const char *ssid = "amt";
const char *password = "amtserver";


 
// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

 CRC16 crc(CRC16_MODBUS_POLYNOME,
           CRC16_MODBUS_INITIAL,
           CRC16_MODBUS_XOR_OUT,
           CRC16_MODBUS_REV_IN,
           CRC16_MODBUS_REV_OUT);

hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
// Set the static IP address to use if the DHCP fails to assign
#define EEPROMsize 4
#define IN1 12
#define IN2 13
#define IN3 14
#define IN4 15
#define IN5 16
#define IN6 17
#define IN7 35
#define IN8 34
#define EN 33
#define SET_WIFI 4
// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
EthernetServer server(80);
ModbusServerEthernet mb;
AsyncWebServer sv(80);
// Variables to measure the speed

//IPAddress ip(192,168,8,180);
uint8_t test_lora_zigbee[55] = {00,0x10,00,0x32,00,0x18,0x30,00,0x01,00,0x01,00,0x01,00,00,00,00,00,0x01,00,00,00,
00,00,00,00,0,00,0,00,01,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00}, *test;
uint8_t ipset[] = {192, 168 ,9 ,180};
uint8_t buffer[8], *bf ;
uint16_t datasend[1], *dts;
uint16_t count = 0, interrupts_counter = 0;
bool eth_status = 1, wifi_status = 0, wifion = 0, onetime = 0;
bool temp[6] = { 0, 0, 0, 0, 0, 0};
bool  pre_sta[6] = { 0 , 0, 0, 0, 0, 0}; 
const char* PARAM_INPUT_1 = "ETH_IP";
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
    .input{ width: 200px; height: 50px; display: inline_block;}
    .button{background-color: #4CAF50; width: 80px; height: 50px; border:none; border-radius: 5px; color: white; display: inline_block;}
  </style>
  </head><body>
  <h2>AMT REMOTE IO CONFIG</h2>
  <form action="/get">
    ETH_IP<input class="input" type="text" name="ETH_IP" placeholder="IP">
    <input class="button" type="submit" value="Submit">
  </form><br>
</body></html>)rawliteral";

ModbusMessage FC03(ModbusMessage request)
{
  Serial.println(request);
  ModbusMessage response; // The Modbus message we are going to give back
  uint16_t addr = 0;      // Start address
  uint16_t words = 0;     // # of words requested
  request.get(2, addr);   // read address from request
  request.get(4, words);  // read # of words from request

  // Address overflow?
  if ((addr + words) > 20)
  {
    // Yes - send respective error response
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  // Set up response
  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));//request.getServerID()
  // Request for FC 0x03?
  if (request.getFunctionCode() == READ_HOLD_REGISTER)
  {
      response.add((uint16_t)(datasend[0]));
  }
  
  return response;
}

void wifiserver(){
   sv.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  sv.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;
    char c;
    int getip=0, count=0;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1;
    }
    else {
      inputMessage = "No message sent";
      inputParam = "none";
    }
    Serial.println(inputMessage);
    if (inputMessage != "No message sent"){
      for (int i = 0; i < inputMessage.length(); i++){
        c = inputMessage[i];
        if (c != '.') getip = getip*10 + (int(inputMessage[i])-0x30);
        else {
          EEPROM.write(count, getip);
          EEPROM.commit();
          getip=0;
          count+=1;
        }
      }
      if (count == 3){
        EEPROM.write(3, getip);
        EEPROM.commit();
        getip = 0;
        count = 0;
      }
    }
    request->send_P(200, "text/html", index_html);
  });
}

void ip_setup(){
  for (int i = 0; i <= 4; i++){
    if (EEPROM.read(i) == 255){
      EEPROM.write(i, ipset[i]);
      EEPROM.commit();
    }
  }
}

void wifista(){
  if (digitalRead(SET_WIFI)){
    if (onetime != 1) {wifi_status = 1; wifion = 1; onetime=1;}
    else {wifion = 0; eth_status = 1; onetime=0;}
  }
  
}
void mode(){
  if (wifi_status == 1){
    server.end();
    mb.stop();
    delay(1000);
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    sv.begin();
    wifi_status = 0;
  }
  else if (eth_status == 1){
    WiFi.softAPdisconnect (true);
    sv.end();
     
    Serial.println("Begin Ethernet");

    Ethernet.init(5);   // MKR ETH Shield
  
    IPAddress ip(EEPROM.read(0), EEPROM.read(1), EEPROM.read(2), EEPROM.read(3));
    Ethernet.begin(mac, ip);
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
  
    Serial.print("Local IP : ");
    Serial.println(Ethernet.localIP());
    server.begin();

    Serial.println("Ethernet Successfully Initialized");

    mb.registerWorker(255, READ_HOLD_REGISTER, &FC03);      
    mb.registerWorker(255, READ_INPUT_REGISTER, &FC03); 

    mb.start(80, 2, 3000);
    eth_status = 0;
  }
}
void handle_input(int input, uint8_t offset){
  switch (input)
  {
    case 0:{
      temp[offset] = 1;
      pre_sta[offset] = 0;
      break;
    }
    case 1:{
      if(pre_sta[offset] == 0){
        temp[offset] = 1;
      }
      else temp[offset] = 0;
      pre_sta[offset] = 1;
      break;
    }
    default:
      break;
  } 
}
void IRAM_ATTR onTimer() {   
  portENTER_CRITICAL_ISR(&timerMux); 
  switch (digitalRead(IN1))
  {
    case 0:{
      temp[0] = 1;
      pre_sta[0] = 0;
      break;
    }
    case 1:{
      if(pre_sta[0] == 0){
        temp[0] = 1;
      }
      else temp[0] = 0;
      pre_sta[0] = 1;
      break;
    }
    default:
      break;
  }
  switch (digitalRead(IN2))
  {
    case 0:{
      temp[1] = 1;
      pre_sta[1] = 0;
      break;
    }
    case 1:{
      if(pre_sta[1] == 0){
        temp[1] = 1;
      }
      else temp[1] = 0;
      pre_sta[1] = 1;
      break;
    }
    default:
      break;
  }
  switch (digitalRead(IN3))
  {
    case 0:{
      temp[2] = 1;
      pre_sta[2] = 0;
      break;
    }
    case 1:{
      if(pre_sta[2] == 0){
        temp[2] = 1;
      }
      else temp[2] = 0;
      pre_sta[2] = 1;
      break;
    }
    default:
      break;
  }
  switch (digitalRead(IN6))
  {
    case 0:{
      temp[3] = 1;
      pre_sta[3] = 0;
      break;
    }
    case 1:{
      if(pre_sta[3] == 0){
        temp[3] = 1;
      }
      else temp[3] = 0;
      pre_sta[3] = 1;
      break;
    }
    default:
      break;
  } 
  switch (digitalRead(IN7))
  {
    case 0:{
      temp[4] = 1;
      pre_sta[4] = 0;
      break;
    }
    case 1:{
      if(pre_sta[4] == 0){
        temp[4] = 1;
      }
      else temp[4] = 0;
      pre_sta[4] = 1;
      break;
    }
    default:
      break;
  }
  switch (digitalRead(IN8))
  {
    case 0:{
      temp[5] = 1;
      pre_sta[5] = 0;
      break;
    }
    case 1:{
      if(pre_sta[5] == 0){
        temp[5] = 1;
      }
      else temp[5] = 0;
      pre_sta[5] = 1;
      break;
    }
    default:
      break;
  }
  portEXIT_CRITICAL_ISR(&timerMux); 
}
void setup() {
    Serial.begin(38400);
    EEPROM.begin(EEPROMsize);
    ip_setup();
    delay(1000);
    pinMode(IN1, INPUT_PULLUP);
    pinMode(IN2, INPUT_PULLUP);
    pinMode(IN3, INPUT_PULLUP);
    pinMode(IN4, INPUT_PULLUP);
    pinMode(IN5, INPUT_PULLUP);
    pinMode(IN6, INPUT_PULLUP);
    pinMode(IN7, INPUT_PULLUP);
    pinMode(IN8, INPUT_PULLUP);
    pinMode(SET_WIFI, OUTPUT);
    pinMode(EN, OUTPUT);
    wifiserver();
    bf = buffer;
    dts = datasend;
    test = test_lora_zigbee;

  timer = timerBegin(0, 80, true);
  
  timerAttachInterrupt(timer, &onTimer, true);
  
  timerAlarmWrite(timer, 20000, true);
  
  timerAlarmEnable(timer);
}
 
void loop() {
  wifista();
  mode(); 
  digitalWrite(EN,1);
    if (temp[0]) {
      *bf = 1; 
      *(test+8)=1;
    }
    else {
      *bf = 0; 
      *(test+8)=0; 
    }
    if (temp[1]) {
      *(bf+1) = 1; 
      *(test+10)=1;}
    else {
      *(bf+1) = 0; 
      *(test+10)=0;
    }
    if (temp[2]) {
      *(bf+2) = 1; 
      *(test+12)=1;
    }
    else {
      *(bf+2) = 0; 
      *(test+12)=0;
    }
    if (!digitalRead(IN4)) {*(bf+3) = 1;}
    else {*(bf+3) = 0; }
    if (!digitalRead(IN5)) {*(bf+4) = 1;}
    else {*(bf+4) = 0;}
    if (temp[3]) {
      *(bf+5) = 1; 
      *(test+20)=1;
    }
    else {
      *(bf+5) = 0; 
      *(test+20)=0;
    }
    if (temp[4]) {
      *(bf+6) = 1; 
      *(test+22)=1;
    }
    else {
      *(bf+6) = 0; 
      *(test+22)=0;
    }
    if (temp[5]) {
      *(bf+7) = 1; 
      *(test+24)=1;
    }
    else {
      *(bf+7) = 0; 
      *(test+24)=0;
    }

  for (int i=0; i < 55 ; i++){
     Serial.write(test_lora_zigbee[i]);
     crc.add(test_lora_zigbee[i]);
  }

  Serial.write(crc.calc());
  Serial.write((crc.calc())>>8);

  *dts=(buffer[7]<<7)|(buffer[6]<<6)|(buffer[5]<<5)|(buffer[4]<<4)|(buffer[3]<<3)|(buffer[2]<<2)|(buffer[1]<<1)|buffer[0];
  crc.restart();

  delay(50);
}