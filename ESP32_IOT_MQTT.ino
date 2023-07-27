//OBSERVACOES
// -- DEVE SE UTILIZAR ESP32 O SCHEME HUGE APP (3MB No OTA/ 1MB SPIFFS) 

#define LED 2 
#define PORTAD21 21
#define PORTAD22 22
#define PORTAD15 15
#define PORTAD14 14
#define PORTAD25 25
#define PORTAD26 26
#define PORTAD27 27
#define PORTAD12 12 // ESSA PORTA ESTA DANDO ERRO PORTA D35
// -- WIFI ----
#include <WiFi.h>
const char* ssid     = "CASA 413";
const char* password = "naopergunte";
WiFiClient nodemcuClient;

// --- MQTT ---
#include <PubSubClient.h>
PubSubClient client(nodemcuClient);
const char* mqtt_Broker = "mqtt.eclipseprojects.io";
const char* mqtt_ClientID = "ESP8266_ANTONIO13254FVSDFVDFV56"; 
// TOPICOS DE ACIONAMENTO REMOTO
const char* mqtt_topico_controle1 =    "labantonio/rele01/dispositivo1/iluminacao";
const char* mqtt_topico_monitoracao1 = "labantonio/rele01/dispositivo1/monitoracao";
bool statusMonitor1 = false;
bool dispositivoPresente1 = false;
const char* mqtt_topico_controle2 =    "labantonio/rele02/dispositivo2/iluminacao";
const char* mqtt_topico_monitoracao2 = "labantonio/rele02/dispositivo2/monitoracao";
bool statusMonitor2 = false;
bool dispositivoPresente2 = false;
const char* mqtt_topico_controle3 =    "labantonio/rele03/dispositivo3/iluminacao";
const char* mqtt_topico_monitoracao3 = "labantonio/rele03/dispositivo3/monitoracao";
bool statusMonitor3 = false;
bool dispositivoPresente3 = false;
const char* mqtt_topico_controle4 =    "labantonio/rele04/dispositivo4/iluminacao";
const char* mqtt_topico_monitoracao4 = "labantonio/rele04/dispositivo4/monitoracao";
bool statusMonitor4 = false;
bool dispositivoPresente4 = false;
const char* mqtt_topico_controle5 =    "labantonio/rele05/dispositivo5/iluminacao";
const char* mqtt_topico_monitoracao5 = "labantonio/rele05/dispositivo5/monitoracao";
bool statusMonitor5 = false;
bool dispositivoPresente5 = false;

const char* mqtt_topico_confirmacao = "labantonio/office/confirmacao";
const char* confirmacao;

// -- BLUETOOTH ---
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
int scanTime = 2; //In seconds
int nivelRSSI = -60; //POTENCIA DE ALCANCE
String dispositivosAutorizados = "cd:65:fd:4d:64:4a";
unsigned long ultimoTempoMedido = 0;
const long intervaloPublicacao = 20000; // Ajustar o tempo de desligamento

// --- DISPLAY OLED ---
#include <Wire.h>
#include <Adafruit_SSD1306.h>
//#include <SSD1306Wire.h>
#define OLED_RESET LED_BUILTIN
// Inicializa o objeto para controlar o display OLED I2C
Adafruit_SSD1306 display(128, 64, &Wire, -1);

//--- INICIA SETUP ---
void setup() {
  pinMode(LED, OUTPUT);  
  pinMode(PORTAD14, OUTPUT);
  pinMode(PORTAD27, OUTPUT);
  pinMode(PORTAD25, OUTPUT);
  pinMode(PORTAD26, OUTPUT);
  pinMode(PORTAD12, OUTPUT);
  configuraDisplay();
  Serial.begin(115200);
  conectaWifi();
  client.setServer(mqtt_Broker, 1883);
  client.setCallback(callback);
  Serial.println("Scanning...");
  BLEDevice::init("dsfdsfd");
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Callback para receber as publicações MQTT
  // Transformar o payload (byte array) em uma string
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if(strcmp(topic, mqtt_topico_monitoracao1) == 0){
    Serial.print("Topico que chegou: ");
    Serial.println(topic);
    if(strcmp(String(message).c_str(), String("1").c_str())==0){
      digitalWrite(PORTAD26, LOW);
      Serial.print("PORTAD25 Acionanda!");
      statusMonitor1 = 1;  
      Serial.print("Conteúdo da mensagem1: ");
      Serial.println(message);
      Serial.print("Conteúdo do Status1: ");
      Serial.println(statusMonitor1);
      mostrarMensagemNoDisplay("Porta PORTAD25 ","Acionada",1,1,2,2);
    }else{
       statusMonitor1 = 0;  
    }
  }else if(strcmp(topic, mqtt_topico_monitoracao2)==0){
    if(strcmp(String(message).c_str(), String("1").c_str())==0){
      Serial.print("PORTAD26 Acionanda!");
      statusMonitor2 = 1;  
      Serial.print("Conteúdo da mensagem2: ");
      Serial.println(message);
      Serial.print("Conteúdo do Status2: ");
      Serial.println(statusMonitor2);
      mostrarMensagemNoDisplay("Porta PORTAD26 ","Acionada",1,1,2,2);
    }else{
      statusMonitor2 = 0;
    }
  }else if(strcmp(topic, mqtt_topico_monitoracao3)==0){
    if(strcmp(String(message).c_str(), String("1").c_str())==0){
      Serial.print("PORTAD14 Acionanda!");
      statusMonitor3 = 1;  
      Serial.print("Conteúdo da mensagem3: ");
      Serial.println(message);
      Serial.print("Conteúdo do Status3: ");
      Serial.println(statusMonitor3);
      mostrarMensagemNoDisplay("Porta PORTAD14 ","Acionada",1,1,2,2);
    }else{
      statusMonitor3 = 0;
    }
  }else if(strcmp(topic, mqtt_topico_monitoracao4)==0){
    if(strcmp(String(message).c_str(), String("1").c_str())==0){
      Serial.print("PORTAD25 Acionanda!");
      statusMonitor4 = 1;  
      Serial.print("Conteúdo da mensagem4: ");
      Serial.println(message);
      Serial.print("Conteúdo do Status4: ");
      Serial.println(statusMonitor4);
      mostrarMensagemNoDisplay("Porta PORTAD25 ","Acionada",1,1,2,2);
    }else{
      statusMonitor4 = 0;
    }
  }else if(strcmp(topic, mqtt_topico_monitoracao5)==0){
    if(strcmp(String(message).c_str(), String("1").c_str())==0){
      Serial.print("PORTAD12 Acionanda!");
      statusMonitor5 = 1;  
      Serial.print("Conteúdo da mensagem 5: ");
      Serial.println(message);
      Serial.print("Conteúdo do Status5: ");
      Serial.println(statusMonitor5);
      mostrarMensagemNoDisplay("Porta PORTAD12 ","Acionada",1,1,2,2);
    }else{
      statusMonitor5 = 0;
    }
  }
  else{
    mostrarMensagemNoDisplay("Sem Ações do ","Mqtt",1,1,2,2);
  }
}

// --- CONFIGURA RELE DE DUAS SAIDAS ---
void configReless(){
   // initialize digital pin LED_BUILTIN as an output.
  pinMode(PORTAD14, OUTPUT);
  pinMode(PORTAD27, OUTPUT);
  pinMode(PORTAD25, OUTPUT);
  pinMode(PORTAD26, OUTPUT);
  pinMode(PORTAD12, OUTPUT);
}

// --- CONFIGURACAO DO DISPLAY --- 
void configuraDisplay(){
  //Inicializa o display Oled
  Wire.begin(PORTAD21, PORTAD22);  // SDA -> D21 (GPIO21), SCL -> D22 (GPIO22)
  // Inicializa a comunicação I2C e o display OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextColor(WHITE);
  display.clearDisplay();
}

//--- Funcoes Auxiliares ---
// nessa parte aqui, depois pode se verificar em qual ambiente a pulceira esta e habilita o rele deacordo com o ambiente
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      String dispositivosEncontrados = advertisedDevice.getAddress().toString().c_str();
      if(dispositivosEncontrados == dispositivosAutorizados && advertisedDevice.getRSSI() > nivelRSSI){
          mostrarMensagemNoDisplay("Pulceira"," On!",1,1,0,0);
          Serial.println("Identificador DETECTADO!");
          Serial.print("RSSI: ");
          Serial.println(advertisedDevice.getRSSI());
          dispositivoPresente1 = true;
          dispositivoPresente2 = true;
          dispositivoPresente3 = true;
          dispositivoPresente4 = true;
          dispositivoPresente5 = false;
          digitalWrite(LED, HIGH);  // turn the LED on (HIGH is the voltage level)
          delay(200);                      // wait for a second
          digitalWrite(LED, LOW);   // turn the LED off by making the voltage LOW
          delay(200);  
          digitalWrite(LED, HIGH);  // turn the LED on (HIGH is the voltage level)
          delay(200);                      // wait for a second
          digitalWrite(LED, LOW);   // turn the LED off by making the voltage LOW
          delay(200);  
          Serial.print("Valor da Variavel: ");
          Serial.println(dispositivoPresente1);
          ultimoTempoMedido = millis();
      }else{
        dispositivoPresente1 = false;
        dispositivoPresente2 = false;
        dispositivoPresente3 = false;
        dispositivoPresente4 = false;
        dispositivoPresente5 = false;
      }
    }
};

// ---- Scann Bluetooth LE ----
void scannBleDevice(){
  BLEScan* pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  BLEScanResults foundDevices = pBLEScan->start(scanTime);
}

// ---- CONECTA WIFI ----
void conectaWifi(){
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");
        display.print(".");
        display.display();
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    mostrarMensagemNoDisplay("IP Esp: ",String(WiFi.localIP()).c_str(),1,1,0,0);
}

// --- RECONECTAR O CLIENT MQTT --- 
void reconectarMQTT() {
  while (!client.connected()) {
      client.connect(mqtt_ClientID);
      client.subscribe(mqtt_topico_monitoracao1);
      client.subscribe(mqtt_topico_monitoracao2);
      client.subscribe(mqtt_topico_monitoracao3);
      client.subscribe(mqtt_topico_monitoracao4);
      client.subscribe(mqtt_topico_monitoracao5);
  }
}

// --- Desabilita o scan ---
void desabilitaScan1(){
  if (statusMonitor1 == 0 ){
    Serial.println("Scan ATIVO 1");
    publicaStatusnoTopico1();
    scannBleDevice();
  }
  else {
    Serial.println("Scan DESLIGADO 1!");
    dispositivoPresente1 = true;
    publicaStatusnoTopico1();
    acionaRele1();
  }
}

// --- Desabilita o scan ---
void desabilitaScan2(){
  if (statusMonitor2 == 0 ){
    Serial.println("Scan ATIVO 2");
    publicaStatusnoTopico2();
    scannBleDevice();
  }
  else {
    Serial.println("Scan DESLIGADO 2!");
    dispositivoPresente2 = true;
    publicaStatusnoTopico2();
    acionaRele2();
  }
}

// --- Desabilita o scan ---
void desabilitaScan3(){
  if (statusMonitor3 == 0 ){
    Serial.println("Scan ATIVO 3");
    publicaStatusnoTopico3();
    scannBleDevice();
  }
  else {
    Serial.println("Scan DESLIGADO 3!");
    dispositivoPresente3 = true;
    publicaStatusnoTopico3();
    acionaRele3();
  }
}

// --- Desabilita o scan ---
void desabilitaScan4(){
  if (statusMonitor4 == 0 ){
    Serial.println("Scan ATIVO 4");
    publicaStatusnoTopico4();
    scannBleDevice();
  }
  else {
    Serial.println("Scan DESLIGADO 4!");
    dispositivoPresente4 = true;
    publicaStatusnoTopico4();
    acionaRele4();
  }
}

// --- Desabilita o scan ---
void desabilitaScan5(){
  if (statusMonitor5 == 0 ){
    Serial.println("Scan ATIVO 5");
    publicaStatusnoTopico5();
    scannBleDevice();
  }
  else {
    Serial.println("Scan DESLIGADO 5!");
    dispositivoPresente5 = true;
    publicaStatusnoTopico5();
    acionaRele5();
  }
}

void acionaRele1(){
  if(dispositivoPresente1 == 1){
    digitalWrite(PORTAD14, LOW);
  }else{
    digitalWrite(PORTAD14, HIGH);
  }
}

void acionaRele2(){
  if(dispositivoPresente2 == 1){
    digitalWrite(PORTAD25, LOW);
  }else{
    digitalWrite(PORTAD25, HIGH);
  }
}

void acionaRele3(){
  if(dispositivoPresente3 == 1){
    digitalWrite(PORTAD26, LOW);
  }else{
    digitalWrite(PORTAD26, HIGH);
  }
}

void acionaRele4(){
  if(dispositivoPresente4 == 1){
    digitalWrite(PORTAD27, LOW);
  }else{
    digitalWrite(PORTAD27, HIGH);
  }
}

void acionaRele5(){
  if(dispositivoPresente5 == 1){
    digitalWrite(PORTAD12, HIGH);
  }else{
    digitalWrite(PORTAD12, LOW);
  }
}

// --- Publica no tópico Iluminacao (liga/desliga o dispositivo) ---
void publicaStatusnoTopico1(){
  if(dispositivoPresente1 == true){
    client.publish(mqtt_topico_controle1, String("on").c_str(), true);
    Serial.println("Power ON 1");
    acionaRele1();
  }
  else{
    if(millis() - ultimoTempoMedido > intervaloPublicacao){
      client.publish(mqtt_topico_controle1, String("off").c_str(), true);
      Serial.println("Power OFF 1");
      acionaRele1();
    }
  }
}

// --- Publica no tópico Iluminacao (liga/desliga o dispositivo) ---
void publicaStatusnoTopico2(){
  if(dispositivoPresente2 == true){
    client.publish(mqtt_topico_controle2, String("on").c_str(), true);
    Serial.println("Power ON 2");
    acionaRele2();
  }
  else{
    if(millis() - ultimoTempoMedido > intervaloPublicacao){
      client.publish(mqtt_topico_controle2, String("off").c_str(), true);
      Serial.println("Power OFF 2");
      acionaRele2();
    }
  }
}

// --- Publica no tópico Iluminacao (liga/desliga o dispositivo) ---
void publicaStatusnoTopico3(){
  if(dispositivoPresente3 == true){
    client.publish(mqtt_topico_controle3, String("on").c_str(), true);
    Serial.println("Power ON 3");
    acionaRele3();
  }
  else{
    if(millis() - ultimoTempoMedido > intervaloPublicacao){
      client.publish(mqtt_topico_controle3, String("off").c_str(), true);
      Serial.println("Power OFF 3");
      acionaRele3();
    }
  }
}

// --- Publica no tópico Iluminacao (liga/desliga o dispositivo) ---
void publicaStatusnoTopico4(){
  if(dispositivoPresente4 == true){
    client.publish(mqtt_topico_controle4, String("on").c_str(), true);
    Serial.println("Power ON 4");
    acionaRele4();
    //  confirmacao = "off_rele";
  }
  else{
    if(millis() - ultimoTempoMedido > intervaloPublicacao){
      client.publish(mqtt_topico_controle4, String("off").c_str(), true);
      Serial.println("Power OFF 4");
      acionaRele4();
    }
  }
}

// --- Publica no tópico Iluminacao (liga/desliga o dispositivo) ---
void publicaStatusnoTopico5(){
  if(dispositivoPresente5 == true){
    client.publish(mqtt_topico_controle5, String("on").c_str(), true);
    Serial.println("Power ON 5");
    acionaRele5();
  }
  else{
    if(millis() - ultimoTempoMedido > intervaloPublicacao){
      client.publish(mqtt_topico_controle5, String("off").c_str(), true);
      Serial.println("Power OFF 5");
      acionaRele5();
    }
  }
}

// --- FUNCAO PARA APRESENTAR MENSAGEM NO DISPLAY OLED --- 
void mostrarMensagemNoDisplay(const char* texto1, const char* texto2, int tamanhoFonte1,int tamanhoFonte2,  int posi1, int posi2 ) {
  display.clearDisplay();
  display.setTextSize(tamanhoFonte1);
  display.setCursor(posi1, posi2);
  display.print(texto1);
  display.setTextSize(tamanhoFonte2);
  display.print(texto2);
  display.display();
  delay(1000);
}

void loop() {
  // O código adicional que você deseja executar pode ser adicionado aqui
  if(!client.connected()){
    reconectarMQTT();
  }
  client.loop();
  desabilitaScan1();
  desabilitaScan2();
  desabilitaScan3();
  desabilitaScan4();
  desabilitaScan5();
  delay(100);
}
