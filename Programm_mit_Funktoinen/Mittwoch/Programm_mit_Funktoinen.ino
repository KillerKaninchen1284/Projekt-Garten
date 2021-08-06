//MOTOR

#define enA  14
#define in1  12 
#define in2  13
#define enB  21
#define in3  18
#define in4  19
//Servo
#define servopin 2
//#include <Servo.h>
//LEDSTRIP
#include <Adafruit_NeoPixel.h>
#define LEDPIN 15
#define NUMPIXELS 12
Adafruit_NeoPixel pixels(NUMPIXELS, LEDPIN, NEO_RGB + NEO_KHZ800);
//Sensoren
#define Tempsen 4
#define Ldr 34
#define Watsens 27

//Web protokoll bibliotheken

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

//W-lan informationen

// SSID/Password combination
const char* ssid = "SICK Summer University";
const char* password = "SSU2021!";

// MQTT Broker IP address:
const char* mqtt_server = "192.168.8.11";


//WERTE FÜR MSQTT
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];


void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

void callback(char* topic, byte* message, unsigned int length) {        //Wenn Nachrich vom MQTT Brooker gesendet wurde
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
    
  if(String(topic) == "garden/light/color")
  {
    LedStrip(messageTemp);
    Serial.println("1");
  }


}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      //client.subscribe("esp32/output");
      //client.subscribe("esp32/light");
      //client.subscribe("esp32/helligkeit");
      //client.subscribe("garden/light/color");
      client.subscribe("test");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void seervo(){
int pos = 0;  
//Servo myservo;  

for (pos = 0; pos <= 180; pos += 1) {
    
   // myservo.write(pos);              
    delay(15);                       
  }
for (pos = 180; pos >= 0; pos -= 1) { 
   // myservo.write(pos);              
    delay(15);                       
  }
  }
void LedStrip(){
  //pixels.clear();
  pixels.fill(pixels.Color(0,0,0));
  pixels.fill(pixels.Color(0,0,0));
  //pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
  }
void LedStrip(String Eingang)   //Format rgb(rValue, gValue, bValue)
{
  

  //2mal bis , suchen

  int rValue;
  int gValue;
  int bValue;

  String rTemp;
  String gTemp;
  String bTemp;

  Serial.println("2");
  //rValue
  bool rUnFinished = true;

  rTemp = Eingang.substring(4,Eingang.length()-1);

  int AnzahlStellenR = 1;
  while (rUnFinished)
  {
    if(rTemp[AnzahlStellenR] != ',')
    {
      AnzahlStellenR = AnzahlStellenR + 1;
      Serial.println("3");
    }
    else
    {
      rUnFinished = false;
    }
  }
  rTemp.trim();

  rValue = rTemp.toInt();
  Serial.println(rValue);
}



int Water(){
  
  int Eingangwat = analogRead(Watsens);
  //Serial.println(Eingangwat);

  int Prozent = (Eingangwat / 40.95f);

  return Prozent;
  }
  
int LDR(){
  /*
   * int mini = 4000;
  int MAxi = 0;
  int Count =0;
  int akt = (analogRead(Ldr));
  if (akt >= MAxi){
    MAxi = akt;
    }
  if(akt <= mini){
    mini = akt;
    }
  Count = Count + 1;
  
    Serial.println("MINI"+String(mini));
    Serial.println("MAx"+String(MAxi));
    
    */
    double Minimum = 1000;   //Bei Helligkeit

    double Maximum = 4000;   //Bei Dunkelheit
    
    double Eingang = analogRead(Ldr);

    int Ausgang = 0;//soll zwischen 255 und 0 liegen

    double Prozent = (Eingang - 1000)/30;

    Prozent = (Prozent * -1) + 100;

    return Prozent;
    
  }

void Mototr(int speedm1, int speedm2,bool M1dir,bool M2dir)
{
  //M1#####################################################
  if(M1dir == 0){
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH); 
  }
  else{
    digitalWrite(in1, HIGH); 
    digitalWrite(in2, LOW);
    }
  //M1#####################################################
  if(M2dir == 0){
  digitalWrite(in3, LOW); 
  digitalWrite(in4, HIGH); 
  }
  else{
    digitalWrite(in3, HIGH); 
    digitalWrite(in4, LOW);
    }
  //MOTORspeed
  sigmaDeltaWrite(1,speedm1);
  sigmaDeltaWrite(0,speedm2);
  
  
  }
void setup() {
//Servo
 // myservo.attach(servopin);
//Sensoren
  pinMode(Ldr, INPUT);
  pinMode(Tempsen, INPUT);
  pinMode(Watsens, INPUT);
  
  
//Sigma
  sigmaDeltaSetup(0, 312500);
  sigmaDeltaSetup(1, 312500);
  sigmaDeltaAttachPin(enA,1);
  sigmaDeltaAttachPin(enB,0);
//MOTOR
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
//LED
  pixels.begin();
//Serial
  Serial.begin(9600);

//MQTT
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  
}



void loop(){

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  

  //client.publish("test", "Hallo Welt");
  //LedStrip();
  //seervo();
  Mototr(255,255,1,0);
  //Serial.println(Water());
  }
