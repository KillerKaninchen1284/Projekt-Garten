#include <DHT.h>
#include <DHT_U.h>

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
#define LdrPin 35
#define Watsens 27

//Web protokoll bibliotheken

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

//W-lan informationen

//Labrarys for Temperature Sensor

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

//Temperatursensor
#define DHTPIN 4
#define DHTTYPE DHT11

//DHT dht(DHTPIN, DHTTYPE);


int HexToInt(String ss)
{
  char arr[2];
  arr[0] = ss[0];
  arr[1] = ss[1];
  return(int(strtol(arr, 0, 16)));
}

bool sendInfo()   //Wasser Temperatur und Licht muss ausgelesen und gesendet werden
{
  bool sendSucces = true;
  
  String StrWasser = String(Water());
  String StrLicht = String(LDR());
  String StrTemperatur = String(Temperatur());

  char chWasser[StrWasser.length()];
  char chLicht[StrLicht.length()];
  char chTemperatur[StrTemperatur.length()];

  StrWasser.toCharArray(chWasser,StrWasser.length());
  StrLicht.toCharArray(chLicht,StrLicht.length());
  StrTemperatur.toCharArray(chTemperatur,StrTemperatur.length());

  Serial.println("Wasser:");
  Serial.println(StrWasser);
  Serial.println("Licht:");
  Serial.println(StrLicht);
  Serial.println("Temperatur:");
  Serial.println(StrTemperatur);

  bool test1 = false;
  bool test2 = false;
  bool test3 = false;
  
  client.publish("garden/wetness", chWasser);
  client.publish("garden/light", chLicht);
  client.publish("garden/temperature", chTemperatur);

}


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
void LedStrip() {
  //pixels.clear();
  pixels.fill(pixels.Color(0, 0, 0));
  pixels.fill(pixels.Color(0, 0, 0));
  //pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
}
void LedStrip(String Eingang)   //Format HEX showed direct
{
  int red = 0;
  int green = 0;
  int blue = 0;

  red = HexToInt(Eingang.substring(0, 2));
  green = HexToInt(Eingang.substring(2, 4));
  blue = HexToInt(Eingang.substring(4, 6));

  pixels.fill(pixels.Color(green, red, blue));

  
  pixels.show();
  Serial.println(red);
  Serial.println(green);
  Serial.println(blue);
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

  Serial.println("");

  if (String(topic) == "garden/light/color")
  {
    LedStrip(messageTemp);
  }


}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("TestESP")) {
      Serial.println("connected");
      client.subscribe("garden/light/color");


    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void seervo() {
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



int Water() {

  int Eingangwat = analogRead(Watsens);
  //Serial.println(Eingangwat);

  int Prozent = (Eingangwat / 40.95f);

  return Prozent;
}

int LDR() {
  /*
     int mini = 4000;
    int MAxi = 0;
    int Count =0;
    int akt = (analogRead(LdrPin));
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
  Serial.println(analogRead(LdrPin));
  
  double Minimum = 1000;   //Bei Helligkeit

  double Maximum = 4000;   //Bei Dunkelheit

  double Eingang = analogRead(LdrPin);

  int Ausgang = 0;//soll zwischen 255 und 0 liegen

  double Prozent = (Eingang - 1000) / 30;

  Prozent = (Prozent * -1) + 100;

  return Prozent;

}

int Temperatur()  //nicht implementiert
{
  int temp;
  
  //temp = dht.readTemperature();
  
  
  return temp;
}

void Bewaessern(int TimeMilli, int konzentration)//konzentration: 0-50%
{
  int requiredSpeed = (double(2.55)*(konzentration*2));
  
  Mototr( 255,  requiredSpeed,  true,  true);

  delay(TimeMilli);
  Mototr( 0,  0,  true,  true);
}


void Mototr(int speedm1, int speedm2, bool M1dir, bool M2dir)   //True ist vorwaertz, False rueckwaerts
{
  //M1#####################################################
  if (M1dir == 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  //M1#####################################################
  if (M2dir == 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  //MOTORspeed
  sigmaDeltaWrite(1, speedm1);
  sigmaDeltaWrite(0, speedm2);


}
void setup() {
  //Servo
  // myservo.attach(servopin);
  //Sensoren
  pinMode(LdrPin, INPUT);
  pinMode(Tempsen, INPUT);
  pinMode(Watsens, INPUT);

  //DEBUG
  pinMode(34,INPUT_PULLUP);

  
  //Sigma
  sigmaDeltaSetup(0, 312500);
  sigmaDeltaSetup(1, 312500);
  sigmaDeltaAttachPin(enA, 1);
  sigmaDeltaAttachPin(enB, 0);
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

  //Temperatursensor beginne Messung
  //dht.begin();


}



int timeLastSend  = 0;

void loop() {
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();  //für den callback methoden check
  


  //sendinfo loop getimed
  
  if(millis()>timeLastSend + 2500)
  {
    sendInfo();
    timeLastSend = millis();
    Serial.println("ping1");
  }
  
  if(digitalRead(34)==LOW)
  {
    Mototr(255,255,true,true);
  }
  else
  {
    Mototr(0,0,true,true);
  }
  





  //Serial.println(analogRead(35));
  //client.publish("test", "Hallo Welt");
  //LedStrip();
  //seervo();
  //Mototr(255,255,1,0);
  //Serial.println(Water());
}
