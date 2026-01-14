#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Arduino.h>

// --- CONFIGURACIÓ DE TEMPS DE SON ---
#define uS_TO_S_FACTOR 1000000ULL  /* Factor de conversió de microsegons a segons */
#define TIME_TO_SLEEP  900         /* Temps que l'ESP32 estarà adormit (15 minuts = 900 segons) */

// Configuració del sensor BME280
Adafruit_BME280 bme; 

// --- CONFIGURACIÓ PLAQUES SOLARS ---
const int pinPlaques = 34;           
const char* topic_carrega = "esp32/plaques/carrega"; 

// Configuració Wi-Fi
const char* ssid = "icse"; 
const char* password = "euss_2021"; 

// Configuració del broker MQTT (HiveMQ)
const char* mqtt_server = "39b9e2d2086643dd879d7cc8fd26b5bd.s1.eu.hivemq.cloud"; 
const int mqtt_port = 8883;                    
const char* mqtt_user = "euss_3";            
const char* mqtt_password = "Barcelona_1";  

// Topics MQTT
const char* topic_temperature = "esp32/bme280/temperature";
const char* topic_humidity = "esp32/bme280/humidity";

WiFiClientSecure espClient;
PubSubClient client(espClient);

void connectToWiFi() {
  Serial.print("Connectant a Wi-Fi...");
  WiFi.begin(ssid, password);
  unsigned long startAttemptTime = millis();
  
  // Esperar fins a 20 segons per connectar
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
    delay(500);
    Serial.print(".");
  }
  
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("\nConnectat!");
  } else {
    Serial.println("\nError: No s'ha pogut connectar al Wi-Fi.");
  }
}

void connectToMQTT() {
  int retries = 0;
  while (!client.connected() && retries < 5) {
    Serial.print("Connectant al broker MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("Connectat!");
    } else {
      Serial.print("Error, rc=");
      Serial.print(client.state());
      Serial.println(" Intentant de nou en 2 segons...");
      delay(2000);
      retries++;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Pausa de seguretat per al Monitor Sèrie

  // 1. Inicialitzar sensor i pins
  if (!bme.begin(0x76)) {
    Serial.println("Error: No s'ha trobat el BME280!");
  }
  pinMode(pinPlaques, INPUT);

  // 2. Comunicacions
  espClient.setInsecure(); 
  connectToWiFi();
  client.setServer(mqtt_server, mqtt_port);
  connectToMQTT();

  // 3. Llegir i enviar dades
  if (client.connected()) {
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    int valorAnalogic = analogRead(pinPlaques);
    float percentatgeCarrega = (valorAnalogic / 4095.0) * 100.0;

    client.publish(topic_temperature, String(temperature).c_str());
    client.publish(topic_humidity, String(humidity).c_str());
    client.publish(topic_carrega, String(percentatgeCarrega).c_str());
    
    Serial.println("Dades enviades correctament.");
    
    // Donar temps al tancament de la connexió MQTT
    client.loop(); 
    delay(1000);
  }

  // 4. Configurar i iniciar el Deep Sleep
  Serial.println("Entrant en mode Deep Sleep per 15 minuts...");
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop() {
  // Aquest codi mai s'executarà
}

