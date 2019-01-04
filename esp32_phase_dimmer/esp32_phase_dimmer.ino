#include <WiFi.h>
#include <PubSubClient.h>

// Update these with values suitable for your network.

const char* ssid = "YOUR WIFI SSID";
const char* password = "YOUR WIFI PASSWORD";
const char* mqtt_server = "broker.mqtt-dashboard.com";
const char* topic = "smartfan/456787654";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

unsigned char PWM = 23;  // Output to Opto Triac pin
unsigned char ZEROCROSS = 4;  // Output to Opto Triac pin
unsigned char dimming = 0;  // Dimming level (0-100)
unsigned char i;
boolean enable = false;

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

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String msg = "";
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.println(msg);

  // set dimming variable in function of speed is received
  if (msg == "stop" || msg == "0") {
    enable = false;
  } else {
    dimming = (11 - msg.toInt()) * 5;
    enable = true;
  }

  Serial.print("dimming: ");
  Serial.println(dimming);

  Serial.print("enable: ");
  Serial.println(enable);

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  pinMode(PWM, OUTPUT);// Set AC Load pin as output
  pinMode(ZEROCROSS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ZEROCROSS), zero_crosss_int, RISING);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback); 
}

void zero_crosss_int()  // function to be fired at the zero crossing to dim the light
{
  // Firing angle calculation : 1 full 50Hz wave =1/50=20ms 
  // Every zerocrossing : (50Hz)-> 10ms (1/2 Cycle) For 60Hz (1/2 Cycle) => 8.33ms 
  // 10ms=10000us

  digitalWrite(2, HIGH);   // Turn the LED on (Note that LOW is the voltage level

  if (enable) {
    int dimtime = (100*dimming);    // For 60Hz =>65    
    delayMicroseconds(dimtime);     // Off cycle
    digitalWrite(PWM, HIGH);        // triac firing
    delayMicroseconds(20);          // triac On propogation delay (for 60Hz use 8.33)
    digitalWrite(PWM, LOW);         // triac Off
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  } 
  else {
    client.loop();
  }
}
