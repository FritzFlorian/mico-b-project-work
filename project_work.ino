#include <SoftwareSerial.h>
// OneWire bus for temperature sensor
#include <OneWire.h>
// General library for the NanoESP board
#include <NanoESP.h>
// Special library for MQQT on the NanoESP
#include <NanoESP_MQTT.h>
// LED Strip
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

// Adjust if receiver or transmitter.
// Comment out one of the two to setup boards.
//#define TRANSMITTER
#define RECEIVER

// Adjust WIFI settings
#define SSID        "TODO"
#define PASSWORD    "TODO"

// Adjust MQTT settings
#define MQTT_HOST "192.168.0.11"
#define MQTT_PORT 1883
#ifdef TRANSMITTER
const String mqttId("mqtt_trans"); 
#endif
#ifdef RECEIVER
const String mqttId("mqtt_rec"); 
#endif

// MQTT Topic Configuration
#define BUTTON_OUT_TOPIC    "rfmq/nanoesp/out"
#define BUTTON_IN_TOPIC     "rfmq/nanoesp/in"
#define TEMPERATURE_TOPIC   "rfmq/nanoesp/temp"

// Pin in/out settings
#define LED_WLAN      8 // On when wifi is connected
#define LED_MQTT      7 // On when mqtt is connected
#define LED_BUTTON    6 // Output for button state
#define LED_STRIP     3 // LED Strip out pin

#define ONE_WIRE_BUS  4 // Bus pin for temperature sensor 
#define D_PIN         3 // Digital Input PIN for button Interrupt (only pin 2 and 3 are interrupt enabled)

#define NUM_LED_STRIP 8 


// DS18B20 Temperature chip IO variables
OneWire ds(ONE_WIRE_BUS);
byte temp_sensor_addr[8];
byte temp_sensor_data[9];
byte temp_sensor_present = 0; // Found correct device on bus
unsigned long last_temp_sent = 0;
const unsigned long temp_intervall = 2000;

//Button variables 
volatile bool button_state = true;
bool button_last_state = button_state;
int button_last_timer = 0;

// NanoESP variables
NanoESP nanoesp = NanoESP();
NanoESP_MQTT mqtt = NanoESP_MQTT(nanoesp);

// LED strip settings
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LED_STRIP, LED_STRIP, NEO_GRB + NEO_KHZ800);


void button_interrupt()
{
  int current_timer = millis();

  // Debounce button (only allow inputs every 50ms)
  if((current_timer - button_last_timer) > 50){
    button_last_timer = current_timer;
    button_state = !button_state;
  }
}

void setup_button_interrupt() {
  pinMode(D_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(D_PIN), button_interrupt, FALLING);
}


bool setup_temperature_sensor() {
  Serial.println("Initializing temperature sensor...");
  
   // initialize search
  ds.reset_search();
  // search exactly one device
  if ( !ds.search(temp_sensor_addr)) {
    Serial.println("Sensor not found!");
    ds.reset_search();
    return false;
  }

  // One Device found.
  if ( OneWire::crc8( temp_sensor_addr, 7) != temp_sensor_addr[7]) {
    Serial.println("CSensor CRC not valid!");
    return false;
  }

  // Check known devices - family code, first byte
  if ( temp_sensor_addr[0] == 0x28) {
    Serial.println("Temperature sensor initialized.");

    //Set Resolution (we choose the highest possible)
    const byte resolution = 0b00111111;
    
    ds.reset();
    ds.select(temp_sensor_addr);
    ds.write(0x4e);
    ds.write(0x42);
    ds.write(0x43);
    ds.write(resolution);
    
    temp_sensor_present = 1;

    return true;
  } else {
    return false;
  }
}

void setup_led_strip() {
  strip.begin();
  strip.setPixelColor(0, strip.Color(127, 127, 127));
  strip.show();
}

bool setup_wifi() {
  Serial.println("Connecting to WIFI...");
  nanoesp.configWifi(STATION, SSID, PASSWORD);
  nanoesp.configWifiMode(STATION);

  if (nanoesp.wifiConnected()) {
    digitalWrite(LED_WLAN, HIGH);
    Serial.println("Successfully connected to WIFI");
    return true;
  } else {
    Serial.println(F("Error while connecting to WIFI!"));
    return false;
  }
}

bool setup_mqtt() {
  Serial.println("Connecting to MQTT broker...");
  if (mqtt.connect(0, MQTT_HOST, MQTT_PORT, mqttId)) { 
    digitalWrite(LED_MQTT, HIGH);
    Serial.println("Sucessfully connected to MQTT broker (id=" + mqttId + ")");
    return true;
  } else {
    Serial.println(F("Error while connecting to MQTT Broker!"));
    return false;
  }
}

bool mqtt_subscribe(String topic) {
  if (mqtt.subscribe(0, topic)) {
    Serial.print("Subscribed to: "); Serial.println(topic);
    return true;
  } else {
    return false;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting up...");
  
  #ifdef TRANSMITTER
  setup_temperature_sensor();
  setup_button_interrupt();
  #endif
  #ifdef RECEIVER
  setup_led_strip();
  #endif
  
  nanoesp.init();

  pinMode(LED_BUTTON, OUTPUT);
  pinMode(LED_WLAN, OUTPUT);
  pinMode(LED_MQTT, OUTPUT);
  bool wifi_connected = setup_wifi();
  bool mqtt_connected = setup_mqtt();
  
  if (mqtt_connected) {
    #ifdef RECEIVER
    mqtt_subscribe(BUTTON_OUT_TOPIC);
    mqtt_subscribe(TEMPERATURE_TOPIC);
    #endif
  }
}

bool read_temperature(float* result_temp) {
  temp_sensor_present = ds.reset();
  if (!temp_sensor_present) {
    Serial.println("Device not responding (start conversion).");
    return false;
  }
  
  ds.select(temp_sensor_addr);      // Select device
  ds.write(0x44, 1);    // start conversion, with parasite power on at the end
  delay(200);     // wait for sensor to read in value

  temp_sensor_present = ds.reset();   // the DS18B20 requires a new initialization, see datasheep p.10
  if (!temp_sensor_present) {
    Serial.println("Device not responding (read scratchpad).");
    return false;
  }
  ds.select(temp_sensor_addr);
  ds.write(0xBE);         // Read Scratchpad command
  ds.read_bytes(temp_sensor_data,9);  // Read scratchpad, 9 bytes including crc
  
  if ( OneWire::crc8( temp_sensor_data, 8) != temp_sensor_data[8]) {
    Serial.println("CRC is not valid!");
    return false;
  }

  // convert temperature
  int t_int = temp_sensor_data[1] << 8 | temp_sensor_data[0]; // integer value
  (*result_temp) = t_int * 0.0625;           // scaling, see datasheet p. 5, independed of resolution see p. 6

  return true;
}

bool try_recv_mqtt(String& topic, String& value) {
  int id, len;

  if (nanoesp.recvData(id, len)) { // Check if there is data
    return mqtt.recvMQTT(id, len, topic, value);
  } else {
    return false;
  }
}

bool publish_mqtt(String topic, String value) {
  if (mqtt.publish(0, topic, value)) {
    Serial.println("Send " + value + " to " + topic);
    return true;
  } else {
    Serial.println("Send " + topic + " error.");
    return false;
  }
}

void button_out_event(String value) {
  Serial.println("Button Out Event: " + value); 
  if(value == "ON") {
    digitalWrite(LED_BUTTON, HIGH);
  } else {
    digitalWrite(LED_BUTTON, LOW);
  }
  
}

void temperature_event(float value) {
  int minimum_temp = 21;
  int temp_range = 6;
  
  float over = value - minimum_temp;
  if (over < 0) {
    over = 0.0f;
  }
  if (over > temp_range) {
    over = temp_range;
  }
  float percentage = over / temp_range;
  int num_leds = 1 + percentage * 7;
  int blue = 255 * (1 - percentage);
  int red = 255 * percentage;
  for(int i = 0; i < strip.numPixels(); i++) {
    if (i < num_leds) {
      strip.setPixelColor(i, strip.Color(red, 0, blue));
    } else {
      strip.setPixelColor(i, strip.Color(0, 0, 0));
    }
  }
  strip.show();
  Serial.print("Temperature Event: "); Serial.println(value);
}

void button_pressed_event() {
    Serial.print("Button State: "); Serial.println(button_state);
    if (button_state) {
      publish_mqtt(BUTTON_IN_TOPIC, "ON");
    } else {
      publish_mqtt(BUTTON_IN_TOPIC, "OFF");
    }
}

void loop() {
  #ifdef RECEIVER
  String topic, value;
  if (try_recv_mqtt(topic, value)) {
    if (topic == BUTTON_OUT_TOPIC) {
      button_out_event(value);
    } else if (topic == TEMPERATURE_TOPIC) {
      temperature_event(value.toFloat());
    }
  }
  #endif

  
  #ifdef TRANSMITTER
  if (button_state != button_last_state){
    button_last_state = button_state;
    button_pressed_event();
  } else {
    // Dont send temperature when we are busy sending the button press
    float temp;
    if(millis() - last_temp_sent > temp_intervall) {
      bool success = read_temperature(&temp);
      if (success) {
        publish_mqtt(TEMPERATURE_TOPIC, String(temp));
        last_temp_sent = millis();
      }
    }
  }
  #endif

  mqtt.stayConnected(0);        //keep the connection to broker alive
}
