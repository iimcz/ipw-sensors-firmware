/****************************************************************************************************************************
  NAKI M5 Atom PoE TCP sensor client
 *****************************************************************************************************************************/

#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include "src/naki_sensor.pb.h"

#include <FastLED.h>
#include <Ethernet.h>

#define SCK 22
#define MISO 23
#define MOSI 33
#define CS 19

// NAKI Server
char serverAddress[] = "192.168.23.47";
int serverPort = 5000;

char sensor_id[] = "atom_1_pir_1";
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE // atom_1
};

//char sensor_id[] = "atom_2_pir_1";
//byte mac[] = {
//  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF // atom_2
//};

#define POLL_RATE 1000
#define SENSOR_PIN 32

#define NUM_LEDS 1
#define ATOM_LED_PIN 27
CRGB leds[NUM_LEDS];

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
//IPAddress ip(192, 168, 1, 177);
EthernetClient client;

uint8_t pbMessageBuffer[128];
boolean initialFalsePositiveTriggered = false;
boolean lastState = false;

void setup() {
  pinMode(SENSOR_PIN, INPUT);
  
  FastLED.addLeds<NEOPIXEL, ATOM_LED_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
  leds[0] = CRGB::Yellow;
  FastLED.show();

  Ethernet.init(CS);
  
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("\nStarting TCP Sensor Client");

  //Ethernet.begin(mac, ip);
  Ethernet.begin(mac);

  Serial.println("=========================");
  Serial.println("Currently Used SPI pinout:");
  Serial.print("MOSI:");
  Serial.println(MOSI);
  Serial.print("MISO:");
  Serial.println(MISO);
  Serial.print("SCK:");
  Serial.println(SCK);
  Serial.print("SS:");
  Serial.println(SS);
}

void sendSensorList()
{
  naki3d_common_protocol_SensorList message = naki3d_common_protocol_SensorList_init_zero;
  pb_ostream_t stream = pb_ostream_from_buffer(pbMessageBuffer, sizeof(pbMessageBuffer));
  bool status = pb_encode(&stream, naki3d_common_protocol_SensorList_fields, &message);

  // Blink to show network traffic
  leds[0] = CRGB::Purple;
  FastLED.show();
  delay(50);
  
  client.write(pbMessageBuffer, stream.bytes_written);
  client.flush();
  
  leds[0] = CRGB::Green;
  FastLED.show();
}

void sendMovementSensorMessage(boolean movementDetected) 
{
  Serial.println("Sending Message");
  naki3d_common_protocol_SensorMessage message = naki3d_common_protocol_SensorMessage_init_zero;
  
  if (movementDetected) {
    message.data.pir_movement.event = naki3d_common_protocol_PirMovementEvent_MOVEMENT_STARTED;
  } else {
    message.data.pir_movement.event = naki3d_common_protocol_PirMovementEvent_MOVEMENT_STOPPED;
  }
  
  message.which_data = naki3d_common_protocol_SensorMessage_pir_movement_tag;
  message.timestamp = micros();
  strcpy(message.sensor_id, sensor_id);
  
  pb_ostream_t stream = pb_ostream_from_buffer(pbMessageBuffer, sizeof(pbMessageBuffer));
  bool status = pb_encode_ex(&stream, naki3d_common_protocol_SensorMessage_fields, &message, PB_ENCODE_DELIMITED);

  // Blink to show network traffic
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(50);
  
  client.write(pbMessageBuffer, stream.bytes_written);
  client.flush();
  
  leds[0] = CRGB::Green;
  FastLED.show();
}

void loop() {
  Serial.println("Connecting to server");
  if (client.connect(serverAddress, serverPort)) {
    Serial.println("connected");
    leds[0] = CRGB::Green;
    //fill_solid(leds, NUM_LEDS, CRGB::Green);
    FastLED.show();
    //sendSensorList();
  } else {
    Serial.println("connection failed");
    leds[0] = CRGB::Red;
    //fill_solid(leds, NUM_LEDS, CRGB::Red);
    FastLED.show();
  }
  
  while (client.connected()) 
  { 
    bool movementDetected = analogRead(SENSOR_PIN) > 0;
    Serial.print("movementDetected: ");
    Serial.println(movementDetected);

    if (!initialFalsePositiveTriggered && !movementDetected) {
      initialFalsePositiveTriggered = true;
    }
      
    if (initialFalsePositiveTriggered && movementDetected != lastState) {
      sendMovementSensorMessage(movementDetected);
      lastState = movementDetected;
    }
    
    // check if a message is available to be received
    //int messageSize = wsClient.parseMessage();

//    if (messageSize > 0) 
//    {
//      Serial.println("Received a message:");
//      Serial.println(wsClient.readString());
//    }

    // wait between messages
    delay(POLL_RATE);
  }
  
  client.stop();
  leds[0] = CRGB::Red;
  FastLED.show();
  
  Serial.println("disconnected");
  delay(500);
}
