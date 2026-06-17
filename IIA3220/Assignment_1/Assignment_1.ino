// Imports
#include <Low_pass_filter.h>
#include <PI_controller.h>
#include <Air_heater_sim.h>
#include <TimeLib.h>
#include <WiFiS3.h>
#include "MCP_DAC.h"

#include "secrets.h"
#include "ThingSpeak.h"

// Module definitions
Low_pass_filter LPF;
PI_controller PI_c;
Air_heater_sim AHS;
WiFiClient client;
MCP4911 MCP(11, 13);

// Wifi variables
int status = WL_IDLE_STATUS;

// PI controller variables
float Ts = 0.1;
float Kp = 0.8;
float Ti = 20;
float controller_signal = 0; // Volt
float signal_error = 0;
float wanted_temperature = 24; // Celsius

// Airheater variables
float environmental_temperature = 22; // Celsius
float air_heater_temperature = environmental_temperature;
float run_time = 0; // sec
float seconds_since_run = 0; // sec

// Low pass filter variables
float filter_constant = 0.1;

// Timer variables
float update_time = 0;
float seconds_since_update = 0;
float minutes_since_update = 0;

// ThingSpeak variables
String variable_changed = "TTTTT";
int nr_fields_to_update = 0;
float passable_array_1[] = {air_heater_temperature, wanted_temperature, filter_constant, Ts, Kp, Ti};
int passable_array_2[] = {1, 2, 3, 4, 5, 6};
String fields_to_update = "";

// Serial variables
char incoming_byte = 'a';
static char message[25];
static unsigned int message_pos = 0;
String Message = "";

// HMI variables
bool green_pressed = false;
bool red_pressed = false;

void setup() {
  // Console coms
  Serial.begin(9600);

  // Physical HMI
  pinMode(3, INPUT_PULLUP); // Green Button
  pinMode(4, INPUT_PULLUP); // Red Button
  pinMode(5, OUTPUT); // Green LED
  pinMode(6, OUTPUT); // Yellow LED

  // ThingSpeak
  connectToWifi();
  ThingSpeak.begin(client);

  // Control signal
  MCP.begin(10);
}

void loop() {
  seconds_since_run = second() - run_time;
  minutes_since_update = minute() - update_time;

  if (seconds_since_run > 1 || seconds_since_run < -1) {
    run_time = second();

    auto [controller_signal_temp, signal_error_temp] = PI_c.get_value(air_heater_temperature, controller_signal, signal_error, wanted_temperature, seconds_since_run, Kp, Ti);
    controller_signal = controller_signal_temp; signal_error = signal_error_temp;
    air_heater_temperature = LPF.filter(AHS.run(air_heater_temperature, controller_signal), air_heater_temperature, filter_constant);

    writeMCP(controller_signal);

    Serial.print(minutes_since_update);
    Serial.print("min  ");
    Serial.print(controller_signal);
    Serial.print("V  ");
    Serial.print(wanted_temperature);
    Serial.print("C  ");
    Serial.print(air_heater_temperature);
    Serial.println("C");

    if (minutes_since_update >= 1 || minutes_since_update < 0) {
      update_time = minute();

      if (variable_changed.indexOf("T") > -1) {
        for (int i = 1; i < 6; i++) {
          if (variable_changed[i - 1] == 'T') {
            fields_to_update += String(i);
            variable_changed[i - 1] = 'F';
          }
        }

        passable_array_1[1] = wanted_temperature;
        passable_array_1[2] = filter_constant;
        passable_array_1[3] = Ts;
        passable_array_1[4] = Kp;
        passable_array_1[5] = Ti;
      }

      passable_array_1[0] = air_heater_temperature;
      thingSpeakSet(passable_array_1, passable_array_2, "0" + fields_to_update);
      thingSpeakWrite();

      fields_to_update = "";

      wanted_temperature = thingSpeakRead(2);
      filter_constant = thingSpeakRead(3);
      Ts = thingSpeakRead(4);
      Kp = thingSpeakRead(5);
      Ti = thingSpeakRead(6);
    }
  }
  readSerial();
  checkButtons();
  updateLeds();

  delay(100);
}

void writeMCP(float voltage) {
  uint16_t adc_value = voltage * 1023. / 5.;
  if (adc_value < 0) {
    adc_value = 0;
  }
  else if (adc_value > 1023) {
    adc_value = 1023;
  }
  MCP.write(adc_value, 0);
}

void checkButtons() {
  if (digitalRead(3) == 0 && green_pressed == false) {
    green_pressed = true;
    wanted_temperature += 0.5;
    variable_changed[0] = 'T';
  }
  else if (digitalRead(3) == 1 && green_pressed == true) {
    green_pressed = false;
  }

  if (digitalRead(4) == 0 && red_pressed == false) {
    red_pressed = true;
    wanted_temperature -= 0.5;
    variable_changed[0] = 'T';
  }
  else if (digitalRead(4) == 1 && red_pressed == true) {
    red_pressed = false;
  }
}

void updateLeds() {
  if (abs(wanted_temperature - air_heater_temperature) < 0.1) {
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW);
  }
  else if (controller_signal > 0) {
    digitalWrite(6, HIGH);
    digitalWrite(5, LOW);
  }
  else {
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
  }
}

void readSerial() {
  // Function for reading message from serial port
  while (Serial.available() > 0) {
    // read the incoming byte:
    incoming_byte = Serial.read();
    // Saving usefull bites and handeling finished message
    if (incoming_byte != '\n') {
      message[message_pos] = incoming_byte;
      message_pos++;
    }
    else {
      message[message_pos] = '\0';
      message_pos = 0;
      Message = String(message);
    }
  }
  if (Message.indexOf("wanted_temperature") > -1) {
    if (Message.indexOf('=') > -1) {
      wanted_temperature = Message.substring(Message.indexOf('=') + 1, -1).toFloat();
      variable_changed[0] = 'T';
    }
    else {
      Serial.print("Wanted Temperature: ");
      Serial.print(wanted_temperature);
      Serial.println(" C");
    }
  }
  else if (Message.indexOf("filter_constant") > -1) {
    if (Message.indexOf('=') > -1) {
      filter_constant = Message.substring(Message.indexOf('=') + 1, -1).toFloat();
      variable_changed[1] = 'T';
    }
    else {
      Serial.print("Filter Constant: ");
      Serial.println(filter_constant);
    }
  }
  else if (Message.indexOf("Ts") > -1) {
    if (Message.indexOf('=') > -1) {
      Ts = Message.substring(Message.indexOf('=') + 1, -1).toFloat();
      variable_changed[2] = 'T';
    }
    else {
      Serial.print("Ts: ");
      Serial.println(Ts);
    }
  }
  else if (Message.indexOf("Kp") > -1) {
    if (Message.indexOf('=') > -1) {
      Kp = Message.substring(Message.indexOf('=') + 1, -1).toFloat();
      variable_changed[3] = 'T';
    }
    else {
      Serial.print("Kp: ");
      Serial.println(Kp);
    }
  }
  else if (Message.indexOf("Ti") > -1) {
    if (Message.indexOf('=') > -1) {
      Ti = Message.substring(Message.indexOf('=') + 1, -1).toFloat();
      variable_changed[4] = 'T';
    }
    else {
      Serial.print("Ti: ");
      Serial.println(Ti);
    }
  }

  Message = "";
}

void connectToWifi() {
  // Function used to connect to wifi on an Arduino Uno R4 WiFi

  char ssid[] = WIFI_NAME;
  char pass[] = PASSWORD;

  // Look for wifi
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with wifi failed");
    while (true);
  }

  // Check firmware
  if (WiFi.firmwareVersion() < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("New firmware version exists. Please upgrade.");
  }

  // Connect to wifi network
  while (status != WL_CONNECTED) {
    Serial.print    ("Connecting to WPA SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);

    // Only check connection every 10 seconds
    delay(10000);
  }

  Serial.println("Connection established");
  printNetwork();
}

void printNetwork() {
  // Function used to print info on connected wifi
  Serial.print("Wifi status: ");
  Serial.println(WiFi.status());

  Serial.print("Wifi name: ");
  Serial.println(WiFi.SSID());

  Serial.print("IP Adresse: ");
  Serial.println(WiFi.localIP());
}

void thingSpeakSet(float values[], int fields[], String ftu) {
  for (int i = 0; i < ftu.length(); i++) {
    int index = ftu.substring(i, i + 1).toInt();
    ThingSpeak.setField(fields[index], values[index]);
  }
}

void thingSpeakWrite() {
  // Function used to write to a single ThingSpeak field

  unsigned long channel_id = CHANNEL_ID;
  const char * write_apikey = WRITE_APIKEY;

  int result = ThingSpeak.writeFields(channel_id, write_apikey);

  if (result == 200 ) {
    Serial.println("Channel updated successfully.");
  }
  else if (result == 0 || result == -401) {
    Serial.println("Channel not updated, to short time since last update attempt.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(result));
  }
}

float thingSpeakRead(int field) {
  // Function used to read a single value from a specified field

  char server[] = "api.thingspeak.com";
  unsigned long channel_id = CHANNEL_ID;
  String read_apikey = READ_APIKEY;
  String query = "/channels/" + String(channel_id) + "/fields/" + String(field) + "/last.json?key=" + String(read_apikey);
  if (client.connect(server, 80)) {
    client.println( "GET " + query + " HTTP/1.1" );
    client.println( "Host: " + String(server));
    client.println();
    String data = client.readString();
    //Serial.println(data);
    int startindex = data.indexOf("field" + field) + 7; //Looking for the Value for field x
    //Serial.println(startindex);
    //Serial.println(data.substring(startindex, startindex + 7));
    float read_value = data.substring(startindex, startindex + 7).toFloat();
    //Serial.print("Read Value = ");
    //Serial.println(read_value);
    return read_value;
    }
    else {
      Serial.println ( "Connection Failed" );
    }
  }
