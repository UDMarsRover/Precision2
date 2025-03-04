#include <CANSAME5x.h>

CANSAME5x CAN;
float input_value = 0.0;
float control_set = 0.0;

// Control and status modes from the second script
enum control_mode {
  Duty_Cycle_Set = 0x2050080,
  Speed_Set = 0x2050480,
  Smart_Velocity_Set = 0x20504C0,
  Position_Set = 0x2050C80,
  Voltage_Set = 0x2051080,
  Current_Set = 0x20510C0,
  Smart_Motion_Set = 0x2051480
};

enum status_frame_id {
  status_0 = 0x2051800,
  status_1 = 0x2051840,
  status_2 = 0x2051880,
  status_3 = 0x20518C0,
  status_4 = 0x2051900
};

// Heartbeat Frame
const uint32_t HEARTBEAT_ID = 0x2052C80;
const uint8_t HEARTBEAT_DATA[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };

// Control Frame
const uint8_t CONTROL_SIZE = 8;

// Status Frame
const uint8_t STATUS_SIZE = 2;

void pack_data(uint8_t *data, const uint8_t *source, const int size);
void send_control_frame(const uint32_t device_id, const control_mode mode, const float setpoint);
void print_control_frame(uint32_t control_id, uint8_t *control_data, uint8_t control_size);
void ramp_speed(float start, float end);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("CAN Sender and Motor Controller");

  // Setup CAN pins and bus for Adafruit Feather M4 CAN
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false);  // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true);   // turn on booster

  if (!CAN.begin(1000000)) {
    Serial.println("Starting CAN failed!");
    while (1) delay(10);
  }
  Serial.println("CAN bus started");
}

void loop() {
  if (Serial.available()) {
      input_value = Serial.parseFloat(); // Read float from Serial
  }
  // Send heartbeat frame on CAN bus using Adafruit Feather M4 CAN
  Serial.print("Sending heartbeat frame... ");
  CAN.beginExtendedPacket(HEARTBEAT_ID);
  CAN.write(HEARTBEAT_DATA, sizeof(HEARTBEAT_DATA));
  CAN.endPacket();
  // Serial.println("done");

  // Send control frame for motor control
  // send_control_frame(1, Speed_Set, 10);
  send_control_frame(1, Smart_Velocity_Set, input_value);
  Serial.println();
  delay(100);
}

void pack_data(uint8_t *data, const uint8_t *source, const int size) {
  for (int i = 0; i < size; i++) {
    data[i] = source[i];
  }
}

void floatToLittleEndian(float value, uint8_t* buffer) {
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&value);
    buffer[0] = bytes[3];
    buffer[1] = bytes[2];
    buffer[2] = bytes[1];
    buffer[3] = bytes[0];
}

void send_control_frame(const uint32_t device_id, const control_mode mode, const float setpoint) {
  Serial.print("Sending control frame... ");
  
  uint32_t control_id = mode + device_id;

  uint8_t control_data[CONTROL_SIZE] = {CONTROL_SIZE};

  memcpy(control_data, &setpoint, sizeof(setpoint));

  Serial.print("Sending CAN message with ID: 0x");

  CAN.beginExtendedPacket(control_id);
  CAN.write(control_data, CONTROL_SIZE);
  if (CAN.endPacket()) {
        Serial.println("CAN packet sent successfully!");
    } else {
        Serial.println("CAN packet failed to send.");
    }

}

void print_control_frame(uint32_t control_id, uint8_t *control_data, uint8_t control_size) {
  // Print the control ID in hexadecimal
  Serial.print("Control ID: 0x");
  Serial.println(control_id, HEX);

  // Print the control data as bytes
  Serial.print("Control Data: ");
  for (int i = 0; i < control_size; i++) {
    Serial.print("0x");
    Serial.print(control_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void ramp_speed(float start, float end) {
  
}
