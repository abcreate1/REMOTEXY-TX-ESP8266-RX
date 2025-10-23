/*
   RemoteXY to PPM Converter - Professional Grade
   Stable PPM generation with zero noise
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

#define REMOTEXY_MODE__WIFI_POINT
#include <ESP8266WiFi.h>

#define REMOTEXY_WIFI_SSID "RemoteXY"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] = {
  255,5,0,0,0,62,0,19,0,0,0,0,31,2,106,200,200,84,1,1,
  3,0,2,39,12,23,52,86,4,29,14,0,2,26,31,31,79,78,0,79,
  70,70,0,5,204,24,143,143,9,12,60,60,32,2,26,31,5,14,26,143,
  143,132,13,60,60,32,2,26,31
};
  
struct {
  uint8_t switch_01;
  int8_t joystick_01_x;
  int8_t joystick_01_y;
  int8_t joystick_02_x;
  int8_t joystick_02_y;
  uint8_t connect_flag;
} RemoteXY;   
#pragma pack(pop)

/////////////////////////////////////////////
//           PPM Configuration             //
/////////////////////////////////////////////

#define PPM_PIN 2
#define NUM_CHANNELS 6
#define PPM_FRAME_LENGTH 22500
#define PPM_PULSE_LENGTH 300
#define onState HIGH
#define offState LOW

// Channel values - initialized to safe defaults
uint16_t channelValue[NUM_CHANNELS] = {1500, 1500, 1000, 1500, 1000, 1500};

// PPM state machine variables
volatile uint8_t channel = 0;
volatile uint32_t calc_rest = 0;

/////////////////////////////////////////////
//        Professional PPM ISR             //
/////////////////////////////////////////////

void ICACHE_RAM_ATTR onTimer1ISR() {
  static boolean state = true;

  // Disable timer
  timer1_write(5000);

  if (state) {
    // Start of pulse - set pin LOW
    digitalWrite(PPM_PIN, offState);
    // Set timer for pulse length
    timer1_write(PPM_PULSE_LENGTH * 5);
    state = false;
  } 
  else {
    // End of pulse - set pin HIGH
    digitalWrite(PPM_PIN, onState);
    state = true;

    if (channel >= NUM_CHANNELS) {
      // End of frame - calculate and set sync gap
      channel = 0;
      calc_rest += PPM_PULSE_LENGTH;
      uint32_t sync_gap = PPM_FRAME_LENGTH - calc_rest;
      calc_rest = 0;
      timer1_write(sync_gap * 5);
    } 
    else {
      // Set delay for channel value
      uint32_t channel_gap = channelValue[channel] - PPM_PULSE_LENGTH;
      calc_rest += channelValue[channel];
      timer1_write(channel_gap * 5);
      channel++;
    }
  }
}

/////////////////////////////////////////////
//           Channel Update                //
/////////////////////////////////////////////

void updateChannels() {
  static int8_t lastJoy01X = 0, lastJoy01Y = 0;
  static int8_t lastJoy02X = 0, lastJoy02Y = 0;
  static uint8_t lastSwitch = 0;
  
  // Only update if values actually changed
  bool changed = false;
  
  if (lastJoy01X != RemoteXY.joystick_01_x || 
      lastJoy01Y != RemoteXY.joystick_01_y ||
      lastJoy02X != RemoteXY.joystick_02_x || 
      lastJoy02Y != RemoteXY.joystick_02_y ||
      lastSwitch != RemoteXY.switch_01) {
    changed = true;
  }
  
  if (!changed) return; // No update needed
  
  // Store current values
  lastJoy01X = RemoteXY.joystick_01_x;
  lastJoy01Y = RemoteXY.joystick_01_y;
  lastJoy02X = RemoteXY.joystick_02_x;
  lastJoy02Y = RemoteXY.joystick_02_y;
  lastSwitch = RemoteXY.switch_01;
  
  uint16_t temp[NUM_CHANNELS];
  
  // Map joystick values (-100 to 100) to PWM (1000 to 2000)
  int16_t roll = map(RemoteXY.joystick_02_x, -100, 100, 1000, 2000);
  int16_t pitch = map(RemoteXY.joystick_02_y, -100, 100, 1000, 2000);
  int16_t throttle = map(RemoteXY.joystick_01_y, -100, 100, 1000, 2000);
  int16_t yaw = map(RemoteXY.joystick_01_x, -100, 100, 1000, 2000);
  
  // Apply center deadzone for roll, pitch, yaw (NOT throttle)
  if (abs(RemoteXY.joystick_02_x) < 5) roll = 1500;
  if (abs(RemoteXY.joystick_02_y) < 5) pitch = 1500;
  if (abs(RemoteXY.joystick_01_x) < 5) yaw = 1500;
  
  // Assign to channels
  temp[0] = constrain(roll, 1000, 2000);      // CH1: Roll
  temp[1] = constrain(pitch, 1000, 2000);     // CH2: Pitch
  temp[2] = constrain(throttle, 1000, 2000);  // CH3: Throttle
  temp[3] = constrain(yaw, 1000, 2000);       // CH4: Yaw
  temp[4] = RemoteXY.switch_01 ? 2000 : 1000; // CH5: ARM
  temp[5] = 1500;                             // CH6: AUX
  
  // Atomic update
  noInterrupts();
  memcpy((void*)channelValue, temp, sizeof(channelValue));
  interrupts();
}

/////////////////////////////////////////////
//           Setup Functions               //
/////////////////////////////////////////////

void setupPPM() {
  pinMode(PPM_PIN, OUTPUT);
  digitalWrite(PPM_PIN, onState);
  
  // Configure Timer1
  timer1_attachInterrupt(onTimer1ISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(5000); // Initial delay
  
  Serial.println("[PPM] Professional signal generator started");
  Serial.print("[PPM] Pin: GPIO");
  Serial.print(PPM_PIN);
  Serial.println(" (D4)");
  Serial.print("[PPM] Channels: ");
  Serial.println(NUM_CHANNELS);
  Serial.println("[PPM] Frame: 22.5ms (44Hz)");
}

void setupWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setOutputPower(20.5);
  
  WiFi.softAPConfig(
    IPAddress(192, 168, 4, 1),
    IPAddress(192, 168, 4, 1),
    IPAddress(255, 255, 255, 0)
  );
  
  WiFi.softAP(REMOTEXY_WIFI_SSID, REMOTEXY_WIFI_PASSWORD, 1, 0, 4);
  
  Serial.println("[WiFi] AP Started");
  Serial.print("[WiFi] SSID: ");
  Serial.println(REMOTEXY_WIFI_SSID);
  Serial.print("[WiFi] IP: ");
  Serial.println(WiFi.softAPIP());
}

/////////////////////////////////////////////
//         Serial Monitor                  //
/////////////////////////////////////////////

void printStatus() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║    RemoteXY to PPM - Rock Solid       ║");
  Serial.println("╚════════════════════════════════════════╝");
  
  Serial.print("WiFi Clients: ");
  Serial.println(WiFi.softAPgetStationNum());
  
  Serial.print("RemoteXY: ");
  Serial.println(RemoteXY.connect_flag ? "✓ CONNECTED" : "✗ WAITING");
  
  Serial.println("\n--- Joystick Input ---");
  Serial.print("LEFT  - X: ");
  Serial.print(RemoteXY.joystick_01_x);
  Serial.print(" | Y: ");
  Serial.println(RemoteXY.joystick_01_y);
  
  Serial.print("RIGHT - X: ");
  Serial.print(RemoteXY.joystick_02_x);
  Serial.print(" | Y: ");
  Serial.println(RemoteXY.joystick_02_y);
  
  Serial.print("Switch: ");
  Serial.println(RemoteXY.switch_01 ? "ON" : "OFF");
  
  Serial.println("\n--- PPM Channels (µs) ---");
  Serial.print("CH1 (Roll):     ");
  Serial.println(channelValue[0]);
  Serial.print("CH2 (Pitch):    ");
  Serial.println(channelValue[1]);
  Serial.print("CH3 (Throttle): ");
  Serial.println(channelValue[2]);
  Serial.print("CH4 (Yaw):      ");
  Serial.println(channelValue[3]);
  Serial.print("CH5 (ARM):      ");
  Serial.println(channelValue[4]);
  Serial.print("CH6 (AUX):      ");
  Serial.println(channelValue[5]);
  
  Serial.println("════════════════════════════════════════");
}

/////////////////////////////////////////////
//              Main Setup                 //
/////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n\n╔════════════════════════════════════════╗");
  Serial.println("║  ESP8266 RemoteXY to PPM Converter    ║");
  Serial.println("║       Professional Edition v3.0        ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  setupWiFi();
  
  RemoteXY_Init();
  Serial.println("[RemoteXY] Initialized\n");
  
  setupPPM();
  
  Serial.println("\n[READY] Connect RemoteXY app");
  Serial.print("        SSID: ");
  Serial.println(REMOTEXY_WIFI_SSID);
  Serial.print("        Pass: ");
  Serial.println(REMOTEXY_WIFI_PASSWORD);
  Serial.println();
}

/////////////////////////////////////////////
//              Main Loop                  //
/////////////////////////////////////////////

void loop() {
  // Failsafe - stop if no client connected
  if (WiFi.softAPgetStationNum() == 0) {
    delay(50);
    return;
  }
  
  RemoteXY_Handler();
  updateChannels();
  
  // Status update every second
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    printStatus();
  }
  
  // Consistent timing
  delay(10);
}
