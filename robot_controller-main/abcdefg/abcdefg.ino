#include <WiFiManager.h>
#include <WebSocketsClient.h>


WebSocketsClient webSocket;

int currentSpeed = 100;

const int deadZoneThreshold = 2;



const int joystickYRight = 39;
const int joystickYLeft = 33;
const int joystickXRight = 35;
const int joystickXLeft = 34;

const int resetButtonPin = 5;
const int BtnM_Left = 0;
const int BtnM_Right = 23;


const int speedUp = 26;
const int speedDown = 18;


const int BtnL_Left = 25;
const int BtnL_Bottom = 22;
const int BtnL_Top = 21;
const int BtnL_Right = 17;


const int BtnR_Left = 13;
const int BtnR_Bottom = 19;
const int BtnR_Top = 16;
const int BtnR_Right = 15;





const int ledSpeed[] = { 2, 12, 4 };
const int numLeds = sizeof(ledSpeed) / sizeof(ledSpeed[0]);


unsigned long lastSendTime = 0;
const unsigned long sendInterval = 50;


int prevJoystickValueXRight = -1;
int prevJoystickValueYRight = -1;
int prevJoystickValueXLeft = -1;
int prevJoystickValueYLeft = -1;

int lastButtonspeedUp = HIGH;
int lastButtonspeedDown = HIGH;

int lastButtonClick = HIGH;


WiFiManager wifiManager;

void setup() {
  Serial.begin(115200);


  pinMode(joystickYRight, INPUT);
  pinMode(joystickYLeft, INPUT);
  pinMode(joystickXRight, INPUT);
  pinMode(joystickXLeft, INPUT);

  pinMode(resetButtonPin, INPUT_PULLUP);

  pinMode(speedUp, INPUT_PULLUP);
  pinMode(speedDown, INPUT_PULLUP);

  for (int i = 0; i < numLeds; i++) {
    pinMode(ledSpeed[i], OUTPUT);
  }

  // pinMode(BtnL_Left, INPUT_PULLUP);
  // pinMode(BtnL_Bottom, INPUT_PULLUP);
  // pinMode(BtnL_Top, INPUT_PULLUP);
  // pinMode(BtnR_Right, INPUT_PULLUP);


  // pinMode(BtnR_Left, INPUT_PULLUP);
  // pinMode(BtnR_Bottom, INPUT_PULLUP);
  // pinMode(BtnR_Top, INPUT_PULLUP);
  // pinMode(BtnR_Right, INPUT_PULLUP);
 
 
  // pinMode(BtnM_Left, INPUT_PULLUP);
  // pinMode(BtnM_Right, INPUT_PULLUP);





  if (!wifiManager.autoConnect("JoystickControllerAP")) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    ESP.restart();
  }


  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());


  webSocket.begin("kukukai2000.ddns.net", 88, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {
  webSocket.loop();


  if (digitalRead(resetButtonPin) == LOW) {
    Serial.println("Reset button pressed, resetting WiFi settings...");
    wifiManager.resetSettings();
    delay(1000);
    ESP.restart();
  }


  unsigned long currentTime = millis();

  if (digitalRead(speedUp) == LOW && lastButtonspeedUp == HIGH) {
    currentSpeed = min(currentSpeed + 100, 255);
    ledSpeedShow(currentSpeed == 100 ? 1 : currentSpeed == 200 ? 2
                                                               : 3);
    sendCommand(0x04);  // Send Speed Up command (0x04)
  }
  lastButtonspeedUp = digitalRead(speedUp);  // Store the new state

  if (digitalRead(speedDown) == LOW && lastButtonspeedDown == HIGH) {
    currentSpeed = max(currentSpeed - 100, 100);
    ledSpeedShow(currentSpeed == 100 ? 1 : currentSpeed == 155 ? 2   
                                                               : 3);
    sendCommand(0x05);  // Send Speed Down command (0x05)
  }
  lastButtonspeedDown = digitalRead(speedDown);



  // if (digitalRead(BtnL_Left) == LOW && lastButtonClick == HIGH) {
  //   sendCommand(0x06);
  // }
  // lastButtonClick = digitalRead(BtnL_Left);
  // if (digitalRead(BtnL_Top) == LOW && lastButtonClick == HIGH) {
  //   sendCommand(0x07);
  // }
  // lastButtonClick = digitalRead(BtnL_Top);
  // if (digitalRead(BtnL_Bottom) == LOW && lastButtonClick == HIGH) {
  //   sendCommand(0x08);
  // }
  // lastButtonClick = digitalRead(BtnL_Bottom);
  // if (digitalRead(BtnL_Right) == LOW && lastButtonClick == HIGH) {
  //   sendCommand(0x09);
  // }
  // lastButtonClick = digitalRead(BtnL_Right);


  //   if (digitalRead(BtnR_Left) == LOW && lastButtonClick == HIGH) {
  //   sendCommand(0x10);
  // }
  // lastButtonClick = digitalRead(BtnR_Left);
  // if (digitalRead(BtnR_Top) == LOW && lastButtonClick == HIGH) {
  //   sendCommand(0x11);
  // }
  // lastButtonClick = digitalRead(BtnR_Top);
  // if (digitalRead(BtnR_Bottom) == LOW && lastButtonClick == HIGH) {
  //   sendCommand(0x12);
  // }
  // lastButtonClick = digitalRead(BtnR_Bottom);
  // if (digitalRead(BtnR_Right) == LOW && lastButtonClick == HIGH) {
  //   sendCommand(0x13);
  // }
  // lastButtonClick = digitalRead(BtnR_Right);
  
  
  // if (digitalRead(BtnM_Left) == LOW && lastButtonClick == HIGH) {
  //   sendCommand(0x14);
  // }
  // lastButtonClick = digitalRead(BtnM_Left);
  // if (digitalRead(BtnM_Right) == LOW && lastButtonClick == HIGH) {
  //   sendCommand(0x15);
  // }
  // lastButtonClick = digitalRead(BtnM_Right);


  if (currentTime - lastSendTime >= sendInterval) {
    bool hasChanged = false;

    int joystickValueXRight = map(analogRead(joystickXRight), 0, 4095, 0, 11);
    int joystickValueYRight = map(analogRead(joystickYRight), 0, 4095, 0, 11);
    int joystickValueXLeft = map(analogRead(joystickXLeft), 0, 4095, 0, 11);
    int joystickValueYLeft = map(analogRead(joystickYLeft), 0, 4095, 0, 11);

    if (abs(joystickValueXRight - prevJoystickValueXRight) > deadZoneThreshold || abs(joystickValueYRight - prevJoystickValueYRight) > deadZoneThreshold) {
      sendJoystickData(0x01, joystickValueXRight, joystickValueYRight);
      prevJoystickValueXRight = joystickValueXRight;
      prevJoystickValueYRight = joystickValueYRight;
      hasChanged = true;
    }


    if (abs(joystickValueXLeft - prevJoystickValueXLeft) > deadZoneThreshold || abs(joystickValueYLeft - prevJoystickValueYLeft) > deadZoneThreshold) {
      sendJoystickData(0x02, joystickValueXLeft, joystickValueYLeft);
      prevJoystickValueXLeft = joystickValueXLeft;
      prevJoystickValueYLeft = joystickValueYLeft;
      hasChanged = true;
    }


    if (hasChanged) {
      lastSendTime = currentTime;
    }
  }
}

void ledSpeedShow(int level) {
  for (int i = 0; i < numLeds; i++) {
    digitalWrite(ledSpeed[i], LOW);
  }
  for (int i = 0; i < level; i++) {
    digitalWrite(ledSpeed[i], HIGH);
  }
}

void sendJoystickData(uint8_t direction, int joystickX, int joystickY) {
  if (webSocket.isConnected()) {
    uint8_t binaryData[5];
    binaryData[0] = direction;
    binaryData[1] = (joystickX >> 8) & 0xFF;
    binaryData[2] = joystickX & 0xFF;
    binaryData[3] = (joystickY >> 8) & 0xFF;
    binaryData[4] = joystickY & 0xFF;

    webSocket.sendBIN(binaryData, sizeof(binaryData));
    Serial.printf("Sent joystick data: Direction=%d, X=%d, Y=%d\n", direction, joystickX, joystickY);
  } else {
    Serial.println("WebSocket not connected. Cannot send joystick data.");
  }
}

void sendCommand(uint8_t command) {
  if (webSocket.isConnected()) {
    uint8_t binaryData[] = { command };
    webSocket.sendBIN(binaryData, sizeof(binaryData));
    Serial.printf("Sent command: %02X\n", command);
  } else {
    Serial.println("WebSocket not connected. Cannot send command.");
  }
}


void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket Disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("WebSocket Connected");
      webSocket.sendTXT("Hello Server");
      break;
    case WStype_TEXT:
      char payloadStr[length + 1];          // Create a buffer to hold the string
      memcpy(payloadStr, payload, length);  // Copy the payload data to the buffer
      payloadStr[length] = '\0';            // Null-terminate the string

      // Convert string to integer
      int currentSpeed = atoi(payloadStr);  // Use atoi() to convert string to int

      // Display the received value
      Serial.printf("Received Speed: %d\n", currentSpeed);

      // Call your custom function with the interpreted speed
      ledSpeedShow(currentSpeed == 100 ? 1 : currentSpeed == 200 ? 2
                                                                 : 3);
      break;
  }
}
