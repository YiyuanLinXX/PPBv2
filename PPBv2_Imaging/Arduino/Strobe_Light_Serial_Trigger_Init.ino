// Hardware trigger controller for the PPBv2 imaging system.

const int dischargePin = 11;
const int chargePin = 13;
const int cameraPin = 10;
const char protocolId[] = "PPBV2_TRIGGER";

// Stop triggering if the Jetson heartbeat disappears. The ROS node sends an
// 'h' every 500 ms, so 2000 ms tolerates transient scheduling delays while
// still putting the illumination system into a safe state after a disconnect.
const unsigned long heartbeatTimeoutMs = 2000;
const unsigned long chargeDurationMs = 493;
const unsigned int chargeRemainderUs = 500;

enum OperatingMode {
  STOPPED,
  LIGHT_ONLY,
  ACQUISITION
};

OperatingMode operatingMode = STOPPED;
unsigned long lastHeartbeatMs = 0;

// ------ strobe light cable setting ------
// GREEN-YELLOW -> GND
// ONE          -> 72V
// TWO          -> pin 11 (discharge)
// THREE        -> pin 13 (charge)

// ------ camera GPIO cable setting ------
// BROWN, BLUE -> GND
// GREEN       -> auxiliary input voltage
// BLACK       -> cameraPin (opto-isolated input)

void stopOutputs() {
  operatingMode = STOPPED;
  digitalWrite(dischargePin, LOW);
  digitalWrite(chargePin, LOW);
  digitalWrite(cameraPin, LOW);
}

void startMode(OperatingMode requestedMode) {
  stopOutputs();
  operatingMode = requestedMode;
  lastHeartbeatMs = millis();
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 's') {
      // Full acquisition: strobe light and camera trigger.
      startMode(ACQUISITION);
      Serial.print("ACK:S:");
      Serial.println(protocolId);
    } else if (command == 'l') {
      // Standalone illumination: strobe light without camera trigger.
      startMode(LIGHT_ONLY);
      Serial.print("ACK:L:");
      Serial.println(protocolId);
    } else if (command == 'h') {
      if (operatingMode != STOPPED) {
        lastHeartbeatMs = millis();
      }
    } else if (command == 'e') {
      stopOutputs();
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(dischargePin, OUTPUT);
  pinMode(chargePin, OUTPUT);
  pinMode(cameraPin, OUTPUT);
  stopOutputs();
  delay(50);
  Serial.print("READY:");
  Serial.println(protocolId);
}

void loop() {
  processSerialCommands();

  if (
    operatingMode != STOPPED &&
    static_cast<unsigned long>(millis() - lastHeartbeatMs) >= heartbeatTimeoutMs
  ) {
    stopOutputs();
  }

  if (operatingMode != STOPPED) {
    // Preserve the original, hardware-verified pulse sequence. The charge and
    // final loop delays are shortened so the complete cycle is approximately
    // 500 ms instead of 506.5 ms.
    if (operatingMode == ACQUISITION) {
      digitalWrite(cameraPin, HIGH);
      delayMicroseconds(5);
    }

    digitalWrite(dischargePin, HIGH);
    if (operatingMode == ACQUISITION) {
      digitalWrite(cameraPin, LOW);
    }
    delayMicroseconds(500);
    digitalWrite(dischargePin, LOW);

    delayMicroseconds(1000);
    digitalWrite(chargePin, HIGH);
    delay(chargeDurationMs);
    delayMicroseconds(chargeRemainderUs);
    digitalWrite(chargePin, LOW);
  }

  delay(5);
}
