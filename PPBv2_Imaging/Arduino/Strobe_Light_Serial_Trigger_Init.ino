// Hardware trigger controller for the PPBv2 imaging system.

const int dischargePin = 11;
const int chargePin = 13;
const int cameraPin = 10;

// Stop triggering if the Jetson heartbeat disappears. The ROS node sends an
// 'h' every 500 ms, so 2000 ms tolerates transient scheduling delays while
// still putting the illumination system into a safe state after a disconnect.
const unsigned long heartbeatTimeoutMs = 2000;

bool acquisitionEnabled = false;
unsigned long lastHeartbeatMs = 0;

// ------ strobe light cable setting ------
// GREEN-YELLOW -> GND
// ONE          -> 72V
// TWO          -> pin 11 (discharge)
// THREE        -> pin 13 (charge)

// ------ camera GPIO cable setting ------
// BROWN, BLUE -> GND
// GREEN       -> auxiliary input voltage
// BLACK       -> pin 10 (opto-isolated input)

void stopOutputs() {
  acquisitionEnabled = false;
  digitalWrite(dischargePin, LOW);
  digitalWrite(chargePin, LOW);
  digitalWrite(cameraPin, LOW);
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 's') {
      acquisitionEnabled = true;
      lastHeartbeatMs = millis();
    } else if (command == 'h') {
      if (acquisitionEnabled) {
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
}

void loop() {
  processSerialCommands();

  if (
    acquisitionEnabled &&
    static_cast<unsigned long>(millis() - lastHeartbeatMs) >= heartbeatTimeoutMs
  ) {
    stopOutputs();
  }

  if (acquisitionEnabled) {
    // Preserve the established falling-edge camera/strobe timing.
    digitalWrite(cameraPin, HIGH);
    delayMicroseconds(5);

    digitalWrite(dischargePin, HIGH);
    digitalWrite(cameraPin, LOW);
    delayMicroseconds(500);
    digitalWrite(dischargePin, LOW);

    delayMicroseconds(1000);
    digitalWrite(chargePin, HIGH);
    delay(500);
    digitalWrite(chargePin, LOW);
  }

  delay(5);
}
