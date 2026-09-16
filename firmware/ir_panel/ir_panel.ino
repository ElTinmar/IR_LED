#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <SPI.h>

// ==========================================
// CONFIGURATION FLAGS
// ==========================================
#define DEBUG_MODE 1  // 1 = Debugging (Spammy Serial), 0 = Production (Silent/Clean Protocol)

// --- Identity (used for robust host-side discovery) ---
#define DEVICE_TYPE "LED_PANEL_CONTROLLER"
#define FW_VERSION  "1.1.0"

// --- Pin Definitions ---
const int ENCODER_A = 2;
const int ENCODER_B = 3;
const int ENCODER_SW = 4;
const int FAN_PWM_PIN = 5;

// Status LEDs
const int LED_CH1 = 6;
const int LED_CH2 = 7;
const int LED_CH3 = 8;
const int LED_SATURATION = 9;

// SPI TMP126
const int TMP126_CS = 10;

// ==========================================
// SMART MCP PROXY CLASS (Runtime Fallback)
// ==========================================
class SmartMCP {
private:
  Adafruit_MCP4728 realMcp;
  bool fallbackActive = false;

public:
  bool begin(uint8_t address) {
    pinMode(A4, INPUT_PULLUP);
    pinMode(A5, INPUT_PULLUP);
    delay(5);

    bool busIsOk = (digitalRead(A4) == HIGH && digitalRead(A5) == HIGH);

    if (!busIsOk) {
      Serial.println("WARN: HW_I2C_BUS_DEAD! Falling back to MOCK MCP.");
      fallbackActive = true;
      return true;
    }

    Wire.begin();
    if (!realMcp.begin(address)) {
      Serial.println("WARN: HW_MCP_MISSING! Falling back to MOCK MCP.");
      fallbackActive = true;
      return true;
    }

    Serial.println("SYS: Physical MCP4728 initialized successfully.");
    fallbackActive = false;
    return true;
  }

  bool setChannelValue(MCP4728_channel_t channel, uint16_t value,
                       MCP4728_vref_t vref = MCP4728_VREF_VDD,
                       MCP4728_gain_t gain = MCP4728_GAIN_1X,
                       MCP4728_pd_mode_t pd = MCP4728_PD_MODE_NORMAL) {
    if (fallbackActive) {
      #if DEBUG_MODE
        Serial.print("MOCK: Ch ");
        Serial.print(channel);
        Serial.print(" -> ");
        Serial.println(value);
      #endif
      return true;
    } else {
      return realMcp.setChannelValue(channel, value, vref, gain, pd);
    }
  }

  bool isMockActive() const { return fallbackActive; }
};

SmartMCP mcp;

// --- Global Variables ---
uint16_t dacValues[3] = {2048, 2048, 2048};
int currentChannel = 0;

// Encoder State
int lastStateA;
bool lastButtonReading = HIGH;
bool debouncedButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Serial Parsing Buffer
String inputString = "";

// ==========================================
// TEMPERATURE PID CONTROLLER
// ==========================================
struct PIDController {
  float kp = 8.0f;
  float ki = 0.5f;
  float kd = 2.0f;
  float setpoint = 45.0f;   // degC
  float integral = 0.0f;
  float lastInput = 0.0f;
  bool initialized = false;
  const float outMin = 0.0f;
  const float outMax = 100.0f;   // matches fan duty cycle range
};

PIDController tempPID;
bool pidModeEnabled = false;
unsigned long lastPidRunTime = 0;
const unsigned long PID_INTERVAL_MS = 1000;   // 1 Hz control loop
uint8_t currentFanDuty = 0;

void init25kHzPWM() {
  TCA0.SPLIT.CTRLA = 0;
  TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV4_gc | TCA_SPLIT_ENABLE_bm;
  TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm;
  TCA0.SPLIT.LPER = 99;
  TCA0.SPLIT.CTRLB &= ~(TCA_SPLIT_LCMP0EN_bm | TCA_SPLIT_LCMP1EN_bm);
  TCA0.SPLIT.CTRLB |= TCA_SPLIT_LCMP2EN_bm;
  TCA0.SPLIT.LCMP2 = 0;
  pinMode(FAN_PWM_PIN, OUTPUT);
}

void setFanDutyCycle(uint8_t duty) {
  if (duty > 100) duty = 100;

  if (duty >= 100) {
    // True 100%: detach WO2 from the pin and drive it statically HIGH.
    // CMP == PER is a degenerate case on TCA split-mode PWM that doesn't
    // reliably produce a full-width pulse (double-buffered CMP/PER/CNT),
    // so we bypass the timer compare channel entirely instead.
    TCA0.SPLIT.CTRLB &= ~TCA_SPLIT_LCMP2EN_bm;
    digitalWrite(FAN_PWM_PIN, HIGH);
  } else if (duty == 0) {
    // Same degenerate-edge concern at the bottom; force it explicitly too.
    TCA0.SPLIT.CTRLB &= ~TCA_SPLIT_LCMP2EN_bm;
    digitalWrite(FAN_PWM_PIN, LOW);
  } else {
    // Normal case: reconnect the timer to the pin and set the duty.
    TCA0.SPLIT.CTRLB |= TCA_SPLIT_LCMP2EN_bm;
    TCA0.SPLIT.LCMP2 = duty;
  }
}

// Reads the ATmega4809 factory-programmed 10-byte unique serial number
String getUniqueID() {
  uint8_t *sernum = (uint8_t*)&SIGROW.SERNUM0;
  char buf[21];
  for (int i = 0; i < 10; i++) {
    sprintf(buf + i * 2, "%02X", sernum[i]);
  }
  buf[20] = '\0';
  return String(buf);
}

// -- PID math -------------------------------------------------------
float pidCompute(PIDController &pid, float input, float dt) {
  float error = pid.setpoint - input;

  // integral accumulation
  pid.integral += error * dt;

  float pTerm = pid.kp * error;

  // integral term with anti-windup clamp: never let the accumulated
  // integral push the output beyond what the actuator can do.
  float iTerm = pid.ki * pid.integral;
  if (iTerm > pid.outMax) {
    iTerm = pid.outMax;
    if (pid.ki != 0.0f) pid.integral = pid.outMax / pid.ki;
  } else if (iTerm < pid.outMin) {
    iTerm = pid.outMin;
    if (pid.ki != 0.0f) pid.integral = pid.outMin / pid.ki;
  }

  // derivative on measurement (avoids "derivative kick" on setpoint changes)
  float dInput = pid.initialized ? (input - pid.lastInput) / dt : 0.0f;
  float dTerm = -pid.kd * dInput;

  float output = pTerm + iTerm + dTerm;
  output = constrain(output, pid.outMin, pid.outMax);

  pid.lastInput = input;
  pid.initialized = true;

  return output;
}

void pidReset(PIDController &pid, float currentTemp) {
  pid.integral = 0.0f;
  pid.lastInput = currentTemp;
  pid.initialized = true;
}

void updatePIDIfNeeded() {
  if (!pidModeEnabled) return;

  unsigned long now = millis();
  if (now - lastPidRunTime < PID_INTERVAL_MS) return;

  float dt = (lastPidRunTime == 0) ? (PID_INTERVAL_MS / 1000.0f)
                                    : (now - lastPidRunTime) / 1000.0f;
  lastPidRunTime = now;

  float temp = readTMP126();

  // Fail-safe: if the sensor reading is out of any plausible physical
  // range (disconnected/miswired SPI, etc.), don't trust it - go to max
  // cooling instead of computing garbage control output.
  if (temp < -40.0f || temp > 150.0f) {
    currentFanDuty = 100;
    setFanDutyCycle(currentFanDuty);
    Serial.println("WARN:PID_TEMP_INVALID");
    return;
  }

  float output = pidCompute(tempPID, temp, dt);
  currentFanDuty = (uint8_t)lroundf(output);
  setFanDutyCycle(currentFanDuty);

  // Unsolicited PID telemetry - rate-limited to PID_INTERVAL_MS already,
  // so safe to always emit (not gated behind DEBUG_MODE).
  Serial.print("PID:TEMP:");
  Serial.print(temp, 2);
  Serial.print(",FAN:");
  Serial.print(currentFanDuty);
  Serial.print(",SP:");
  Serial.print(tempPID.setpoint, 2);
  Serial.print(",MODE:");
  Serial.println(pidModeEnabled ? 1 : 0);
}

void setup() {
  Serial.begin(115200);

  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 2000)) {
    delay(10);
  }

  Serial.println("SYS:BOOTING...");
  inputString.reserve(30);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  pinMode(LED_CH1, OUTPUT);
  pinMode(LED_CH2, OUTPUT);
  pinMode(LED_CH3, OUTPUT);
  pinMode(LED_SATURATION, OUTPUT);

  pinMode(TMP126_CS, OUTPUT);
  digitalWrite(TMP126_CS, HIGH);

  SPI.begin();
  init25kHzPWM();

  mcp.begin(0x60);

  lastStateA = digitalRead(ENCODER_A);
  updateLEDs();
  updateDAC();

  Serial.println("SYS:READY");
}

void loop() {
  handleEncoderButton();
  handleEncoderRotation();
  updatePIDIfNeeded();

  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        processSerialCommand(inputString);
        inputString = "";
      }
    } else {
      inputString += inChar;
    }
  }
}

// --- Telemetry Output ---
void transmitSystemState() {
  bool isSaturated = (dacValues[currentChannel] == 0 || dacValues[currentChannel] == 4095);
  Serial.print("CH:");
  Serial.print(currentChannel + 1);
  Serial.print(",VAL:");
  Serial.print(dacValues[currentChannel]);
  Serial.print(",SAT:");
  Serial.print(isSaturated ? "1" : "0");
  Serial.print(",MOCK:");
  Serial.println(mcp.isMockActive() ? "1" : "0");
}

// --- Serial Command Parser ---
void processSerialCommand(String command) {
  command.trim();

  if (command == "ID" || command == "WHOAMI") {
    Serial.print("RSP:ID,");
    Serial.print(DEVICE_TYPE);
    Serial.print(",");
    Serial.print(FW_VERSION);
    Serial.print(",");
    Serial.println(getUniqueID());
  }
  else if (command.startsWith("SET_FAN:")) {
    if (pidModeEnabled) {
      Serial.println("ERR:PID_ACTIVE");
    } else {
      int valueStringIndex = command.indexOf(':');
      int fanValue = command.substring(valueStringIndex + 1).toInt();
      fanValue = constrain(fanValue, 0, 100);
      setFanDutyCycle(fanValue);
      currentFanDuty = fanValue;

      Serial.print("RSP:FAN,");
      Serial.println(fanValue);
    }
  }
  else if (command.startsWith("SET_DAC:")) {
    int colonIndex = command.indexOf(':');
    int commaIndex = command.indexOf(',');

    if (colonIndex != -1 && commaIndex != -1) {
      int targetChannel = command.substring(colonIndex + 1, commaIndex).toInt() - 1;
      int dacValue = command.substring(commaIndex + 1).toInt();

      if (targetChannel >= 0 && targetChannel <= 2) {
        dacValues[targetChannel] = constrain(dacValue, 0, 4095);

        updateDAC();
        checkSaturation();

        Serial.print("RSP:DAC_SET,");
        Serial.print(targetChannel + 1);
        Serial.print(",");
        Serial.println(dacValues[targetChannel]);
      } else {
        Serial.println("ERR:INVALID_CH");
      }
    } else {
      Serial.println("ERR:INVALID_FORMAT");
    }
  }
  else if (command == "GET_DAC") {
    Serial.print("RSP:DAC_VALS,");
    Serial.print(dacValues[0]);
    Serial.print(",");
    Serial.print(dacValues[1]);
    Serial.print(",");
    Serial.println(dacValues[2]);
  }
  else if (command == "GET_TEMP") {
    float temperature = readTMP126();
    Serial.print("RSP:TMP,");
    Serial.println(temperature, 2);
  }
  else if (command.startsWith("SET_PID_MODE:")) {
    int idx = command.indexOf(':');
    int mode = command.substring(idx + 1).toInt();
    pidModeEnabled = (mode != 0);

    if (pidModeEnabled) {
      float currentTemp = readTMP126();
      pidReset(tempPID, currentTemp);
      lastPidRunTime = 0;  // force PID to run on the very next loop() pass
    } else {
      currentFanDuty = 0;
      setFanDutyCycle(0);  // safe default: fan off, host must SET_FAN explicitly
    }

    Serial.print("RSP:PID_MODE,");
    Serial.println(pidModeEnabled ? 1 : 0);
  }
  else if (command.startsWith("SET_PID_SETPOINT:")) {
    int idx = command.indexOf(':');
    float sp = command.substring(idx + 1).toFloat();
    tempPID.setpoint = sp;

    Serial.print("RSP:PID_SETPOINT,");
    Serial.println(tempPID.setpoint, 2);
  }
  else if (command.startsWith("SET_PID_GAINS:")) {
    int idx = command.indexOf(':');
    String rest = command.substring(idx + 1);
    int c1 = rest.indexOf(',');
    int c2 = rest.indexOf(',', c1 + 1);

    if (c1 != -1 && c2 != -1) {
      tempPID.kp = rest.substring(0, c1).toFloat();
      tempPID.ki = rest.substring(c1 + 1, c2).toFloat();
      tempPID.kd = rest.substring(c2 + 1).toFloat();

      Serial.print("RSP:PID_GAINS,");
      Serial.print(tempPID.kp, 3);
      Serial.print(",");
      Serial.print(tempPID.ki, 3);
      Serial.print(",");
      Serial.println(tempPID.kd, 3);
    } else {
      Serial.println("ERR:INVALID_FORMAT");
    }
  }
  else if (command == "GET_PID") {
    Serial.print("RSP:PID_STATE,");
    Serial.print(pidModeEnabled ? 1 : 0);
    Serial.print(",");
    Serial.print(tempPID.setpoint, 2);
    Serial.print(",");
    Serial.print(tempPID.kp, 3);
    Serial.print(",");
    Serial.print(tempPID.ki, 3);
    Serial.print(",");
    Serial.print(tempPID.kd, 3);
    Serial.print(",");
    Serial.print(currentFanDuty);
    Serial.print(",");
    Serial.println(readTMP126(), 2);
  }
  else {
    Serial.println("ERR:CMD");
  }
}

float readTMP126() {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(TMP126_CS, LOW);
  uint8_t msb = SPI.transfer(0x00);
  uint8_t lsb = SPI.transfer(0x00);
  digitalWrite(TMP126_CS, HIGH);
  SPI.endTransaction();

  int16_t rawData = (msb << 8) | lsb;
  return (rawData >> 2) * 0.03125;
}

// --- UI & Peripheral Control Logic ---
void handleEncoderButton() {
  bool reading = digitalRead(ENCODER_SW);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != debouncedButtonState) {
      debouncedButtonState = reading;

      if (debouncedButtonState == LOW) {
        currentChannel++;
        if (currentChannel > 2) currentChannel = 0;

        updateLEDs();

        #if DEBUG_MODE
          transmitSystemState();
        #endif
      }
    }
  }

  lastButtonReading = reading;
}

void handleEncoderRotation() {
  int currentStateA = digitalRead(ENCODER_A);
  if (currentStateA != lastStateA && currentStateA == LOW) {
    if (digitalRead(ENCODER_B) == currentStateA) {
      if (dacValues[currentChannel] < 4045) dacValues[currentChannel] += 50;
      else dacValues[currentChannel] = 4095;
    } else {
      if (dacValues[currentChannel] > 50) dacValues[currentChannel] -= 50;
      else dacValues[currentChannel] = 0;
    }

    updateDAC();
    checkSaturation();

    #if DEBUG_MODE
      transmitSystemState();
    #endif
  }
  lastStateA = currentStateA;
}

void updateLEDs() {
  digitalWrite(LED_CH1, currentChannel == 0 ? HIGH : LOW);
  digitalWrite(LED_CH2, currentChannel == 1 ? HIGH : LOW);
  digitalWrite(LED_CH3, currentChannel == 2 ? HIGH : LOW);
  checkSaturation();
}

void updateDAC() {
  mcp.setChannelValue(MCP4728_CHANNEL_A, dacValues[0]);
  mcp.setChannelValue(MCP4728_CHANNEL_B, dacValues[1]);
  mcp.setChannelValue(MCP4728_CHANNEL_C, dacValues[2]);
}

void checkSaturation() {
  if (dacValues[currentChannel] == 0 || dacValues[currentChannel] == 4095) {
    digitalWrite(LED_SATURATION, HIGH);
  } else {
    digitalWrite(LED_SATURATION, LOW);
  }
}