// WiFi robot car firmware (ESP32 + L298N)
// Upload this ONCE over USB. After that, programs are sent wirelessly
// by:  python compiler.py program.rbt --send 192.168.4.1
//
// The car makes its own WiFi network (access point), so no router is needed.
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// ---------- settings you may change ----------
const char*    AP_SSID = "RobotCar";
const char*    AP_PASS = "robot1234";   // at least 8 characters
const uint16_t PORT    = 5000;

#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14

const int   MS_PER_UNIT = 500;    // time per 1 distance unit
const float MS_PER_DEG  = 3.04;   // time per 1 degree of turn
const int   MAX_CMDS    = 1000;
// ---------------------------------------------

struct Cmd { char op; int val; };
Cmd prog[MAX_CMDS];
int progLen = 0;

WiFiServer server(PORT);
WiFiClient client;
bool stopRequested = false;

void setMotors(int a, int b, int c, int d) {
  digitalWrite(IN1, a); digitalWrite(IN2, b);
  digitalWrite(IN3, c); digitalWrite(IN4, d);
}
void stopMotors() { setMotors(0, 0, 0, 0); }

// Wait while watching for a STOP message or a dropped connection.
// Returns false if the run must end.
bool waitMs(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (!client.connected()) { stopRequested = true; return false; }
    if (client.available()) {
      String s = client.readStringUntil('\n');
      s.trim(); s.toUpperCase();
      if (s == "STOP") { stopRequested = true; return false; }
    }
    delay(5);
  }
  return true;
}

bool runCmd(const Cmd& c) {
  const char* name = "";
  unsigned long ms = 0;
  int a = 0, b = 0, p = 0, q = 0;
  switch (c.op) {
    case 'F': name = "FORWARD";  a = 1; b = 0; p = 1; q = 0; ms = (unsigned long)c.val * MS_PER_UNIT; break;
    case 'B': name = "BACKWARD"; a = 0; b = 1; p = 0; q = 1; ms = (unsigned long)c.val * MS_PER_UNIT; break;
    case 'L': name = "LEFT";     a = 1; b = 0; p = 0; q = 1; ms = (unsigned long)(c.val * MS_PER_DEG); break;
    case 'R': name = "RIGHT";    a = 0; b = 1; p = 1; q = 0; ms = (unsigned long)(c.val * MS_PER_DEG); break;
    default: return true;
  }
  client.printf("RUN %s %d\n", name, c.val);
  setMotors(a, b, p, q);
  bool ok = waitMs(ms);
  stopMotors();
  return ok;
}

void runProgram() {
  stopRequested = false;
  client.printf("ACK %d\n", progLen);
  for (int i = 0; i < progLen; i++) {
    if (!runCmd(prog[i])) break;
    if (!waitMs(200)) break;          // short pause between commands
  }
  stopMotors();
  if (client.connected()) client.println(stopRequested ? "STOPPED" : "DONE");
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  line.toUpperCase();

  if (line == "CLEAR") { progLen = 0; client.println("OK"); return; }
  if (line == "STOP")  { stopMotors(); client.println("STOPPED"); return; }
  if (line == "END")   { runProgram(); progLen = 0; return; }

  int sp = line.indexOf(' ');
  if (sp < 0) { client.println("ERR format"); progLen = 0; return; }
  String w = line.substring(0, sp);
  int v = line.substring(sp + 1).toInt();

  char op = 0;
  if (w == "FORWARD")  op = 'F';
  if (w == "BACKWARD") op = 'B';
  if (w == "LEFT")     op = 'L';
  if (w == "RIGHT")    op = 'R';
  if (!op || v <= 0) { client.println("ERR " + line); progLen = 0; return; }
  if (progLen >= MAX_CMDS) { client.println("ERR too many commands"); return; }
  prog[progLen++] = Cmd{op, v};
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("WiFi network: "); Serial.println(AP_SSID);
  Serial.print("Car IP: ");       Serial.println(WiFi.softAPIP());

  ArduinoOTA.setHostname("robotcar");
  ArduinoOTA.setPassword(AP_PASS);
  ArduinoOTA.onStart([]() { stopMotors(); });
  ArduinoOTA.begin();

  server.begin();
  Serial.println("READY");
}

void loop() {
  ArduinoOTA.handle();

  if (!client.connected()) {
    WiFiClient c = server.available();
    if (c) {
      client = c;
      client.setNoDelay(true);
      progLen = 0;
      client.println("READY");
    }
    return;
  }
  if (client.available()) handleLine(client.readStringUntil('\n'));
}
