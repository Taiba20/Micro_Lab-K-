/*
  cleaning Robot - ESP8266 (NodeMCU / WeMos D1 mini)
  2x N20 motors via L298N + 1x EDF via ESC
  + 2x HC-SR04 ultrasonic sensors (shared TRIG pin)
  + 1x mop DC motor via NPN/MOSFET transistor switch (PWM speed)

  IMPORTANT: D9 (RX) AND D10 (TX) are both USB-serial pins.
  Disconnect the ECHO2 wire and the mop motor's transistor base
  wire before re-uploading new code, then reconnect both.

  WiFi: connect to "WallClimber" / "climb1234", open http://192.168.4.1
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

// ---------- Motor / ESC pins ----------
#define ENA   5    // D1
#define IN1   4    // D2
#define IN2   14   // D5
#define IN3   13   // D7
#define IN4   16   // D0
#define ENB   12   // D6
#define ESC_PIN 15 // D8

// ---------- Sensor pins ----------
#define TRIG_PIN   0   // D3  (shared trigger for both ultrasonic sensors)
#define ECHO1_PIN  2   // D4  (ultrasonic 1 - front)
#define ECHO2_PIN  3   // D9 / RX (ultrasonic 2 - side)  ** disconnect before upload **
#define MOP_PIN    1   // D10 / TX (mop motor transistor base/gate, PWM) ** disconnect before upload **

// ---------- Settings ----------
const char* AP_SSID = "WallClimber";
const char* AP_PASS = "climb1234";

const int ESC_MIN_US = 1000;
const int ESC_MAX_US = 2000;
const int FAN_LIMIT  = 100;
const unsigned long DRIVE_TIMEOUT = 500;

const float OBSTACLE_STOP_CM = 8.0;   // auto-stop drive if something closer than this
const unsigned long SENSOR_INTERVAL = 100; // ms between sensor reads

Servo esc;
ESP8266WebServer server(80);

unsigned long lastDrive = 0;
int fanTarget = 0, fanNow = 0;
unsigned long lastRamp = 0;
unsigned long lastSensor = 0;

float dist1 = -1, dist2 = -1;
int lastL = 0, lastR = 0;
int mopSpeed = 0; // 0-255

// ---------- Motors ----------
void setMotor(int en, int inA, int inB, int spd) {
  spd = constrain(spd, -255, 255);
  digitalWrite(inA, spd > 0);
  digitalWrite(inB, spd < 0);
  analogWrite(en, abs(spd));
}

void drive(int left, int right) {
  setMotor(ENA, IN1, IN2, -left);  // left motor (Channel A) is wired reversed - inverted here in software
  setMotor(ENB, IN3, IN4, right);
}

// ---------- Fan ----------
void writeFan(int pct) {
  pct = constrain(pct, 0, 100);
  esc.writeMicroseconds(map(pct, 0, 100, ESC_MIN_US, ESC_MAX_US));
}

// ---------- Mop motor ----------
void writeMop(int pct) {
  mopSpeed = constrain(pct, 0, 100);
  analogWrite(MOP_PIN, map(mopSpeed, 0, 100, 0, 255));
}

int parseClampedInt(const String& value, int fallback, int minVal, int maxVal) {
  if (value.length() == 0) return fallback;
  int parsed = value.toInt();
  if (parsed < minVal) return minVal;
  if (parsed > maxVal) return maxVal;
  return parsed;
}

// ---------- Ultrasonic ----------
float readEcho(int echoPin) {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000); // 25ms timeout (~4m range)
  if (duration == 0) return -1; // no echo / out of range
  return duration * 0.0343 / 2.0; // cm
}

void readSensors() {
  dist1 = readEcho(ECHO1_PIN);
  delay(10); // small gap so the two echoes don't overlap
  dist2 = readEcho(ECHO2_PIN);
}

bool obstacleTooClose() {
  if (dist1 > 0 && dist1 < OBSTACLE_STOP_CM) return true;
  if (dist2 > 0 && dist2 < OBSTACLE_STOP_CM) return true;
  return false;
}

// ---------- Web page ----------
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Wall Climber</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;text-align:center;margin:0;padding:10px;touch-action:none}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:320px;margin:10px auto}
button{font-size:28px;padding:22px 0;border:0;border-radius:14px;background:#2a6;color:#fff}
button:active{background:#1c4}
.stop{background:#c33}.fan{background:#37c}
input[type=range]{width:90%}
#sensors{margin-top:14px;font-size:13px;text-align:left;max-width:320px;margin-left:auto;margin-right:auto;background:#222;padding:10px;border-radius:10px}
.bad{color:#f66;font-weight:bold}
</style></head><body>
<h2>Wall Climber</h2>
<div class="pad">
 <span></span><button id="f">&#9650;</button><span></span>
 <button id="l">&#9664;</button><button id="s" class="stop">&#9632;</button><button id="r">&#9654;</button>
 <span></span><button id="b">&#9660;</button><span></span>
</div>
Drive speed: <b id="sv">180</b><br>
<input type="range" id="spd" min="80" max="255" value="180"><br><br>
Fan: <b id="fv">0</b>%<br>
<input type="range" id="fan" min="0" max="100" value="0"><br><br>
<button class="fan" onclick="setFan(0)" style="padding:14px 40px">FAN OFF</button><br><br>
Mop: <b id="mv">0</b>%<br>
<input type="range" id="mop" min="0" max="100" value="0"><br>
<button style="background:#a67c00" onclick="setMop(0)">MOP OFF</button>
<button style="background:#a67c00" onclick="setMop(100)">MOP FULL</button>
<div id="sensors">reading sensors...</div>
<script>
let spd=180,busy=false;
const $=id=>document.getElementById(id);
$('spd').oninput=e=>{spd=+e.target.value;$('sv').textContent=spd};
function send(l,r){fetch('/drive?l='+l+'&r='+r).catch(()=>{})}
function hold(id,fn){
  const el=$(id);let t;
  const go=e=>{e.preventDefault();const[a,b]=fn();send(a,b);clearInterval(t);t=setInterval(()=>send(a,b),150)};
  const end=()=>{clearInterval(t);send(0,0)};
  el.addEventListener('pointerdown',go);
  ['pointerup','pointerleave','pointercancel'].forEach(ev=>el.addEventListener(ev,end));
}
hold('f',()=>[spd,spd]);
hold('b',()=>[-spd,-spd]);
hold('l',()=>[-spd,spd]);
hold('r',()=>[spd,-spd]);
$('s').onclick=()=>send(0,0);
function setFan(v){$('fan').value=v;$('fv').textContent=v;fetch('/fan?v='+v).catch(()=>{})}
$('fan').oninput=e=>{$('fv').textContent=e.target.value;
  if(!busy){busy=true;setTimeout(()=>{busy=false;fetch('/fan?v='+$('fan').value)},120)}};
$('fan').onchange=e=>fetch('/fan?v='+e.target.value);
function setMop(v){$('mop').value=v;$('mv').textContent=v;fetch('/mop?v='+v).catch(()=>{})}
$('mop').oninput=e=>{$('mv').textContent=e.target.value;
  if(!busy){busy=true;setTimeout(()=>{busy=false;fetch('/mop?v='+$('mop').value)},120)}};
$('mop').onchange=e=>fetch('/mop?v='+e.target.value);
setInterval(()=>{
  fetch('/sensors').then(r=>r.json()).then(d=>{
    let h='Ultrasonic 1: '+(d.d1<0?'--':d.d1.toFixed(1)+' cm')+'<br>';
    h+='Ultrasonic 2: '+(d.d2<0?'--':d.d2.toFixed(1)+' cm')+'<br>';
    if(d.blocked) h+='<span class="bad">AUTO-STOP: obstacle too close</span>';
    $('sensors').innerHTML=h;
  }).catch(()=>{});
},300);
</script></body></html>
)rawliteral";

// ---------- Handlers ----------
void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleDrive() {
  lastL = parseClampedInt(server.arg("l"), 0, -255, 255);
  lastR = parseClampedInt(server.arg("r"), 0, -255, 255);
  lastDrive = millis();
  server.send(200, "text/plain", "ok");
}

void handleFan() {
  fanTarget = parseClampedInt(server.arg("v"), 0, 0, FAN_LIMIT);
  if (fanTarget == 0) { fanNow = 0; writeFan(0); }
  server.send(200, "text/plain", "ok");
}

void handleMop() {
  writeMop(parseClampedInt(server.arg("v"), 0, 0, 100));
  server.send(200, "text/plain", "ok");
}

void handleSensors() {
  bool blocked = obstacleTooClose();
  String json = "{";
  json += "\"d1\":" + String(dist1, 1) + ",";
  json += "\"d2\":" + String(dist2, 1) + ",";
  json += "\"blocked\":" + String(blocked ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

// ---------- Setup / loop ----------
void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  analogWriteRange(255);
  analogWriteFreq(1000);
  drive(0, 0);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(ECHO2_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(MOP_PIN, OUTPUT);
  writeMop(0); // mop off at startup

  esc.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);
  writeFan(0);
  delay(3000); // let ESC arm

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", handleRoot);
  server.on("/drive", handleDrive);
  server.on("/fan", handleFan);
  server.on("/mop", handleMop);
  server.on("/sensors", handleSensors);
  server.begin();
}

void loop() {
  server.handleClient();

  if (millis() - lastSensor >= SENSOR_INTERVAL) {
    lastSensor = millis();
    readSensors();
  }

  if (millis() - lastDrive > DRIVE_TIMEOUT) {
    drive(0, 0);
  } else if (obstacleTooClose()) {
    drive(0, 0); // safety cutoff overrides manual drive command
  } else {
    drive(lastL, lastR);
  }

  if (millis() - lastRamp >= 20) {
    lastRamp = millis();
    if (fanNow < fanTarget)      fanNow++;
    else if (fanNow > fanTarget) fanNow--;
    writeFan(fanNow);
  }
}