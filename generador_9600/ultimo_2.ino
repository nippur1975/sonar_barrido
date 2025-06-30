#include <AltSoftSerial.h>
#include <TimeLib.h>
#include <math.h>

// Pines
const int sw1 = 4, sw2 = 5, sw3 = 6, sw4 = 7;
const int ledPort = 8, ledStarboard = 12;
const int potPin = A0;

unsigned long previousMillis = 0;
const unsigned long interval = 200;

float lat = -12.055078, lon = -77.502755;
float heading = 0.0;
float speedKn = 0.0;
float pitch = 0.0, roll = 0.0;
float tempSea = 18.5;
float depth = 75.0;
float distanceNM = 0.0;
unsigned long startTime;
int potMax = 410;

AltSoftSerial nmeaSerial;

void setup() {
  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);
  pinMode(sw3, INPUT_PULLUP);
  pinMode(sw4, INPUT_PULLUP);
  pinMode(ledPort, OUTPUT);
  pinMode(ledStarboard, OUTPUT);

  Serial.begin(9600);
  nmeaSerial.begin(9600);
  setTime(0, 0, 0, 25, 6, 2025);
  startTime = millis();
}

void loop() {
  updateSpeed();
  updateHeading();
  updatePosition();
  updatePitchRoll();
  updateTemperature();
  updateDepth();
  updateLEDs();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendNMEASentences();
  }
}

void updateSpeed() {
  static bool estabaAdelante = false;

  int potVal = analogRead(potPin);
  if (potVal > potMax && potVal <= 1023) potMax = potVal;

  if (potVal < 400 && speedKn > 0.5 && estabaAdelante) speedKn = 0.0;
  estabaAdelante = (potVal > 410);

  float targetSpeed = 0.0;
  if (potVal > 410) {
    if (potMax - 410 == 0) return;
    float ratio = float(potVal - 410) / float(potMax - 410);
    ratio = constrain(ratio, 0.0, 1.0);
    targetSpeed = 13.0 * (exp(3.0 * ratio) - 1.0) / (exp(3.0) - 1.0);
  } else if (potVal >= 400 && potVal <= 410) {
    float timeConst = (speedKn > 6.5) ? 240000.0 : 180000.0;
    float alpha = interval / timeConst;
    speedKn *= (1.0 - alpha);
    return;
  } else if (potVal < 400) {
    if (speedKn > 0.1) {
      float alpha = interval / 60000.0;
      speedKn *= (1.0 - alpha);
      return;
    } else {
      float ratio = float(400 - potVal) / 400.0;
      ratio = constrain(ratio, 0.0, 1.0);
      targetSpeed = -5.0 * log(1 + 9 * ratio) / log(10);
    }
  }

  float maxDelta = (interval / 60000.0) * 13.0;
  float delta = targetSpeed - speedKn;
  delta = constrain(delta, -maxDelta, maxDelta);
  speedKn += delta;
}

void updateHeading() {
  static unsigned long lastUpdate = 0;
  static unsigned long lastOsc = 0;
  unsigned long now = millis();

  bool babor = (digitalRead(sw3) == LOW);
  bool estribor = (digitalRead(sw4) == LOW);

  if ((babor || estribor) && now - lastUpdate >= 100) {
    if (babor && !estribor) heading -= 0.5;
    else if (estribor && !babor) heading += 0.5;
    lastUpdate = now;
  }

  if (now - lastOsc >= 1000) {
    heading += random(-10, 11) * 0.1;
    lastOsc = now;
  }

  if (heading < 0) heading += 360;
  if (heading >= 360) heading -= 360;
}

void updatePitchRoll() {
  float t = millis() / 1000.0;
  float waveEffect = sin(t * 1.1) * 2.0;

  if (speedKn == 0) {
    pitch = sin(t) * 1.0;
    roll = sin(t * 1.3) * 1.0;
  } else {
    pitch = (speedKn / 13.0) * 12.0 + sin(t * 0.7) * 0.5;
    if (digitalRead(sw3) == LOW)
      roll = (speedKn / 13.0) * 25.0 + waveEffect;
    else if (digitalRead(sw4) == LOW)
      roll = -(speedKn / 13.0) * 25.0 + waveEffect;
    else
      roll = waveEffect;
  }
}

void updatePosition() {
  float dt = interval / 1000.0;
  float distance = speedKn * dt / 3600.0;
  distanceNM += abs(distance);

  float dLat = distance * cos(radians(heading)) / 60.0;
  float dLon = distance * sin(radians(heading)) / (60.0 * cos(radians(lat)));
  lat += dLat;
  lon += dLon;
}

void updateTemperature() {
  tempSea = 18.5 + fmod(distanceNM, 1.7);
}

void updateDepth() {
  depth = 75.0 + 25.0 * sin(millis() / 10000.0); // Oscila entre 50 y 100 m
}

void updateLEDs() {
  digitalWrite(ledPort, digitalRead(sw3) == LOW ? HIGH : LOW);
  digitalWrite(ledStarboard, digitalRead(sw4) == LOW ? HIGH : LOW);
}

void sendNMEASentences() {
  String gll = formatGLL();
  String hdt = formatHDT();
  String vtg = formatVTG();
  String att = formatATT();
  String xdr = formatXDR();
  String zda = formatZDA();
  String gga = formatGGA();
  String rmc = formatRMC();
  String dbt = formatDBT();

  nmeaSerial.println(gll);
  nmeaSerial.println(hdt);
  nmeaSerial.println(vtg);
  nmeaSerial.println(att);
  nmeaSerial.println(xdr);
  nmeaSerial.println(zda);
  nmeaSerial.println(gga);
  nmeaSerial.println(rmc);
  nmeaSerial.println(dbt);

  Serial.println("============== NMEA Data ==============");
  Serial.println("GLL: " + gll);
  Serial.println("HDT: " + hdt);
  Serial.println("VTG: " + vtg);
  Serial.println("ATT: " + att);
  Serial.println("XDR: " + xdr);
  Serial.println("ZDA: " + zda);
  Serial.println("GGA: " + gga);
  Serial.println("RMC: " + rmc);
  Serial.println("DBT: " + dbt);
  Serial.println();
}

String addChecksum(String sentence) {
  int checksum = 0;
  for (int i = 1; i < sentence.length(); i++) checksum ^= sentence[i];
  char hex[6];
  sprintf(hex, "*%02X", checksum);
  return sentence + String(hex);
}

String formatGLL() {
  char latHem = lat < 0 ? 'S' : 'N';
  char lonHem = lon < 0 ? 'W' : 'E';
  float absLat = abs(lat), absLon = abs(lon);
  int latDeg = int(absLat);
  float latMin = (absLat - latDeg) * 60;
  int lonDeg = int(absLon);
  float lonMin = (absLon - lonDeg) * 60;

  String timeStr = (hour() < 10 ? "0" : "") + String(hour()) +
                   (minute() < 10 ? "0" : "") + String(minute()) +
                   (second() < 10 ? "0" : "") + String(second());

  String latStr = (latDeg < 10 ? "0" : "") + String(latDeg) +
                  (latMin < 10 ? "0" : "") + String(latMin, 4);
  String lonStr = (lonDeg < 100 ? (lonDeg < 10 ? "00" : "0") : "") + String(lonDeg) +
                  (lonMin < 10 ? "0" : "") + String(lonMin, 4);

  String sentence = "$GPGLL," + latStr + "," + latHem + "," + lonStr + "," + lonHem + "," + timeStr + ",A";
  return addChecksum(sentence);
}

String formatHDT() {
  return addChecksum("$GPHDT," + String(heading, 1) + ",T");
}

String formatVTG() {
  float magHeading = heading + 3.9;
  float kph = speedKn * 1.852;
  return addChecksum("$GPVTG," + String(heading, 1) + ",T," + String(magHeading, 1) + ",M," +
                     String(speedKn, 1) + ",N," + String(kph, 1) + ",K");
}

String formatATT() {
  return addChecksum("$PFEC,GPatt," + String(heading, 1) + "," + String(pitch, 1) + "," + String(roll, 1));
}

String formatXDR() {
  return addChecksum("$IIXDR,A," + String(tempSea, 1) + ",C,CSThh");
}

String formatZDA() {
  String timeStr = (hour() < 10 ? "0" : "") + String(hour()) +
                   (minute() < 10 ? "0" : "") + String(minute()) +
                   (second() < 10 ? "0" : "") + String(second());

  String dayStr = (day() < 10 ? "0" : "") + String(day());
  String monthStr = (month() < 10 ? "0" : "") + String(month());

  return addChecksum("$GPZDA," + timeStr + ".00," + dayStr + "," + monthStr + "," + String(year()) + ",-05,00");
}

String formatGGA() {
  char latHem = lat < 0 ? 'S' : 'N';
  char lonHem = lon < 0 ? 'W' : 'E';
  float absLat = abs(lat), absLon = abs(lon);
  int latDeg = int(absLat);
  float latMin = (absLat - latDeg) * 60;
  int lonDeg = int(absLon);
  float lonMin = (absLon - lonDeg) * 60;

  String latStr = (latDeg < 10 ? "0" : "") + String(latDeg) +
                  (latMin < 10 ? "0" : "") + String(latMin, 4);
  String lonStr = (lonDeg < 100 ? (lonDeg < 10 ? "00" : "0") : "") + String(lonDeg) +
                  (lonMin < 10 ? "0" : "") + String(lonMin, 4);

  String timeStr = (hour() < 10 ? "0" : "") + String(hour()) +
                   (minute() < 10 ? "0" : "") + String(minute()) +
                   (second() < 10 ? "0" : "") + String(second());

  return addChecksum("$GPGGA," + timeStr + "," + latStr + "," + latHem + "," +
                     lonStr + "," + lonHem + ",1,08,0.9,12.3,M,0.0,M,,");
}

String formatRMC() {
  char latHem = lat < 0 ? 'S' : 'N';
  char lonHem = lon < 0 ? 'W' : 'E';
  float absLat = abs(lat), absLon = abs(lon);
  int latDeg = int(absLat);
  float latMin = (absLat - latDeg) * 60;
  int lonDeg = int(absLon);
  float lonMin = (absLon - lonDeg) * 60;

  String latStr = (latDeg < 10 ? "0" : "") + String(latDeg) +
                  (latMin < 10 ? "0" : "") + String(latMin, 4);
  String lonStr = (lonDeg < 100 ? (lonDeg < 10 ? "00" : "0") : "") + String(lonDeg) +
                  (lonMin < 10 ? "0" : "") + String(lonMin, 4);

  String timeStr = (hour() < 10 ? "0" : "") + String(hour()) +
                   (minute() < 10 ? "0" : "") + String(minute()) +
                   (second() < 10 ? "0" : "") + String(second());

  String dateStr = (day() < 10 ? "0" : "") + String(day()) +
                   (month() < 10 ? "0" : "") + String(month()) +
                   String(year() % 100);

  float speedKnots = abs(speedKn);

  return addChecksum("$GPRMC," + timeStr + ",A," + latStr + "," + latHem + "," +
                     lonStr + "," + lonHem + "," + String(speedKnots, 1) + "," +
                     String(heading, 1) + "," + dateStr + ",,");
}

String formatDBT() {
  float feet = depth * 3.28084;
  float fathoms = depth * 0.546807;
  return addChecksum("$SDDBT," + String(feet, 1) + ",f," + String(depth, 1) + ",M," + String(fathoms, 1) + ",F");
}
