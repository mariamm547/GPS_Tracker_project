#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <math.h>

#define R 6371000  // Earth's radius in meters

int RXPin = 2;
int TXPin = 3;
int GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);
SoftwareSerial sim800(10, 11);  // SIM RX, TX

String number = "+201032557678";  // Phone number
String smsData = "";
String fullSMS;

double lng1 = 31.277572894058487, lat1 = 30.06218242628831;
double lng2, lat2, distance;

int buzzer_pin = 5;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);
  sim800.begin(9600);
  pinMode(buzzer_pin, OUTPUT);

  startGSM();
  Serial.println("GSM Started");
}

void loop() {
  if (sim800.available()) {
    String fullSMS = readSMS();
    Serial.println("Parsing SMS...");

    extractCoordinates(fullSMS, lat2, lng2);
    
    if (lat2 != 0 && lng2 != 0) {
      Serial.print("Extracted Latitude: ");
      Serial.println(lat2, 6);
      Serial.print("Extracted Longitude: ");
      Serial.println(lng2, 6);

      doAction(distance);
      clearSerialBuffer();
    }
  }
}

void startGSM() {
  delay(1000);
  sim800.println("AT");
  delay(500);
  sim800.println("AT+CMGF=1");
  delay(500);
  sim800.println("AT+CNMI=1,2,0,0,0");
  delay(500);
}

void SendMessage(String distStr) {
  sim800.println("AT+CMGF=1");
  delay(1000);
  sim800.println("AT+CMGS=\"" + number + "\"\r");
  delay(1000);
  String SMS = "The Distance between you and the tracked one is: " + distStr + " meters.";
  sim800.println(SMS);
  delay(100);
  sim800.println((char)26);  // Ctrl+Z to send
  delay(1000);

  Serial.println("SMS Sent:");
  Serial.println(SMS);
}

void callNumber() {
  sim800.print(F("ATD"));
  sim800.print(number);
  sim800.print(F(";\r\n"));
}

double toRadians(double degree) {
  return degree * (PI / 180);
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
  double dLat = toRadians(lat2 - lat1);
  double dLon = toRadians(lon2 - lon1);

  lat1 = toRadians(lat1);
  lat2 = toRadians(lat2);

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) *
             sin(dLon / 2) * sin(dLon / 2);

  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;  // Distance in meters
}

void buzzer(int freq) {
  tone(buzzer_pin, freq);
  delay(1000);
  noTone(buzzer_pin);
  delay(500);
  Serial.println(freq);
}

void doAction(double distance) {
  String distStr = String(distance, 2);  // Convert to string with 2 decimal places

  if (distance >= 730) {
    // SendMessage(distStr);
    callNumber();
    delay(300);
    SendMessage(distStr);
    buzzer(2000);
  } else if (distance < 730 && distance >= 280) {
    SendMessage(distStr);
    buzzer(2500);
  } else if (distance < 280 && distance >= 70) {
    SendMessage(distStr);
    buzzer(3500);
  } else if (distance < 70 && distance >= 0) {
    noTone(buzzer_pin);
  }
}

String readSMS() {
  String sms = "";
  bool messageStarted = false;
  unsigned long startTime = millis();
  int linesAfterHeader = 0;

  while (millis() - startTime < 7000) {
    while (sim800.available()) {
      char c = sim800.read();
      sms += c;
      Serial.write(c);

      if (!messageStarted && sms.indexOf("+CMT:") != -1) {
        messageStarted = true;
      }

      if (messageStarted && c == '\n') {
        linesAfterHeader++;
      }

      if (messageStarted && linesAfterHeader >= 2) {
        delay(300);
        break;
      }
    }
  }

  sms.trim();
  return sms;
}

void extractCoordinates(String sms, double &lat, double &lng) {
  lat = 0;
  lng = 0;

  int newLine = sms.indexOf('\n');
  if (newLine != -1) {
    String coordLine = sms.substring(newLine + 1);
    coordLine.trim();
    coordLine.replace("\r", "");

    int comma = coordLine.indexOf(',');
    if (comma != -1) {
      String latStr = coordLine.substring(0, comma);
      String lngStr = coordLine.substring(comma + 1);

      lat = latStr.toFloat();
      lng = lngStr.toFloat();

      Serial.println("Parsed full coordinates:");
      Serial.println("Lat: " + latStr);
      Serial.println("Lng: " + lngStr);
    } else {
      Serial.println("Missing comma separator.");
    }
  } else {
    Serial.println("No newline found in SMS body.");
  }

  distance = haversine(lat1, lng1, lat2, lng2);
  Serial.println("Calculated distance: " + String(distance, 2));
}

void clearSerialBuffer() {
  while (sim800.available()) {
    sim800.read();
  }
}