#include <TinyGPS++.h>
#include <WiFi.h>
#include <WebServer.h>

TinyGPSPlus gps;

// =======================================================
// (1) WiFi Access Point Configuration
// =======================================================
const char* ap_ssid = "Robot-GPS";
const char* ap_pass = "12345678";

WebServer server(80);

// (1.1) Last known GPS coordinates
double lastLat = 0.0;
double lastLng = 0.0;

// =======================================================
// (2) HTML Web Page Builder
// =======================================================
// Generates a simple mobile-friendly webpage showing GPS coordinates
String buildPage() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Robot GPS</title></head><body>";
  html += "<h2>Robot GPS Location</h2>";

  // (2.1) If no GPS fix yet
  if (lastLat == 0.0 && lastLng == 0.0) {
    html += "<p>No GPS fix yet... 📡</p>";
  } 
  else {
    // (2.2) Display coordinates
    html += "<p><b>Latitude:</b> " + String(lastLat, 6) + "</p>";
    html += "<p><b>Longitude:</b> " + String(lastLng, 6) + "</p>";

    // (2.3) Google Maps link
    String mapsUrl = "https://www.google.com/maps?q=" + 
                     String(lastLat, 6) + "," + 
                     String(lastLng, 6);

    html += "<p><a href='" + mapsUrl + "' target='_blank'>Open in Google Maps 📍</a></p>";
  }

  html += "</body></html>";
  return html;
}

// =======================================================
// (3) Web Server Handlers
// =======================================================
void handleRoot() {
  server.send(200, "text/html", buildPage());
}

// =======================================================
// (4) Setup: GPS + WiFi + Web Server
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("GPS + WiFi AP Started...");

  // (4.1) Initialize GPS module on Serial2 (RX = IO4, TX = IO2)
  Serial2.begin(9600, SERIAL_8N1, 4, 2);

  // (4.2) Start WiFi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);

  // Show AP IP (typically 192.168.4.1)
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // (4.3) Web server routes
  server.on("/", handleRoot);

  // (4.4) Start server
  server.begin();
  Serial.println("Web server started.");
}

// =======================================================
// (5) Loop: GPS Update + Web Server Handling
// =======================================================
void loop() {

  // (5.1) Read incoming GPS data from Serial2
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }

  // (5.2) If GPS location updated
  if (gps.location.isUpdated()) {
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();

    Serial.println("===== GPS FIX =====");
    Serial.print("Lat: "); Serial.println(lastLat, 6);
    Serial.print("Lng: "); Serial.println(lastLng, 6);
    Serial.print("Satellites: "); Serial.println(gps.satellites.value());
    Serial.println();
  }

  // (5.3) Handle client browser requests
  server.handleClient();
}
