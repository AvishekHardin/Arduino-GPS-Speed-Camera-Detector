#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#define GPSECHO  false

float speedCameraLoc[40][2]={{-20.243286, 57.749267}, {-20.108813,57.560806},{-20.129616,57.530229},
{-20.135307,57.527611},{-20.146408,57.508868},{-20.164738,57.487121},{-20.173832,57.482625},{-20.179532,57.480844},
{-20.179029,57.470207},{-20.189914,57.482185},{-20.195916,57.469997},{-20.197486,57.444806},{-20.218811,57.485318},
{-20.187296,57.578402},{-20.200427,57.571793},{-20.218811,57.485318},{-20.246817,57.421074},{-20.262700,57.407126},
{-20.277495,57.385840},{-20.280333,57.419271},{-20.291524,57.402534},{-20.253269,57.491262},{-20.259782,57.489052},
{-20.281390,57.478645},{-20.281500,57.485361},{-20.279447,57.501497},{-20.285103,57.506583},{-20.292349,57.506111},
{-20.297139,57.536345},{-20.302552,57.496583},{-20.308413,57.514485},{-20.342142,57.566321},{-20.380033,57.575912},
{-20.415319,57.562169},{-20.183470,57.664704},{-20.202375,57.672118},{-20.200809,57.720494},{-20.227973,57.741265},
{-20.214502, 57.700449},{-20.244804,57.655392}};

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
boolean usingInterrupt = false;
void useInterrupt(boolean); 

int speakerPin = 12;
float dlat, dlon, a, c, d;
float currentLat, currentLon;


void setup(){
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
}

SIGNAL(TIMER0_COMPA_vect){
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
#endif
}

void useInterrupt(boolean v){
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void loop(){
  if (! usingInterrupt) {
    char c = GPS.read();
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  if (GPS.newNMEAreceived()){ 
    if (!GPS.parse(GPS.lastNMEA()))
      return; 
  }
  
  if (GPS.fix) {
    currentLat = GPS._deciDegLat;
    currentLon = GPS._deciDegLon;
    
    for (int x=0; x<40; x++){
      dlat = (speedCameraLoc[x][0] - currentLat) * (3.142 / 180);
      dlon = (speedCameraLoc[x][1] - currentLon) * (3.142 / 180);
      a = pow((sin(dlat/2)),2) + cos(speedCameraLoc[x][0]) * cos(currentLat) * pow((sin(dlon/2)),2);
      c = 2 * atan2( sqrt(a), sqrt(1-a));
      d = 6373000 * c;  
      Serial.print("Distance/m :");
      Serial.println(d,6);
      if( d < 25 ){
        sound_alert(100);
      }else if( d < 50 ){
        sound_alert(200);
      }else if( d < 75 ){
        sound_alert(300);
      }else if( d < 100 ){
        sound_alert(400);
      }else{
      }
    }
    Serial.println("\n"); 
  }
}

void sound_alert(int pitch){
  tone(speakerPin, 1200);
  delay(pitch);
  noTone(speakerPin);
  delay(pitch);
  noTone(speakerPin);
}
