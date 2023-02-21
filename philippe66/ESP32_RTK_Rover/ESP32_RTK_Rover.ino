/*
Application avec GPS F9P RTK Rover
liaison UART2 F9P recupere trames UBX
Affichage sur console Bluetooth
Permet Enregistrer point particuliers pk, Waypoint
enregistrement sur SD card
liaison avec regles de mesure
*/

#include <Arduino.h>
#include "data.h"
#include <TimeAlarms.h>
#include <SPIFFS.h>
#include "BluetoothSerial.h"
#include <SPI.h>
#include <SD.h>
// #include <SdFat.h>

#define VT_CLS                  "\x1B[2J\x1B[H"                         // Code VT-100 "clearscreen + cursorhome"
#define VT_DEFAULT_COLOR        "\x1B[49m"                              // Code VT-100 de la couleur d'arrière-plan par défaut
#define VT_REVERSE              "\x1B[7m"                               // Code VT-100 de la couleur d'arrière-plan inversée
#define VT_NORMAL               "\x1B[m"                                // Code VT-100 de la couleur d'arrière-plan normale
#define LIGNE_SEPAR             "================================================================================"
#define op_timing               12 // sortie mesure temps
#define op_syncRg               13 // sortie synchro règle
#define SD_MISO                 19 // SD card
#define SD_MOSI                 23
#define SD_SCLK                 18
#define SD_CS                   5

BluetoothSerial SerialBT;

String bidon = "";
byte nbrsat         = 0;     // UBX_PVT
float speed         = 0;     // UBX_PVT
float course        = 0;     // UBX_PVT
bool headVehValid   = false; // UBX_PVT
byte fix            = false; // UBX_PVT
String RTK_solution = "";    // UBX_PVT

long double lon_double = 0; // lon from UBX_NAV_HPPOSLLH
long double lat_double = 0; // lat from UBX_NAV_HPPOSLLH
float hAcc        = 0.0;

int timer1 = 1000; // temps boucle affichage
unsigned long last_timer1  = 0; // variable timer 1
unsigned long last_UBX_PVT = 0; // time(millis) last UBX_PVT
unsigned long last_UBX_HPP = 0; // time(millis) last UBX_HPP

bool first = true; // premiere mise à l'heure
bool SPIFFS_present = false;
char currentfile[16]      = "/";   // fichier en SPIFFS "/yyyymmddhhmmss"
String fl = "\n";
String bidons;    // string temporaire
char bidonc[101]; // char temporaire
int lastfileline = 0; // derniere ligne du fichier courant
String lastData = ""; // Données de lastfileline

UBXMessage ubxMessage;
//---------------------------------------------------------------------------
void setup(){

  // SPIFFS.format();

  Serial.begin(115200);
  Serial2.begin(115200); // UART2 du F9P
  SerialBT.begin("ESP32_Rover"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  if (!SPIFFS.begin(true)) {
    Serial.println(F("SPIFFS initialisation failed..."));
    SerialBT.println(F("SPIFFS initialisation failed..."));
    SPIFFS_present = false;
  }
  else {
    Serial.println(F("SPIFFS initialised... file access enabled..."));
    SerialBT.println(F("SPIFFS initialised... file access enabled..."));
    SPIFFS_present = true;
  }

  Serial.println("========SDCard Detect.======");
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI);
  if (!SD.begin(SD_CS)) {
      Serial.println("SDCard MOUNT FAIL");
  } else {
    uint32_t cardSize = SD.cardSize() / (1024 * 1024);
    String str = "SDCard Size: " + String(cardSize) + "MB";
    Serial.println(str);
  }
  Serial.println("===========================");
  File root;
  root = SD.open("/");
  printDirectory(root, 0);
  Serial.println("done!");

  pinMode(op_timing,OUTPUT);
  digitalWrite(op_timing,LOW);
}

//---------------------------------------------------------------------------
void loop(){
  receiveBT();    // Capture reception depuis BT
  // digitalWrite(op_timing,HIGH); // mesure temps execution
  receiveUART2(); // Capture reception depuis F9P UART2
  // digitalWrite(op_timing,LOW); // mesure temps execution

  if(millis() - last_timer1 > timer1){
    last_timer1 = millis();
    printEcran();
  }
}
//---------------------------------------------------------------------------
void printEcran(){
  if(lastfileline > 0){
    // affichage info ligne correspondant dernier enregistrement du fichier courant
    Serial.println(LIGNE_SEPAR);
    Serial.println(LIGNE_SEPAR);
    Serial.print(lastData);
    int pos = lastData.indexOf(";");
    pos = lastData.indexOf(";",pos + 1);
    long double last_lat_double = lastData.substring(pos+1,lastData.indexOf(";",pos+1)).toDouble();

    pos = lastData.indexOf(";",pos+1);
    long double last_lon_double = lastData.substring(pos+1,lastData.indexOf(";",pos+1)).toDouble();  
    
    double dist = distance(last_lat_double,last_lon_double,lat_double,lon_double);
    Serial.print("dist(m):"),Serial.print(dist);
    int cap = bearing(last_lat_double,last_lon_double,lat_double,lon_double);
    Serial.print(" bearing(°):"),Serial.println(cap);
    

    SerialBT.print("\x1B[2J\x1B[H"); // "\x1B[2J\x1B[H"
    SerialBT.println(LIGNE_SEPAR);
    SerialBT.println(LIGNE_SEPAR);
    SerialBT.print(lastData);
    SerialBT.print("dist(m):"),SerialBT.print(dist);
    SerialBT.print(" bearing(°):"),SerialBT.println(cap);
  }
  // data live
  bidons = LIGNE_SEPAR + fl;
  sprintf(bidonc,"UTC:%02d/%02d/%4d %02d:%02d:%02d ",day(),month(),year(),hour(),minute(), second());
  bidons += String(bidonc);
  sprintf(bidonc,"lat: %02.9f lon: %02.9f ",lat_double,lon_double);
  bidons += String(bidonc);
  // if(headVehValid){
    sprintf(bidonc," speed: %02.1f course: %03.0f ",speed,course);
  // } else {
  //   sprintf(bidonc," speed: %02.2f course: %s ",speed, "-");
  // }
  
  bidons += String(bidonc);
  bidons += ";"+ String(fix) + ";" + RTK_solution+ ";" + String(nbrsat);
  sprintf(bidonc,";hAcc: %3.3f\n",hAcc);
  bidons += String(bidonc);

  Serial.print(bidons);
  Serial.print("last PVT:"),Serial.print((millis()-last_UBX_PVT)/1);
  Serial.print(", last HPP:"),Serial.println((millis()-last_UBX_HPP)/1);
  if(lastfileline == 0){
    SerialBT.print("\x1B[2J\x1B[H"); // "\x1B[2J\x1B[H"
  }  
  SerialBT.print(bidons);    
}

//---------------------------------------------------------------------------
// interprete message arrivée BT
void interpretemessage(String demande) {
  
  if (demande.indexOf("save")>-1) {
    // Serial.print("BT demande:"),Serial.println(demande);
    // Verification file existe, sinon creer file
    if(!SPIFFS.exists(currentfile)){
      appendFile(SPIFFS, currentfile, "");
      Serial.print("nouveau fichier jour:"),Serial.println(currentfile);      
    }
    // save format demande = yyymmddhhmmss;pk410000;lat;lon;speed;course;Q;hAcc
    sprintf(bidonc,"%04d/%02d/%02d %02d:%02d:%02d;",year(),month(),day(),hour(),minute(), second());
    bidons = String(bidonc);
    // Serial.print(demande.indexOf("texte"));
    if(demande.indexOf("texte")==4){
      bidons += demande.substring(demande.indexOf("save")+10) + ";";
    } else {
      bidons += demande.substring(demande.indexOf("save")+5) + ";";
    }    
    sprintf(bidonc,"%02.9f;%02.9f;",lat_double,lon_double);
    bidons += String(bidonc);
    sprintf(bidonc,"%02.0f;%03.0f;",speed,course);
    bidons += String(bidonc);
    bidons += String(fix) + ";";
    sprintf(bidonc,"%3.3f\n",hAcc);
    bidons += String(bidonc);
    bidons.toCharArray(bidonc, bidons.length()+1);
    appendFile(SPIFFS, currentfile, bidonc); // Ajouter au fichier
    lastfileline ++;
    if (lastfileline > 0){ // si ligne nouvelle
      // enregistrement des données pour calcul position relative
      lastData = String(bidonc);
    }

    Serial.print("Save:"),Serial.println(bidonc);
    SerialBT.print("Save:"),SerialBT.println(bidonc);
    waitForBTKeyPress(2);
  } else if (demande.indexOf("dir")>-1){
    SPIFFS_dir();
    waitForBTKeyPress(5);
  } else if (demande.indexOf("openfile")>-1){    // Open file
    bidons = demande.substring(demande.indexOf("openfile")+9);
    bidons.toCharArray(bidonc, bidons.length()+1);
    // Serial.println(bidonc);
    readFile(SPIFFS,bidonc,-1);
    waitForBTKeyPress(10);

  } else if (demande.indexOf("opencurrent")>-1){ // Open current file
    readFile(SPIFFS,currentfile,-1);
    waitForBTKeyPress(10);
  } else if (demande.indexOf("deletefile")>-1){ // Delete file
    bidons = demande.substring(demande.indexOf("deletefile")+11);
    if(SPIFFS.remove(bidons)){
      Serial.print(bidons),Serial.println(" supprimé");
      SerialBT.print(bidons),Serial.println(" supprimé");
    }
    waitForBTKeyPress(2);
  } else if (demande.indexOf("definefile")>-1){ // Define file
    // file utilisée en lecture/ecriture, devient current file
    // verifier file exist    
    bidons = demande.substring(demande.indexOf("definefile")+11);
    if(SPIFFS.exists(bidons)){
      bidons.toCharArray(currentfile,bidons.length()+1);
      lastfileline = readFile(SPIFFS,currentfile,-1);
    } else {
      Serial.print("le fichier n'existe pas :"),Serial.println(bidons);
      SerialBT.print("le fichier n'existe pas :"),SerialBT.println(bidons);
    }
    waitForBTKeyPress(5);
  } else if (demande.indexOf("precedent")>-1){ // remonter une ligne
    if(lastfileline > 0) lastfileline --;
    readFile(SPIFFS,currentfile,lastfileline);
    Serial.print("lastfileline :"),Serial.println(lastfileline);
    SerialBT.print("lastfileline :"),SerialBT.println(lastfileline);
    waitForBTKeyPress(3);
  } else if (demande.indexOf("suivant")>-1){ // descendre une ligne
    int nbrline = readFile(SPIFFS,currentfile,-1);
    if(lastfileline < nbrline) lastfileline ++;
    readFile(SPIFFS,currentfile,lastfileline);
    Serial.print("lastfileline :"),Serial.println(lastfileline);
    SerialBT.print("lastfileline :"),SerialBT.println(lastfileline);
    waitForBTKeyPress(3);
  } else if (demande.indexOf("reset")>-1){ // Reset soft
    ESP.restart();
  } else if (demande.indexOf("toSD")>-1){ // copie file vers SD
    
  }
}
//---------------------------------------------------------------------------
// Reception data depuis F9P UART2
void receiveUART2(){
  int msgType = processGPS();
  if (msgType == MT_NAV_HPPOSLLH){
    // Calculate the latitude and longitude
    // High precision with double (same result as int and frac)
    lat_double  = ((double)ubxMessage.navPPosllh.lat)   / 10000000.0;
    lat_double += ((double)ubxMessage.navPPosllh.latHp) / 1000000000.0;
    lon_double  = ((double)ubxMessage.navPPosllh.lon)   / 10000000.0;
    lon_double += ((double)ubxMessage.navPPosllh.lonHp) / 1000000000.0;

    
    last_UBX_HPP = millis();
  }
  else if (msgType == MT_NAV_PVT){
    if (first ){// premier passage pour mise à l'heure et UTC valid
      if(ubxMessage.navPvt.valid.bits.validDate && ubxMessage.navPvt.valid.bits.validTime){// Time and Date valid
        setTime(int(ubxMessage.navPvt.UTC_hour),int(ubxMessage.navPvt.UTC_min),int(ubxMessage.navPvt.UTC_sec),int(ubxMessage.navPvt.UTC_day),int(ubxMessage.navPvt.UTC_month),int(ubxMessage.navPvt.UTC_year));
        sprintf(currentfile,"/%04d%02d%02d%02d%02d%02d",int(ubxMessage.navPvt.UTC_year),int(ubxMessage.navPvt.UTC_month),int(ubxMessage.navPvt.UTC_day),int(ubxMessage.navPvt.UTC_hour),int(ubxMessage.navPvt.UTC_min),int(ubxMessage.navPvt.UTC_sec));
        if(year() > 2022) first = false; // evite majheure fausse 20220501
      }
    }
    speed  = ubxMessage.navPvt.gSpeed;// mm/s
    speed *=(3600/1000000.0); // km/h
    course = ubxMessage.navPvt.headMot;// 1e-5 deg
    course /=100000.0;
    hAcc = ubxMessage.navPvt.hAcc/1000.0;
    headVehValid = ubxMessage.navPvt.flags.bits.headVehValid;
    fix = ubxMessage.navPvt.fixType;
    nbrsat = ubxMessage.navPvt.numSV;
    // Serial.print("head valid:"),Serial.println(headVehValid);
    // Serial.print("courseM:"),Serial.println(ubxMessage.navPvt.headMot);
    // Serial.print("courseV:"),Serial.println(ubxMessage.navPvt.headVeh);
    // Serial.print("vAcc pvt:"),Serial.println(ubxMessage.navPvt.vAcc);
    // Serial.print("nbr sat:"),Serial.println(ubxMessage.navPvt.numSV);
    
    switch (ubxMessage.navPvt.flags.bits.carrSoln){
      case 0:
        RTK_solution = "NRTK";// no RTK
        break;
      case 1:
        RTK_solution = "FRTK";// FRTK
        break;
      case 2:
        RTK_solution = "RTK ";// RTK 
        break;
      default:
        RTK_solution = "udef";
        break;
    }
    last_UBX_PVT = millis();
  }
}

//---------------------------------------------------------------------------
// Reception data depuis console BT
void receiveBT(){
  char   receivedChar;
  static String serialmessage = "";
  static bool   newData = false;

  if (SerialBT.available() > 0) {
    receivedChar = SerialBT.read();
    if (receivedChar != 10 && receivedChar != 13) {
      serialmessage += receivedChar;
    }
    else {
      newData = true;
      return;
    }
  }
  if (newData == true) {
    Serial.println(serialmessage);
    interpretemessage(serialmessage);
    newData = false;
    serialmessage = "";
  }
}

//---------------------------------------------------------------------------
void appendFile(fs::FS &fs, const char * path, const char * message) {
  // Serial.printf("Appending to file: %s\r\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    // Serial.println("- failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    // Serial.println("- message appended");
  } else {
    // Serial.println("- append failed");
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_dir() {
  if (SPIFFS_present) {
    File root = SPIFFS.open("/");
    if (root) {
      root.rewindDirectory();
      printDirectory("/", 0);
      root.close();
    }
  } else {
    Serial.println("SPIFFS not present");
    SerialBT.println("SPIFFS not present");
  };
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void printDirectory(const char * dirname, uint8_t levels) {
  File root = SPIFFS.open(dirname);
  if (!root) {
    return;
  }
  if (!root.isDirectory()) {
    return;
  }
  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.println(String(file.isDirectory() ? "Dir " : "File") + "|" + String(file.name()));
      SerialBT.println(String(file.isDirectory() ? "Dir " : "File") + "|" + String(file.name()));
      printDirectory(file.name(), levels - 1);
    }
    else
    {
      Serial.println(String(file.name()) + "|" + String(file.isDirectory() ? "Dir " : "File")+ "|" + file_size(file.size()));
      SerialBT.println(String(file.name()) + "|" + String(file.isDirectory() ? "Dir " : "File")+ "|" + file_size(file.size()));
    }
    file = root.openNextFile();
  }
  file.close();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
String file_size(int bytes) {
  String fsize = "";
  if (bytes < 1024)                      fsize = String(bytes) + "| B";
  else if (bytes < (1024 * 1024))        fsize = String(bytes / 1024.0, 3) + "| KB";
  else if (bytes < (1024 * 1024 * 1024)) fsize = String(bytes / 1024.0 / 1024.0, 3) + "| MB";
  else                                   fsize = String(bytes / 1024.0 / 1024.0 / 1024.0, 3) + "| GB";
  return fsize;
}

//---------------------------------------------------------------------------
// Read file, print lines and return nbr of lines, nbrline numero de la ligne désirée, -1=toutes les lignes
int readFile(fs::FS &fs, const char * path, int nbrline) {
  Serial.printf("Reading file: %s\r\n", path);
  SerialBT.printf("Reading file: %s\r\n", path);
  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println(F("- failed to open file for reading"));
    SerialBT.println(F("- failed to open file for reading"));
    return 0;
  }
  String buf = "";
  String tempbuf = "";
  int i = 0;
  while (file.available()){
    tempbuf = file.readStringUntil('\n');
    buf += tempbuf;
    buf += fl;
    i++;
    lastData = tempbuf + fl;
    if(i == nbrline) return i;
  }
  Serial.print(buf);
  SerialBT.print(buf);
  file.close();
  return i;
}
//---------------------------------------------------------------------------
// The last two bytes of the message is a checksum value, used to confirm that the received payload is valid.
// The procedure used to calculate this is given as pseudo-code in the uBlox manual.
void calcChecksum(unsigned char* CK, int msgSize) {
  memset(CK, 0, 2);
  for (int i = 0; i < msgSize; i++) {
    CK[0] += ((unsigned char*)(&ubxMessage))[i];
    CK[1] += CK[0];
  }
}

//---------------------------------------------------------------------------
// Compares the first two bytes of the ubxMessage struct with a specific message header.
// Returns true if the two bytes match.
boolean compareMsgHeader(const unsigned char* msgHeader) {
  unsigned char* ptr = (unsigned char*)(&ubxMessage);
  return ptr[0] == msgHeader[0] && ptr[1] == msgHeader[1];
}

//---------------------------------------------------------------------------
// Reads in bytes from the GPS module and checks to see if a valid message has been constructed.
// Returns the type of the message found if successful, or MT_NONE if no message was found.
// After a successful return the contents of the ubxMessage union will be valid, for the 
// message type that was found. Note that further calls to this function can invalidate the
// message content, so you must use the obtained values before calling this function again.
int processGPS() {
  // digitalWrite(op_timing,HIGH); // mesure temps execution
  static bool FlagNmea = false;
  String TrameNMEA;
  static int fpos = 0;
  static unsigned char checksum[2];
  
  static byte currentMsgType = MT_NONE;
  static int payloadSize = sizeof(UBXMessage);

  while ( Serial2.available() ) {
    // temps d'execution :
    // PVT 600µs, HPPOSLLH 300µs
    // repetition 38µs
    byte c = Serial2.read();    
    // Serial.write(c);
    if ( fpos < 2 ) {
      // For the first two bytes we are simply looking for a match with the UBX header bytes (0xB5,0x62)
      if ( c == UBX_HEADER[fpos] ){
        fpos++;
      }
      else{
        fpos = 0; // Reset to beginning state.
      }
    }
    else {
      // If we come here then fpos >= 2, which means we have found a match with the UBX_HEADER
      // and we are now reading in the bytes that make up the payload.
      
      // Place the incoming byte into the ubxMessage struct. The position is fpos-2 because
      // the struct does not include the initial two-byte header (UBX_HEADER).
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&ubxMessage))[fpos-2] = c;

      fpos++;
      
      if ( fpos == 4 ) {
        // We have just received the second byte of the message type header, 
        // so now we can check to see what kind of message it is.
        if ( compareMsgHeader(NAV_HPPOSLLH_HEADER) ) {
          currentMsgType = MT_NAV_HPPOSLLH;
          payloadSize = sizeof(UBX_NAV_HPPOSLLH);
        }
        else if (compareMsgHeader(NAV_PVT_HEADER)){
          currentMsgType = MT_NAV_PVT;
          payloadSize = sizeof(UBX_NAV_PVT);
        }
        else {
          // unknown message type, bail
          fpos = 0;
          continue;
        }
      }

      if ( fpos == (payloadSize+2) ) {
        // All payload bytes have now been received, so we can calculate the 
        // expected checksum value to compare with the next two incoming bytes.
        calcChecksum(checksum, payloadSize);
      }
      else if ( fpos == (payloadSize+3) ) {
        // First byte after the payload, ie. first byte of the checksum.
        // Does it match the first byte of the checksum we calculated?
        if ( c != checksum[0] ) {
          // Checksum doesn't match, reset to beginning state and try again.
          fpos = 0; 
        }
      }
      else if ( fpos == (payloadSize+4) ) {
        // Second byte after the payload, ie. second byte of the checksum.
        // Does it match the second byte of the checksum we calculated?
        fpos = 0; // We will reset the state regardless of whether the checksum matches.
        if ( c == checksum[1] ) {
          // Checksum matches, we have a valid message.
          return currentMsgType; 
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        // We have now read more bytes than both the expected payload and checksum 
        // together, so something went wrong. Reset to beginning state and try again.
        fpos = 0;
      }
    }
  }
  // digitalWrite(op_timing,LOW); // mesure temps execution
  return MT_NONE;
}

//---------------------------------------------------------------------------
// Calcul distance entre 2 points GPS en m (Haversine)
// https://stackoverflow.com/questions/63916877/gps-distance-calculations-between-two-points-using-arduino
long double distance( long double lat1, long double long1, long double lat2, long double long2) {
  // Convert the latitudes and longitudes from degree to radians. 
  lat1  = toRadians(lat1); 
  long1 = toRadians(long1); 
  lat2  = toRadians(lat2); 
  long2 = toRadians(long2); 
    
  // Haversine Formula 
  long double dlong = long2 - long1; 
  long double dlat = lat2 - lat1; 

  long double ans = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlong / 2), 2);
  ans = 2 * asin(sqrt(ans)); 

  // Radius of Earth in Kilometers, R = 6371 
  // Use R = 3956 for miles 
  long double R = 6378137.0; //https://geodesie.ign.fr/contenu/fichiers/Distance_longitude_latitude.pdf
    
  // Calculate the result 
  ans = ans * R; 
  return ans; 
}
//---------------------------------------------------------------------------
// Utility function for converting degrees to radians 
long double toRadians(const long double degree) { 
  long double one_deg = (M_PI) / 180; 
  return (one_deg * degree); 
}

//---------------------------------------------------------------------------
// Calcul angle entre 2 points GPS en °
int bearing(long double lat1, long double long1, long double lat2, long double long2){
  lat1  = toRadians(lat1);
  long1 = toRadians(long1);
  lat2  = toRadians(lat2);
  long2 = toRadians(long2);
  long double dlong = long2 - long1;
  long double y = sin(dlong) * cos(lat2);
  long double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dlong);
  long double teta = atan2(y, x);
  long double brng = teta*180/M_PI + 360; // in degrees
  brng = int(brng) % 360; // in degrees
  return int(brng);

/*
  https://www.movable-type.co.uk/scripts/latlong.html
  Formula:	θ = atan2( sin Δλ ⋅ cos φ2 , cos φ1 ⋅ sin φ2 − sin φ1 ⋅ cos φ2 ⋅ cos Δλ )
  where	φ1,λ1 is the start point, φ2,λ2 the end point (Δλ is the difference in longitude)
  JavaScript:
  (all angles
  in radians)
  const y = Math.sin(λ2-λ1) * Math.cos(φ2);
  const x = Math.cos(φ1)*Math.sin(φ2) -
            Math.sin(φ1)*Math.cos(φ2)*Math.cos(λ2-λ1);
  const θ = Math.atan2(y, x);
  const brng = (θ*180/Math.PI + 360) % 360; // in degrees
*/
}

//---------------------------------------------------------------------------
// Print SD card Directory
void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry = dir.openNextFile();
    if (! entry) {
        // no more files
        break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
        Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
        Serial.println("/");
        printDirectory(entry, numTabs + 1);
    } else {
        // files have sizes, directories do not
        Serial.print("\t\t");
        Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
//---------------------------------------------------------------------------
// Attente appuie touche, timeout en s
void waitForBTKeyPress(int timeout) {
  int debut = millis();
  while (!SerialBT.available() && millis()- debut < (timeout*1000)) {
    // Do nothing
    delay(1);
  }

  while (SerialBT.available()) {
    SerialBT.read();
  }
}