#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <SparkFunLSM6DSO.h>
#include "SparkFunBMP384.h"

const int LED_PIN = 13;
const int LOOP_RATE = 50; 
unsigned long loopInterval = 1000000 / LOOP_RATE; 
SFE_UBLOX_GNSS myGPS;
LSM6DSO myIMU;
BMP384 myPressure;
File logFile;
char fileName[16];
void errorBlink();

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  //SDカード初期化
  Serial.print("Initializing SD card...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD Card initialization failed!");
    errorBlink();
  }
  Serial.println("OK.");

  //ファイル名決定
  int n = 0;
  snprintf(fileName, sizeof(fileName), "FLIGHT%02d.CSV", n);
  while (SD.exists(fileName)) {
    n++;
    snprintf(fileName, sizeof(fileName), "FLIGHT%02d.CSV", n);
  }
  
  //ヘッダー書き込み
  logFile = SD.open(fileName, FILE_WRITE);
  if (logFile) {
    logFile.println("Time(ms),AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Pressure(Pa),Altitude(m),Temp(C),Lat,Lon,GPSAlt,SIV");
    logFile.close(); 
    Serial.print("Logging to: ");
    Serial.println(fileName);
  } else {
    Serial.println("Error creating file!");
    errorBlink();
  }

  Serial.println("Initializing Sensors...");

  //GPS
  if (myGPS.begin() == false) {
    Serial.println("GPS not detected. Freezing.");
  }
  myGPS.setI2COutput(COM_TYPE_UBX); 

  //6軸
  if (myIMU.begin() == false) {
    Serial.println("LSM6DSO not detected. Freezing.");
    errorBlink();
  }
  myIMU.initialize(BASIC_SETTINGS);

  //気圧
  Serial.print("Checking BMP384... ");
  if (myPressure.beginI2C(0x77) != BMP3_OK) { 
      Serial.println("Failed at 0x77! Trying 0x76...");
      // ダメなら0x76も試す
      if (myPressure.beginI2C(0x76) != BMP3_OK) {
         Serial.println("BMP384 not detected. Freezing.");
         errorBlink();
      }
  }

  Serial.println("Ready to fly!");
}

void loop() {
  unsigned long currentMicros = micros();
  static unsigned long previousMicros = 0;

  if (currentMicros - previousMicros >= loopInterval) {
    previousMicros = currentMicros;

    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 

    unsigned long timestamp = millis();
    
    //データ読み取り
    
    // 6軸
    float ax = myIMU.readFloatAccelX();
    float ay = myIMU.readFloatAccelY();
    float az = myIMU.readFloatAccelZ();
    float gx = myIMU.readFloatGyroX();
    float gy = myIMU.readFloatGyroY();
    float gz = myIMU.readFloatGyroZ();

    // 気圧
    bmp3_data data;
    float press = 0.0;
    float temp = 0.0;
    float alt = 0.0;

    // データ取得を実行
    if (myPressure.getSensorData(&data) == BMP3_OK) {
        press = data.pressure;
        temp = data.temperature;
        // 簡易高度計算
        alt = 44330.0 * (1.0 - pow(press / 101325.0, 0.1903)); 
    } else {
        //読み取り失敗時
        Serial.println("BMP384 Read Error");
    }

    // GPS
    long lat = myGPS.getLatitude();
    long lon = myGPS.getLongitude();
    long gpsAlt = myGPS.getAltitude();
    byte siv = myGPS.getSIV(); 

    //データ保存
    logFile = SD.open(fileName, FILE_WRITE);
    if (logFile) {
      logFile.print(timestamp); logFile.print(",");
      
      logFile.print(ax, 3); logFile.print(",");
      logFile.print(ay, 3); logFile.print(",");
      logFile.print(az, 3); logFile.print(",");
      logFile.print(gx, 3); logFile.print(",");
      logFile.print(gy, 3); logFile.print(",");
      logFile.print(gz, 3); logFile.print(",");
      
      logFile.print(press); logFile.print(",");
      logFile.print(alt); logFile.print(",");
      logFile.print(temp); logFile.print(",");
      
      logFile.print(lat); logFile.print(",");
      logFile.print(lon); logFile.print(",");
      logFile.print(gpsAlt); logFile.print(",");
      logFile.println(siv); 
      
      logFile.close(); 
    }

    //デバッグ表示
    if (Serial) {
      Serial.print("Alt: "); Serial.print(alt);
      Serial.print(" m | GPS Sats: "); Serial.println(siv);
    }
  }
}

void errorBlink() {
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}