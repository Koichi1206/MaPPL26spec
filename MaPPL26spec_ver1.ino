//必要なライブラリを用意する
#include <Wire.h>                     // センサとI2C通信するためのライブラリ
#include <SPI.h>                      // SDカードとSPI通信するためのライブラリ
#include <SD.h>                       // SDカードの読み書きを行うためのライブラリ
#include <SparkFun_u-blox_GNSS_v3.h>  // u-blox製GPSモジュールを制御するライブラリ
#include <SparkFunLSM6DSO.h>          // 6軸センサを制御するライブラリ
#include "SparkFunBMP384.h"           // 高精度気圧センサを制御するライブラリ

//設定を決める
const int LED_PIN = 13;                            // 本体についている動作確認用LEDのピン番号
const int LOOP_RATE = 50;                          // 1秒間に50回データを取るように設定
unsigned long loopInterval = 1000000 / LOOP_RATE;  // 1回の処理にかける時間を計算
const int TRIG_PIN = 34;                           // 超音波センサの音波を出すピン番号
const int ECHO_PIN = 33;                           // 超音波センサの音波を受け取るピン番号

//オブジェクトを作るエリア
SFE_UBLOX_GNSS myGPS;  // GPSを管理するmyGPSを作成
LSM6DSO myIMU;         // 6軸センサを管理するmyIMUを作成
BMP384 myPressure;     // 気圧センサを管理するmyPressureを作成
//データを一時的に入れておく変数
File logFile;                     // SDカード内のファイルを操作するための変数
char fileName[32];                // ファイル名を入れておくための箱
unsigned long lastFlushTime = 0;  // 最後にSDカードにデータを保存した時刻を記録する変数
float groundPressure = 1013.25;   // 地面の気圧

//エラーが起きたときの処理
void test() {  // testという名前の関数を定義
  // while(1)は無限ループを作る。ここで処理を永遠に止める
  while (1) {
    digitalWrite(LED_PIN, HIGH);  // LEDピンに電気を流して点灯させる
    delay(100);                   // 100ミリ秒待機する
    digitalWrite(LED_PIN, LOW);   // LEDピンの電気を止めて消灯させる
    delay(100);                   // 100ミリ秒待機する
    // ここでループの先頭に戻り、点滅を繰り返す
  }
}

//電源オンで最初に1回だけ動く設定用の関数
void setup() {
  // 各ピンの役割を設定する
  pinMode(LED_PIN, OUTPUT);   // LEDピンは電気を出すので出力
  pinMode(TRIG_PIN, OUTPUT);  // 超音波発信ピンは信号を出すので出力
  pinMode(ECHO_PIN, INPUT);   // 超音波受信ピンは信号を受け取るので入力

  // パソコンとのシリアル通信を速度115200bpsで開始
  Serial.begin(115200);
  // センサとのI2C通信を開始
  Wire.begin();
  Wire.setClock(400000);  // 通信速度を高速に設定して処理落ちを防ぐ
  // 電源投入直後は不安定なため1秒（1000ms）待機する
  delay(1000);

  // SDカードの初期化
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD Card failed!");  // 初期化に失敗したら画面に表示
    test();                             // エラー関数へ飛び、LED点滅させて停止
  }
  // ファイル名の自動作成処理
  int n = 0;  // ファイル番号のカウンタを0から始める
  // まず最初の候補となるファイル名を作成する
  snprintf(fileName, sizeof(fileName), "FLIGHT%02d.CSV", n);
  // その名前のファイルが既にSDカードに存在するか確認し、ある場合はループする
  while (SD.exists(fileName)) {
    n++;                                                        // 番号を1つ増やす
    snprintf(fileName, sizeof(fileName), "FLIGHT%02d.CSV", n);  // 新しい番号でファイル名を作り直す
  }

  // 決定した名前でファイルを新規作成して書き込みモードで開く
  logFile = SD.open(fileName, FILE_WRITE);
  if (logFile) {
    // ファイルが開けたら、1行目にヘッダーを書き込む
    logFile.println("Time(ms),AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Pressure(Pa),AltBaro(m),Temp(C),Lat,Lon,GPSAlt(m),SIV,SonarDist(m)");
    logFile.flush();               // 書き込んだ内容をSDカードに保存する
    Serial.print("Logging to: ");  // 画面にログファイル名を表示する準備
    Serial.println(fileName);      // ファイル名を表示して改行
  } else {
    Serial.println("Error creating file!");  // ファイル作成に失敗した場合のメッセージ
    test();                                  // エラー関数へ飛び、停止
  }

  Serial.println("Initializing Sensors...");  // センサの初期化開始を画面に表示

  // GPSの初期化と接続確認
  if (myGPS.begin() == false) Serial.println("GPS not detected.");  // 接続失敗してもログは取るため一旦進む
  // GPSを飛行機モードに設定
  myGPS.setDynamicModel(DYN_MODEL_AIRBORNE1g);
  // 通信方式を効率の良いUBXバイナリ形式に変更
  myGPS.setI2COutput(COM_TYPE_UBX);

  // 6軸センサの初期化と接続確認
  if (myIMU.begin() == false) {
    Serial.println("IMU not detected.");  // 接続できない場合は姿勢がわからないので停止
    test();                               // エラー関数へ
  }
  myIMU.initialize(BASIC_SETTINGS);  // センサを標準的な設定で起動する

  // 気圧センサの初期化
  // まずアドレス0x77で接続を試みる
  if (myPressure.beginI2C(0x77) != BMP3_OK) {
    // 失敗したらアドレス0x76で再試行する
    if (myPressure.beginI2C(0x76) != BMP3_OK) {
      Serial.println("BMP384 not detected.");  // どちらもダメならエラー表示
      test();                                  // 停止
    }
  }

  // 高度の0mリセット
  Serial.println("Calibrating Ground Pressure...");  // キャリブレーション開始を表示
  float sum = 0;                                     // 気圧の合計値を入れる変数
  // 10回測定して平均を取るためのループ
  for (int i = 0; i < 10; i++) {
    bmp3_data data;                   // データを受け取るための構造体を作成
    myPressure.getSensorData(&data);  // センサからデータを取得
    sum += data.pressure;             // 取得した気圧を合計値に足す
    delay(100);                       // 0.1秒待機
  }
  groundPressure = sum / 10.0;               // 合計を10で割って平均値を出し、基準気圧として保存
  Serial.print("Ground Pressure set to: ");  // 決定した基準気圧を表示する準備
  Serial.println(groundPressure);            // 基準気圧を表示

  Serial.println("Ready to fly!");  // 全準備完了を表示
}

//メイン処理
void loop() {
  unsigned long currentMicros = micros();   // 起動してからの時間を取得
  static unsigned long previousMicros = 0;  // 前回のループ実行時間を記憶しておく変数

  // 現在時刻と前回の差が、設定した間隔以上になったかチェック
  if (currentMicros - previousMicros >= loopInterval) {
    previousMicros = currentMicros;      // 次回のために実行時間を更新
    unsigned long timestamp = millis();  // ログ記録用に現在の時間（ミリ秒）を取得

    // 6軸センサからデータを取得
    float ax = myIMU.readFloatAccelX();  // X軸の加速度
    float ay = myIMU.readFloatAccelY();  // Y軸の加速度
    float az = myIMU.readFloatAccelZ();  // Z軸の加速度
    float gx = myIMU.readFloatGyroX();   // X軸の角速度
    float gy = myIMU.readFloatGyroY();   // Y軸の角速度
    float gz = myIMU.readFloatGyroZ();   // Z軸の角速度

    // 気圧・高度データの取得準備
    bmp3_data data;       // データを入れる箱
    float press = 0.0;    // 気圧用変数
    float temp = 0.0;     // 温度用変数
    float altBaro = 0.0;  // 計算した高度を入れる変数

    // 気圧センサから正常にデータが取れた場合
    if (myPressure.getSensorData(&data) == BMP3_OK) {
      press = data.pressure;    // 気圧データを代入
      temp = data.temperature;  // 温度データを代入
      // 測定した気圧と地上の気圧の差から高度を計算する式
      altBaro = 44330.0 * (1.0 - pow(press / groundPressure, 0.1903));
    }

    // 超音波センサで距離測定
    float sonarDist = -1.0;  // 取得失敗時は-1にするための初期値

    // トリガー信号を送る処理
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);  // ピンをLOWにしてほんの少し待機
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);        // 10μsだけHIGHにしてパルスを送る
    digitalWrite(TRIG_PIN, LOW);  // すぐにLOWに戻す

    // エコーが返ってくるまでの時間を計測する
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);
    // 時間が0より大きい場合
    if (duration > 0) {
      // 音速340m/sを使って時間を距離[m]に変換する計算式
      sonarDist = (duration * 0.034 / 2.0) / 100.0;
    }

    // GPSデータの取得
    double lat = myGPS.getLatitude() / 10000000.0;    // 緯度を生データから度単位に変換
    double lon = myGPS.getLongitude() / 10000000.0;   // 経度を生データから度単位に変換
    double gpsAlt = myGPS.getAltitudeMSL() / 1000.0;  // 海抜高度をmmからmに変換
    byte siv = myGPS.getSIV();                        // 現在捕捉している衛星の数を取得

    // SDカードへのデータ書き込み
    if (logFile) {
      logFile.print(timestamp);
      logFile.print(",");  // 時刻
      logFile.print(ax, 3);
      logFile.print(",");  // 加速度X
      logFile.print(ay, 3);
      logFile.print(",");  // 加速度Y
      logFile.print(az, 3);
      logFile.print(",");  // 加速度Z
      logFile.print(gx, 3);
      logFile.print(",");  // ジャイロX
      logFile.print(gy, 3);
      logFile.print(",");  // ジャイロY
      logFile.print(gz, 3);
      logFile.print(",");  // ジャイロZ
      logFile.print(press);
      logFile.print(",");  // 気圧
      logFile.print(altBaro);
      logFile.print(",");  // 気圧高度
      logFile.print(temp);
      logFile.print(",");  // 温度
      logFile.print(lat, 9);
      logFile.print(",");  // 緯度
      logFile.print(lon, 9);
      logFile.print(",");  // 経度
      logFile.print(gpsAlt, 2);
      logFile.print(",");  // GPS高度
      logFile.print(siv);
      logFile.print(",");          // 衛星数
      logFile.println(sonarDist);  // 超音波距離

      // 1秒経過するごとにデータをSDカードに保存し、LEDを反転させる
      if (millis() - lastFlushTime > 1000) {
        logFile.flush();                               // データ保存確定
        lastFlushTime = millis();                      // 保存時間を更新
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // LEDの状態を反転
      }
    }

    // パソコン画面への表示（デバッグ用）緯度・経度・衛星数・2種類の高度をまとめて表示する
    Serial.print("Loc: ");  // 場所ラベル
    Serial.print(lat, 7);   // 緯度を表示
    Serial.print(", ");
    Serial.print(lon, 7);  // 経度を表示
    Serial.print(" | Sats: ");
    Serial.print(siv);  // 衛星数を表示
    Serial.print(" | Alt: ");
    Serial.print(altBaro, 2);  // 気圧高度を表示
    Serial.print("m");
    Serial.print(" | Dist: ");
    Serial.println(sonarDist, 2);  // 超音波距離を表示
  }
}