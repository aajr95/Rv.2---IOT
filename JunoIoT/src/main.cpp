#include <Wire.h>
#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLECharacteristic.h>

BLECharacteristic *pCharacteristic;

bool deviceConnected = false;
uint8_t txValue[16];




//0x69 - grideye
//0x0E - grideye initialize
#define pirpin 25
#define pirbigpin 35
#define pir_activation 500
#define MAXTIMINGS  85
#define DHTPIN    33l
#define airquality_sensor_pin 32
#define pingPin 15
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};


bool pirsm = 0;
bool pirbg = 0;

float h;
float c;
float tempprev;
float prob_correction = 460;
float prob_base = 280;
int detectioncnt;
float duration;
int cm;
int pir;
float a;
uint8_t sep = 200;


uint8_t line[71] = {
  1, 2, 3, 4, 5, 6, 7, 8, 9,
  10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
  20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
  30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
  40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
  50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
  60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
  70,
};

uint8_t txValueRem[sizeof(line) % 16];
int dht11_dat[5] = { 0, 0, 0, 0, 0 };

void temp102(int tmpaddress){
unsigned char first, second;
int tempbytes;
float convertedtmp;
float correctedtmp;

Wire.beginTransmission(tmpaddress); //Say hi to the sensor.
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(tmpaddress, 2);
  Wire.endTransmission();

first = (Wire.read());
second = (Wire.read());

tempbytes = ((first) << 4);
tempbytes |= (second >> 4);

convertedtmp = tempbytes * 0.0625;
correctedtmp = convertedtmp - 5;
};


void i2ccomms(int address,int start){
Wire.beginTransmission(address);
Wire.write(start);  // address low byte
Wire.endTransmission();
Wire.requestFrom(0x69, 1);
}

void read_dht11_dat()
{
  uint8_t laststate = HIGH;
  uint8_t counter   = 0;
  uint8_t j   = 0, i;
  float f;

  dht11_dat[0] = dht11_dat[1] = dht11_dat[2] = dht11_dat[3] = dht11_dat[4] = 0;

  pinMode( DHTPIN, OUTPUT );
  digitalWrite( DHTPIN, LOW );
  delay( 18 );
  digitalWrite( DHTPIN, HIGH );
  delayMicroseconds( 40 );
  pinMode( DHTPIN, INPUT );

  for ( i = 0; i < MAXTIMINGS; i++ )
  {
    counter = 0;
    while ( digitalRead( DHTPIN ) == laststate )
    {
      counter++;
      delayMicroseconds( 1 );
      if ( counter == 255 )
      {
        break;
      }
    }
    laststate = digitalRead( DHTPIN );

    if ( counter == 255 )
      break;

    if ( (i >= 4) && (i % 2 == 0) )
    {
      dht11_dat[j / 8] <<= 1;
      if ( counter > 16 )
        dht11_dat[j / 8] |= 1;
      j++;
    }
  }

  if ( (j >= 40) &&
       (dht11_dat[4] == ( (dht11_dat[0] + dht11_dat[1] + dht11_dat[2] + dht11_dat[3]) & 0xFF) ) )
  {


  }

}

int pixels[8][8] ={
  {11, 12, 13, 14, 15, 16, 17, 18},
  {21, 22, 23, 24, 25, 26, 27, 28},
  {31, 32, 33, 34, 35, 36, 37, 38},
  {41, 42, 43, 44, 45, 46, 47, 48},
  {51, 52, 53, 54, 55, 56, 57, 58},
  {61, 62, 63, 64, 65, 66, 67, 68},
  {71, 72, 73, 74, 75, 76, 77, 78},
  {81, 82, 83, 84, 85, 86, 87, 88}

};

void setup() {
  pinMode(airquality_sensor_pin, INPUT);
  pinMode(pirpin, INPUT);
  delay(20);
  i2ccomms(0x69, 0x0e);
  delay(20);
  Wire.begin(14,27);
  Serial.begin(115200);

  // Create the BLE Device
BLEDevice::init("ESP");

// Create the BLE Server
BLEServer *pServer = BLEDevice::createServer();
pServer->setCallbacks(new MyServerCallbacks());

// Create the BLE Service
BLEService *pService = pServer->createService(SERVICE_UUID);

// Create a BLE Characteristic
pCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );

pCharacteristic->addDescriptor(new BLE2902());

BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                       CHARACTERISTIC_UUID_RX,
                                       BLECharacteristic::PROPERTY_WRITE
                                     );

pCharacteristic->setCallbacks(new MyCallbacks());

// Start the service
pService->start();

// Start advertising
pServer->getAdvertising()->start();
Serial.println("Waiting a client connection to notify...");


for(int i=0;i<64;i++){
Serial.print("ta");
Serial.print(i);
Serial.print(",");
}
Serial.print("h,");
Serial.print("c,");
Serial.print("air,");
Serial.print("dist,");
Serial.print("pirbg,");
Serial.print("pirsm");
Serial.println(":");
}
void loop() {


/*********THERMAL ARRAY**************/

  for (int i=0x00; i<=0x07; i++){
    for (int j=0x00; j<=0x07; j++){
        int pixel = j*2 + i * 0x08 + 0x80;
  i2ccomms(0x69, pixel);
  pixels[i][j] = Wire.read();
       }
     }//writes value for each thermal array pixel

/***************TMP******************/
temp102(0x00);

/***************AIRQUAL****************/
int airquality_value = analogRead(airquality_sensor_pin);

/***************PIR****************/
int pir_inst=analogRead(pirpin);
if (pir_inst > 1000){
  pirsm = 1;
  }
  else{
    pirsm = 0;
    }

/***************PIR****************/
pir_inst=analogRead(pirbigpin);
if (pir_inst > 1000){
  pirbg = 1;
  }
  else{
    pirbg = 0;
    }

if(pirsm == 0 && pirbg ==0){
  pir = 00;
}
else if(pirsm == 1 && pirbg ==0){
  pir = 10;
}
else if(pirsm == 0 && pirbg ==1){
  pir = 01;
}
else if(pirsm == 1 && pirbg ==1){
  pir = 11;
}

/*************Hum******************/
read_dht11_dat();

/************PING****************/
delay(29);
pinMode(pingPin, OUTPUT);
digitalWrite(pingPin, LOW);
delayMicroseconds(2);
digitalWrite(pingPin, HIGH);
delayMicroseconds(5);
digitalWrite(pingPin, LOW);
pinMode(pingPin, INPUT);
duration = pulseIn(pingPin, HIGH);

a = (duration / 29 / 2);

cm = a * 0.80634920634920634920634920634921;


/***********COMMS************/


for(int k = 0x00; k < 0x07; k++){
  for(int l = 0x00; l < 0x07; l++){
    line[k + 8*l] = pixels[k][l];
}};


line[64] = h;
line[65] = c;
line[66] = airquality_value;
line[67] = cm;
line[68] = pirsm;
line[69] = pirbg;
line[70] = 255;

  sep = line[70];
  Serial.print(" ");
  Serial.print(sep);
  Serial.print(" ");



  Serial.print(" ");
  Serial.println(sizeof(line));

for(int i = 0; i < 4 ; i++){
  for(int k = 0; k<16; k++){
    txValue[k] = line[(k + 16 * i)];
    }
    pCharacteristic->setValue(txValue, sizeof(txValue));
    pCharacteristic->notify();
  }
for(int i = 0; i < 7 ; i++){
  txValueRem[i] = line[i];
  }
  pCharacteristic->setValue(txValueRem, sizeof(txValue));
  pCharacteristic->notify();
}
