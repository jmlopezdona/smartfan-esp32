/**
 * A BLE client example that is rich in capabilities.
 */

#include "BLEDevice.h"

unsigned char PWM = 23;  // Output to Opto Triac pin
unsigned char ZEROCROSS = 4;  // Output to Opto Triac pin
unsigned char dimming = 0;  // Dimming level (0-100)
boolean enable = false;

// for CSC data handling
int wheel_size_;
unsigned int curr_wheel_revs_;
unsigned int curr_crank_revs_;
double distance_;
double total_distance_;
double prev_wheel_event_;
double curr_wheel_event_;
double curr_crank_event_;
double prev_crank_event_;
unsigned int prev_wheel_revs_;
unsigned int prev_crank_revs_;
unsigned int exer_wheel_revs_;
unsigned int last_wheel_rounds_;
time_t last_wheel_event_;
time_t last_crank_event_;
double last_speed_;
int last_cadence_;
int max_wheel_time_;

struct cscflags
{
  uint8_t _wheel_revolutions_present:1;
  uint8_t _crank_revolutions_present:1;
  uint8_t _reserved:6;
};

#define MIN_SPEED_VALUE 2.5
#define MAX_CEVENT_TIME 4
#define MAX_EVENT_TIME  64.0
#define KSPEED_UNINIT -1
#define KCADENCE_UNINT -1

// The remote service we wish to connect to.
static BLEUUID serviceUUID("00001818-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("00002a5b-0000-1000-8000-00805f9b34fb");

static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;

static void initValues() {
  prev_wheel_event_ = 0;
  curr_wheel_event_ = 0;
  curr_crank_event_ = 0;
  prev_crank_event_ = 0;
  prev_wheel_revs_  = 0;
  curr_wheel_revs_  = 0;
  prev_crank_revs_  = 0;
  curr_crank_revs_  = 0;
  exer_wheel_revs_  = 0;
  last_wheel_event_ = 0;
  last_wheel_rounds_ = 0;
  last_crank_event_  = 0;
  last_speed_ = KSPEED_UNINIT;
  last_cadence_ = KCADENCE_UNINT;
  wheel_size_ = 2000; // default 2000 mm wheel size
  max_wheel_time_ = (int)((wheel_size_*0.001*1*3.6)/MIN_SPEED_VALUE);
}

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.print(length);
    Serial.print(" data: ");
    for (int i = 0; i < length; i++) {
      Serial.print(pData[i]);
    }
    Serial.println();

    cscflags flags={0};
    memcpy(&flags, pData, sizeof(flags));
    int offset(sizeof(flags));

    time_t abs_time = time(NULL);

    // if speed present can be combo sensor or speed sensor only
    if( flags._wheel_revolutions_present )
    {
        prev_wheel_revs_ = curr_wheel_revs_;

        memcpy(&curr_wheel_revs_,pData+offset,4);
        offset += 4;

        prev_wheel_event_ = curr_wheel_event_;

        uint16_t tmp_curr_wheel_event_(0);
        memcpy(&tmp_curr_wheel_event_,pData+offset,sizeof(tmp_curr_wheel_event_));
        curr_wheel_event_ = (double)tmp_curr_wheel_event_/1024.0;
        offset += sizeof(tmp_curr_wheel_event_);

        if( last_speed_ == KSPEED_UNINIT )
        {
          // skip first packet
          last_speed_ = 0;
          last_wheel_event_ = abs_time;
        }
        else
        {
          double event_diff = curr_wheel_event_;
          if( prev_wheel_event_ )
          {
            if( prev_wheel_event_ <= curr_wheel_event_ )
              event_diff = curr_wheel_event_ - prev_wheel_event_;
            else
            {
              // rollover
              event_diff = curr_wheel_event_ + ( ((double)0xFFFF/1024.0) - prev_wheel_event_);
            }
          }

          unsigned int wheel_rounds(curr_wheel_revs_ - prev_wheel_revs_);

          if( curr_wheel_revs_ < prev_wheel_revs_ )
          {
            prev_wheel_revs_ = curr_wheel_revs_;
            wheel_rounds = 0;
          }

          if( wheel_rounds > 0 )  last_wheel_rounds_ = wheel_rounds;

          exer_wheel_revs_ += wheel_rounds;
          double speed(0);
          if( (!event_diff || !wheel_rounds) && (abs_time-last_wheel_event_) < max_wheel_time_ )
          {
            speed = last_speed_;
            exer_wheel_revs_ += last_wheel_rounds_;
          }
          if( event_diff && wheel_rounds )
          {
            speed = ((((double)wheel_size_*0.001)*wheel_rounds)/event_diff)*3.6;
            last_wheel_event_ = abs_time;
          }

          last_speed_ = speed;
          distance_ = ((double)wheel_size_*0.001)*exer_wheel_revs_; // in meters
          total_distance_ = ((double)wheel_size_*0.001)*curr_wheel_revs_; // in meters, assumption that the wheel size has been the same 
          Serial.print("Speed: ");
          Serial.println(speed);
        }

    }
  
}

bool connectToServer(BLEAddress pAddress) {
    Serial.print("Forming a connection to ");
    Serial.println(pAddress.toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    // Connect to the remove BLE Server.
    pClient->connect(pAddress);
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    //BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    BLERemoteService* pRemoteService = pClient->getService("00001816-0000-1000-8000-00805f9b34fb");
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());

    pRemoteCharacteristic->registerForNotify(notifyCallback);
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) {
      Serial.print("Found our device!  address: "); 
      advertisedDevice.getScan()->stop();

      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;
    }
  }
};

void zero_crosss_int()  // function to be fired at the zero crossing to dim the light
{
  // Firing angle calculation : 1 full 50Hz wave =1/50=20ms 
  // Every zerocrossing : (50Hz)-> 10ms (1/2 Cycle) For 60Hz (1/2 Cycle) => 8.33ms 
  // 10ms=10000us

  enable = true;
  if (last_speed_ > 10) {
    dimming = 50;
  }
  else if (last_speed_ > 20) {
    dimming = 40;
  }
  else if (last_speed_ > 30) {
    dimming = 30;
  }
  else if (last_speed_ > 40) {
    dimming = 20;
  }
  else if (last_speed_ > 50) {
    dimming = 10;
  }
  else if (last_speed_ > 60) {
    dimming = 0;
  }
  else if (last_speed_ <= 10) {
    enable = false;
  }
  
  if (enable) {
    int dimtime = (100*dimming);    // For 60Hz =>65    
    delayMicroseconds(dimtime);    // Off cycle
    digitalWrite(PWM, HIGH);   // triac firing
    delayMicroseconds(20);         // triac On propogation delay (for 60Hz use 8.33)
    digitalWrite(PWM, LOW);    // triac Off
  }
}

void setup() {
  Serial.begin(115200);
    
  initValues();

  pinMode(PWM, OUTPUT);// Set AC Load pin as output
  pinMode(ZEROCROSS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ZEROCROSS), zero_crosss_int, RISING);

  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device. Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30); 
}

// This is the Arduino main loop function.
void loop() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect. Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      connected = true;
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }
}