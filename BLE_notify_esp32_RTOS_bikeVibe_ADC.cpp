/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updated by chegewara

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
   And has a characteristic of: beb5483e-36e1-4688-b7f5-ea07361b26a8

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   A connect hander associated with the server starts a background task that performs notification
   every couple of seconds.

   rjw
   Run BLE server on core 1 and bikeVibe measurement on core 0
   the measurements will be available as an HRM service

   The static and dynamic memory requirements are 73% and 11% respectively
   
   A8 peak and A8 cumulative values are reported
   # Directive 2002/44/EC of European Parliament Council (2002) provides details of the minimum health and safety requirements
   # regarding exposure of workers to the risks arising from mechanical vibration.
   # The Directive defines exposure limit values for hand-arm vibration based upon a standardized eight hour reference period (simulating a working day).
   # The Directive defines the thresholds of hand-arm vibration exposure from which the employer has to control
   # (Exposure Action Value, EAV = 2.5 ms^-2 rms) and threshold exposure limits to which workers must not be subjected
   # (Exposure Limit Value, ELV = 5.0 ms^-2 rms)
   
   the double precision variables are only needed for the filter calculation; subsequent calculations of stats could be with single precision
   
   scope on testPin shows the core loop occasionally is delayed by 400 microsec but recovers its timing on the next tick
   
   no timing data are reported; we use the timestamp from the BLE client.  Could add the timestamp to the RR data packet
   
   
   

*/




#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <driver/adc.h> // for ADC reading



#include <SparkFun_ADXL345.h>         // SparkFun ADXL345 Library



//Beginning of Auto generated function prototypes by Atmel Studio
void IRAM_ATTR onTimer();
void initTime(uint32_t period);
void calibrate(double coeffs32, double calibratedValues334, int16_t measurements, uint8_t last);
void applyFilter(double filterCoeffs36, double filteredValues334, uint8_t lastValue);
//End of Auto generated function prototypes by Atmel Studio



////////////////////////////////////// bikeVibe parameters //////////////////////////////////////////

#define ADXL345_MG2G_MULTIPLIER (0.004)  // 4mg per lsb
#define ADXL345_MG2G_DIVIDER (250)  // 4mg per lsb
#define ADXL345_MG16G_MULTIPLIER (0.032)  // 32mg per lsb
#define ADXL345_MG16G_DIVIDER (31.5)  // 32mg per lsb
#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON              (1.6F)                  /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN               (275.0F)                /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
#define TWOPI (2. * PI)
#define METRES_TO_FEET (3.2808) // not used
#define RR_CENTIMETRE (100. / 1.024)
#define RR_MILLIMETRE (1000. / 1.024)
#define SIMULATE 0
#define SIZE_BUFF 49
#define USE_SER 1
#define TEST_PORT 0 // used to test timing of the core ADXL read loop
char data[SIZE_BUFF + 1] = {0}; /* Line buffer */
char temp[SIZE_BUFF + 1] = {0}; /* Temporary buffer */

// Buffer for data read from sensors.
#define BUFFER_SIZE 11
uint8_t buff[BUFFER_SIZE];

// pins
const int ledPin = 12;
const int adcPin = 13;
const int accelerometerInt1Pin = 4;
#if TEST_PORT
const int signalPin = 14;
const int gndPin = 27;
#endif

//initialise vspi with default pins
//SCLK = 18, MISO = 19, MOSI = 23, SS = 5
const uint32_t spiFrequency = 4000000;
const uint32_t maxRuns = 100;

ADXL345 adxl = ADXL345(SS, spiFrequency);           // USE FOR SPI COMMUNICATION

//configure accelerometer data capture
const uint32_t nOmit = 128;    // no of early measurements to omit to allow filter to stabilise

//configure timer
const uint32_t tickFrequency = 1000;           // 1 ms; the filter coefficients are determined for a particular sampling frequency
const double tickPeriod = 1. / (double)tickFrequency;  // used in simulation
const double time0 = 28800.0;                    // the reference duration of eight hours (28,800s)
volatile bool timeOut = false;
volatile bool tick = false;


// reporting
const uint32_t nMeasureADXL = 1000; // number of ticks before pass to BLE server
const uint32_t nMeasureADC = 30000; // number of ticks before pass to BLE server
volatile uint16_t ahvInteger; // exposure data
volatile uint16_t a8RideInteger; // cumulative exposure data
volatile uint16_t maxAhvInteger; // max exposure
volatile uint32_t batteryVoltage; // battery voltage measured with ADC (Vin not regulator voltage)



////////////////////////////////////// BLE parameters //////////////////////////////////////////

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
volatile bool newDataADXL = false;
volatile bool newDataADC = false;
//uint32_t dataPacket;
uint8_t dataPacket[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // change type of datapacket to allow more bytes to be sent


TaskHandle_t Task1;






////////////////////////////////////// BLE functions //////////////////////////////////////////


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

//#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
//#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
// rjw mod 12/12/19
#define SERVICE_UUID        "0000180d-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "00002a37-0000-1000-8000-00805f9b34fb"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

////////////////////////////////////// hw timer functions //////////////////////////////////////////


hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;



void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

void initTime(uint32_t period)
// check type for period
{
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, period, true);

  // Start an alarm
  timerAlarmEnable(timer);
}
////////////////////////////////////// bikeVibe functions //////////////////////////////////////////

double calibrationCoeffs[3][2] = {-10.87728, 26.16823, 8.805525, 27.00157, -26.74323, 25.53217}; // [x,y,z] (x - a0) / a1 to convert 10 bit integer to acceleration in ms-2
// measured for sum of 16 readings

double filterCoeffs[3][6] = {
  //{0.310349844,-0.567696069,0.260868419,0.016714604,0.001761097,-0.014953507}, {41.08403,46.5508,16.78048,26.10383,52.20766,26.10383}, {2.918487,-5.652972,2.74225,2.828427,-5.656854,2.828427} // fs = 900 Hz
  {0.276879253,-0.511128066,0.236814985,0.013462109,0.001283086,-0.012179023}, {10.221917,0.6697712,1.7615627,3.1633127,6.3266254,3.1633127}, {2.909304,-5.65371,2.750695,2.828427,-5.656854,2.828427}  // fs = 1000 Hz
//  {0.2277253333,-0.4261585181,0.1999171031,0.0092696133,0.0007419592,-0.0085276541}, {3.5184959,-5.52018664,2.27502596,0.06833381,0.13666761,0.06833381}, {2.895603,-5.654671,2.763435,2.828427,-5.656854,2.828427}   // fs = 1200 Hz
//  {0.2180439689,-0.4091495028,0.192418231,0.0085284104,0.0006563486,-0.0078720618}, {2.919330238,-5.65402299,2.74035527,0.00141563,0.002831259,0.00141563}, {2.892874,-5.654842,2.765993,2.828427,-5.656854,2.828427} // fs = 1250 Hz
//  {0.1798093454,-0.3410695507,0.1620194965,0.0058825744,0.0003796456,-0.0055029288}, {1.4815962,-3.9284474,5.9036649,0.8642034,1.7284069,0.8642034}, {2.881991,-5.655457,2.77626,2.828427,-5.656854,2.828427} // fs = 1500 Hz (not stable)
}; // [Hw,Hl,Hh][a0,a1,a2,b0,b1,b2]
// can scale these values by a fixed factor without change to the calculation

double filteredValues[3][3][4] = {
  
  
  {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}},
  {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}},
  {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}
  
};
// [rows][columns] [x,y,z][buffer index][aa,bb,cc,dd] HwHlHh converts successively from aa->dd

////////////////////////////////////// calibrate accelerometer measurements //////////////////////////////////////////
void calibrate(double coeffs[3][2], double calibratedValues[3][3][4], int16_t * measurements, uint8_t last) {
  uint8_t i;
  for (i = 0; i < 3; i++) {
    calibratedValues[i][last][0] = ((double)(measurements[i]) - coeffs[i][0]) / coeffs[i][1];
  }
}


////////////////////////////////////// applyFilter //////////////////////////////////////////
void applyFilter(const double filterCoeffs[3][6], double filteredValues[3][3][4], uint8_t lastValue) {
  uint8_t i, j, k;
  for (i = 0; i < 3; i++) {   // over x, y, z
    for (j = 0; j < 3; j++) { // over Hw, Hl, Hh
      double result = 0;
      uint8_t l = lastValue;
      for (k = 0; k < 3; k++) { // over cyclic buffer
        result += filterCoeffs[j][k + 3] * filteredValues[i][l][j];
        if (k != 0)
        result -= filterCoeffs[j][k] * filteredValues[i][l][j + 1];
        if (l == 0) {
          l = 2;
          } else {
          --l;
        }
      }
      filteredValues[i][lastValue][j + 1] = result / filterCoeffs[j][0];
    }
  }
}

////////////////////////////////////// calcAcc //////////////////////////////////////////

void calcAcc(uint32_t dataCount, int16_t * buff )
{
  //float frequency = pow(10.,(float)runCount / 33.3); //scans from 1 Hz to 1000 Hz in 100 runs
  float frequency = 10; // peak response in Hz 
  float t = (float)dataCount * tickPeriod; //in sec
  uint16_t x = sin(TWOPI * frequency * t) * 8. * ADXL345_MG16G_DIVIDER; // simulate +/- 8g
  buff[0] = x;
  buff[1] = x;
  buff[2] = x;
}



////////////////////////////////////// Task1code //////////////////////////////////////////



//Task1code: measures vibration every 1 ms
void Task1code( void * pvParameters ){
  //Serial.print("Task1 running on core ");
  //Serial.println(xPortGetCoreID());
  uint32_t nextCount = nMeasureADXL + nOmit; // determines when data available for BT comm
  uint32_t nextADC = nMeasureADC; // determines when data available for BT comm
  uint32_t count = 0;
  double sum = 0; // sum of frequency-weighted acceleration for xyz axes for nMeasure
  double maxAhvSquared = 0; // maximum frequency-weighted acceleration for xyz axes for nMeasure
  double sumRide = 0; // sum since last reset
  double ahvRMS; // RMS exposure over integration period
  double a8Ride = 0; // total exposure to vibration since last reset

  for (;;) {// wait til tick
    if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
      #if TEST_PORT
      digitalWrite(signalPin, HIGH);
      #endif
      uint8_t lastValue = (uint8_t)(count % 3); // index on filteredValues which is a cyclic buffer
      #if SIMULATE
      calcAcc(count, (int16_t *)buff);
      #else
      adxl.readAcc((int16_t *)buff);
      #endif //simulate
      calibrate(calibrationCoeffs, filteredValues, (int16_t *)buff, lastValue);
      if (count < 2) {
        for (uint8_t j = 0; j < 3; j++) {
          for (uint8_t k = 1; k < 4; k++) {
            filteredValues[j][count][k] = filteredValues[j][count][0];
          }
        }
        } else {
        applyFilter(filterCoeffs, filteredValues, lastValue);
      }
      ++count;
      // calculate rms values and ahvSquared
      
      // omit first few readings to allow infinite impulse filter to settle
      if (count > nOmit)
      {
        double result = 0; // temporary result to allow calculation of maximum ahv
        for (uint8_t i = 0; i < 3; i++) {
          result += (filteredValues[i][lastValue][3] * filteredValues[i][lastValue][3]);
        }
        sum += result;
        // measure max
        if (maxAhvSquared < result)
        {
          maxAhvSquared = result;
        }
        
      }

      if (count > nextCount)
      {
        nextCount = count + nMeasureADXL;
        // calculate contribution to daily exposure according to ISO5349
        ahvRMS = sqrtf(sum / (double)nMeasureADXL); // root mean square
        if (ahvRMS > 0.06)
        {
          sumRide += sum; // sum ahvi^2 for whole ride
          ahvInteger = uint16_t(ahvRMS * RR_CENTIMETRE + 0.5); //cm^-2; 0.5 is for rounding
          maxAhvInteger = uint16_t(sqrtf(maxAhvSquared) * RR_CENTIMETRE + 0.5);
        }
        else // consider the measurement is noise
        {
          ahvInteger = 0;
          maxAhvInteger = 0;
        }
        a8Ride = sqrtf(sumRide * tickPeriod / time0);
        a8RideInteger = uint16_t(a8Ride * RR_MILLIMETRE);
        // the result is in mm/s/s
        newDataADXL = true;
        sum = 0;
        maxAhvSquared = 0;
        #if USE_SER
        sprintf(data, "Count = %i, ahvRMS = %e, a8Ride = %e\n", count, ahvRMS, a8Ride);
        Serial.print(data);
        #endif
      }

      if (count > nextADC)
      {
        // measure battery voltage
        // probably need an average
        batteryVoltage = 0;
        for (uint16_t i = 0; i < 64; i++ )
        {
          batteryVoltage += (uint32_t)readADC();
        }
        nextADC = count + nMeasureADC;
        newDataADC = true;
        batteryVoltage *= 163L;
        batteryVoltage >>= 12L;
        batteryVoltage += 320L;
        // from calibration
        #if USE_SER
        sprintf(data, "\nBattery = %i V\n", batteryVoltage);
        Serial.print(data);
        #endif
      }

      
      
      // Could sleep til next tick
      #if TEST_PORT
      digitalWrite(signalPin, LOW);
      #endif

    }  // end of main for loop
  }
}  // end of task1


////////////////////////////////////// readADC //////////////////////////////////////////
  int readADC()
  {
    int read_raw;
    adc2_config_channel_atten( ADC2_CHANNEL_4, ADC_ATTEN_11db );

    esp_err_t r = adc2_get_raw( ADC2_CHANNEL_4, ADC_WIDTH_12Bit, &read_raw);
  #if USE_SER
    if ( r == ESP_OK ) {
      printf("%d, ", read_raw );
      } else if ( r == ESP_ERR_TIMEOUT ) {
      printf("ADC2 used by Wi-Fi.\n");
    }
  #endif
  return(read_raw);
  }

////////////////////////////////////// printPower //////////////////////////////////////////
void printPower(esp_power_level_t power)
   {
     #if USE_SER
     switch (power)   {
       case ESP_PWR_LVL_N12:
       Serial.println("Power set to N12");
       break;
       case ESP_PWR_LVL_N9:
       Serial.println("Power set to N9");
       break;
       case ESP_PWR_LVL_N6:
       Serial.println("Power set to N6");
       break;
       case ESP_PWR_LVL_N3:
       Serial.println("Power set to N3");
       break;
       case ESP_PWR_LVL_N0:
       Serial.println("Power set to N0");
       break;
       case ESP_PWR_LVL_P3:
       Serial.println("Power set to P3");
       break;
       case ESP_PWR_LVL_P6:
       Serial.println("Power set to P6");
       break;
       case ESP_PWR_LVL_P9:
       Serial.println("Power set to P9");
       break;
       default:
       Serial.println("default");
     }
     #endif
   }


////////////////////////////////////// setTimeInact //////////////////////////////////////////


void setTimeInact(int timeInactive) {
	timeInactive = constrain(timeInactive,0,255);
	byte _b = byte (timeInactive);
	writeTo(ADXL345_TIME_INACT, _b);

}
////////////////////////////////////// sleepADXL //////////////////////////////////////////


void sleepADXL() {
	//D7 D6 D5		D4			D3			D2		D1		D0
	//0  0  Link	AUTO_SLEEP	Measure		Sleep	Wakeup
	byte _b;
	readFrom(ADXL345_POWER_CTL, 1, &_b);
	_b |= b00000100;
	writeTo(ADXL345_POWER_CTL, _b);
}

////////////////////////////////////// powerUpADXL //////////////////////////////////////////


void powerUpADXL() {
	//D7 D6 D5		D4			D3			D2		D1		D0
	//0  0  Link	AUTO_SLEEP	Measure		Sleep	Wakeup
	writeTo(ADXL345_POWER_CTL, 8);	// Measure
}

////////////////////////////////////// standByADXL //////////////////////////////////////////


void standByADXL() {
	//D7 D6 D5		D4			D3			D2		D1		D0
	//0  0  Link	AUTO_SLEEP	Measure		Sleep	Wakeup
	writeTo(ADXL345_POWER_CTL, 0);	// Measure
}

////////////////////////////////////// standByADXL //////////////////////////////////////////


void setActivityAC(bool state) { 
	/*
	state = 0 is DC; 1 is AC
	In ac-coupled operation for activity detection, the acceleration
	value at the start of activity detection is taken as a reference
	value. New samples of acceleration are then compared to this
	reference value, and if the magnitude of the difference exceeds
	the THRESH_ACT value, the device triggers an activity interrupt.
	*/
	
	setRegisterBit(ADXL345_ACT_INACT_CTL, 7, state);
}
////////////////////////////////////// setup //////////////////////////////////////////


void setup(){
  #if USE_SER
  Serial.begin(115200);
  #endif
  pinMode(ledPin, OUTPUT);
  #if TEST_PORT
  pinMode(signalPin, OUTPUT); //used to measure speed of adxl345 loop
  pinMode(gndPin, OUTPUT); //used as ground for test probe
  pinMode(accelerometerInt1Pin, INPUT); //used to check for accelerometer wake-up
  digitalWrite(gndPin, LOW); // 
  #endif
  delay(5000);
  
  // Set the CPU speed
  setCpuFrequencyMhz(80); //Set CPU clock frequency 80, 160, 240; 240 default
  #if USE_SER
  Serial.println(getCpuFrequencyMhz());
  #endif

  

  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
  CHARACTERISTIC_UUID,
  BLECharacteristic::PROPERTY_READ   |
  BLECharacteristic::PROPERTY_WRITE  |
  BLECharacteristic::PROPERTY_NOTIFY |
  BLECharacteristic::PROPERTY_INDICATE
  );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  delay(5000); // added by rjw so as to start serial client
  #if USE_SER
  Serial.println("Waiting a client connection to notify...");
  #endif

//set advertising power   
   /*
    The power level can be one of:
    ESP_PWR_LVL_N12
    ESP_PWR_LVL_N9
    ESP_PWR_LVL_N6
    ESP_PWR_LVL_N3
    ESP_PWR_LVL_N0
    ESP_PWR_LVL_P3
    ESP_PWR_LVL_P6
    ESP_PWR_LVL_P9
*/
   if (esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,ESP_PWR_LVL_N0) == OK)
   {
     #if USE_SER
     Serial.println("Advertising power changed");
     #endif
   }
   esp_power_level_t powerAdv = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);
   printPower(powerAdv);
   
 
  adxl.powerOn();                     // Power on the ADXL345

  adxl.setRangeSetting(16);           // Give the range settings
  // Accepted values are 2g, 4g, 8g or 16g
  // Higher Values = Wider Measurement Range
  // Lower Values = Greater Sensitivity

  adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1

  adxl.setFullResBit(1);        // 4mg/lsb is the resolution for all ranges
    
  adxl.setRate(1600);

  adxl.getRangeSetting(buff);
  
  initTime(1000000L / tickFrequency); // Configure timer
  
  #if USE_SER
  Serial.print("Full res bit ");  Serial.println(adxl.getFullResBit());
  Serial.print("Data rate ");  Serial.println(adxl.getRate());
  Serial.print("Range ");  Serial.println(buff[0]);
  Serial.println("Initialised accelerometer");
  #endif

  
  
  adxl.setActivityXYZ(1, 1, 1);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(16);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255) // 16 = 1g
  setActivityAC(1);
  adxl.setImportantInterruptMapping(0, 0, 0, 1, 0);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);"
  // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
  // This library may have a problem using INT2 pin. Default to INT1 pin.
  adxl.ActivityINT(1);
  sleepADXL();

  
  #if USE_SER
  Serial.println("accelerometer in sleep mode.  Move to wake.  Reading INT1 on GPIO ");
  #endif
  
  while(digitalRead(accelerometerInt1Pin)) {;;} // do nothing
  
  byte intSource = adxl.getInterruptSource();

  #if USE_SER
  Serial.print("intSource = "); Serial.println(byte, BIN);
  Serial.print("accelerometerInt1Pin = "); Serial.println(digitalRead(accelerometerInt1Pin));
  #endif
  
  // enable measurement
  standByADXL();
  powerUpADXL();
	  
  



  
  delay (1000);
  
  disableCore0WDT(); // source https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal.h#L76-L91
  // see epic thread https://github.com/espressif/arduino-esp32/issues/595
  
  //create a task that will be executed in the Task1code() function, with priority 2 and executed on core 0
  xTaskCreatePinnedToCore(
  Task1code,   /* Task function. */
  "Task1",     /* name of task. */
  10000,       /* Stack size of task */
  NULL,        /* parameter of the task */
  2,           /* priority of the task */
  &Task1,      /* Task handle to keep track of created task */
  0);          /* pin task to core 0 */
  delay(1000);
  

  
}


////////////////////////////////////// main loop core 1 //////////////////////////////////////////

void loop() {
  // notify changed value
  if (deviceConnected && newDataADXL) {
    /*
    most OTS BLE Android apps only accept 1 byte HRM data, although the 0x2a37 characteristic also supports 2 bytes
    https://www.bluetooth.com/specifications/gatt/characteristics/
    send: instantaneous ahvi as one byte in BPM field
    cumulative ahvi as two bytes in energy field
    two byte instantaneous ahvi in 1st RR field
    two byte cumulative ahvi in 2nd RR field
    two byte battery voltage in 3rd RR field
    */
    
    dataPacket[0] = 0b00010111;
    dataPacket[1] = ahvInteger & 0x00ff;
    dataPacket[2] = ahvInteger >> 8;
    dataPacket[3] = batteryVoltage & 0x00ff;
    dataPacket[4] = batteryVoltage >> 8;
    dataPacket[5] = a8RideInteger & 0x00ff;
    dataPacket[6] = a8RideInteger >> 8;
    dataPacket[7] = maxAhvInteger & 0x00ff;
    dataPacket[8] = maxAhvInteger >> 8;
    dataPacket[9] = ahvInteger & 0x00ff;
    dataPacket[10] = ahvInteger >> 8;

    pCharacteristic->setValue(dataPacket, 11);
    pCharacteristic->notify();
    newDataADXL = false;
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
  #if USE_SER
    Serial.println("start advertising");
  #endif
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    
    if (esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_N0) == OK)
    {
      #if USE_SER
      Serial.println("Connecting power changed");
      #endif
    }
    esp_power_level_t powerConn = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_CONN_HDL0);
    printPower(powerConn);

    
    #if USE_SER
    Serial.println("connected");
    #endif
    oldDeviceConnected = deviceConnected;
  }
}
