/*
Program to make a high frequency (~1kHz) measurement of vibration (acceleration), filter it using an infinite impulse filter and report it out by a BLE connection. 
The intended use is to measure cyclist hand-arm vibration; the filter values correspond to the British Standard BS6841 for ocupational exposure

The code was designed to run on a dual core Esp32 with an ADXL345 accelerometer and prototyped on the Esp32 devkit v1. Run BLE server on core 0 and accelerometer measurement on core 1
The Arduino library is precompiled.  The sdkconfig file used for the compilation pins bluedroid to core 0.  The static and dynamic memory requirements are 76% and 11% respectively

The BLE server uses Neil Kolban's library;  Accerometer measurement uses a modified version of the SparkADXL345 library

The measurements are reported as RR values of a Heart Rate Monitor service, which can then be logged using off-the-shelf mobile phone apps such as HR Log and RideWithGPS, 
the first logs RR values, the last logs heart rate and location.
https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.heart_rate_measurement.xml
The GATT chacteristics shows RR values are reported as 1LSB = 1/1024th msec.  Output values are multiplied by 1.024.  Clearly this is a cludge.

Output values (in order of transmission in data packet):
1) ahv - RMS frequency-weighted acceleration over integration period in cms^-2.  Integration period set by nMeasureADXL, e.g. 1s
2) Battery voltage in mV
3) a8 - Total exposure to vibration since last reset in mms^-2
4) ahvMax - maximum frequency-weighted acceleration for xyz axes for nMeasure in cms^-2
5) ahv - RMS frequency-weighted acceleration over integration period in cms-2 (repeat)

ahv is repeated:  the first value maps to the BPM field and can be read by any HRM app that accepts a 16 bit integer for BPM, e.g. RidewithGPS; the second value will appear in the list of RRs
There seem to be few android apps that log RRs; I only found HR Log
No timing data are reported; we use the timestamp from the BLE client.  Could add the timestamp to the data packet from the BLE server


Directive 2002/44/EC of European Parliament Council (2002) provides details of the minimum health and safety requirements
regarding exposure of workers to the risks arising from mechanical vibration.
The Directive defines exposure limit values for hand-arm vibration based upon a standardized eight hour reference period (simulating a working day).
The Directive defines the thresholds of hand-arm vibration exposure from which the employer has to control
(Exposure Action Value, EAV = 2.5 ms^-2 rms) and threshold exposure limits to which workers must not be subjected
(Exposure Limit Value, ELV = 5.0 ms^-2 rms)

https://www.napier.ac.uk/research-and-innovation/research-search/outputs/cyclist-exposure-to-hand-arm-vibration-and-pavement-surface-improvement-in-the-city-of
discusses use of bike vibration sensors to characterise roads in Edinburgh

The design of the digital filter is described in:
Industrial Health 2007, 45, 512–519
https://www.ncbi.nlm.nih.gov/pubmed/17878622

The double precision variables are only needed for the filter calculation; subsequent calculations of stats could be with single precision.

Scope on signalTestPin shows the duration of each measurement is 400msec with a 80MHz esp32 CPU clock frequency.

If ahv values are zero for timeBeforeSleep sec then the output ports are shutdown and the device enters deep sleep.  The device is woken from deep sleep by movement detected by the accelerometer

In deep sleep current consumption is 12.5 mA when the code is run on the development board Esp DevKit V1. In use current consumption is 50mA
The board has a USB-UART converter CP2102 which accounts for most of the deep sleep current; ~1mA goes to the power LED

The ADXL345 supports a data rate of up to 3.2KHz i.e. can measure frequencies of up to 1.6KHz.  The SPI clock frequency is set to 4MHz, high enough to support the equivalent rate of data transfer 6 x 8 x 3.2 kbit/s.  See datasheet
https://www.analog.com/en/products/adxl345.html#

It is set to maximum resolution of 13bit, one of which is the sign bit

The digital filter values are set to values calculated with an R script.  It would be straightforward to incorporate the calculation in this code.  To do!



*/




#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <driver/adc.h> // for ADC reading
#include <driver/rtc_io.h> // for use of RTC GPIO pins


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
#define BPM_CENTIMETRE (100.0) // BPM values are reported in cms-2
#define RR_CENTIMETRE (102.4) // RR values are converted 1LSB = 1/1024ms
#define RR_MILLIMETRE (1024)
#define SIMULATE 0 // used to simulate the accelerometer output in order to test the filter response 
#define SIZE_BUFF 49 // used for serial comms
#define USE_SER 0
#define TEST_PORT 1 // used to test timing of the core 1 measurement loop
//#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds used to wake from deep sleep */
#define TIME_TO_SLEEP  30        /* Time ESP32 will go to sleep (in seconds) */
#define SHUTDOWN 1 // enter deep sleep if no movement

RTC_DATA_ATTR int bootCount = 0; // logs number of times rebooted
// to use the data over reboot, store it into the RTC memory by defining a global variable with RTC_DATA_ATTR attribute





char data[SIZE_BUFF + 1] = {0}; /* Line buffer */
char temp[SIZE_BUFF + 1] = {0}; /* Temporary buffer */

// Buffer for data read from sensors.
#define BUFFER_SIZE 11
uint8_t buff[BUFFER_SIZE];

// pins
const int ledPin = 12;
const int adcPin = 13;
#if TEST_PORT
const int signalPin = 14;
const int gndPin = 27;
#endif
// RTC GPIO 4 is used as EXT0 to wakeup ESP32 from deep sleep


//initialise vspi with default pins
//SCLK = 18, MISO = 19, MOSI = 23, SS = 5
const uint32_t spiFrequency = 4000000;
const uint32_t maxRuns = 100;

ADXL345 adxl = ADXL345(SS, spiFrequency);           // USE FOR SPI COMMUNICATION

//configure accelerometer data capture
const uint32_t nOmit = 128;    // no of early measurements to omit to allow IIR filter to stabilise

//configure timer
const uint32_t tickFrequency = 1000;           // the filter coefficients are determined for a particular sampling frequency fs
// IIR filter not stable for fs > 1250Hz
const double tickPeriod = 1. / (double)tickFrequency;  // used in simulation
const double time0 = 28800.0;                    // the reference duration of eight hours (28,800s)
volatile bool tick = false;
volatile bool shutDown = false; // flag passed from core 1 to core 0 in shutdown process



// reporting
const uint32_t nMeasureADXL = 1000; // number of ticks before pass to BLE server
const uint32_t nMeasureADC = 30000; // number of ticks before pass to BLE server
volatile uint16_t ahvInteger; // exposure data
volatile uint16_t ahvIntegerScaled; // exposure data reported as RR value
volatile uint16_t a8RideInteger; // cumulative exposure data
volatile uint16_t maxAhvInteger; // max exposure
volatile uint32_t batteryVoltage; // battery voltage measured with ADC (Vin not regulator voltage)

// sleep
const uint32_t timeBeforeSleep = 30000; // in millisec


////////////////////////////////////// BLE parameters //////////////////////////////////////////

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool ledOn = false;
const uint32_t timeLedOn = 100; //msec
uint32_t timeLedOff;
volatile bool newDataADXL = false;
volatile bool newDataADC = false;
//uint32_t dataPacket;
uint8_t dataPacket[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // change type of datapacket to allow more bytes to be sent


TaskHandle_t vibeTask;
TaskHandle_t commsTask;






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



////////////////////////////////////// onTimer //////////////////////////////////////////


void IRAM_ATTR onTimer(){
	// Increment the counter and set the time of ISR
	// These counters are not used in this program
	portENTER_CRITICAL_ISR(&timerMux);
	isrCounter++;
	lastIsrAt = millis();
	portEXIT_CRITICAL_ISR(&timerMux);
	
	
	// Give a semaphore that we can check in the loop
	xSemaphoreGiveFromISR(timerSemaphore, NULL);
	
	//BaseType_t xHigherPriorityTaskWoken;
	//xHigherPriorityTaskWoken = pdFALSE;	
	//xTaskNotifyFromISR(&vibeTask, 0, eNoAction, &xHigherPriorityTaskWoken);
	
	
	// It is safe to use digitalRead/Write here if you want to toggle an output
	//#if TEST_PORT
	//digitalWrite(signalPin, ISRmonitor);
	//ISRmonitor = !ISRmonitor;
	//#endif


}
////////////////////////////////////// initTime //////////////////////////////////////////


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
// for simulation of accelerometer input

void calcAcc(uint32_t dataCount, int16_t * buff )
{
	uint32_t runCount = dataCount / nMeasureADXL;
	//float frequency = pow(10.,(float)runCount / 33.3); //scans from 1 Hz to 1000 Hz in 100 runs
	float frequency = 10; // peak response of IIR filter in Hz
	float t = (float)dataCount * tickPeriod; //in sec
	uint16_t x = sin(TWOPI * frequency * t) * 8. * ADXL345_MG16G_DIVIDER; // simulate +/- 8g
	buff[0] = x;
	buff[1] = x;
	buff[2] = x;
}


////////////////////////////////////// readADC //////////////////////////////////////////
int readADC()
{
	int read_raw;
	adc2_config_channel_atten( ADC2_CHANNEL_4, ADC_ATTEN_11db );

	esp_err_t r = adc2_get_raw( ADC2_CHANNEL_4, ADC_WIDTH_12Bit, &read_raw);
	//#if USE_SER
	//if ( r == ESP_OK ) {
		//printf("%d, ", read_raw );
		//} else if ( r == ESP_ERR_TIMEOUT ) {
		//printf("ADC2 used by Wi-Fi.\n");
	//}
	//#endif
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

////////////////////////////////////// print_wakeup_reason //////////////////////////////////////////
/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
	esp_sleep_wakeup_cause_t wakeup_reason;

	wakeup_reason = esp_sleep_get_wakeup_cause();
	#if USE_SER


	switch(wakeup_reason)
	{
		case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
		case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
		case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
		case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
		case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
		default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
	}
	#endif
}
////////////////////////////////////// enterDeepSleep //////////////////////////////////////////

void enterDeepSleep()
{
	adxl.setActivityXYZ(1, 1, 1);            // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
	adxl.setActivityThreshold(4);          // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255) // 16 = 1g
	adxl.setActivityAC(1);
	adxl.setImportantInterruptMapping(0, 0, 0, 1, 0); // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);"
	// Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
	// This library may have a problem using INT2 pin. Default to INT1 pin.
	adxl.ActivityINT(1);
	#if USE_SER
	Serial.print("isInterruptEnabled = "); Serial.println(adxl.isInterruptEnabled(ADXL345_INT_ACTIVITY_BIT));
	#endif
	adxl.sleep();
	
	
	#if USE_SER
	Serial.println("accelerometer in sleep mode.  Move to wake.  Reading INT1 on GPIO ");
	#endif
	
	
	/*
	configure the wake up source as RTC GPIO 4; connect INT1 from ADXL345 to this pin
	*/
	
	rtc_gpio_init(GPIO_NUM_4);
	rtc_gpio_set_direction(GPIO_NUM_4, RTC_GPIO_MODE_INPUT_ONLY);
	
	//esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF); // power down slow memory on RTC module
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF); // power down fast memory on RTC module
	
	esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 1); //1 = High, 0 = Low

	//esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // added to test timer init problem

	/*
	Next we decide what all peripherals to shut down/keep on
	By default, ESP32 will automatically power down the peripherals
	not needed by the wakeup source, but if you want to be a poweruser
	this is for you. Read in detail at the API docs
	http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
	Left the line commented as an example of how to configure peripherals.
	The line below turns off all RTC peripherals in deep sleep.
	*/
	//esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
	//Serial.println("Configured all RTC Peripherals to be powered down in sleep");
	
	/*
	Now that we have setup a wake cause and if needed setup the
	peripherals state in deep sleep, we can now start going to
	deep sleep.
	In the case that no wake up sources were provided but deep
	sleep was started, it will sleep forever unless hardware
	reset occurs.
	*/
	#if USE_SER
	Serial.println("Going to sleep now");
	Serial.flush();
	#endif
	esp_deep_sleep_start();
}

////////////////////////////////////// flash //////////////////////////////////////////
void flash(int nTimes, int ledPinNo)
{
	for (int i = 0; i <= nTimes; i++)
	{
		digitalWrite(ledPinNo, HIGH);
		delay(50);
		digitalWrite(ledPinNo, LOW);
		delay(200);
	}
}

////////////////////////////////////// setup //////////////////////////////////////////

void setup(){
	// Set the CPU speed
	setCpuFrequencyMhz(80); //Set CPU clock frequency 80, 160, 240; 240 default

	#if USE_SER
	Serial.begin(115200);
	delay(5000);
	#endif
	pinMode(ledPin, OUTPUT);
	#if TEST_PORT
	pinMode(signalPin, OUTPUT); //used to measure speed of adxl345 loop
	pinMode(gndPin, OUTPUT); //used as ground for test probe
	digitalWrite(gndPin, LOW); //
	#endif
	
	#if USE_SER
	Serial.print("CPU frequency = "); Serial.println(getCpuFrequencyMhz());

	
	//Increment boot number and print it every reboot
	++bootCount;
	Serial.println("Boot number: " + String(bootCount));

	//Print the wakeup reason for ESP32
	print_wakeup_reason();
	#endif


	// configure accelerometer

	adxl.ActivityINT(0); // accelerometer state might be interrupt from sleep
	byte intSource = adxl.getInterruptSource(); // to ensure INT1 is low
	#if USE_SER
	Serial.print("intSource = "); Serial.println(intSource, BIN);
	/*
	D7      D6      D5      D4
	DATA_READY  SINGLE_TAP  DOUBLE_TAP  Activity
	D3      D2      D1      D0
	Inactivity  FREE_FALL Watermark Overrun
	*/
	#endif
	adxl.standBy();  // cycle measure bit in power control register in order to discard noisy measurements
	adxl.powerUp();

	adxl.setRangeSetting(16);           // Give the range settings
	// Accepted values are 2g, 4g, 8g or 16g
	// Higher Values = Wider Measurement Range
	// Lower Values = Greater Sensitivity

	adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1

	adxl.setFullResBit(1);        // 4mg/lsb is the resolution for all ranges
	
	adxl.setRate(1600);

	adxl.getRangeSetting(buff);
	
	
	#if USE_SER
	Serial.print("Full res bit ");  Serial.println(adxl.getFullResBit());
	Serial.print("Data rate ");  Serial.println(adxl.getRate());
	Serial.print("Range ");  Serial.println(buff[0]);
	Serial.println("Initialised accelerometer");
	#endif

	////////////////////////////////////// init timer //////////////////////////////////////////


	
	
	disableCore1WDT(); // source https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal.h#L76-L91
	// see epic thread https://github.com/espressif/arduino-esp32/issues/595
	// needed else main measurement task is blocked, so that it synchronises with alternate ticks
	
	//create a task that will be executed in the measureVibration() function, with priority 2 and executed on core 1
	xTaskCreatePinnedToCore(
	measureVibration,   /* Task function. */
	"vibeTask",     /* name of task. */
	10000,       /* Stack size of task */
	NULL,        /* parameter of the task */
	1,           /* priority of the task */
	&vibeTask,      /* Task handle to keep track of created task */
	1);          /* pin task to core 1 */

	
	//create a task that will be executed in the BLEComms() function, with priority 1 and executed on core 0
	xTaskCreatePinnedToCore(
	BLEComms,   /* Task function. */
	"commsTask",     /* name of task. */
	28800,       /* Stack size of task */
	NULL,        /* parameter of the task */
	2,           /* priority of the task */
	&commsTask,      /* Task handle to keep track of created task */
	0);          /* pin task to core 0 */

	
	flash(3, ledPin);
}

////////////////////////////////////// loop //////////////////////////////////////////
void loop()
{
	// Empty. Things are done in Tasks.
}

////////////////////////////////////// measureVibration //////////////////////////////////////////
//measureVibration: measures vibration every 1 ms
//
void measureVibration( void * pvParameters ){
	uint32_t nextCount = nMeasureADXL + nOmit; // count when measurement integration period ends 
	uint32_t nextADC = nMeasureADC; // count when battery voltage is measured
	uint32_t count = 0;
	uint32_t lastTimeNonZero = 0;
	double sum = 0; // sum of frequency-weighted acceleration for xyz axes for nMeasure
	double maxAhvSquared = 0; // maximum frequency-weighted acceleration for xyz axes for nMeasure
	double ahvRMS; // RMS frequency-weighted acceleration over integration period
	double sumRide = 0; // sum since last reset
	double a8Ride = 0; // total exposure to vibration since last reset

	initTime(1000000L / tickFrequency); // Configure timer - period in microseconds
	delay(50); // Added to allow timer to stabilise before entering main measurement loop
	// bug where get two sequential measurements when code operates after flash.  Cleared by cycling power off/on


	xSemaphoreTake(timerSemaphore, portMAX_DELAY);
	while (xSemaphoreTake(timerSemaphore, portMAX_DELAY) == pdFALSE) {;;} // wait for semaphore
	for (;;) {// wait til tick
		if (xSemaphoreTake(timerSemaphore, portMAX_DELAY) == pdTRUE) {
		
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
				ahvInteger = uint16_t(ahvRMS * BPM_CENTIMETRE + 0.5); //cm^-2; 0.5 is for rounding
				ahvIntegerScaled = uint16_t(ahvRMS * RR_CENTIMETRE + 0.5); //cm^-2
				maxAhvInteger = uint16_t(sqrtf(maxAhvSquared) * RR_CENTIMETRE + 0.5);
				lastTimeNonZero = count;
			}
			else // consider the measurement is noise
			{
				ahvInteger = 0;
				ahvIntegerScaled = 0;
				maxAhvInteger = 0;
				#if SHUTDOWN
				// shutdown is initiated by the measurement task which sets a flag, powers down the ADXL345 and terminates.  
				// On receipt of the flag the BLEcomms task shuts down:
				// the BLE transceiver, the measurement task and finally the esp32
				if ((count - lastTimeNonZero) > timeBeforeSleep) // shutdown if there is no movement for timeBeforeSleep msec
				{
					#if TEST_PORT
					digitalWrite(signalPin, LOW); // ensure output ports released before deep sleep
					pinMode(signalPin, INPUT);
					pinMode(gndPin, INPUT);
					#endif
					#if USE_SER
					Serial.println("Shutting down ADXL measurement task");
					#endif
					shutDown = true;
					vTaskDelete(NULL);     //Delete own task by passing NULL(task handle can also be used)
					while(true) {}; // do nothing while waiting for task to end
				}
				#endif
			}
			a8Ride = sqrtf(sumRide * tickPeriod / time0);
			a8RideInteger = uint16_t(a8Ride * RR_MILLIMETRE); // the result is in mm^s-2
			newDataADXL = true; // flag to BLE comms task that new data are ready
			sum = 0;
			maxAhvSquared = 0;
			#if USE_SER
			sprintf(data, "Time = %i, ahvRMS = %e, a8Ride = %e\n", millis(), ahvRMS, a8Ride);
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

		#if TEST_PORT
		digitalWrite(signalPin, LOW);
		#endif

		}  // end of if semaphore
	}  // end of main for loop
}  // end of measureVibration task

////////////////////////////////////// BLEComms //////////////////////////////////////////
/*
Information about the BLE server

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
when the newdata flag is set by the measurement task.

*/


void BLEComms(void * pvParameters ) {
	
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
	#if USE_SER
	//delay(5000); // added by rjw so as to start serial client
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
	
	
	while(true)
	{
		
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
			dataPacket[9] = ahvIntegerScaled & 0x00ff;
			dataPacket[10] = ahvIntegerScaled >> 8;

			pCharacteristic->setValue(dataPacket, 11);
			pCharacteristic->notify();
			newDataADXL = false;
			digitalWrite(ledPin, HIGH);
			timeLedOff = millis() + timeLedOn; //msec
			ledOn = true;
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
		/*
		the measurement task sets a flag, shutdown, if there is no movement 
		the test for no movement could equally well be made in this task (BLEcomm)
		we deinit BLE and then enter deep sleep; the first step is probably not needed
		*/
		if (shutDown)
		{
			BLEDevice::deinit(true);
			flash(5, ledPin);
			pinMode(ledPin, INPUT);
			enterDeepSleep();
		}
		if (ledOn)
		{
			if (millis() > timeLedOff)
			{
				digitalWrite(ledPin, LOW);
				ledOn = false;
			}

		}
		vTaskDelay(2); // needed to avoid triggering error from task WDT
	} // end of main while loop
}
