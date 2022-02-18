#define VERSION "V2.1"
#define BANNER "Bus Monitor  Kyuho_Kim"
//#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

#define REBOOT reboot(180);
void reboot(int);

#define DEFAULT1 "{\"user\":1000,\"gps\":0,\"topic\":\"s2m\",\"device\":\"bus\",\"target\":\"busan\",\"ssid\":\"cookie2\",\"password\":\"0317137263\",\"secure\":\"no\",\"eap_login\":\"wifi\",\"eap_password\":\"wifi\",\"mqttserver\":\"damoa.io\"}"

/*
{"user":100005,"topic":"s2m","device":"bus","gps":0,"ssid":"","password":"","target":"seoul","secure":"no","eap_login":"wifi","eap_password":"wifi","mqttserver":"damoa.io"}
											0:내부용,1:외부용
																				seoul,busan.raspAP
																									secure,insecure
*/
//#define SHT1X   if not, BME680 or DHT21
//#define OLED
//#define GYRO  auto config
//#define DS18B20

#define dataP  19
#define clockP 18
#include <Wire.h>

#include <esp_task_wdt.h>
TaskHandle_t main_task;
TaskHandle_t update_task;

#include "WiFi.h"
#include <HTTPClient.h>
HTTPClient http;
#include <PubSubClient.h>
#include <HTTPUpdate.h>
WiFiClient espClient;
PubSubClient mqttClient(espClient);
const char* mqttUser = NULL;
const char* mqttPassword = NULL;


#include "screen.h"
Screen myscreen;

#include <QuickStats.h>
QuickStats stats;
class Median {
public:
	float get(int a[], int cnt) {
		float qbuf[100];
		for (int i=0; i<cnt; i++) qbuf[i] = float(a[i]);
		return get(qbuf, cnt);
	}

	float get(float a[], int cnt) {
		return stats.median(a, cnt);
	}
} mymedian;

class Led {
public:
	unsigned long mark=0;
	void begin() {
		pinMode(2, OUTPUT);
		digitalWrite(2, LOW);
	}
	void on() {
		digitalWrite(2, HIGH);
	}
	void off() {
		digitalWrite(2, LOW);
	}
	void on(int time) {
		mark = millis() + time;
		on();
	}
	void update() {
		if (mark && mark < millis()) {
			mark = 0;
			off();
		}
	}
} myled;

class Mhz14 
{
public:
	int readings[100];
	int numreadings;
	int ppm;

	void begin(void) {
		//const int co2_tx = 16;  //ESP32 16
		//const int co2_rx = 17;  //ESP32 17
		numreadings = 0;
		Serial2.begin(9600);
	}
	
	void measure() {
		while (Serial2.available()) Serial2.read();
		Serial2.write(cmd, 9);
		//Serial.printf("\nco2 sent cmd");
	}
	
	void calibrate() {
		Serial2.write(calibrate_cmd, 9);
		Serial.printf("\nco2 sent calibration");
	}
	
	int append(int ppm) {
		if (numreadings==100) {
			for(int i=0;i<99;i++) readings[i]=readings[i+1];
			numreadings--;
		}
			
		readings[numreadings] = ppm;
		return numreadings++;
	}
	
	int update(void) {
		int c;
		if (!(c=Serial2.available())) return -2;
		
		uint8_t response[9];
		//Serial.printf(" > available %d",c);
		c=Serial2.readBytes(response, 9);
		if (c == 9) {
			int responseHigh = (int) response[2];
			int responseLow = (int) response[3];
			ppm = (256 * responseHigh) + responseLow;
			byte chksum = 0;
			for (int i=1;i<=8;i++) chksum += response[i];
			if (chksum) {
				Serial.println(" > co2 checksum error:");
				for (int i=0;i<9;i++) Serial.printf(" %02x", response[i]);
				Serial2.flush();
				return -1;
			}
			Serial.printf("\n > read co2: %dppm@%d",ppm,append(ppm));
			return ppm;
		} else {
			Serial.printf(" > read(9) fail got %dB",c);
			Serial2.flush();
			return -1;
		}
	}
private:
	const uint8_t cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
	const uint8_t calibrate_cmd[9] = {0xFF,0x01,0x87,0x00,0x00,0x00,0x00,0x00,0x78};
	byte header[2] = { 0xFF, 0x86 };
	byte chksum;
} myco2;

#include <TinyGPS.h>
TinyGPS gps;
class GPS {
public:
	float lat=0.0;
	float lon=0.0;
	bool first=true;
	void begin() {
		//const int gps_tx = 16;  //ESP32 16
		//const int gps_rx = 17;  //ESP32 17
		Serial2.begin(9600);
	}
	void update() {
		unsigned long age;
		while (Serial2.available())
			gps.encode(Serial2.read());
		
		if (gps.satellites() != TinyGPS::GPS_INVALID_SATELLITES) {
			gps.f_get_position(&lat, &lon, &age);
			if(first) {
				Serial.printf("\n Found GPS");
				first=false;
			}
		}
	}
} mygps;

HardwareSerial dustport(1);

class PmsA003 {
public:
	int pm25_readings[100];
	int pm10_readings[100];
	int numreadings;
	int pm25, pm10;

	void begin(void) {
		const int dust_tx = 5;  //ESP32 5
		numreadings = 0;
		dustport.begin(9600, SERIAL_8N1, dust_tx, 4);
		while(dustport.available()) dustport.read();
	}
	
	int append(int pm25, int pm10) {
		if (numreadings==100) {
			for(int i=0;i<99;i++) pm25_readings[i]=pm25_readings[i+1];
			for(int i=0;i<99;i++) pm10_readings[i]=pm10_readings[i+1];
			numreadings--;
		}
			
		pm25_readings[numreadings] = pm25;
		pm10_readings[numreadings] = pm10;
		return numreadings++;
	}
	
	int update(void) {
		int c;
		if (!(c=dustport.available())) return -2;
		bool first=true;
		while (dustport.available()>0 && dustport.peek() != 0x42) {
			if (first) {
				first=false;
				Serial.printf("\n  dust sensor: finding start byte...");
			}
			Serial.printf(" %02X", dustport.read());
		}
		
		uint8_t r[34];
		//Serial.printf(" > available %d",c);
		c=dustport.readBytes(r, 32);
		if (c == 32 && r[0]==header[0] && r[1]==header[1]) {
			chksum=r[0]+r[1];
			for(int i=0;i<14;i++) {
				//Serial.printf("%s %d", i==0?"\n" :"", r[i*2+2]*256+r[i*2+3]);
				chksum += r[i*2+2] + r[i*2+3];
			}
			if (chksum != r[30]*256+r[31]) {
				Serial.printf(" > dust checksum error:");
				for (int i=0;i<32;i++) Serial.printf("%s %02x", i==0?"\n":"", r[i]);
				dustport.flush();
				return -1;
			}
			pm25=r[12]*256+r[13];
			pm10=r[14]*256+r[15];
			Serial.printf("\n > read dust: %d,%d@%d",pm25,pm10,append(pm25,pm10));
			return 0;
		} else {
			Serial.printf(" > read(32) fail got %dB",c);
			for (int i=0;i<c;i++) Serial.printf("%s %02x", i==0?"\n":"", r[i]);
			dustport.flush();
			
			return -1;
		}
		
		
	}
private:
	byte header[2] = { 0x42, 0x4d };
	int chksum;
} mydust;


#include <MPU6050_light.h>
MPU6050 mpu(Wire);
class Gyro {
public:
	float Mvalues[6] ={0,0,0,0,0,0};
	float v[6]={0,0,0,0,0,0};
	float oldv[6]={0,0,0,0,0,0};
	int ready=0;
	
	void begin() {
		if (!mpu.begin()) {
			Serial.printf("\n found MPU6050");
			ready=1;
			mpu.calcOffsets(true,true); 
		} else
			Serial.printf("\n no MPU6050");
	}
	void update() {
		if (!ready) return;
		mpu.update();
		
		int k=0;
		Mvalues[k++]+=(v[0]=abs(mpu.getAccX()));
		Mvalues[k++]+=(v[1]=abs(mpu.getAccY()));
		Mvalues[k++]+=(v[2]=abs(mpu.getAccZ()));
		Mvalues[k++]+=(v[3]=abs(mpu.getGyroX()));
		Mvalues[k++]+=(v[4]=abs(mpu.getGyroY()));
		Mvalues[k++]+=(v[5]=abs(mpu.getGyroZ()));
		Mvalues[k++] = mpu.getTemp();
		
		bool got=false;
		for (int i=0; i<6; i++) 
			if (abs(oldv[i]-v[i])> oldv[i]*20) got=true;
		if (got)
			for (int i=0; i<6; i++)
				Serial.printf(" %s%.2f(%.0f%%)", i==0?"\n":"", v[i], 100*abs(oldv[i]-v[i])/oldv[i]);
		for (int i=0; i<6; i++)
			oldv[i]=v[i];

		/*
		Mvalues[k++] = mpu.getAccAngleX();	//6
		Mvalues[k++] = mpu.getAccAngleY();	//7
		Mvalues[k++] = mpu.getGyroAngleX();	//8
		Mvalues[k++] = mpu.getGyroAngleY();	//9
		Mvalues[k++] = mpu.getGyroAngleZ();	//10
		Mvalues[k++] = mpu.getAngleX();	//11
		Mvalues[k++] = mpu.getAngleY();	//12
		Mvalues[k++] = mpu.getAngleZ();	//13
		
		*/
	}
} mygyro;

#ifndef SHT1X

#include "bsec.h"
Bsec iaqSensor;

class BME680 {
public:
	// 0:temp 1:humid 2:gas 3:pressure 4:alt
	float readings[11][100];
	float reading[11];
	int numreadings=0;
	int ready=0;
	bsec_virtual_sensor_t sensorList[10] = {
		BSEC_OUTPUT_RAW_TEMPERATURE,
		BSEC_OUTPUT_RAW_PRESSURE,
		BSEC_OUTPUT_RAW_HUMIDITY,
		BSEC_OUTPUT_RAW_GAS,
		BSEC_OUTPUT_IAQ,
		BSEC_OUTPUT_STATIC_IAQ,
		BSEC_OUTPUT_CO2_EQUIVALENT,
		BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
		BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
		BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
	};
	String title[11]={"raw temperature [°C]", "pressure [hPa]", "raw relative humidity [%]", "gas [Ohm]", "IAQ", "IAQ accuracy", "temperature [°C]", "relative humidity [%]", "Static IAQ", "CO2 equivalent", "breath VOC equivalent"};
	
	void begin() {
		iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
		String output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
		//Serial.print(output);
		checkIaqSensorStatus();
		if (!ready) {
			Serial.printf("\n no BME680");
			return;
		}
		
		iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
	}
	
	void checkIaqSensorStatus(void) {
		String output;
	  if (iaqSensor.status != BSEC_OK) {
		if (iaqSensor.status < BSEC_OK) {
		  output = "\n BSEC error code : " + String(iaqSensor.status);
		  Serial.println(output);
		} else {
		  output = "\n BSEC warning code : " + String(iaqSensor.status);
		  Serial.print(output);
		}
		return;
	  }

	  if (iaqSensor.bme680Status != BME680_OK) {
		if (iaqSensor.bme680Status < BME680_OK) {
		  output = "\n BME680 error code : " + String(iaqSensor.bme680Status);
		  //Serial.print(output);
		} else {
		  output = "\n BME680 warning code : " + String(iaqSensor.bme680Status);
		  Serial.print(output);
		}
		return;
	  }
	  Serial.printf("\n found bme680");
	  ready=1;
	}

	int append(float reading[]) {
		if (numreadings==100) {
			for(int i=0;i<99;i++)
				for (int j=0;j<11;j++) readings[j][i]=readings[j][i+1];
			numreadings--;
		}
			
		for (int j=0;j<11;j++) readings[j][numreadings]=reading[j];
		return numreadings++;
	}

	void update(void) {
		if (!ready) return;
		if (!iaqSensor.run()) {
			Serial.printf("\n bme680 is not ready");
			return;
		}

		// "T+H+I++CVBPG"
		reading[0]=iaqSensor.temperature;
		reading[1]=iaqSensor.rawTemperature;
		reading[2]=iaqSensor.humidity;
		reading[3]=iaqSensor.rawHumidity;
		reading[4]=iaqSensor.staticIaq;
		reading[5]=iaqSensor.iaq;
		reading[6]=iaqSensor.iaqAccuracy;
		reading[7]=iaqSensor.co2Equivalent;
		reading[8]=iaqSensor.breathVocEquivalent;
		reading[9]=iaqSensor.pressure/100;
		reading[10]=iaqSensor.gasResistance/1000;



		Serial.printf("\n > T+H+I++CVPG:");
		for (int i=0; i<11;i++) Serial.printf(" %.f", reading[i]);
		Serial.printf("@%d", append(reading));
	}
} mybme;

#include <HTU21D.h>
HTU21D htu21(HTU21D_RES_RH12_TEMP14);
class HTU {
public:
	float temp_readings[100];
	float humid_readings[100];
	int numreadings;
	float temp, humid;
	int ready=0;
	
	void begin() {
		numreadings = 0;
		if (!htu21.begin(dataP, clockP)) {
			Serial.printf("\n No HTU21 Sensor.");
		} else {
			Serial.printf("\n found HTU21");
			ready=1;
		}
	}
	
	int append(float temp, float humid) {
		if (numreadings==100) {
			for(int i=0;i<99;i++) temp_readings[i]=temp_readings[i+1];
			for(int i=0;i<99;i++) humid_readings[i]=humid_readings[i+1];
			numreadings--;
		}
			
		temp_readings[numreadings] = temp;
		humid_readings[numreadings] = humid;
		return numreadings++;
	}
	
	void update(void) {
		if (!ready) return;
		temp=htu21.readTemperature();
		humid=htu21.readHumidity();
		Serial.printf("\n > read temp,humid: %.1f,%.1f@%d",temp,humid,append(temp,humid));
	}
} mytemphumid;

#else
	
#include <SHT1X.h>  //19@esp32-SDA@sht, 18@esp32-SCL@sht
SHT1x sht15(dataP, clockP);
class TempHumid {
public:
	float temp_readings[100];
	float humid_readings[100];
	int numreadings;
	float temp, humid;

	void begin(void) {
		numreadings = 0;
	}
	
	int append(float temp, float humid) {
		if (numreadings==100) {
			for(int i=0;i<99;i++) temp_readings[i]=temp_readings[i+1];
			for(int i=0;i<99;i++) humid_readings[i]=humid_readings[i+1];
			numreadings--;
		}
			
		temp_readings[numreadings] = temp;
		humid_readings[numreadings] = humid;
		return numreadings++;
	}
	
	void update(void) {
		temp=sht15.readTemperatureC();
		humid=sht15.readHumidity();
		Serial.printf("\n > read temp,humid: %.1f,%.1f@%d",temp,humid,append(temp,humid));
	}
} mytemphumid;
#endif

#ifdef DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature _ds18b20(&oneWire);
class Ds18b20 {
public:
	bool ready;
	float temp_readings[8][100];
	int numreadings;
	int count;
	bool available;
	DeviceAddress address[8];
	
	Ds18b20() {
		ready = available = false;
	}
	
	void read() {
	  if (!available) return;
	  if (!ready) {
		ready = true;
		_ds18b20.begin();
		if (_ds18b20.getDeviceCount()>0) {
		  Serial.printf("\nds18b20 found %d devices", _ds18b20.getDeviceCount());
		  count = _ds18b20.getDeviceCount();
		  for (int i=0; i<_ds18b20.getDeviceCount(); i++) {
			if (_ds18b20.getAddress(address[i], i)) Serial.printf("\n _ds18b20[%d] address ok, ", i);
			_ds18b20.setResolution(11);
			Serial.printf("resolution %d", _ds18b20.getResolution(address[i]));
		  }
		  _ds18b20.setWaitForConversion(false);
		  _ds18b20.requestTemperatures();
		} else Serial.printf("\nfound no _ds18b20");
		return;
	  }
	  for (int i=0; i<_ds18b20.getDeviceCount(); i++) {
		float x = _ds18b20.getTempC(address[i]);
		//Serial.printf(" %.1f", x);
		temp_readings[i][numreadings++]=x;
	  }
	  _ds18b20.requestTemperatures();
	}
} ds18b20;
#endif


#include <ArduinoJson.h>
#include <EEPROM.h>
#define EEPROM_SIZE 256
char json[EEPROM_SIZE];
StaticJsonDocument<EEPROM_SIZE*2> doc;
JsonObject root = doc.to<JsonObject>();

class Conf {
public:
	Conf(void) {
		ssid[0]=0;
	}
	void read(void) {
		EEPROM.begin(EEPROM_SIZE);
		for (int i=0; i<EEPROM_SIZE; i++) json[i] = EEPROM.read(i);
		if (json[0] != '{') {
			Serial.printf("\nInitialized with default values");
			strcpy(json, DEFAULT1);
			for (int i=0; i<EEPROM_SIZE; i++) EEPROM.write(i, json[i]);
			EEPROM.commit();
		}
		DeserializationError error = deserializeJson(doc, json);
		if (error) {
			Serial.print(F("deserializeJson() failed: "));
			Serial.println(error.c_str());
			return;
		}

		Serial.printf("\nGot EEPROM ");
		serializeJson(doc, eeprom);
		Serial.printf("%s", eeprom.c_str());
		if(doc["user"]) user=doc["user"];
		if(doc["topic"]) strcpy(topic, (const char*)doc["topic"]); else strcpy(topic, "no_topic");
		if(doc["device"]) strcpy(device, (const char*)doc["device"]); else strcpy(device, "no_name");
		if(doc["ssid"]) strcpy(ssid, (const char*)doc["ssid"]); else ssid[0]=0;
		if(doc["password"]) strcpy(password, (const char*)doc["password"]); 
		if(doc["mqttserver"]) strcpy(server, (const char*)doc["mqttserver"]);
		if(doc["secure"]) if (String((const char*)doc["secure"]) == "yes") secure=true;
		if(doc["target"]) target = String((const char*)doc["target"]);
		if(doc["eap_login"]) strcpy(eap_login, (const char*)doc["eap_login"]); 
		if(doc["eap_password"]) strcpy(eap_password, (const char*)doc["eap_password"]); 
		if(doc["gps"]) gps=doc["gps"]; 
	}
	String eeprom;
	int user;
	char server[32];
	char device[32];
	char topic[64];
	char ssid[32];
	char password[32];
	int co2[2]; //co2tx,co2rx
	int sht[2];
	int ds18b20;
	bool secure=false;  // EAP/PEAP
	char eap_login[32];
	char eap_password[32];
	String target="";
	int gps=0;
} myconf;

class Console {
public:
	void update(void) {
		char buffer[256];
		while (Serial.available()) {
			int c = Serial.readBytesUntil('\n', buffer, 256);
			if (buffer[c-1] == 13) buffer[--c]=0;
			if (c==256) Serial.printf("\nfyi, got 128b: %s", buffer);
			buffer[c] = 0;
			if (c<2) {
				Serial.printf("\ncurrent:");
				serializeJson(doc, Serial);
				return;
			}
			got("console", buffer);
		}
	}

	void got(String from, const char *cmd) {
		if (!strncmp(cmd, "reboot", 6)) ESP.restart();
		
		if (!strncmp(cmd, "co2 calibration", 16)) myco2.calibrate();
		
		if (cmd[0]=='{') {
		  strcpy(json, cmd);
		  DeserializationError error = deserializeJson(doc, json);
		  if (error) {
			Serial.printf("format error: %s  ", cmd);
			Serial.println(error.c_str());
			return;
		  }

		  serializeJson(doc, Serial);
		  EEPROM.begin(EEPROM_SIZE);
		  for (int i=0; i<EEPROM_SIZE; i++) EEPROM.write(i, cmd[i]);
		  EEPROM.commit();
		  Serial.printf("\nWrote EEPROM %s", cmd);

		  ESP.restart();
		}
		
		Serial.printf("\nunknown cmd=%s", cmd);
	}
} myconsole;

#include <Regexp.h>
MatchState ms;
char tbuf[4096], tb1[1024], tb2[1024], tb3[1024];

#include "esp_wifi.h"
#include "esp_wpa2.h"
class _wifi { //ZZ
public:
	int pub_wifi[64];
	int n_wifi=0;
	String myssid;
	String mypassword;
	String netstat="init";
	unsigned long serial=0;
	char macaddr[18]={0};
	byte mac[6];
	Console *_console;
	String url1, url2;
	bool the_ssid =false;
	/*
		case (0): return "Open";
		case (1): return "WEP";
		case (2): return "PSK";
		case (3): return "WPA2_PSK";
		case (4): return "WPA_WPA2_PSK";
		case (5): return "WPA2_ENTERPRISE";
	*/
	int scan_order[6]={0,5,1,2,3,4};
	
	int verify(String ssid, String password, int channel, uint8_t *bssid){ //ZZ
		Serial.printf("\ntrying WiFi ssid=%s ch=%d", ssid.c_str(), channel);
		
		WiFi.disconnect(true);
		WiFi.mode(WIFI_STA);
		if (myconf.secure) {
			Serial.printf("\n WPA2-Enterprise login using %s:%s", myconf.eap_login, myconf.eap_password);
			wifi_country_t country = {
				.cc = "JP",
				.schan = 1,
				.nchan = 14, 
				.max_tx_power = 20, 
				.policy = WIFI_COUNTRY_POLICY_AUTO,
			};
			ESP_ERROR_CHECK ( esp_wifi_set_country ( &country ) );
			//esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)myconf.eap_login, strlen(myconf.eap_login));
			esp_wifi_sta_wpa2_ent_set_username((uint8_t *)myconf.eap_login, strlen(myconf.eap_login));
			esp_wifi_sta_wpa2_ent_set_password((uint8_t *)myconf.eap_password, strlen(myconf.eap_password));
			esp_wifi_sta_wpa2_ent_enable();
		}
		delay(1);
		if (channel == -1) {
			Serial.printf("\n WiFi.begin(%s,%s)", ssid.c_str(), password==""?"NULL":password.c_str());
			if (password=="") WiFi.begin(ssid.c_str(), NULL);
			else WiFi.begin(ssid.c_str(), password.c_str());
		} else {
			Serial.printf("\n WiFi.begin(%s,%s,%d,%d)", ssid.c_str(), password==""?"NULL":password.c_str(), channel, bssid);
			if (password=="") WiFi.begin(ssid.c_str(), NULL, channel,bssid, true);
			else WiFi.begin(ssid.c_str(), password.c_str(), channel,bssid, true);
		}
		Serial.printf(" ... ok. Waiting to be connected.");
		
		unsigned long m2=millis();
		unsigned long m1=millis();
		myled.off();
		myled.on(100);
		int k=0;
		while (WiFi.status() != WL_CONNECTED) {
			if (m2+60000<millis()) {
				Serial.printf("60 sec... failed");
				return 0;
			}
			_console->update();
			myled.update();
			if (m1+1000<millis()) {
				m1=millis();
				myled.on(100);
				Serial.printf(".");
			}
		}
		Serial.printf(" connected. ip=%s", WiFi.localIP().toString().c_str());
		
		WiFi.macAddress(mac);
		sprintf(macaddr, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		Serial.printf(", mac=%s", macaddr);
		return 1;
	}
	int best() { //ZZ
		int v=-999;
		int k=-1;
		static int prev=-1;
		if (prev>0) pub_wifi[prev]=-1;
		Serial.printf("\n\n wifi for connect: from %d options", n_wifi);
		for (int i=0; i<n_wifi; i++) {
			int w=pub_wifi[i];
			if (w == -1) continue;
			//Serial.printf("\n pub_wifi[%d]=WiFi(%d) ssid=%s", i,w,WiFi.SSID(w).c_str());
			Serial.printf("\n  %d ssid=%s rssi=%d", i,WiFi.SSID(w).c_str(), WiFi.RSSI(w));
			if (WiFi.RSSI(w)>v) {
				v=WiFi.RSSI(w);
				k=w;
				prev=i;
			}
		}
		Serial.printf("\n best = %s %d ch%d %s", WiFi.SSID(k).c_str(), WiFi.RSSI(k), WiFi.channel(k),WiFi.BSSIDstr(k).c_str());
		return k;
	}
	void scan(){ //ZZ
		n_wifi=0;
		WiFi.disconnect();
		WiFi.scanDelete();
		int n = WiFi.scanNetworks(false, true);
		if(n<=0) {
			Serial.printf("\nscan ssid, got nothing.");
			reboot(5);
		}

		Serial.printf("\n found %d ssid", n);
		for (int j=0;j<7;j++) { //0~5 보안방법, 6 Hidden
			if (j==2) Serial.println();
			int kn;
			if (j==0) kn=2; else kn=1; // 개방형은 Public 표시 여부 구분
			for (int k=0;k<kn;k++) {
				for (int i=0;i<n;i++) {
					if (WiFi.SSID(i)=="splavice") the_ssid=true;
					if (j<6 && (WiFi.SSID(i)==""||WiFi.encryptionType(i) != scan_order[j])) continue;
					if (j==0 && k==0 && WiFi.SSID(i).indexOf("Public") <0) continue;
					if (j==0 && k==1 && WiFi.SSID(i).indexOf("Public") >=0) continue;
					if (j==6 && WiFi.SSID(i)!="") continue;
					
					// Print SSID and RSSI for each network found
					Serial.printf("\n%2d:",i);
					Serial.print(WiFi.SSID(i)==""?"HIDDEN":WiFi.SSID(i));
					Serial.printf(" [%s]", translateEncryptionType(WiFi.encryptionType(i)).c_str());
					if ((myconf.secure==true && WiFi.encryptionType(i)==5) || (myconf.secure==false && WiFi.encryptionType(i)==0)) {
						if (WiFi.SSID(i)=="") { Serial.printf(" > Hidden"); continue; }
						if (WiFi.SSID(i).indexOf("Public")<0) { Serial.printf(" > !Public"); continue; }
						if (myconf.secure==true && WiFi.SSID(i).indexOf("Secure")<0) { Serial.printf(" > !Secure"); continue; }
						Serial.printf(" ch=%d", WiFi.channel(i));
						//Serial.printf(" bssid=%d ", WiFi.BSSID(i));
						Serial.print(WiFi.BSSIDstr(i));
						Serial.print(" (");
						Serial.print(WiFi.RSSI(i));
						Serial.print(") ");
						
						pub_wifi[n_wifi]=i;
						Serial.printf(" pub_wifi[%d]=%d", n_wifi, i);
						if (n_wifi++ == 63) {
							Serial.printf("\n n_wifi >64. break.");
							break;
						}
					}
				}
			}
		}

		for (int i=0; i<n_wifi; i++) {
			if (the_ssid && verify("splavice", "means.success", -1, 0)) {
				Serial.printf("\n using splavice");
				myssid = "splavice";
				return; 
			}
			
			int j = best();
			if (j<0) break;
			Serial.printf("\n verifying %s@%d %d %s", WiFi.SSID(j).c_str(),WiFi.channel(j), WiFi.RSSI(j), WiFi.BSSIDstr(j).c_str());
			if (verify(WiFi.SSID(j), "", WiFi.channel(j), WiFi.BSSID(j))) {
				Serial.printf("\n found good ssid %s", WiFi.SSID(j).c_str());
				myssid = WiFi.SSID(j);
				return;
			}
			Serial.printf("\n %s bad ssid.", WiFi.SSID(j).c_str());
		}
		Serial.printf("\nfound no good wifi.");
		reboot(5);
	}
	void connect() { //ZZ
		if (WiFi.status() == WL_CONNECTED) return;
		
		if (myssid =="") scan();
		else verify(myssid, mypassword, -1, 0);
	}
	void begin(char* _ssid, char* _password, Console* console) { //ZZ
		myssid=String(_ssid);
		mypassword=String(_password);
		myssid.trim();
		mypassword.trim();
		_console = console;
		void WiFiEvent(WiFiEvent_t event);
		WiFi.onEvent(WiFiEvent);
		WiFiEventId_t eventID = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
			Serial.print("\n  WiFi lost connection. Reason: ");
			if (info.wifi_sta_disconnected.reason==200) Serial.printf("REASON_BEACON_TIMEOUT"); else
			if (info.wifi_sta_disconnected.reason==201) Serial.printf("REASON_NO_AP_FOUND"); else
			if (info.wifi_sta_disconnected.reason==202) Serial.printf("REASON_AUTH_FAIL"); else
			if (info.wifi_sta_disconnected.reason==203) Serial.printf("REASON_ASSOC_FAIL"); else
			if (info.wifi_sta_disconnected.reason==204) Serial.printf("REASON_HANDSHAKE_TIMEOUT"); else
			if (info.wifi_sta_disconnected.reason==2) Serial.printf("REASON_AUTH_EXPIRE"); else
			if (info.wifi_sta_disconnected.reason==8) Serial.printf("REASON_ASSOC_LEAVE"); else
			if (info.wifi_sta_disconnected.reason==23) Serial.printf("REASON_802_1X_AUTH_FAILED"); else
			Serial.print(info.wifi_sta_disconnected.reason);
		}, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
		connect();
		myled.on();
	}
	String translateEncryptionType(wifi_auth_mode_t encryptionType) {
		switch (encryptionType) {
			case (0): return "Open";
			case (1): return "WEP";
			case (2): return "PSK";
			case (3): return "WPA2_PSK";
			case (4): return "WPA_WPA2_PSK";
			case (5): return "WPA2_ENTERPRISE";
			default: return "UNKOWN";
		}
	}
	void ota_update(String url) {
		t_httpUpdate_return ret = httpUpdate.update(espClient, url.c_str());
		switch (ret) {
			case HTTP_UPDATE_FAILED:
				Serial.printf("\n OTA Fail(%d)%s", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
				break;
			case HTTP_UPDATE_NO_UPDATES:
				Serial.println("\n OTA Nothing to do");
				break;
			case HTTP_UPDATE_OK:
				Serial.println("\n OTA Success.");
			break;
		}
	}
} mywifi;

class URL { //ZZ
public:
	char host[64];
	char sport[8];
	char path[32];
	String param;
	String url;
	
	void decompose(String _url) { //ZZ
		url = _url;
		strcpy(tb1, _url.c_str());
		ms.Target(tb1);
		ms.Match(".*//(.-)(/.*)");
		//Serial.printf("\n ms.match=%d", ms.level);
		if (ms.level != 2) {
			Serial.printf("\n bad match level=%d", ms.level);
			for (int i=0;i<ms.level;i++) Serial.printf("\n %d %s", i, ms.GetCapture(tb2, i));
			reboot(5);
		}
		ms.GetCapture(tb2, 0);
		ms.GetCapture(tb3, 1);
		//Serial.printf("\n capture=%s %s", tb1, tb2);
		ms.Target(tb2);
		ms.Match("(.*):(.*)");
		if (ms.level==2) {
			ms.GetCapture(host, 0);
			ms.GetCapture(sport, 1);
		} else {
			strcpy(host, tb2);
			strcpy(sport, "80");
			//Serial.printf("\n port defaulted to 80");
		} 
		ms.Target(tb3);
		ms.Match("(.*)%?(.*)");
		//Serial.printf("\n ms.match=%d", ms.level);
		if (ms.level==2) {
			ms.GetCapture(path, 0);
			ms.GetCapture(tb2, 1);
			param="";
			for (int i=0; i<strlen(tb2); i++) {
				if (tb2[i]==':') param +=String("%3a"); else
				if (tb2[i]=='/') param +=String("%2f"); else
				if (tb2[i]=='?') param +=String("%3f"); else
				param +=String(tb2[i]);
			}
		} else {
			strcpy(path, tb3);
			param="";
		}
		//Serial.printf("\n decompose: %s %s%s %s %s", host, sport, sport, path, param.c_str());
	}
	void refresh() {
		url = String("http://")+host+":"+sport+path+param;
	}
} myurl;

class Captive { //ZZ
public:
	char busmac[18];
	int status=0;
	bool doit = false;


	/*
	int redirect_process_page(String url2) {
		int http_code=0;
		int content_length=0;
		               
		myurl.decompose(url2);
		WiFiClient client;
		
		int port = myurl.sport[0]?atoi(myurl.sport):80;
		Serial.printf("\nfetching %s:%d%s...", myurl.host, port, myurl.path);
		if (!client.connect(myurl.host, port)) {
			Serial.printf("failed. %s:%d", myurl.host, port);
			REBOOT
		}
		client.print(String("GET ") + myurl.path + " HTTP/1.1\r\n" +
			"Host: " + myurl.host + ":"+ String(port) +"\r\n" +
			"Connection: close\r\n\r\n");
		unsigned long timeout = millis();
		while (client.available() == 0) {
			if (millis() - timeout > 10000) {
				Serial.printf("\n timedout. failed to fetch http://%s:%d%s.", myurl.host, port, myurl.path);
				client.stop();
				REBOOT
			}
		}
		int c=0;
		while(client.available()) {
			c=client.read((uint8_t*)tbuf, 4096);
			buf[c]=0;
		}
		
		client.stop();
		return c;
	}
	*/
/*
	String extract_bus_submit_url(String page) {
		String current = page;
		int i=0, j;
		int len=current.length();
		int k=0;
#define START 1
#define INAJAX 2
#define INFORM 3
		int stat=START;
		myurl.param="{";
		String del="";
		while (i<len) {
			current = current.substring(i);
			j=current.indexOf("\n");
			if (j<0) break;
			strncpy(tb1, current.c_str(), j>1024?1024:j);
			tb1[j++]=0;
			for (int i=0; i<j; i++) tb1[i]=tolower(tb1[i]);
			Serial.printf("\n  [%d] %s", k++, tb1);
			ms.Target(tb1);
			if (ms.Match("(<form.*action.*>)")>0) {
				ms.GetCapture(tb2, 0);
				Serial.printf(" -> match %s", tb2);
				stat=INFORM;
			}
			if (stat==INFORM && ms.Match("<input.*name=\"?([a-zA-Z0-9_-]*)\"?.*value=\"?([a-zA-Z0-9%_%-%.]*)\"?.*>")>0) {
				ms.GetCapture(tb2, 0); 
				//if (tb2[strlen(tb2)-2]=='\"') tb2[strlen(tb2)-2]=0;
				Serial.printf(" -> bus match %s", tb2);
				ms.GetCapture(tb3, 1);
				//if (tb2[strlen(tb2)-2]=='\"') tb2[strlen(tb2)-2]=0;
				Serial.printf(" -> bus match %s", tb3);
				myurl.param+= del+String("\"")+tb2 +"\":\""+tb3+"\"";
				del=",";
			}
			i=j;
			delay(0);
		}
		myurl.param+="}";
		Serial.printf("extract_bus_submit_url %s", myurl.url.c_str());
		return "ok";
	}
*/
/*
	String login(String url) {
		if (myconf.target=="raspAP") {
			int i=url.indexOf("?");
			if (i>=0) url=url.substring(0, i);
		}
		myurl.decompose(url);
		http.begin(url);
		Serial.printf("\ntrying captive login page %s ...", url.c_str());
		http.addHeader("Host", "www.msftconnecttest.com");
		
		int r=http.GET();
		if(r<=0) {
			Serial.printf("\n failed. got %d(%s)", r, http.errorToString(r).c_str());
			REBOOT
		}

		String payload = http.getString();
		Serial.printf(" got %d bytes. \n%s", payload.length(), payload.c_str());
		http.end();
		if (myconf.target=="raspAP")
			return extract_submit_url(payload);
		else
			return extract_bus_submit_url(payload);
	}
*/
/*
	String auth(String url) {
		http.begin(url);
		Serial.printf("\ntrying auth %s ...", url.c_str());
		int r=http.GET();
		if(r<=0) {
			Serial.printf("\n failed. got %d(%s)", r, http.errorToString(r).c_str());
			REBOOT
		}

		String payload = http.getString();
		Serial.printf("\ngot %d bytes. \n%s", payload.length(), payload.c_str());
		http.end();
		return "";
	}
	void bus_success() {
		//String url="http://192.168.182.1/www/coova.html?url=http://captive.nexpector.com/auth/wifisucess";
		String url="vpncall://entrolink";
		http.begin(url);
		Serial.printf("\ntrying bus success %s", url.c_str());
		
		int r=http.GET();
		if(r<=0) {
			Serial.printf("\n failed. got %d(%s)", r, http.errorToString(r).c_str());
			REBOOT
		}

		String payload = http.getString();
		Serial.printf("\ngot %d bytes. \n%s", payload.length(), payload.c_str());
		http.end();
		return;
	}
		
	void bus_post() {
		String url="http://captive.nexpector.com/auth/view?svccode=NA";
		http.begin(url);
		Serial.printf("\ntrying bus post %s", url.c_str());
		
		int r=http.GET();
		if(r<=0) {
			Serial.printf("\n failed. got %d(%s)", r, http.errorToString(r).c_str());
			REBOOT
		}

		String payload = http.getString();
		Serial.printf("\ngot %d bytes. \n%s", payload.length(), payload.c_str());
		http.end();
		return;
	}
	*/
	void begin(void) { //ZZ
		byte *macp=mywifi.mac;
		sprintf(busmac, "%02X-%02X-%02X-%02X-%02X-%02X", *macp++, *macp++, *macp++, *macp++, *macp++, *macp++);
	}
	
	String get(String url) { //ZZ
		myurl.decompose(url);
		WiFiClient client;
		
		String param1 = myurl.param==""?"":"?"+myurl.param;
		Serial.printf("\nfetching http:// %s : %d %s %s", myurl.host, atoi(myurl.sport), myurl.path, param1.c_str());
		if (!client.connect(myurl.host, atoi(myurl.sport))) {
			Serial.printf(" failed to open tcp");
			reboot(5);
		}
		client.print(String("GET ") + myurl.path + param1 +" HTTP/1.1\r\n" +
			"Host: " + myurl.host + ":"+ myurl.sport +"\r\n" +
			"Connection: close\r\n\r\n");
		unsigned long timeout = millis();
		while (client.available() == 0) {
			if (millis() - timeout > 30000) {
				Serial.printf(" ... tcp response time out 30s. failed");
				client.stop();
				reboot(5);
			}
		}
		//Serial.printf("\n retrieve data");

		String s1="";
		while(client.available()) {
			int c=client.read((uint8_t*)tb1, 1024);
			delay(100);
			tb1[c]=0;
			s1 += String(tb1);
		}
		
		client.stop();
		return s1;
	}
	void parse_header(String h) { //ZZ
		//Serial.println();
		header.code="";
		header.location="";
		header.length=0;
		String current = h;
		int len=current.length();
		int i=0;
		int k=0;
		//Serial.printf("\n len=%d", len);
		while(true) {
			current = current.substring(i);
			//for (int i=0;i<60;i++) {
			//	Serial.printf("%c", current.charAt(i));
			//	if (!isgraph(current.charAt(i))) Serial.printf("/%02x/", current.charAt(i));
			//}
			int j=current.indexOf("\r\n");
			if (j<0) {
				if (current.length()>0) {
					strncpy(tb1, current.c_str(), current.length());
					tb1[current.length()]=0;
					i=current.length();
				} else break;
			} else {
				strncpy(tb1, current.c_str(), j);
				tb1[j]=0;
				i=j+2;
			}
			Serial.printf("\n [%d] %s", k++, tb1);
			ms.Target(tb1);
			if (ms.Match("HTTP/%d%.%d +(%d+) .*")==REGEXP_MATCHED) {
				header.code=String(ms.GetCapture(tb2, 0));
				Serial.printf(" --> match %s", header.code.c_str());
			}
			if (ms.Match("Location: *(http://.*) *")>0) {
				String s1 = String(ms.GetCapture(tb2, 0));
				s1.toLowerCase();
				int t=s1.indexOf("=http");
				if (t<0) {
					header.location = s1;
				} else {
					header.location=s1.substring(0,t+1);
					for (int i=t+1; i<s1.length(); i++) {
						if (tb2[i]==':') header.location +=String("%3a"); else
						if (tb2[i]=='/') header.location +=String("%2f"); else
						if (tb2[i]=='?') header.location +=String("%3f"); else
						header.location +=String(tb2[i]);
					}
				}
				Serial.printf(" --> match %s", header.location.c_str());
			}
			delay(0);
			if (k>100) {
				Serial.printf("\n too long header");
				REBOOT;
			}
		}
		Serial.printf("\n got http header: code=%s location=%s", header.code, header.location.c_str());
	}
	String bus_auth(void) { //ZZ
		String url = String("http://") + myurl.host + ":" + myurl.sport + myurl.path;
		//String url = String("http://damoa.io:8084/testt");
		
		http.begin(url);
		Serial.printf("\ntrying bus auth POST %s %s", url.c_str(), myurl.param.c_str());
		http.addHeader("Content-Type", "application/x-www-form-urlencoded; charset=UTF-8");
		http.addHeader("User-Agent", "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/98.0.4758.82 Safari/537.36");
		
//svcCode=NA&url=&proxy=&uip=&username=&password=&id=&ip=&client_mac=&mac=&webAuthTypeCode=IMAGE&mexusCallUrl=http%3A%2F%2F192.168.182.1%2Fwww%2Fcoova.html%3Furl%3Dhttp%3A%2F%2Fcaptive.nexpector.com%2Fauth%2FwifiSucess&language=ko&deviceType=Desktop&deviceModel=NoteBook-PC

		int r=http.POST(myurl.param);
		if(r<=0) {
			Serial.printf("\n failed. got %d(%s)", r, http.errorToString(r).c_str());
			REBOOT
		}

		String payload = http.getString();
		Serial.printf("\ngot %d bytes. \n%s", payload.length(), payload.c_str());
		http.end();
		return payload;
	}
	String extract_redirect_from_body(String p) { //ZZ
		//<script>top.self.location.href='http://cpnia.com/kt_test.jsp?usermac=10521c68fb54&apmac=54aed003fa3a&url=http%3A%2F%2Fwww.msftconnecttest.com%3A80%2Fconnecttest.txt&vendor=DASAN';</script>
		String url="";
		int k=0;
		int i=0;
		int len=p.length();
		String current=p;
		current.toLowerCase();
		Serial.printf("\nlen=%d", len);
		while (true) {
			current = current.substring(i);
			int j=current.indexOf("\n");
			if (j<0) {
				if (current.length()>0) {
					strncpy(tb1, current.c_str(), current.length());
					tb1[current.length()]=0;
					i=current.length();
				} else break;
			} else {
				strncpy(tb1, current.c_str(), j);
				tb1[j]=0;
				i=j+1;
			}
			Serial.printf("\n [%d] %s", k++, tb1);
			
			ms.Target(tb1);
			if (ms.Match("location%.href *= *[\'\"](.*)[\'\"]")>0) {
				String s1 = String(ms.GetCapture(tb2, 0));
				s1.toLowerCase();
				int t=s1.indexOf("=http");
				if (t<0) {
					url = s1;
				} else {
					url=s1.substring(0,t+1);
					for (int i=t+1; i<s1.length(); i++) {
						if (tb2[i]==':') url +=String("%3a"); else
						if (tb2[i]=='/') url +=String("%2f"); else
						if (tb2[i]=='?') url +=String("%3f"); else
						url +=String(tb2[i]);
					}
				}
				Serial.printf(" --> match \n%s", tb2);
				//Serial.printf("\n%s", url.c_str());
			}
		}
		return String(url);
	}
	String extract_bus_post_auth(String p) { //ZZ
		Serial.printf("\n extract post_auth url\n");
		//Serial.println(buf);
		bool form = false;;
		String url="";
		int k=0;
		int i=0;
		int len=p.length();
		String current=p;
		current.toLowerCase();
		Serial.printf("\nlen=%d", len);
		while (true) {
			current = current.substring(i);
			int j=current.indexOf("\n");
			if (j<0) {
				if (current.length()>0) {
					strncpy(tb1, current.c_str(), current.length());
					tb1[current.length()]=0;
					i=current.length();
				} else break;
			} else {
				strncpy(tb1, current.c_str(), j);
				tb1[j]=0;
				i=j+1;
			}
			Serial.printf("\n [%d] %s", k++, tb1);
			
			ms.Target(tb1);
			if (ms.Match("function +deviceCallBack")>0) {
				Serial.printf(" -> match fuction deviceCallBack");
				form=true;
			}
			if (form && ms.Match("var +url *= *[\'\"](.*)[\'\"] *;")>0) {
				ms.GetCapture(tb2, 0); 
				Serial.printf(" -> match %s", tb2);
				url= tb2;
				break;
			}
		}
		return String(url);
	}
	String process_bus(String p) { //ZZ
		Serial.printf("\n process Public....Open ssid. searching FORM and etc\n");
		//Serial.println(buf);
		bool form = false;
		bool func = false;
		String url="";  // auth
		String del="";
		int k=0;
		int i=0;
		int len=p.length();
		String current=p;
		current.toLowerCase();
		Serial.printf("\nlen=%d", len);
		while (true) {
			current = current.substring(i);
			int j=current.indexOf("\n");
			if (j<0) {
				if (current.length()>0) {
					strncpy(tb1, current.c_str(), current.length());
					tb1[current.length()]=0;
					i=current.length();
				} else break;
			} else {
				strncpy(tb1, current.c_str(), j);
				tb1[j]=0;
				i=j+1;
			}
			Serial.printf("\n [%d] %s", k++, tb1);
			
			ms.Target(tb1);
			if (ms.Match("function +devicecallback")>0) {
				Serial.printf(" -> match fuction deviceCallBack");
				func=true;
			}
			if (func && ms.Match("%s*var +url *= *[\'\"](.*)[\'\"] *;")>0) {
				ms.GetCapture(tb2, 0); 
				Serial.printf(" -> match %s", tb2);
				post_auth_url = String(tb2);
				Serial.printf("\n filled post_auth_url %s", tb2);
				func = false;
			}
			if (ms.Match("<form .*>")>0) {
				Serial.printf(" -> match form begin");
				form=true;
			}
			if (form && ms.Match("<input .*name=\"(.*)\" .*value=\"(.*)\".*>")>0) {
				ms.GetCapture(tb2, 0); 
				Serial.printf(" -> match %s", tb2);
				url+= del+tb2+"=";
				ms.GetCapture(tb3, 1);
				Serial.printf(" -> match %s", tb3);
				String s3 = String(tb3);
				s3.toLowerCase();
				if (s3.startsWith("http"))
					for (int i=0;i<strlen(tb3);i++) {
						if (tb3[i]==':') url +=String("%3a"); else
						if (tb3[i]=='/') url +=String("%2f"); else
						if (tb3[i]=='?') url +=String("%3f"); else
						url +=String(tb3[i]);
					}
				else url+= s3;
				del="&";
			}
			if (form && ms.Match("</form>")>0) {
				form = false;
			}
			delay(0);
		}
		return url;
	}

	String process_raspAP(String p) { //ZZ
		Serial.printf("\n process rsaspAP searching FORM and etc\n");
		//Serial.println(buf);
		bool form = false;
		String url="";
		String del="?";
		int k=0;
		String current = p;
		int i=0;
		while (true) {
			current = current.substring(i);
			int j=current.indexOf("\n");
			if (j<0) {
				if (current.length()>0) {
					strncpy(tb1, current.c_str(), current.length());
					tb1[current.length()]=0;
					i=current.length();
				} else break;
			} else {
				strncpy(tb1, current.c_str(), j);
				tb1[j]=0;
				i=j+1;
			}
			Serial.printf("\n [%d] %s", k++, tb1);
			ms.Target(tb1);
			if (ms.Match("<form method.*action=\"(.*)\">")>0) {
				ms.GetCapture(tb2, 0);
				Serial.printf(" -> match %s", tb2);
				form=true;
				url += String(tb2);
			}
			if (form && ms.Match("<input .*name=\"(.*)\" .*value=\"(.*)\".*>")>0) {
				ms.GetCapture(tb2, 0); 
				Serial.printf(" -> match %s", tb2);
				url+= del+tb2+"=";
				ms.GetCapture(tb3, 1);
				Serial.printf(" -> match %s", tb3);
				String t1 = String(tb3);
				t1.toLowerCase();
				if (t1.startsWith("http"))
					for (int i=0;i<strlen(tb3);i++) {
						if (tb3[i]==':') url +=String("%3a"); else
						if (tb3[i]=='/') url +=String("%2f"); else
						url +=String(tb3[i]);
						//Serial.println(url);
					}
				else url+= t1;
				del="&";
			}
			delay(0);
		}
		return String(url);
	}

	
	String post_auth_url;
	
	void go_captive() { //ZZ
		String url="http://www.msftconnecttest.com/connecttest.txt";
		//String url="http://damoa.io/a.html";
		String param;
		
		String page;
		//bool got_captive=false;
		bool got_captive=false;
		String stage = "";
		Serial.printf("\n captive stage1, fetching %s", url.c_str());
		for (int i=0; i<16; i++) {
			
			page=get(url);
			Serial.printf(" ... got %d bytes.", page.length());
			int j=page.indexOf("\r\n\r\n");
			if (j<0) {
				Serial.printf(" web page has no header ?? \n%s", page);
				reboot(10);
			}
			//Serial.printf("\nheader:\n%s\n", page.substring(0,j).c_str());
			parse_header(page.substring(0,j)); 
			
			if (header.code == "404") { 
				Serial.printf("\n got error 404");
				break;
			}
			if (stage == "get_bus_auth") {
				String payload = bus_auth(); // POST
				Serial.printf("\n got auth result %s", payload.c_str());
				url = post_auth_url;
				stage = "";
				continue;
			}
			if (i==0 && header.code == "200") { // tcp working ok
				Serial.printf("\ndata-----\n%s\n-----", page.substring(j+4).c_str());
				if (page.substring(j+4) == "Microsoft Connect Test") {
					Serial.printf("\n got M$. tcp ready. going normal operation mode.\n");
					return;
				} else {
					url = extract_redirect_from_body(page.substring(j+4));
					Serial.printf("\n got reload url from redirection page \n    %s", url.c_str());
					if (url.indexOf("cpnia.com")>=0) {
						url += "&hotspot=kt";
						url.replace("kt_test.jsp", "main/form.jsp");
						myurl.decompose(url);
						strcpy(myurl.path, "/main/handle.jsp");
						myurl.refresh();
						url = myurl.url;
						stage = "get_bus_auth";
						got_captive=true;
					}
					continue;
				}
			}
			
			if (!got_captive && header.code.startsWith("3") && header.location.startsWith("http")) { // redirect 반복
				Serial.printf("\ndata-----\n%s\n-----", page.substring(j+4).c_str());
				Serial.printf("\n found redirect url %s", header.location.c_str());
				url=header.location;
				continue;
			}
			if (!got_captive && header.code == "200") { // captive page, AP 마다 달라지는 부분
				got_captive=true;
				if (mywifi.myssid == "cookieR") url=process_raspAP(page.substring(j+4)); else
				if (mywifi.myssid.indexOf("nexpector.com")>=0) {
					param =process_bus(page.substring(j+4));  // return param and url 
					param += "&language=ko&deviceType=Desktop&deviceModel=NoteBook-PC";
					myurl.decompose(url);
					myurl.param = param;

					Serial.printf("\n got auth url and param=%s %s %s %s", myurl.host, myurl.sport, myurl.path, myurl.param.c_str()); 
					stage = "get_bus_auth";
				} else 
				if (mywifi.myssid.indexOf("cpnia.com")>=0) {
					;
					return;
				} else {
					Serial.printf("\n please configure target either raspAP or bus if not ok. Or tcp might be ready. mywifi.myssid=%s", mywifi.myssid.c_str());
					Serial.printf("\ndata-----\n%s\n-----", page.substring(j+4).c_str());
				}
					
				Serial.printf("\n got submit_url %s", url.c_str());
				continue;
			}
			if (got_captive &&header.code.startsWith("3") && header.location.startsWith("http")) {
				Serial.printf("\ndata-----\n%s\n-----", page.substring(j+4).c_str());
				Serial.printf("\n got post-captive redirect url %s", header.location.c_str());
				url=header.location;
				continue;
			}
			if (got_captive && header.code == "200") { // tcp working ok
				Serial.printf("\ndata-----\n%s\n-----", page.substring(j+4).c_str());
				Serial.printf("\n tcp ready. going normal operation mode.");
				return;
			}
		}
		Serial.printf("\n repeated redirection failed."); //ZZ @ redirect(){}
		reboot(10);
	}

	void update() {
		if (!doit) return;
		doit = false;
		switch(status) {
			case 1: // got connected test connection and begin captive cycle if needed.
				
				return;

		}
	}
	typedef struct _Header {
		String code;
		int length;
		String location;
	} Header;
	Header header = {"",0,""};
/*
		String captive_page=mycaptive.redirect();  //ZZ STEP1 GET REDIRECT PAGE @ verify(){}
		String captive_submit_url = mycaptive.extract_submit_url(captive_page);
		Serial.printf("\n captive_submit_url=%s", captive_submit_url.c_str());

		//String submit_url = mycaptive.login(captive_submit_url);
		//if (submit_url =="") {
		//	Serial.printf("\n fails auth url");
		//	REBOOT
		//}

		String r = mycaptive.auth(captive_submit_url);
		if (myconf.target=="raspAP") {
			r=
		} else {
			if (captive_submit_url=="ok") {
				r=mycaptive.bus_auth(captive_submit_url);
				delay(1);
				mycaptive.bus_success();
				//mycaptive.bus_post();
			} else {
				Serial.printf("\n error in getting submit url");
				REBOOT
			}
		}
*/
} mycaptive;

class Mqtt {//ZZ
public:
	const int port = 1883;
	const char* mqttUser = NULL;
	const char* mqttPassword = NULL;
	String mytopic="";
	String clientId = "";
	
	void begin() {
		mqttClient.setServer(myconf.server, port);
		for (int i=0;i<8;i++) clientId += String(random(16), HEX);
		clientId = String(myconf.user)+"_"+clientId;
		mytopic = String(myconf.topic)+"/"+myconf.user+"/";
	}
	
	void publish(String specific, const char* message) { //ZZ
		if (connect()) {
			Serial.printf("\n %s %s", (mytopic+specific).c_str(), message);
			mqttClient.publish((mytopic+specific).c_str(), message);
		} else
			Serial.printf("\n failed to mqtt.connect()");
	}
	void callback(char* rtopic, byte* _payload, unsigned int length) {
		String payload = "";
		for (int i=0;i<length;i++) payload +=String((char)_payload[i]);
		Serial.printf("\n got callback mytopic=%s, received=%s payload(%dB)=[%s]", (mytopic+"cmd").c_str(), rtopic, length, payload.c_str());
		if (payload.startsWith("reset")) {
			reboot(1);
		}
		if (payload.startsWith("co2-calibrate")) {
			publish("update", "will calibrate co2");
			myco2.calibrate();
		}
		if (payload.startsWith("update")) {
			String path= payload.substring(payload.indexOf(" "));
			path.trim();
			Serial.printf("\n OTA Update %s", path.c_str());
			publish("update", "will update");
			mywifi.ota_update(path);
		}
		if (payload.startsWith("current")) {
			publish("current", myconf.eeprom.c_str());
			Serial.printf("\n respond with %s", myconf.eeprom.c_str());
		}
		if (payload.startsWith("config")) {
			String path= payload.substring(payload.indexOf(" "));
			path.trim();
			publish("config", (String("will config ")+path).c_str());
			myconsole.got("mqtt", path.c_str());
		}
	}
	const char* status2str(int code) {
		switch(code) {
			case -4 : return("MQTT_CONNECTION_TIMEOUT - the server didn't respond within the keepalive time");
			case -3 : return("MQTT_CONNECTION_LOST - the network connection was broken");
			case -2 : return("MQTT_CONNECT_FAILED - the network connection failed");
			case -1 : return("MQTT_DISCONNECTED - the client is disconnected cleanly");
			case 0 : return("MQTT_CONNECTED - the client is connected");
			case 1 : return("MQTT_CONNECT_BAD_PROTOCOL - the server doesn't support the requested version of MQTT");
			case 2 : return("MQTT_CONNECT_BAD_CLIENT_ID - the server rejected the client identifier");
			case 3 : return("MQTT_CONNECT_UNAVAILABLE - the server was unable to accept the connection");
			case 4 : return("MQTT_CONNECT_BAD_CREDENTIALS - the username/password were rejected");
			case 5 : return("MQTT_CONNECT_UNAUTHORIZED - the client was not authorized to connect");
		}
		return("unknown");
	}

	int connect(){ //ZZ
		mywifi.connect();
		
		if (!mqttClient.connected()) {
			Serial.printf("\n connecting mqtt server...");
			if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword )) {
				mqttClient.setKeepAlive(60);
				mqttClient.setSocketTimeout(60);
				Serial.printf("connected as %s", clientId.c_str());
				char topic[128];
				sprintf(topic, "%s/%d/cmd", myconf.topic, myconf.user);
				Serial.printf("\nmqtt subscribed %s", topic);
				mqttClient.subscribe(topic);
			} else {
				Serial.printf("\nmqtt client_id %s, failed with %d %s", clientId.c_str(), mqttClient.state(), status2str(mqttClient.state()));
				Serial.printf("\n check network connection. ");
				for (int i=0;i<2;i++) {
					Serial.printf("\n %d. trying http://damoa.io/ip and got %s", i, mycaptive.get("http://damoa.io/ip").c_str());
				}
				return 0;
			}
		}
		return 1;
	}
} mymqtt;

void callback(char* rtopic, byte* payload, unsigned int length) {
	myled.on();
	mymqtt.callback(rtopic, payload, length);
	myled.off();
}

void do_tick_1sec() {
	if (mygyro.ready) mygyro.update();
}

void do_tick_6sec() {
	if (mybme.ready) mybme.update();
	else if(mytemphumid.ready) mytemphumid.update();
}

void do_tick_2sec() {
	myled.on(30);
	if (!myconf.gps) myco2.measure();
	#ifdef OLED
	// CO2 1, Humidity 2, Dust 3, O2 4, Temperature 5, CO 6, Volt 7, voc 8
	static int sensors[5] = {1, 2, 5, 3, 8};
	switch(sensors[myscreen.current++]) {
		case 1: myscreen.visualize(1, float(myco2.ppm), mywifi.netstat); break;
		case 2: myscreen.visualize(2, mytemphumid.humid, mywifi.netstat); break;
		case 3: myscreen.visualize(3, float(mydust.pm25), float(mydust.pm10), mywifi.netstat); break;
		case 5: myscreen.visualize(5, mytemphumid.temp, mywifi.netstat); break;
		default:
			Serial.printf("\n Unknown sensor type");
	}
	if (myscreen.current==4) myscreen.current=0;
	#endif
}

void do_tick_60sec() {
	String format="", values="", comma="";
	
	myled.on();
	if (mydust.numreadings>0) {
		sprintf(tbuf, "%.0f,%.0f", mymedian.get(mydust.pm25_readings, mydust.numreadings), mymedian.get(mydust.pm10_readings, mydust.numreadings));
		mydust.numreadings=0;
		format += "D+";
		values += comma+String(tbuf);
		comma=",";
		mydust.numreadings =0;
	}
	if (myco2.numreadings>0) {
		sprintf(tbuf, "%.0f", mymedian.get(myco2.readings, myco2.numreadings));
		mydust.numreadings=0;
		format += "C";
		values += comma+String(tbuf);
		comma=",";
		myco2.numreadings =0;
	}

	if (mybme.ready) {
		//char f1[11][5]={"%.1f","%.0f","%f","%.0f","%.0f"};
		for (int i=0;i<11;i++) {
			sprintf(tbuf, "%f", mymedian.get(mybme.readings[i], mybme.numreadings));
			char *p=&tbuf[strlen(tbuf)]-1;
			while (*p=='0') {
				*p=0;
				p--;
			}
			values += comma+tbuf;
			comma=",";
		}
		format += "T+H+I++CVPG";
		mybme.numreadings=0;
	} else {
		if (mytemphumid.numreadings>0) {
			sprintf(tbuf, "%.1f,%.0f", mymedian.get(mytemphumid.temp_readings, mytemphumid.numreadings),mymedian.get(mytemphumid.humid_readings, mytemphumid.numreadings));
			//mytemphumid.numreadings=0;  wait form BME
			format += "TH";
			values += comma+String(tbuf);
			comma=",";
			mytemphumid.numreadings=0;
		}
	}
	
	if (mygyro.ready) {
		values += comma+mygyro.Mvalues[6]; // temperature
		for (int i=0;i<6;i++) {
			values += comma+mygyro.Mvalues[i];
			comma=",";
			mygyro.Mvalues[i]=0;
		}
		format +="TG+++++";
	}
	
	if (myconf.gps==1 && mygps.lat >1 && mygps.lon >1) {
		format +="L+";
		sprintf(tbuf, "%f,%f", mygps.lat, mygps.lon);
		values += comma+tbuf;
		comma=",";
	}
	
	sprintf(tbuf, "{\"user\":%d,\"mac\":\"%s\",\"rssi\":%d,\"topic\":\"%s\",\"device\":\"%s\",\"version\":\"%s\",\"serial\":%lu,\"attribute\":\"%s\",\"data\":\"%s\"}", myconf.user, mywifi.macaddr, WiFi.RSSI(), myconf.topic, myconf.device, VERSION, mywifi.serial++,format.c_str(),values.c_str());

	mymqtt.publish("data", tbuf);
	myled.off();
}

void reboot(int s) {
	Serial.printf("\nrestart in %d sec...", s); 
	static int m=millis()+s*1000;
	while (millis()<m) {
		myconsole.update();
		mqttClient.loop();
		delay(100);
	}
	ESP.restart();
}

unsigned minute=0;
void setup() {
	Serial.begin(115200);
	delay(250);
	Serial.printf("\n\n\n\nKyuho's Bus Environment Monitoring Platform is beginning... Yeah!");
	Serial.printf("\n%s %s", BANNER, VERSION);
	Serial.printf("\n%s %s %s", __DATE__, __TIME__, __FILENAME__);
	myled.begin();
	myconf.read();
	Serial.printf("\n waiting for configuration command, if you need..");
	for (int i=0;i<10;i++) {
		myconsole.update();
		Serial.printf(".");
		delay(300);
	}
	Serial.printf("  done waiting.");
	
	if (myconf.gps) {
		Serial.printf("\n using GPS");
		mygps.begin();
	} else {
		Serial.printf("\n using CO2 Sensor");
		myco2.begin(); //16,17 Serial
	}

	Serial.printf("\n using Dust Sensor");
	mydust.begin(); //5,4 Serial
	delay(1);
	
	Wire.begin(dataP, clockP);  //19,18  I2C
	mygyro.begin(); //I2C
	mybme.begin(); //I2C
	delay(1);
	if (!mybme.ready) mytemphumid.begin(); //I2C
	myscreen.begin(myconf.user, myconf.ssid, myconf.password); //19,18 I2C
	delay(1);
	
	mywifi.begin(myconf.ssid, myconf.password, &myconsole);
	delay(1);
	mywifi.connect();
	mycaptive.begin();
	mycaptive.go_captive(); //ZZ
	
	mymqtt.begin();
	mqttClient.setCallback(callback);
	sprintf(tb1, "{\"user\":%d,\"ssid\":\"%s\",\"rssi\":%d,\"version\":\"%s\"}", myconf.user, mywifi.myssid.c_str(), WiFi.RSSI(), VERSION);
	mymqtt.publish("start", tb1);
	
	xTaskCreatePinnedToCore(
		update_loop,          // 태스크 함수
		"update",           // 테스크 이름
		10000,             // 스택 크기(워드단위)
		NULL,              // 태스크 파라미터
		1,                 // 태스크 우선순위
		&update_task,            // 태스크 핸들
		0);                // 실행될 코어
		
	xTaskCreatePinnedToCore(
		main_loop,         // 태스크 함수
		"main",           // 테스크 이름
		10000,             // 스택 크기(워드단위)
		NULL,              // 태스크 파라미터
		1,                 // 태스크 우선순위
		&main_task,            // 태스크 핸들
		1);                // 실행될 코어
	
	minute = millis()+10000;
}

void loop() {
	delay(2000);
}

void main_loop(void *param) { Serial.printf("\n# main_loop task running on core %d", xPortGetCoreID()); delay(1000); while (1) {
	static unsigned mark=0;
	static unsigned mark2=0;
	static unsigned mark3=0;
	
	if (mark3<millis()) {
		static bool first=1;
		if (first) {
			first=0;
			
		}
		mark3 = millis()+6000;
		do_tick_6sec();
	}
	if (mark2<millis()) {
		mark2 = millis()+1000;
		do_tick_1sec();
	}
	if (mark<millis()) {
		mark = millis()+2000;
		do_tick_2sec();
	}

	if (minute< millis()) {
		minute=millis()+60000;
		do_tick_60sec();
	}

	mqttClient.loop();
	if (!myconf.gps) myco2.update();
	mydust.update();
}}

void update_loop(void *param) {	Serial.printf("\n# update_task running on core %d", xPortGetCoreID()); 	while(1) { vTaskDelay(1);
	myconsole.update();
	mygps.update();
	myled.update();
	mycaptive.update();
}}

