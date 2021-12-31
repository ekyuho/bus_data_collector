#define VERSION "V1.0"
#define BANNER "Bus Monitor  Kyuho_Kim"
#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

//#define KBOX

#ifdef KBOX
#define SHT1X
#define OLED
#else
//#define HTU
#define BME
#endif
//#define SHT1X
//#define OLED
//#define GYRO
//#define DS18B20

#include <esp_task_wdt.h>
TaskHandle_t main_task;
TaskHandle_t update_task;

#include "WiFi.h"
#include <HTTPClient.h>
#include <PubSubClient.h>
WiFiClient espClient;
PubSubClient mqttClient(espClient);
const int mqttPort = 1883;
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
				Serial.println(" > checksum error:");
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
	byte header[2] = { 0xFF, 0x86 };
	byte chksum;
} myco2;

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
				Serial.printf(" > checksum error:");
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

#define dataP  19
#define clockP 18
#include <Wire.h>
bool wired = false;
#ifdef GYRO
#include <MPU6050_tockn.h>
MPU6050 mpu6050(Wire);
class Gyro {
public:
	float Mvalues[6];
	float v[6]={0,0,0,0,0,0};
	float oldv[6]={0,0,0,0,0,0};
	void begin() {
		if (!wired) {
			Wire.begin(dataP, clockP);  //data, clock
			wired=true;
		}
		mpu6050.begin();
		mpu6050.calcGyroOffsets(true);
		Serial.printf("\n init mpu6050");
		for (int k=0; k<6; k++) Mvalues[k]=0;
	}
	void update() {
		mpu6050.update();
		
		int k=0;
		Mvalues[k++]+=(v[0]=abs(mpu6050.getAccX()));
		Mvalues[k++]+=(v[1]=abs(mpu6050.getAccY()));
		Mvalues[k++]+=(v[2]=abs(mpu6050.getAccZ()));
		Mvalues[k++]+=(v[3]=abs(mpu6050.getGyroX()));
		Mvalues[k++]+=(v[4]=abs(mpu6050.getGyroY()));
		Mvalues[k++]+=(v[5]=abs(mpu6050.getGyroZ()));
		
		bool got=false;
		for (int i=0; i<6; i++) 
			if (abs(oldv[i]-v[i])> oldv[i]*5) got=true;
		if (got)
			for (int i=0; i<6; i++)
				Serial.printf(" %s%.2f(%.0f%%)", i==0?"\n":"", v[i], 100*abs(oldv[i]-v[i])/oldv[i]);
		for (int i=0; i<6; i++)
			oldv[i]=v[i];

		/*
		Mvalues[k++] = mpu6050.getAccAngleX();	//6
		Mvalues[k++] = mpu6050.getAccAngleY();	//7
		Mvalues[k++] = mpu6050.getGyroAngleX();	//8
		Mvalues[k++] = mpu6050.getGyroAngleY();	//9
		Mvalues[k++] = mpu6050.getGyroAngleZ();	//10
		Mvalues[k++] = mpu6050.getAngleX();	//11
		Mvalues[k++] = mpu6050.getAngleY();	//12
		Mvalues[k++] = mpu6050.getAngleZ();	//13
		Mvalues[k++] = mpu6050.getTemp();	//14
		*/
	}
} mygyro;
#endif

#ifdef BME
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
Adafruit_BME680 bme;
#define SEALEVELPRESSURE_HPA (1033.8)

#define mytemphumid mybme
#define temp_readings readings[0]
#define humid_readings readings[1]
class BME680 {
public:
	// 0:temp 1:humid 2:gas 3:pressure 4:alt
	float readings[5][100];
	float reading[5];
	int numreadings;
	int ready=0;
	float temp,humid, pressure, gas, alt;
	
	
	void begin() {
		numreadings = 0;
		if (!wired) {
			Wire.begin(dataP, clockP);  //data, clock
			wired=true;
		}
		if (!bme.begin()) Serial.println("\n No BME680 sensor.");
		else {
			ready=1;
			bme.setTemperatureOversampling(BME680_OS_8X);
			bme.setHumidityOversampling(BME680_OS_2X);
			bme.setPressureOversampling(BME680_OS_4X);
			bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
			bme.setGasHeater(320, 150); // 320*C for 150 ms
		}
	}

	int append(float reading[]) {
		if (numreadings==100) {
			for(int i=0;i<99;i++)
				for (int j=0;j<5;j++) readings[j][i]=readings[j][i+1];
			numreadings--;
		}
			
		for (int j=0;j<5;j++) readings[j][numreadings]=reading[j];
		return numreadings++;
	}

	void update(void) {
		if (! bme.performReading()) {
			Serial.println("\n Failed to read BME680.");
			return;
		}
		reading[0]=bme.temperature;
		reading[1]=bme.humidity;
		reading[2]=bme.gas_resistance / 1000.0;
		reading[3]=bme.pressure / 100.0;
		reading[4]=bme.readAltitude(SEALEVELPRESSURE_HPA);
		Serial.printf("\n > read temp,humid,gas(Kohm),pressureZ(Hpa),alt(m): %.1f,%.1f,%.1f,%.1f,%.1f@%d",reading[0],reading[1],reading[2],reading[3],reading[4],append(reading));
	}
} mybme;

#elif defined(HTU)

#include <HTU21D.h>
HTU21D htu21(HTU21D_RES_RH12_TEMP14);
class HTU {
public:
	float temp_readings[100];
	float humid_readings[100];
	int numreadings;
	float temp, humid;
	
	void begin() {
		numreadings = 0;
		if (!wired) {
			Wire.begin(dataP, clockP);  //data, clock
			wired=true;
		}
		htu21.begin(dataP, clockP);
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
		temp=htu21.readTemperature();
		humid=htu21.readHumidity();
		Serial.printf("\n > read temp,humid: %.1f,%.1f@%d",temp,humid,append(temp,humid));
	}
} mytemphumid;

#elif defined(SHT1X)
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
#else
	not defined
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
StaticJsonDocument<EEPROM_SIZE> doc;
JsonObject root = doc.to<JsonObject>();

class Conf {
#define DEFAULT1 "{\"user\":4000,\"topic\":\"s2m\",\"device\":\"bus\",\"ssid\":\"cookie2\",\"password\":\"0317137263\",\"mqttserver\":\"damoa.io\"}"
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
		serializeJson(doc, Serial);
		if(doc["user"]) user=doc["user"];
		if(doc["topic"]) strcpy(topic, (const char*)doc["topic"]); else strcpy(topic, "no_topic");
		if(doc["device"]) strcpy(device, (const char*)doc["device"]); else strcpy(device, "no_name");
		if(doc["ssid"]) strcpy(ssid, (const char*)doc["ssid"]); 
		if(doc["password"]) strcpy(password, (const char*)doc["password"]); 
		if(doc["mqttserver"]) strcpy(server, (const char*)doc["mqttserver"]); 
	}
	int user;
	char server[32];
	char device[32];
	char topic[64];
	char ssid[32];
	char password[32];
	int co2[2]; //co2tx,co2rx
	int sht[2];
	int ds18b20;
} myconf;

class Console {
public:
	void update(void) {
		char buffer[128];
		while (Serial.available()) {
			int c = Serial.readBytesUntil('\n', buffer, 128);
			if (buffer[c-1] == 13) buffer[--c]=0;
			if (c==128) Serial.printf("\nfyi, got 128b: %s", buffer);
			buffer[c] = 0;
			if (c<2) {
				Serial.printf("\ncurrent:");
				serializeJson(doc, Serial);
				return;
			}
			got("console", buffer);
		}
	}

	void got(String from, char *cmd) {
		if (!strncmp(cmd, "reboot", 6)) {
		  Serial.printf("\nreboot immediately");
		  ESP.restart();
		}
		
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

		  Serial.printf("\nrestart in 3 sec...");
		  delay(3000);
		  ESP.restart();
		}
		
		Serial.printf("\nunknown cmd=%s", cmd);
	}
} myconsole;

class _wifi {
public:
	int pub_wifi[10];
	int n_wifi;
	String myssid;
	String mypassword;
	String netstat;
	HTTPClient http;
	unsigned long serial;
	String clientId;
	
	_wifi() {
		n_wifi=0;
		serial =0;
		netstat = "init";
	}

	void begin(char* _ssid, char* _password) {
		myssid=String(_ssid);
		mypassword=String(_password);
		clientId = "";
		for (int i=0;i<8;i++) 
			clientId += String(random(16), HEX);
	}
	
	int connect(Console* c) {
		if (WiFi.status() == WL_CONNECTED) return 1;
		
		Serial.printf("\nConnecting WiFi ssid=%s (password=%s) ", myssid.c_str(),mypassword==""?"NULL":mypassword.c_str());
		WiFi.mode(WIFI_STA);
		WiFi.begin(myssid.c_str(), mypassword.c_str());
		int count=0;
		while (WiFi.status() != WL_CONNECTED) {
			if (count++ > 20) {
				Serial.printf("failed");
				return 0;
			}
			Serial.print('.');
			c->update();
			delay(1000);
		}
		Serial.printf("connected. ip=%s", WiFi.localIP().toString().c_str());
		netstat = "AP ok";
		return 1;
	}
	int verify(String ssid, String password){
		if (password=="") WiFi.begin(ssid.c_str(), NULL);
		else WiFi.begin(ssid.c_str(), password.c_str());
		Serial.printf("\nTrying WiFi SSID=%s (password=%s) ", ssid.c_str(),password==""?"NULL":password.c_str());
		int count=0;
		while (WiFi.status() != WL_CONNECTED) {
			if (count++ > 20) {
				Serial.printf("failed");
				return 0;
			}
			Serial.print('.');
			delay(1000);
		}
		Serial.printf("connected. ip=%s", WiFi.localIP().toString().c_str());
		
		http.begin("http://damoa.io:8080/");
		Serial.printf("\nTrying http...");
		int r=http.GET();
		if(r>0) {
			String payload = http.getString();
			//Serial.printf("payload=%s", payload.c_str());
			if (payload.startsWith("Your")) {
				Serial.printf("got success");
				http.end();
				return 1;
			}
		} 
		Serial.printf("got %d. failed", r);
		http.end();
		return 0;
	}

	String scan(){
		if (myssid){
			Serial.printf("\nusing %s",myssid.c_str()); 
			if (verify(myssid, mypassword)){
				//Serial.printf("\nfound active good pub wifi %s", WiFi.SSID(j).c_str());
				return myssid;
			} else {
				Serial.printf("\ninvalid ssid,password=%s,%s", myssid.c_str(), mypassword?mypassword.c_str():"NULL");
				return "invalid ssid="+myssid ;
			}
		}

		n_wifi=0;
		WiFi.scanDelete();
		int n = WiFi.scanNetworks();
		if(n<0) {
			Serial.printf("\ngot %d. restart device in 2 sec...", n);
			delay(2000);
			ESP.restart();
		}
		Serial.println("scan done");
	  
	  if (n ==0) {
		  Serial.println("no networks found");
	  } else {
		Serial.print(n);
		Serial.println(" networks found");
		for (int i = 0; i < n; ++i) {
		  // Print SSID and RSSI for each network found
		  Serial.print(i + 1);
		  Serial.print(": ");
		  Serial.print(WiFi.SSID(i));
		  Serial.print(" (");
		  Serial.print(WiFi.RSSI(i));
		  Serial.print(")");
		  Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
		  if (WiFi.encryptionType(i)== WIFI_AUTH_OPEN) pub_wifi[n_wifi++]=i;
		  delay(10);
		}
		Serial.printf("\nfound %d public wifi", n_wifi);
		for (int i = 0; i < n_wifi; ++i) {
			int j=pub_wifi[i];
			//Serial.printf("\n%d. ssid=%s rssi=%d", i, WiFi.SSID(j).c_str(), WiFi.RSSI(j));
			WiFi.disconnect();
			delay(1000);
			if (verify(WiFi.SSID(j), "")){
				//Serial.printf("\nfound active good pub wifi %s", WiFi.SSID(j).c_str());
				myssid = WiFi.SSID(j);
				return WiFi.SSID(j);
			}
		}
	  }
		Serial.printf("\nfound no good pub wifi");
		return "";
	}

} mywifi;

bool mqtt_connect(){
	if (!mywifi.connect(&myconsole)) {
		Serial.print("\n WiFi failed");
		return false;
	}
	
	if (!mqttClient.connected()) {
		Serial.printf("\n connecting mqtt server...");
		if (mqttClient.connect(mywifi.clientId.c_str(), mqttUser, mqttPassword )) {
			mqttClient.setKeepAlive(40);
			//mqttClient.setSocketTimeout(60);
			Serial.printf("connected as %s", mywifi.clientId.c_str());
			return true;
		} else {
			Serial.print("failed with state ");
			Serial.print(mqttClient.state());
			return false;
		}
	}
	return true;
}

void do_tick_1sec() {
	#ifdef GYRO
	mygyro.update();
	#endif
}

void do_tick_2sec() {
	#ifdef BME
	mybme.update();
	#else
	mytemphumid.update();
	#endif
	myco2.measure();
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
	char tbuf[256];
	
	if (mydust.numreadings>0) {
		sprintf(tbuf, "%.0f,%.0f", mymedian.get(mydust.pm25_readings, mydust.numreadings), mymedian.get(mydust.pm10_readings, mydust.numreadings));
		mydust.numreadings=0;
		format += "D+";
		values += comma+String(tbuf);
		comma=",";
	}
	if (mytemphumid.numreadings>0) {
		sprintf(tbuf, "%.1f,%.0f", mymedian.get(mytemphumid.temp_readings, mytemphumid.numreadings),mymedian.get(mytemphumid.humid_readings, mytemphumid.numreadings));
		//mytemphumid.numreadings=0;  wait form BME
		format += "TH";
		values += comma+String(tbuf);
		comma=",";
	}
	if (myco2.numreadings>0) {
		sprintf(tbuf, "%.1f", mymedian.get(mydust.pm25_readings, mydust.numreadings));
		mydust.numreadings=0;
		format += "C";
		values += comma+String(tbuf);
		comma=",";
	}

	#ifdef BME
	char f1[3][5]={"%f","%.0f","%.0f"};
	for (int i=2;i<5;i++) {
		sprintf(tbuf, f1[i-2], mymedian.get(mybme.readings[i], mybme.numreadings));
		values += comma+tbuf;
		comma=",";
	}
	format += "OPA";
	#endif
	mytemphumid.numreadings=0;  // this is actually BME
	
	#ifdef GYRO
	for (int i=0;i<6;i++) {
		sprintf(tbuf, "%f", mygyro.Mvalues[i]);
		values += comma+tbuf;
		comma=",";
		mygyro.Mvalues[i]=0;
	}
	format +="G+++++";
	#endif
	
	mydust.numreadings = mytemphumid.numreadings = myco2.numreadings= 0;
	
	char topic[128];
	sprintf(topic, "%s/%d/data", myconf.topic, myconf.user);
	sprintf(tbuf, "{\"user\":%d,\"topic\":\"%s\",\"device\":\"%s\",\"version\":\"%s\",\"serial\":%lu,\"attribute\":\"%s\",\"data\":\"%s\"}", myconf.user, myconf.topic, myconf.device, VERSION, mywifi.serial++,format.c_str(),values.c_str());

	if (mqtt_connect()) {
		mqttClient.publish(topic, tbuf);
		Serial.printf("\n %s  %s", topic, tbuf);
	}
	
}

unsigned minute=0;
void setup() {
	Serial.begin(115200);
	delay(400);
	Serial.println("\n\nBus Monitor");
	myconf.read();
	
	myco2.begin(); //16,17 Serial
	mydust.begin(); //5,4 Serial
	#ifdef GYRO
	mygyro.begin(); //19,18  I2C
	#endif
	#ifdef BME
	mybme.begin();
	#else
	mytemphumid.begin(); //19,18 I2C  data,clock
	#endif
	myscreen.begin(myconf.user, myconf.ssid, myconf.password); //19,18 I2C
	
	if (myconf.ssid) mywifi.begin(myconf.ssid, myconf.password);
	else Serial.printf("\nusing ssid=%s", mywifi.scan().c_str());
	
	mqttClient.setServer(myconf.server, mqttPort);
	mywifi.clientId = String(myconf.user)+"_"+mywifi.clientId;
	if (mqtt_connect()) mywifi.netstat = "";
	else mywifi.netstat = "mqtt fail";
	
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
	myco2.update();
	mydust.update();
}}

void update_loop(void *param) {	Serial.printf("\n# update_task running on core %d", xPortGetCoreID()); 	while(1) { vTaskDelay(1);
	myconsole.update();
}}
