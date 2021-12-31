#ifndef OLED
class Screen {
public:
	void begin(int userno, char* ssid, char* password) {}
	void visualize(int type, float v, String netstat) {}
	void visualize(int type, float p1, float p2, String netstat) {}
};
#else
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
SSD1306  display(0x3c, 23, 22);  //Data, Clock

class Screen {
public:
	String user;
	int current;
	
void graph(int *v, int *vo, int wid, int ht, int new_value, float *scale, int low, int high) {
    float s = *scale;
    for (int i=0; i<wid; i++) v[i] = v[i+1];
    v[wid-1] = new_value;     
    
    int dmax = 0, dmin = 9999;
    for (int i=0; i<wid; i++) {
        dmax = v[i]>dmax?v[i]:dmax;
        if (v[i] > 0) dmin = v[i]<dmin?v[i]:dmin;
    }
	yield();
    if (high*10 > dmax) dmax = high*10;
    if (dmin > low*10) dmin = low*10;
    //Serial.println(String("min=")+ String(dmin) +", max="+ String(dmax));

    s = float(ht)/float(dmax-dmin);
    if (s > 1.) s = 1.;

    for (int i=0; i<wid; i++) vo[i] = v[i]>0?(v[i]-dmin) * s:0;
    *scale = s;
	yield();
}

	void visualize(int type, float v, String netstat) {
	  visualize(type, v, 0.0, netstat);
	}

	void visualize(int type, float p1, float p2, String netstat) {
	  //static boolean first = true;
	  const int GTOP = 37;
	  
	  display.clear();
	  if (type == 1) {  //CO2
		int x=0, y=0;

		display.setTextAlignment(TEXT_ALIGN_LEFT);  
		display.setFont(ArialMT_Plain_16);
		display.drawString(x, y, "C");
		x += display.getStringWidth("C");
		display.setFont(ArialMT_Plain_10);
		display.drawString(x, y+4, "O2 (ppm)");
		x += display.getStringWidth("O2 (ppm)");    
		y = 13;
		display.setFont(ArialMT_Plain_24);    
		String t_str = String(int(p1));
		display.drawString(0, y, t_str);
	   
		static int v1[128], vo[128];
		float scale;

		graph(v1, vo, 128, 64-GTOP, int(p1*10), &scale, 400, 800);
		for (int i=0; i<128;i++) 
			display.setPixel(i, (63-vo[i]));
			//display.drawVerticalLine(i, (63-vo[i]), !vo[i]?1:vo[i]);    
		   
		display.setFont(ArialMT_Plain_10);
		display.setTextAlignment(TEXT_ALIGN_RIGHT);
		display.drawString(127, 4, String("@ ")+user);  
		display.drawString(127, 13, netstat); 
		//display.drawString(127, 23, String("x") + String(scale));    
	  } else
	  if (type == 2) { //Humidity
		int x=0, y=0;

		display.setTextAlignment(TEXT_ALIGN_LEFT);
		display.setFont(ArialMT_Plain_16);
		display.drawString(x, y, "H");
		display.setFont(ArialMT_Plain_10);
		display.drawString(11, y+4, "umidity");
		y = 13;
		display.setFont(ArialMT_Plain_24);    
		String t_str = String(int(p1));
		display.drawString(x, y, t_str);
		x +=  display.getStringWidth(t_str);    

		static int v1[128], vo[128];
		float scale;

		graph(v1, vo, 128, 64-GTOP, int(p1*10), &scale, 25, 90);
		for (int i=0; i<128;i++) display.setPixel(i, (63-vo[i])); //display.drawVerticalLine(i, (63-vo[i]), !vo[i]?1:vo[i]);       
	 
		display.setFont(ArialMT_Plain_10);
		display.setTextAlignment(TEXT_ALIGN_RIGHT);
		display.drawString(127, 4, String("@ ")+user);  
		display.drawString(127, 13, netstat);
		//display.drawString(127, 23, String("x")+String(scale)); 
	  } else
	  if (type == 3) { // Dust 
		int x=0, y=0, x1=0;
		
		display.setTextAlignment(TEXT_ALIGN_LEFT);
		display.setFont(ArialMT_Plain_16);
		display.drawString(x, y, "P");
		x += display.getStringWidth("P");
		display.setFont(ArialMT_Plain_10);
		display.drawString(x, y+4, "m 2.5");
		x += display.getStringWidth("m2.5");    
		y = 13;
		display.setFont(ArialMT_Plain_24);    
		String t_str = String(int(p1));
		display.drawString(0, y, t_str);
		
		x1 =  display.getStringWidth(t_str);
		x = x1>x?x1:x;
		x += 10;  // gap between PM2.5 and PM10
		y = 0;
		display.setFont(ArialMT_Plain_16);
		display.drawString(x, y, "P");  
		x1 =  display.getStringWidth("P");
		display.setFont(ArialMT_Plain_10);
		display.drawString(x+x1, y+4, "m 10");      
		y = 13;
		display.setFont(ArialMT_Plain_24);
		display.drawString(x, y, String(int(p2))); 

		static int v1[128], v2[128];
		for (int i=0; i<128; i++) {
		  if (i <127) v1[i] = v1[i+1]; else v1[i] = p1;  
		  if (i <127) v2[i] = v2[i+1]; else v2[i] = p2;      
		}

		int dmax=0, dmin=9999;
		float scale;
		for (int i=0; i<128; i++) {
		  dmax = v1[i]>dmax?v1[i]:dmax;
		  dmax = v2[i]>dmax?v2[i]:dmax;
		  dmin = v1[i]<dmin?v1[i]:dmin;
		  dmin = v2[i]<dmin?v2[i]:dmin;
		}

		scale = (dmax-dmin)>(63-GTOP)?float(63-GTOP)/float(dmax-dmin):1.;
		int y1, y2, len;
		for (int i=0; i<128; i++) {
		  y1 = float(v1[i]-dmin)*scale;
		  y2 = float(v2[i]-dmin)*scale;
		  len = y2>y1?y2-y1:1;
		  
		  //display.drawVerticalLine(i, 63-y2<GTOP?GTOP:63-y2, len>63-GTOP?63-GTOP:len); 
		  display.drawVerticalLine(i, 63-(y2), len);
		  //Serial.println(String(v1[i])+","+String(v2[i])+","+String(scale)+","+String(y1)+","+String(y2)+","+String(63-(y2))+","+String(len));
		}
		//Serial.println(String("[y1,y2,len]=")+ String(y1)+","+String(y2)+","+String(len));
		display.setTextAlignment(TEXT_ALIGN_RIGHT);
		display.setFont(ArialMT_Plain_10);
		display.drawString(127, 4, String("@ ")+user);  
		display.drawString(127, 13, netstat);
		//display.drawString(127, 23, String("x")+String(scale));
	  } else
	  if (type == 4) {  //O2
		static float v[128];
		int x=0, y=0, x2, y2;
		float scale;

		display.setFont(ArialMT_Plain_16);
		display.drawString(x, y, "O");
		x += display.getStringWidth("O");
		display.setFont(ArialMT_Plain_10);
		display.drawString(x, y+4, "2 %");
		x += display.getStringWidth("2 %");    
		y = 13;
		display.setFont(ArialMT_Plain_24);    
		String t_str = String(p1);
		display.drawString(0, y, t_str);
		x2 = display.getStringWidth(t_str) +3;
		y2 = y+1;

		float cmax=0., cmin=9999.;
		for (int i=0; i<128; i++) {
		  if (i <127) v[i] = v[i+1]; else v[i] = p1;
		  cmax = v[i]>cmax?v[i]:cmax;
		  cmin = !cmin?v[i]:cmin;
		  cmin = v[i]&&v[i]<cmin?v[i]:cmin;
		}  

		if ((cmax-cmin)*10.<26.) {
		  scale = 1.;
		} else scale = float(63-GTOP)/((cmax-cmin)*10.);
		for (int i=0; i<128; i++) {
		  int y =  63 - (v[i]-cmin)*scale*10.;
		  y = y<GTOP?GTOP:y;
		  y = y>63?63:y;
		  display.drawVerticalLine(i, y, 63);      
		}
		display.setFont(ArialMT_Plain_10);
		display.drawString(x2, y2, String(cmax));
		display.drawString(x2, y2+10, String(cmin));      
		display.setTextAlignment(TEXT_ALIGN_RIGHT);
		display.drawString(127, 4, String("@ ")+user);  
		display.drawString(127, 13, netstat);
		//display.drawString(127, 23, String(scale)); 
		display.setTextAlignment(TEXT_ALIGN_LEFT);  
		//Serial.println(String(" val/max/min/scale=")+String(p1)+","+String(cmax)+","+String(cmin)+","+String(scale));    
	  } else
	  if (type == 5) { // Temp
		int x=0, y=0;
		const int GTOP = 38;

		display.setTextAlignment(TEXT_ALIGN_LEFT);
		display.setFont(ArialMT_Plain_16);
		display.drawString(x, y, "T");
		display.setFont(ArialMT_Plain_10);
		display.drawString(7, y+4, "emperature");
		y = 13;
		display.setFont(ArialMT_Plain_24);    
		String t_str = String(int(p1));
		display.drawString(x, y, t_str);
		x +=  display.getStringWidth(t_str);  // temperature digits > 1
		display.setFont(ArialMT_Plain_16);  
		String t2_str = String(".")+ String(int((p1-int(p1))*10));
		display.drawString(x, y+7, t2_str);    

		static int v1[128],vo[128];
		float scale;

		graph(v1, vo, 128, 64-GTOP, int(p1*10), &scale, 20, 35);
		for (int i=0; i<128;i++) display.setPixel(i, (63-vo[i])); //display.drawVerticalLine(i, (63-vo[i]), !vo[i]?1:vo[i]);       
	 
		display.setFont(ArialMT_Plain_10);
		display.setTextAlignment(TEXT_ALIGN_RIGHT);
		display.drawString(127, 4, String("@ ")+user);  
		display.drawString(127, 13, netstat);
		//display.drawString(127, 23, String("x")+String(scale));  
	  } else  
	  if (type == 6) { // CO
		int x=0, y=0;
		const int GTOP = 38;

		display.setTextAlignment(TEXT_ALIGN_LEFT);
		display.setFont(ArialMT_Plain_16);
		display.drawString(x, y, "CO");
		display.setFont(ArialMT_Plain_10);
		display.drawString(28, y+4, "ppm");    
		y = 13;
		display.setFont(ArialMT_Plain_24);    
		String t_str = String(int(p1));
		display.drawString(x, y, t_str); 

		static int v1[128],vo[128];
		float scale;

		graph(v1, vo, 128, 64-GTOP, int(p1*10), &scale, 20, 35);
		for (int i=0; i<128;i++) display.setPixel(i, (63-vo[i])); //display.drawVerticalLine(i, (63-vo[i]), !vo[i]?1:vo[i]);       
	 
		display.setFont(ArialMT_Plain_10);
		display.setTextAlignment(TEXT_ALIGN_RIGHT);
		display.drawString(127, 4, String("@ ")+user);  
		display.drawString(127, 13, netstat);
		//display.drawString(127, 23, String("x")+String(scale));  
	  } else
	  if (type == 7) { // Volts
		int x=0, y=0;
		//const int GTOP = 38;

		display.setTextAlignment(TEXT_ALIGN_LEFT);
		display.setFont(ArialMT_Plain_16);
		display.drawString(x, y, "value");
		display.setFont(ArialMT_Plain_10);
		display.drawString(40, y+4, "(rms)");    
		y = 13 + 5;
		display.setFont(ArialMT_Plain_24);    
		String t_str = String(int(p1));
		display.drawString(x, y, t_str); 

		//static int v1[128],vo[128];
		//float scale;

		//graph(v1, vo, 128, 64-GTOP, p1, &scale, 20, 35);
		//for (int i=0; i<128;i++) display.setPixel(i, (63-vo[i])); //display.drawVerticalLine(i, (63-vo[i]), !vo[i]?1:vo[i]);       
	 
		display.setFont(ArialMT_Plain_10);
		display.setTextAlignment(TEXT_ALIGN_RIGHT);
		display.drawString(127, 4, String("@ ")+user);  
		display.drawString(127, 13, netstat);
		//display.drawString(127, 23, String("x")+String(scale));  
	  } else      
		Serial.println("Wrong type");
	  display.display();
	}

	void begin(int userno, char* ssid, char* password) {
		user = String(userno);
		current=0;
		
		Serial.printf("\ninit SSD1306");
		display.init();
		display.clear();
		display.flipScreenVertically();
		display.setFont(ArialMT_Plain_10);
		display.drawString(0, 0, "Initializing.." );
		const char *s1 = "id= ";
		display.drawString(0, 13, String(s1)); 
		int i1 = display.getStringWidth(s1);
		display.setFont(ArialMT_Plain_16);
		display.drawString(i1+1, 13, user);
		display.setFont(ArialMT_Plain_10); 
		display.drawString(0, 26, String("ssid= ")+ String(ssid));
		display.drawString(0, 36, String("password= ") + String(password));
		display.display();
	}
};
#endif