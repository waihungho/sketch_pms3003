// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Arduino Software Version v1.6.12
// Arduino Board UNO  
//
// Sensors:
//  1. PMS3003 (PM2.5 / PM1.0 / PM10.0)
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//  Program Specification 
//    1. setup()
//        initialize all sensors（PMS3003)        
//    2. loop()
//        Read data from PMS3003 [readPMS3003()]
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------

// ------------------------------------------
//  Software Configuration (BEGIN)
// ------------------------------------------
#define SERIAL_BAUD_RATE 9600
#define READING_SENSOR_INTERVAL 10000     // Interval to read ALL sensors. 10s interval.
// ------------------------------------------
//  Software Configuration (END)
// ------------------------------------------

// PMS3003 (PM2.5 / PM1.0 / PM10.0) PIN:
#define PMS3003_RX_PIN 4 
#define PMS3003_TX_PIN 5 
//  PMS3003     Arduino(UNO)
//  PIN1        5V
//  PIN2        GND
//  PIN4        TX Pin 5  (Reference:Arduino Mega2560:18 [Serial1])
//  PIN5        RX Pin 4  (Reference:Arduino Mega2560:19 [Serial1])  

// PMS3003 Configuration (BEGIN)
// http://www.icshop.com.tw/product_info.php/products_id/20460
//首先，它可以採樣測定的空氣懸浮微粒有三種規格 0.3-1.0um/1.0-2.5um/2.5-10um，
//也就是說我們可以拿到 PM1.0/PM2.5/PM10 的測定資料(ug/m3)。
//而且PMS3003 的 datasheet 寫到他有兩套檢定空氣品質濃度的方法，
//分別是可以獲得「大氣環境下」和「標準顆粒物」兩組資料值，所以程式裡面每一次從 sensor 那邊得到的資料就會有2組，6個測定值。（這裡我會在意的是「大氣環境下」測得的這組）
#include <SoftwareSerial.h>
SoftwareSerial PMS3003Serial(PMS3003_RX_PIN, PMS3003_TX_PIN); // RX, TX
int pmcf10 = 0; // PM1.0  標準顆粒物
int pmcf25 = 0; // PM2.5  標準顆粒物
int pmcf100 = 0; // PM10.0 標準顆粒物
int pmat10 = 0; // PM1.0  大氣環境下
int pmat25 = 0; // PM2.5  大氣環境下
int pmat100 = 0; // PM10.0 大氣環境下
#define PMS3003_DATALEN 32
uint8_t PMS3003_buf[PMS3003_DATALEN];
//#define PMS3003_DISPLAY_DURATION 10000  // The duration to display PM2.5 message on LCD
// PMS3003 Configuration (BEGIN)


// Software variables(BEGIN)
#define FIRST_LOOP_DELAY 2000  // delay seconds to wait for PMS3003 sensor on first-loop.
boolean firstLoop = true; 
// Software variables (BEGIN)

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                    FUNCTIONS
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  initPMS3003(); // initialize PMS3003
  firstLoop = true;
}

void loop() { 
  // Read from PMS3003 sensor, put it into global variables (pmcf10,pmcf25,pmcf100, pmat10,pmat25,pmat100).
  readPMS3003(); 
  
  // Result
  Serial.println("PMS3003 PM2.5: " + pmat25);
  Serial.println("PMS3003 PM10.0: " + pmat100);
  
  delay(READING_SENSOR_INTERVAL);
  firstLoop = false;
}

// Initialize the PMS3003 sensor
void initPMS3003(){
  PMS3003Serial.begin(SERIAL_BAUD_RATE);
}

// Read value from PMS3003
void readPMS3003(){
  // Reset the values.
  pmcf10 = 0; // PMS3003 PM1.0 標準顆粒物.
  pmcf25 = 0; // PMS3003 PM2.5 標準顆粒物.
  pmcf100 = 0; // PMS3003 PM10.0 標準顆粒物.
  pmat10 = 0; // PMS3003 PM1.0  大氣環境下.
  pmat25 = 0; // PMS3003 PM2.5  大氣環境下.
  pmat100 = 0; // PMS3003 PM10.0 大氣環境下.
  
  if (firstLoop) {  
    delay(FIRST_LOOP_DELAY); // Wait for seconds, otherwise PMS3003Serial is not available to read in the first loop.
  }
  memset(PMS3003_buf, 0, PMS3003_DATALEN);
  int idx = 0;
  const int MAX_INDEX = 23;
  while (PMS3003Serial.available() && idx<=MAX_INDEX)
  {
      PMS3003_buf[idx++] = PMS3003Serial.read();
  }
  while (PMS3003Serial.available()) PMS3003Serial.read(); // Clear all the buffer read. Must do this here, otherwise it may cause "Check Failed" problem.
  
  if (PMS3003_buf[0] == 0x42 && PMS3003_buf[1] == 0x4d)
  {

    int checksum =   (PMS3003_buf[22]<<8) + PMS3003_buf[MAX_INDEX];
    int checkvalue = (PMS3003_buf[0]+PMS3003_buf[1]+PMS3003_buf[2]+PMS3003_buf[3]+PMS3003_buf[4]+PMS3003_buf[5]+PMS3003_buf[6]+PMS3003_buf[7]+PMS3003_buf[8]+PMS3003_buf[9]+PMS3003_buf[10]+
                      PMS3003_buf[11]+PMS3003_buf[12]+PMS3003_buf[13]+PMS3003_buf[14]+PMS3003_buf[15]+PMS3003_buf[16]+PMS3003_buf[17]+PMS3003_buf[18]+PMS3003_buf[19]+PMS3003_buf[20]+PMS3003_buf[21]);

    if ( checksum == checkvalue ) { // Compare the checksum.
      pmat25  = ( PMS3003_buf[12] << 8 ) | PMS3003_buf[13];  // PMS3003 PM2.5  大氣環境下.
      pmat10  = ( PMS3003_buf[10] << 8 ) | PMS3003_buf[11];  // PMS3003 PM1.0  大氣環境下.
      pmat100 = ( PMS3003_buf[14] << 8 ) | PMS3003_buf[15];  // PMS3003 PM10.0 大氣環境下.
      
      pmcf25  = ( PMS3003_buf[6] << 8 ) | PMS3003_buf[7];    // PMS3003 PM2.5  標準顆粒物.
      pmcf10  = ( PMS3003_buf[4] << 8 ) | PMS3003_buf[5];    // PMS3003 PM1.0  標準顆粒物.
      pmcf100 = ( PMS3003_buf[8] << 8 ) | PMS3003_buf[9];    // PMS3003 PM10.0 標準顆粒物.
    } 
  } 
}





