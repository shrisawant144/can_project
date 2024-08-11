#include <time.h> 
#include <TinyGPSPlus.h>
#include <math.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
//#include <mcp_can.h>
//#include <SPI.h>
//#include <SPIFFS.h>
#include "esp_attr.h"
#include "mcp2515.h"
#include "can.h"
#include <WiFi.h>
#include <HTTPClient.h>
#define EARTH_RADIUS 6371000 // Earth's radius in meters
#define PIN2_NUM_MISO 12
#define PIN2_NUM_MOSI 13
#define PIN2_NUM_CLK 14
#define PIN2_NUM_CS 15
#define PIN2_NUM_INT 26
#define SPI2_Channel HSPI_HOST

can_frame canMsg;
const size_t JSON_BUFFER_SIZE = 8192; // Adjust the size as needed
//long unsigned int rxId;
//unsigned char len = 0;
//unsigned char rxBuf[8];
int i = 0;
const String ssid = "Redmi Note 9";
const String password = "shrikrishna";
const int utcToIstOffset = 5 * 3600 + 30 * 60;
uint16_t rpm;
int  previous_millis;
struct data
{
  float decimalSpeed;
  int32_t  battery_volt;
  int32_t  DC_current;
  uint16_t MOTOR_temp;
  uint16_t hsTemp;
  uint16_t MCU_Voltage;
  int8_t speedMode;
  int16_t motorCurrent;
  double lattitude;
  double longitude;
  double Altitude;
  double Course;
  double Speed;
  char timestamp[10];
  unsigned char BMS_failureMode;
  unsigned char BMS_errorCode;
  unsigned char BMS_ErrorType;
  unsigned char throttlePercent;
  
};

struct data Data[100];

// A sample NMEA stream.
const char *gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

int flag1, flag2, flag3, flag4, flag5, count;
#define DESIRED_CAN_ID1  0x230 // Replace with your desired CAN ID
#define DESIRED_CAN_ID2  0x104 // Replace with your desired CAN ID
#define DESIRED_CAN_ID3  0x232 // Replace with your desired CAN ID
#define DESIRED_CAN_ID4  0x233 // Replace with your desired CAN ID


spi_device_handle_t spi2;
MCP2515 mcp2515_2(&spi2);

bool led_on = false;
int currentIndex = 0; // Current index for data storage
TinyGPSPlus gps;
SoftwareSerial mySerial(4, 5);


double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

void convertLatLongToMeters(double latDeg, double lonDeg, float x, float y) {
    double latRad = degreesToRadians(latDeg);
    double lonRad = degreesToRadians(lonDeg);

    x = EARTH_RADIUS * lonRad;
    y = EARTH_RADIUS * log(tan(M_PI / 4.0 + latRad / 2.0));
}





void initializeCan2()
{
    spi_bus_config_t buscfg2 = {
        .mosi_io_num = PIN2_NUM_MOSI,
        .miso_io_num = PIN2_NUM_MISO,
        .sclk_io_num = PIN2_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = -1,
        .flags = 0};

    spi_device_interface_config_t devcfg2 = {
        .mode = 0,
        .clock_speed_hz = 16 * 1000 * 1000,
        .spics_io_num = PIN2_NUM_CS,
        .queue_size = 40,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_Channel, &buscfg2, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_Channel, &devcfg2, &spi2));

    mcp2515_2.reset();
    mcp2515_2.setBitrate(CAN_500KBPS);
    mcp2515_2.setNormalMode();
}


void clearCandata(struct data structuredArray[], size_t arraySize) {
  struct data emptyData; // Create an empty struct with all zeros
  memset(&emptyData, 0, sizeof(emptyData)); // Fill the struct with zeros

  for (int i = 0; i < arraySize; i++) {
    memcpy(&structuredArray[i], &emptyData, sizeof(emptyData));
    Serial.print(".");
  }
}



String structuretojson(struct data structuredArray[], size_t arraySize) {
  DynamicJsonDocument jsonDocument(JSON_BUFFER_SIZE);
  JsonArray jsonArray = jsonDocument.createNestedArray(); // Create a JSON array

  for (int i = 0; i < arraySize; i++) {
    JsonObject obj = jsonArray.createNestedObject();
    obj["timestamp"] = structuredArray[i].timestamp; // Add current timestamp in seconds since 1970
    obj["Speed"] = structuredArray[i].decimalSpeed;
    obj["battery_volt"] = structuredArray[i].battery_volt;
    obj["DC_current"] = structuredArray[i].DC_current;
    obj["MOTOR_temp"] = structuredArray[i].MOTOR_temp;
    obj["HS_temp"] = structuredArray[i].hsTemp;
    obj["MCU_Voltage"] = structuredArray[i].MCU_Voltage;
    obj["SPEED_MODE"] = structuredArray[i].speedMode;
    obj["MOTOR_CURR"] = structuredArray[i].motorCurrent;
    obj["ThrottlePercent"] = structuredArray[i].throttlePercent;
    obj["BMS_failureMode"] = structuredArray[i].BMS_failureMode;
    obj["BMS_errorCode"] = structuredArray[i].BMS_errorCode;
    obj["BMS_ErrorType"] = structuredArray[i].BMS_ErrorType;
    obj["Lattitude"] = structuredArray[i].lattitude; // Fixed spelling mistake here
    obj["Longitude"] = structuredArray[i].longitude;
    obj["Altitude"] = structuredArray[i].Altitude; // Used lowercase "altitude"
    obj["Course"] = structuredArray[i].Course;     // Used lowercase "course"
    obj["Speed_GPS"] = structuredArray[i].Speed;       // Used lowercase "speed"
  }

  String jsonString;
  serializeJson(jsonArray, jsonString);
  Serial.println(jsonString);

  jsonDocument.clear();
  return jsonString;
}



void sendToGoogleAppsScript(String jsonData) {
  // Create HTTPClient object
  HTTPClient http;

  // Set up the URL of your Google Apps Script web app
  String url = "https://script.google.com/macros/s/AKfycbxqSutij299B8DP-WIj9JQH_BOkZ9QDTPBk9IEr1xBE_vvjj3a6xAzMHsAf2__YdwWE0g/exec";
  // Send POST request
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(jsonData);

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String response = http.getString();
    Serial.println(response);
  } else {
    Serial.print("Error in HTTP request. HTTP Response code: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

void setupWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nConnected to WiFi");
}

void reconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost. Reconnecting...");
    WiFi.disconnect();
    setupWiFi();
  }
}



void setup() {
  Serial.begin(115200);
  mySerial.begin(9600);
  Serial.println("Initializing SPI...");
  Serial.println("Initializing CAN...");
//  if (CAN0.begin(CAN_500KBPS) == CAN_OK)
//    Serial.println("can init ok!!\r\n");
//  else
//    Serial.println("Can init fail!!\r\n");
  initializeCan2();
  pinMode(4, INPUT); // Setting pin 4 for /INT input
  Serial.println("MCP2515 Library Receive Example...");
  // Initialize Wi-Fi connection
  setupWiFi();
  
  // Synchronize time with NTP server
  configTime(0, 0, "pool.ntp.org");


  

}

void loop()
{
  // ... (your loop code above)
 
  if (i < 30)
  {
     // Obtain the current UTC time
    time_t utcTime;
    time(&utcTime);
      // Calculate the IST time by adding the offset
    time_t istTime = utcTime + utcToIstOffset;
    struct tm *timeinfo;
    timeinfo = localtime(&istTime);
    
    int year = timeinfo->tm_year + 1900; // Years since 1900
    int month = timeinfo->tm_mon + 1;     // Months since January (0-11)
    int day = timeinfo->tm_mday;          // Day of the month (1-31)
    int hour = timeinfo->tm_hour;         // Hours since midnight (0-23)
    int minute = timeinfo->tm_min;        // Minutes after the hour (0-59)
    int second = timeinfo->tm_sec;        // Seconds after the minute (0-59)
    sprintf(Data[i].timestamp,"%d:%d:%d",hour,minute,second);
    float latitude, longitude;
    int sentenceLength = strlen(gpsStream);
//    for (int i = 0; i < sentenceLength; i++) {
//    char c = gpsStream[i];
//    gps.encode(c); // Parse one character at a time
//    if(c == '\n')
//    flag4 = 1;
//    else
//    flag4 = 0;
//    }

    while (mySerial.available())
      gps.encode(mySerial.read());

//    delay(1000);
    
    if (mcp2515_2.readMessage(&canMsg) == MCP2515::ERROR_OK)
    {
        switch(canMsg.can_id)
        {
           case 0x98FF28F4:
                  Data[i].battery_volt = (float)(canMsg.data[4] | (canMsg.data[5] << 8)) / 10;
                  Data[i].DC_current = (5000 - ((256.0 * canMsg.data[3]) + canMsg.data[2])) / 10.0;
                  Data[i].BMS_failureMode = canMsg.data[6];
                  Data[i].BMS_errorCode = canMsg.data[7];
                  flag1 = 1;
                  Serial.println("0x98FF28F4");
                  break;
          
        
    
          case 0x98FF3007: 
                //Assuming the CAN message structure is followed as in your example
                rpm = ((uint16_t)canMsg.data[0] | ((uint16_t)canMsg.data[1] << 8));
                Data[i].decimalSpeed = rpm * 0.0265677;
                Data[i].MOTOR_temp = ((uint16_t)canMsg.data[2] | ((uint16_t)canMsg.data[3] << 8));
                Data[i].hsTemp = ((uint16_t)canMsg.data[4] | ((uint16_t)canMsg.data[5] << 8)); 
                //     kph = ((uint16_t)canMsg.data[6] | ((uint16_t)canMsg.data[7] << 8));
                flag2 = 1;
                Serial.println("0x98FF3007");
                break;
        
    
          case 0x98FF3006:
                Data[i].speedMode = ((canMsg.data[6] & 0b00000110) >> 1);
                //Assuming the CAN message structure is followed as in your example
                Data[i].MCU_Voltage = ((uint16_t)canMsg.data[0] | ((uint16_t)canMsg.data[1] << 8));
                Data[i].motorCurrent = ((int16_t)canMsg.data[2] | ((int16_t)canMsg.data[3] << 8));
                Data[i].BMS_ErrorType = canMsg.data[5];
                //         highBrakeStatus = (canMsg.data[6] >> 6) & 0x01;
                //         fwdRvsStatus = (canMsg.data[6] >> 3) & 0x01;
                //      speedMode = (canMsg.data[6] >> 4) & 0x03;
                Data[i].throttlePercent = canMsg.data[4];
                flag3 = 1;
                Serial.println("0x98FF3006");
                break;   
        }

  
      //  convertLatLongToMeters(gps.location.lat(),gps.location.lat(),Data[i].lattitude,Data[i].longitude);
        if(gps.location.isValid() == 1 && gps.altitude.isValid() == 1 && gps.speed.isValid() == 1 && gps.course.isValid() == 1)
        {
          Data[i].lattitude = gps.location.lat();
          Data[i].longitude = gps.location.lng();
          Data[i].Altitude = gps.altitude.meters();
          Data[i].Speed = gps.speed.kmph();
          Data[i].Course = gps.course.deg();
          flag4 = 1;
        }
        else
        {
          Data[i].lattitude = 0.00;
          Data[i].longitude = 0.00;
          Data[i].Altitude = 0.00;
          Data[i].Speed = 0.00;
          Data[i].Course = 0.00;
        }
        
    
      if (flag1 == 1 && flag2 == 1 && flag3 == 1 && flag4 == 1 )
      {
        Serial.println(i++);
        flag1 = 0;
        flag2 = 0;
        flag3 = 0;
        flag4 = 0;
      }
      //Serial.println();
    }

  }
  if (i >= 30) {
    String jsonData = structuretojson(Data, 30);
 //   Serial.println(jsonData);
// Remove the square brackets here'
    if (WiFi.status() == WL_CONNECTED) {
      sendToGoogleAppsScript(jsonData);
      delay(10000); // Send data every 10 seconds
    }
    clearCandata(Data, 30);
    i = 0; // Reset the counter
    //        while(1);
  }
 reconnectWiFi() ;


  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

//https://script.google.com/macros/s/AKfycbxqSutij299B8DP-WIj9JQH_BOkZ9QDTPBk9IEr1xBE_vvjj3a6xAzMHsAf2__YdwWE0g/exec
