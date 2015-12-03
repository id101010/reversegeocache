/*
 * Reverse Geocache
 *
 * A geocache device which unlocks in a specific area.
 *
 * Parts used:
 * -----------
 * - Arduino UNO R3
 * - Ultimate GPS logger shield
 * - 16x2 LCD Shield
 * - 9V Battery
 */
#include <math.h>
#include <LiquidCrystal.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Defines
#define GPSECHO         false                   // True -> debug GPS raw
#define ANTIFLICKER     1000                    // Timeout for lcd
#define DEG2RAD         (0.01745329251994f)     // Multiply deg to get rad
#define REARTH          (6371000.0f)            // Earths radius in meters

// Declarations
float range = 9999;                             // Distance between src and pos
String position = "";                           // read from GPS
String tmp = "";

SoftwareSerial mySerial(3,2);                   // Software Serial on 3 (rx) and 2 (tx)
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);            // Init 16x2 LCD
Adafruit_GPS GPS(&mySerial);                    // Use software serial for gps com

float lat1 = 0;
float lat2 = 0;
float lon1 = 0;
float lon2 = 0;

//String destination = "N46 59.776, E007 27.771";   // Test Zollikofen
String destination = "N46 55.090, E007 26.442";     // Gurten

// Prototypes
void useInterrupt(boolean);
void print_message(String Line1, String Line2);
void useInterrupt(boolean v);
void print_message(String Line1, String Line2);
float haversine (float lat1, float lon1, float lat2, float lon2);
float string2lon(String position);
float string2lat(String position);
String gps2string(void);
String int2fw(int x, int n);

void setup()
{
    lcd.begin(16, 2);
    lcd.clear();

    //Serial.begin(9600);
    //Serial.println("[DEBUGGING INFORMATIONS]:");

    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Request RMC and GGA
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // 1 Hz update rate
    useInterrupt(true);                           // Reads in bbackground

    delay(1000);
}

void loop()
{
    if (GPS.newNMEAreceived()) {                // Read nmea string if available
        if (!GPS.parse(GPS.lastNMEA())) {       // Parse string if possible
            return;
        }
    }

    if (GPS.fix) {                              // If there was a fix
        position = gps2string();                // get position string

        lat1 = string2lat(position);
        lat2 = string2lat(destination);
        lon1 = string2lon(position);
        lon2 = string2lon(destination);

        range = haversine(lat1, lon1, lat2, lon2);   // calculate dist to target

        tmp = String(range);
        tmp += " [m]";
        print_message("Walk further!", tmp);

    } else {
        print_message("No GPS!", "Take me outside!"); // No fix available
    }

    if (range < 20.0) {
        print_message("Unlocked!", "Password"); // Destination reached
    }
}


/**************************************************
 *  Name:       print_message()
 *  Returns:    nothing
 *  Params:     Line1, Line2
 *  Descr:      Print two strings on a 16x2 lcd
 **************************************************/
void print_message(String Line1, String Line2)
{
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(Line1);
    lcd.setCursor(0,1);
    lcd.print(Line2);
    delay(ANTIFLICKER);
}


/**************************************************
 *  Name:       SIGNAL()
 *  Returns:    nothing
 *  Params:     nothing
 *  Descr:      Timer 0 interrupt service routine
 *              Gets called on timer0 overflow [1ms]
 **************************************************/
SIGNAL(TIMER0_COMPA_vect)
{
    char c = GPS.read();    // Read from serial input

    if (GPSECHO) {          // If GPSECHO is enabled
        if (c) {
            UDR0 = c;       // Send nmea char by char
        }
    }
}

/**************************************************
 *  Name:       useInterrupt()
 *  Returns:    nothing
 *  Params:     true/false
 *  Descr:      Sets up timer0 and Interrupt
 **************************************************/
void useInterrupt(boolean v)
{
    if (v) {
        OCR0A = 0xAF;           // Set up timer0
        TIMSK0 |= _BV(OCIE0A);  // Enable timer0
    } else {
        TIMSK0 &= ~_BV(OCIE0A); // Disable timer0
    }
}


/**************************************************
 *  Name:       int2fw()
 *  Returns:    string s
 *  Params:     int x, int n
 *  Descr:      returns a fixed-width string s
 **************************************************/
String int2fw(int x, int n)
{
    String s = (String)x;

    while (s.length() < n) {
        s = "0" + s;
    }

    return s;
}

/***************************************************************
 *  Name:       gps2string()
 *  Returns:    string s
 *  Params:     nothing
 *  Descr:      returns a string "Ndd mm.mmm, Wddd mm.mmm";
 ***************************************************************/
String gps2string(void)
{
    // fetch data
    String lat = (String)GPS.lat;
    String lon = (String)GPS.lon;
    String gps2lat, gps2lon;
    float latitude = GPS.latitude;
    float longitude = GPS.longitude;
    int dd = 0;
    int mm = 0;
    int mmm = 0;

    // lat string
    dd = (int) latitude/100;
    mm = (int) latitude % 100;
    mmm = (int) round(1000 * (latitude - floor(latitude)));
    gps2lat = lat + int2fw(dd, 2) + " " + int2fw(mm, 2) + "." + int2fw(mmm, 3);

    // lon string
    dd = (int) longitude/100;
    mm = (int) longitude % 100;
    mmm = (int) round(1000 * (longitude - floor(longitude)));
    gps2lon = lon + int2fw(dd, 3) + " " + int2fw(mm, 2) + "." + int2fw(mmm, 3);

    // Assemble
    return (gps2lat + ", " + gps2lon);
}

/**************************************************
 *  Name:       string2lat()
 *  Returns:    float latitude
 *  Params:     String position
 *  Descr:      returns the float representation of
 *              the current latitude.
 **************************************************/
float string2lat(String position)
{
    // returns radians: e.g. String position = "N38 58.892, W076 29.177";
    float lat = ((position.charAt(1) - '0') * 10.0) + \
                ((position.charAt(2) - '0') * 1.0) + \
                ((position.charAt(4) - '0') / 6.0) + \
                ((position.charAt(5) - '0') / 60.0) + \
                ((position.charAt(7) - '0') / 600.0) + \
                ((position.charAt(8) - '0') / 6000.0) + \
                ((position.charAt(9) - '0') / 60000.0);

    lat *= DEG2RAD;

    if (position.charAt(0) == 'S') {
        lat *= -1;    // Correct for hemispposition
    }

    return lat;
}

/**************************************************
 *  Name:       string2lon()
 *  Returns:    float longitude
 *  Params:     String position
 *  Descr:      returns the float representation of
 *              the current longitude.
 **************************************************/
float string2lon(String position)
{
    // returns radians: e.g. String position = "N38 58.892, W076 29.177";
    float lon = ((position.charAt(13) - '0') * 100.0) + \
                ((position.charAt(14) - '0') * 10.0) + \
                ((position.charAt(15) - '0') * 1.0) + \
                ((position.charAt(17) - '0') / 6.0) + \
                ((position.charAt(18) - '0') / 60.0) + \
                ((position.charAt(20) - '0') / 600.0) + \
                ((position.charAt(21) - '0') / 6000.0) + \
                ((position.charAt(22) - '0') / 60000.0);

    lon *= DEG2RAD;

    if (position.charAt(12) == 'W') {
        lon *= -1;    // Correct for hemispposition
    }

    return lon;
}

/**************************************************
 *  Name:       haversine()
 *  Returns:    float distance
 *  Params:     float lat1, lon1, lat2, lon2
 *  Descr:      Calculates the distance between
 *              two points given by (lat1/2 lon1/2)
 **************************************************/
float haversine(float lat1, float lon1, float lat2, float lon2)
{
    // returns the great-circle distance between two points
    float h =   sq((sin((lat1 - lat2) / 2.0))) + \
                (cos(lat1) * cos(lat2) * \
                 sq((sin((lon1 - lon2) / 2.0))));

    float d = 2.0 * REARTH * asin (sqrt(h));

    return d;
}
