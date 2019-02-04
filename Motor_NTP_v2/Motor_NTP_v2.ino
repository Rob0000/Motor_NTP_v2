/*
 Name:		Motor_NTP_v2.ino
 Created:	1/28/2019 7:45:59 AM
 Author:	rob_b

NTP Time Server en aansturing polarisatie filter:
*/

#define vers "NTP GPS Stepper V03"
#define debug true

#include <SPI.h>           // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <TinyGPS++.h> // toegevoegd als alternatieve gps module
#include <SoftwareSerial.h>
#include <Stepper.h>

const int stepsPerRevolution = 12;  // change this to fit the number of steps for the polarisation filter
Stepper myStepper(stepsPerRevolution, 3, 5, 6, 7);  //  Initialiseer pins stepper 

byte mac[] = { 0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02 }; // Server MAC address
IPAddress ip(192, 168, 1, 20); // IP adres camera setup
IPAddress myDns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

#define NTP_PORT 123 // Time Server Port
static const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE]; // buffers for receiving and sending data
EthernetUDP Udp; // An Ethernet UDP instance 

EthernetServer telnet(23); // telnet defaults to port 23
boolean alreadyConnected = false; // whether or not you got a message from the client yet
String commandString;

TinyGPSPlus gps; // The TinyGPS++ object  toegevoegd
SoftwareSerial Serial1(9, 8);   // Init softwareSerial rx-gps en tx-gps (Geel op 8 en blauw op 9)

uint32_t timestamp, tempval;


void setup() {

	Ethernet.begin(mac, ip, myDns, gateway, subnet);   // start Ethernet:
	Udp.begin(NTP_PORT);   // start UDP:
	telnet.begin();   // start telnet:
	myStepper.setSpeed(60);   // set the stepper speed at 60 rpm:
	Serial1.begin(9600); // start GPS module UART

#if debug
	Serial.begin(115200);
	Serial.print("Version:");
	Serial.println(vers);
#endif

	// Disable everything but $GPRMC
	// Note the following sentences are for UBLOX NEO6MV2 GPS 

	Serial1.write("$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n");
	Serial1.write("$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n");
	Serial1.write("$PUBX,40,GSV,0,0,0,0,0,0*59\r\n");
	Serial1.write("$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n"); 
    Serial1.write("$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n");

}


void loop() {
	EthernetClient motorclient = telnet.available();   // wait for a new client:
	if (motorclient) {                            // when the client sends the first byte, say hello:
		if (!alreadyConnected) {
			motorclient.flush();                      // clear input buffer
			commandString = "";                  //clear the commandString variable
			telnet.println("--> Please type your command end hit Return.....");
			alreadyConnected = true;
		}
		while (motorclient.available()) {            // lees character totdat 0D ontvangen
													 //read the bytes from the client
			char newChar = motorclient.read();
			if (newChar == 0x0D)
			{
				processCommand(commandString);
			}
			else
			{
				Serial.println(newChar);
				commandString += newChar;
			}
		}
		
	}
	int packetSize = Udp.parsePacket();   // Wait for UDP request
	if (packetSize)
	{
		processNTP(packetSize);           // run NTP proces to send date-time
	}
	delay(3);
}


void processNTP(int packetSize)
{
	Udp.read(packetBuffer, NTP_PACKET_SIZE);
	IPAddress Remote = Udp.remoteIP();
	int PortNum = Udp.remotePort();

#if debug
	Serial.println();
	Serial.print("Received UDP packet size ");
	Serial.println(packetSize);
	Serial.print("From ");

	for (int i = 0; i < 4; i++)
	{
		Serial.print(Remote[i], DEC);
		if (i < 3)
		{
			Serial.print(".");
		}
	}
	Serial.print(", port ");
	Serial.print(PortNum);

	byte LIVNMODE = packetBuffer[0];
	Serial.print("  LI, Vers, Mode :");
	Serial.print(packetBuffer[0], HEX);

	byte STRATUM = packetBuffer[1];
	Serial.print("  Stratum :");
	Serial.print(packetBuffer[1], HEX);

	byte POLLING = packetBuffer[2];
	Serial.print("  Polling :");
	Serial.print(packetBuffer[2], HEX);

	byte PRECISION = packetBuffer[3];
	Serial.print("  Precision :");
	Serial.println(packetBuffer[3], HEX);

	for (int z = 0; z < NTP_PACKET_SIZE; z++) {
		Serial.print(packetBuffer[z], HEX);
		if (((z + 1) % 4) == 0) {
			Serial.println();
		}
	}
	Serial.println();

#endif


	packetBuffer[0] = 0b00100100;   // LI, Version, Mode
	packetBuffer[1] = 1;   // stratum
	packetBuffer[2] = 6;   // polling minimum
	packetBuffer[3] = 0xFA; // precision

	packetBuffer[7] = 0; // root delay
	packetBuffer[8] = 0;
	packetBuffer[9] = 8;
	packetBuffer[10] = 0;

	packetBuffer[11] = 0; // root dispersion
	packetBuffer[12] = 0;
	packetBuffer[13] = 0xC;
	packetBuffer[14] = 0;

	GetGPSData(1000);    // lees GPS data

#if debug
	Serial.print("Datum value= "); Serial.println(gps.date.value()); // Raw date in DDMMYY format (u32)
	Serial.print("Year value= "); Serial.println(gps.date.year()); // Year (2000+) (u16)
	Serial.print("Month value= "); Serial.println(gps.date.month()); // Month (1-12) (u8)
	Serial.print("Day value= "); Serial.println(gps.date.day()); // Day (1-31) (u8)
	Serial.print("Time value= "); Serial.println(gps.time.value()); // Raw time in HHMMSSCC format (u32)
	Serial.print("Hour value= "); Serial.println(gps.time.hour()); // Hour (0-23) (u8)
	Serial.print("Minute value= "); Serial.println(gps.time.minute()); // Minute (0-59) (u8)
	Serial.print("Seconds value= "); Serial.println(gps.time.second()); // Second (0-59) (u8)
	Serial.print("100th value= "); Serial.println(gps.time.centisecond()); // 100ths of a second (0-99) (u8)
	Serial.println();
#endif // debug

	timestamp = numberOfSecondsSince1900Epoch(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());
	tempval = timestamp;

	packetBuffer[12] = 71; //"G";
	packetBuffer[13] = 80; //"P";
	packetBuffer[14] = 83; //"S";
	packetBuffer[15] = 0; //"0";

						  // reference timestamp
	packetBuffer[16] = (tempval >> 24) & 0XFF;
	tempval = timestamp;
	packetBuffer[17] = (tempval >> 16) & 0xFF;
	tempval = timestamp;
	packetBuffer[18] = (tempval >> 8) & 0xFF;
	tempval = timestamp;
	packetBuffer[19] = (tempval) & 0xFF;

	packetBuffer[20] = 0;
	packetBuffer[21] = 0;
	packetBuffer[22] = 0;
	packetBuffer[23] = 0;


	//copy originate timestamp from incoming UDP transmit timestamp
	packetBuffer[24] = packetBuffer[40];
	packetBuffer[25] = packetBuffer[41];
	packetBuffer[26] = packetBuffer[42];
	packetBuffer[27] = packetBuffer[43];
	packetBuffer[28] = packetBuffer[44];
	packetBuffer[29] = packetBuffer[45];
	packetBuffer[30] = packetBuffer[46];
	packetBuffer[31] = packetBuffer[47];

	//receive timestamp
	packetBuffer[32] = (tempval >> 24) & 0XFF;
	tempval = timestamp;
	packetBuffer[33] = (tempval >> 16) & 0xFF;
	tempval = timestamp;
	packetBuffer[34] = (tempval >> 8) & 0xFF;
	tempval = timestamp;
	packetBuffer[35] = (tempval) & 0xFF;

	packetBuffer[36] = 0;
	packetBuffer[37] = 0;
	packetBuffer[38] = 0;
	packetBuffer[39] = 0;

	//transmitt timestamp
	packetBuffer[40] = (tempval >> 24) & 0XFF;
	tempval = timestamp;
	packetBuffer[41] = (tempval >> 16) & 0xFF;
	tempval = timestamp;
	packetBuffer[42] = (tempval >> 8) & 0xFF;
	tempval = timestamp;
	packetBuffer[43] = (tempval) & 0xFF;

	packetBuffer[44] = 0;
	packetBuffer[45] = 0;
	packetBuffer[46] = 0;
	packetBuffer[47] = 0;

	// Reply to the IP address and port that sent the NTP request
	Udp.beginPacket(Remote, PortNum);
	Udp.write(packetBuffer, NTP_PACKET_SIZE);
	Udp.endPacket();
}

const uint8_t daysInMonth[] PROGMEM = {
	31,28,31,30,31,30,31,31,30,31,30,31 }; //const or compiler complains

const unsigned long seventyYears = (2208902400UL); // to convert unix time to epoch ** Aanpassing voor verloop van 1 dag(2208988800UL - 84600 seconden)

												   // NTP since 1900/01/01
static unsigned long int numberOfSecondsSince1900Epoch(uint16_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t mm, uint8_t s) {
	if (y >= 1970)
		y -= 1970;
	uint16_t days = d;
	for (uint8_t i = 1; i < m; ++i)
		days += pgm_read_byte(daysInMonth + i - 1);
	if (m > 2 && y % 4 == 0)
		++days;
	days += 365 * y + (y + 3) / 4 - 1;
	return days * 24L * 3600L + h * 3600L + mm * 60L + s + seventyYears;
}

// command string voor de steppermotor
void processCommand(String command)
{
	if (command.indexOf("+") > -1) {             // zoek naar een + in de commandstring
		telnet.println("Clockwise command received");
		myStepper.step(stepsPerRevolution);
		commandString = "";
		return;
	}

	if (command.indexOf("-") > -1) {
		telnet.println("Counterclockwise command received");
		myStepper.step(-stepsPerRevolution);
		commandString = "";
		return;
	}

	commandString = "";
	instructions();
}
void instructions()
{
	telnet.println(" *****INVALID COMMAND******");
	telnet.println("Valid commands are: ");
	telnet.println(" + ,       This will turn the motor clockwise");
	telnet.println(" - ,       This will turn the motor counterclockwise");
}

static void GetGPSData(unsigned long ms) // This custom version of delay() ensures that the gps object is being "fed".
{
	unsigned long start = millis();
	do
	{
		while (Serial1.available())
			gps.encode(Serial1.read());
	} while (millis() - start < ms);
}