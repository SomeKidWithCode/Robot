// ---------- Libraries ---------- //

// For line sensor
#include <QTRSensors.h>
QTRSensors qtr;

// For card reader
#include<SPI.h>
#include<MFRC522.h>
#define SS_PIN -1
#define RST_PIN -1
MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;
int blockCount = 2;
byte rfidReadbackBlocks[blockCount][18];

// ---------- Variables ---------- //

// Line Sensor Array
const uint8_t sensorCount = 4;
uint16_t sensorValues[sensorCount];
byte left = 0;
byte right = 0;

// Ultra sonic sensor
const int echoPin = 6;
const int trigPin = 7;
const int changeThreshhold = 70;

int lastDistance = 32767;
bool firstCheckRun = false;
bool timmyFound = false;
long duration, cm;

// Motor
const int motorAIn1 = 2;
const int motorAIn2 = 3;
const int motorBIn1 = 4;
const int motorBIn2 = 5;
int motorTime = 15;

// Color sensor
const int S0 = A4;
const int S1 = A3;
const int S2 = A2;
const int S3 = A1;
const int sensorOut = A0;

// Button pin
const int btn = 12;

// I didn't write this
// For context, I went over to a friends house
// They have a little sister
// For some reason, she wanted to write something
// So, I let her and that's what she wrote
// I don't know why but I'm going to keep it
// Because no one can stop me
//kael is cool!!!!! hooty the bird tube 

// ---------- Setup ---------- //
void setup() {
  	// Start serial
  	Serial.begin(9600);
	
  	// Configure the line sensor
  	qtr.setTypeRC();
  	qtr.setSensorPins((const uint8_t[]){8, 9, 10, 11}, sensorCount);

  	// Sonic
  	pinMode(trigPin, OUTPUT);
  	pinMode(echoPin, INPUT);

  	// Motor
  	pinMode(motorAIn1, OUTPUT);
  	pinMode(motorAIn2, OUTPUT);
  	pinMode(motorBIn1, OUTPUT);
  	pinMode(motorBIn2, OUTPUT);

  	// Color
  	pinMode(S0, OUTPUT);
  	pinMode(S1, OUTPUT);
  	pinMode(S2, OUTPUT);
  	pinMode(S3, OUTPUT);
  	pinMode(sensorOut, INPUT);

  	// Setting frequency-scaling to 20%
  	digitalWrite(S0, HIGH);
  	digitalWrite(S1, LOW);

  	// RFID
  	SPI.begin();
  	mfrc522.PCD_Init();
  	for (byte i = 0; i < 6; i++)
  	    key.keyByte[i] = 0xFF;

	// Button pin
  	pinMode(btn, INPUT);
}

// ---------- Loop ---------- //
int thresh = 100;
bool buttoned = false;
bool finishedLineThreashing = false;
bool finishedReadingCard = false;
bool hasReadCard = false;

int sonemin = 69420;
int sonemax = 0;
int stwomin = 69420;
int stwomax = 0;
int sthreemin = 69420;
int sthreemax = 0;
int sfourmin = 69420;
int sfourmax = 0;

int codeSelector = 1;

// Also for a quick note, loop can be returned without causing the loop to stop entirely
void loop() {
  	if (digitalRead(btn) == HIGH)
  		buttoned = true;
	
  	if (buttoned) {
    	if (codeSelector == 0) {
    		if (finishedLineThreashing) {
    			lineSensor();

    			motorPulse(true, left <= thresh, true, right <= thresh);
    		} else {
    			int scanCount = 30;
    			for (int i = 0; i < scanCount; i++) {
    				qtr.read(sensorValues);
    				if (i > scanCount / 2)
    					motorPulse(false, false, true, true);
    				else
    					motorPulse(false, false, true, false);

    				sonemin = min(sonemin, sensorValues[0]);
    				sonemax = max(sonemax, sensorValues[0]);
    		
    				stwomin = min(stwomin, sensorValues[1]);
    				stwomax = max(stwomax, sensorValues[1]);
    		
    				sthreemin = min(sthreemin, sensorValues[2]);
    				sthreemax = max(sthreemax, sensorValues[2]);

    				sfourmin = min(sfourmin, sensorValues[3]);
    				sfourmax = max(sfourmax, sensorValues[3]);

    				delay(200);
    			}

    			finishedLineThreashing = true;
    		}
    	} else if (codeSelector == 1) {
    		sonicSensor();

    		if (timmyFound)
    			motorPulse(true, false, true, false);
    		else
    			motorPulse(true, true, true, false);
    	} else if (codeSelector == 2) {
			if (!hasReadCard) {
				// Look for new cards
				if (!mfrc522.PICC_IsNewCardPresent())
					return;
				// Select one of the cards
				if (!mfrc522.PICC_ReadCardSerial())
					return;
	
				motorTime = 500;
				hasReadCard = true;
	
				for (int i = 0; i < blockCount; i++)
					readBlock(i, rfidReadbackBlocks[i]);
			}
			
    		for (int j = 0; j < blockCount; j++) {
    			for (int a = 0; a < 18; a++) {
    				if (rfidReadbackBlocks[j][a] == 0)
    					continue;

    				moveByChar((char)rfidReadbackBlocks[j][a]);
    			}	
    		}
    	}
  	}
}

// ---------- Other functions ---------- //

// Line sensor fn
void lineSensor() {
  	qtr.read(sensorValues);

  	left = map(sensorValues[0] + sensorValues[1], sonemin+stwomin, sonemax+stwomax, 0, 255);
  	right = map(sensorValues[2] + sensorValues[3], sthreemin+sfourmin, sthreemax+sfourmax, 0, 255);

  	Serial.println("left: " + String(left));
  	Serial.println("right: " + String(right));
  	Serial.println("");

  	delay(200);
}

// Sonic sensor fn
void sonicSensor() {
  	if (timmyFound)
  	  	return;
  	digitalWrite(trigPin, LOW);
  	delayMicroseconds(2);
  	digitalWrite(trigPin, HIGH);
  	delayMicroseconds(5);
  	digitalWrite(trigPin, LOW);
  	duration = pulseIn(echoPin, HIGH);
  	cm = microsecondsToCentimetres(duration);
	
  	// figure out how much more or less the new value is as a decimal
  	// ie 17 / 68 perfectly gives 0.25
  	Serial.println("cm: " + String(cm) + ", cm prev: " + String(lastDistance));
	
  	// You cannot divide by/with non-float numbers. You must use two floats.
  	float change = (float)cm / (float)lastDistance;
  	// If the change (as a number) is lower than the threshhold, then something is mucher closer
  	// Trust me this works, I just can't explain it with words
  	if (firstCheckRun && change * 100 <= changeThreshhold) {
  	  	timmyFound = true;
  	  	Serial.println("Found timmy");
  	}
	
  	lastDistance = cm;
  	if (!firstCheckRun)
  	  	firstCheckRun = true;

	  delay(250);
}

// Motor control fn
void motorPulse(bool left, bool leftReverse, bool right, bool rightReverse) {
  	if (left) {
  	  	digitalWrite(motorAIn1, leftReverse ? HIGH : LOW);
  	  	digitalWrite(motorAIn2, leftReverse ? LOW : HIGH);
  	}
	
  	if (right) {
  	  	digitalWrite(motorBIn1, rightReverse ? HIGH : LOW);
  	  	digitalWrite(motorBIn2, rightReverse ? LOW : HIGH);
  	}
	
  	if (left || right)
  	  	delay(motorTime);
	
  	digitalWrite(motorAIn1, LOW);
  	digitalWrite(motorAIn2, LOW);
	
  	digitalWrite(motorBIn1, LOW);
  	digitalWrite(motorBIn2, LOW);
}

// Color sensor fn
void colorSensor() {
  	// Setting red filtered photodiodes to be read
  	digitalWrite(S2, LOW);
  	digitalWrite(S3, LOW);
  	// Reading the output frequency
  	int red = pulseIn(sensorOut, LOW);
  	delay(100);
	
  	// Setting Green filtered photodiodes to be read
  	digitalWrite(S2, HIGH);
  	digitalWrite(S3, HIGH);
  	// Reading the output frequency
  	int green = pulseIn(sensorOut, LOW);
  	delay(100);
	
  	// Setting Blue filtered photodiodes to be read
  	digitalWrite(S2, LOW);
  	digitalWrite(S3, HIGH);
  	// Reading the output frequency
  	int blue = pulseIn(sensorOut, LOW);

  	// Figuring out the color
  	if (blue > red && green > red)
        Serial.println("yellow");
    else if (red > blue && green > blue)
        Serial.println("purple");
    else if (blue > green && red > green)
        Serial.println("green");
  	else
  	  	Serial.println("Something else");

  	Serial.println("r: " + String(red) + ", g: " + String(green) + ", b: " + String(blue));
  	delay(100);
}

// 
long microsecondsToCentimetres(long microseconds) { return microseconds / 29 / 2; }

// RFID block reader fn
int readBlock(int blockNumber, byte arrayAddress[])  {
	int largestModulo4Number = blockNumber / 4 * 4;
  	int trailerBlock = largestModulo4Number + 3; // Determine trailer block for the sector

  	// Authentication of the desired block for access

  	byte status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));

  	if (status != MFRC522::STATUS_OK) {
        Serial.print("PCD_Authenticate() failed (read): ");
        Serial.println(mfrc522.GetStatusCodeName(status));
        return 3; // Return "3" as error message
  	}

	// Reading a block

	byte buffersize = 18; // We need to define a variable with the read buffer size, since the MIFARE_Read method below needs a pointer to the variable that contains the size... 

	status = mfrc522.MIFARE_Read(blockNumber, arrayAddress, &buffersize); // &buffersize is a pointer to the buffersize variable; MIFARE_Read requires a pointer instead of just a number

  	if (status != MFRC522::STATUS_OK) {
        Serial.print("MIFARE_read() failed: ");

        Serial.println(mfrc522.GetStatusCodeName(status));

        return 4; // Return "4" as error message
  	}

  	Serial.println("block was read");
}

// Char mover fn
void moveByChar(char c) {
	if (c == "f")
		motorPulse(true, false, true, false);
	else if (c == "b")
		motorPulse(true, true, true, true);
	else if (c == "l")
		motorPulse(true, false, true, true);
	else if (c == "r")
		motorPulse(true, true, true, false);
}
