#include <SoftwareSerial.h>
#include <FastCRC.h>
#include <EnhancedServo.h>
#include "MotorDriverTB6612FNG.h"
#include "Servo.h"

SoftwareSerial BTHM10(A3, A4); // RX, TX

// Protocol definition
#define PROT_ARRAY_LENGTH 17
#define PROT_STICK_LX 0
#define PROT_STICK_LY 1
#define PROT_STICK_RX 2
#define PROT_STICK_RY 3
#define PROT_BTN_SQUARE 4
#define PROT_BTN_CROSS 5
#define PROT_BTN_TRIANGLE 6
#define PROT_BTN_CIRCLE 7
#define PROT_BTN_L1 8
#define PROT_BTN_L2 9
#define PROT_BTN_R1 10
#define PROT_BTN_R2 11
#define PROT_UP 12
#define PROT_DONW 13
#define PROT_LEFT 14
#define PROT_RIGHT 15
#define PROT_DIGITAL_BUTTONS 16
#define PROT_DIGITAL_START 0
#define PROT_DIGITAL_SELECT 1
#define PROT_DIGITAL_STICK_BTN_L 2
#define PROT_DIGITAL_STICK_BTN_R 3

uint8_t rawMessage[PROT_ARRAY_LENGTH];
uint8_t message[PROT_ARRAY_LENGTH];
uint8_t messageCRC=0;

uint8_t messageBytesRead = 0 ; // we will mark message only as valid, if we read at least a complete message

bool isMessageValid=false;
unsigned long messageReceivedTime = 0; // will be used for timeout

#define PROT_TIMEOUT 10000 // timeout in milliseconds for protocol
const unsigned long protocolTimeout = PROT_TIMEOUT;


//  STEERING SERVO
#define STEERING_SERVO_PIN 6 // need to use PWM Port
#define STEERING_SERVO_LEFT_MAX 2
#define STEERING_SERVO_RIGHT_MAX 180
#define STEERING_SERVO_TRIM 0

//EnhancedServo steeringServo;
EnhancedServo steeringServo;


// MOTOR DRIVER TB6612FNG
#define MOTOR_1_IN_1 4
#define MOTOR_1_IN_2 7
#define MOTOR_1_PWM 3
#define MOTOR_1_STDBY 8

#define MOTOR_2_IN_1 12
#define MOTOR_2_IN_2 13
#define MOTOR_2_PWM 5
#define MOTOR_2_STDBY  8

MotorDriverTB6612FNG motor;





// CRC
FastCRC8 CRC8;

void swapMessage()
{
	for (int i=0; i < sizeof(rawMessage); i++)
	{
		message[i] = rawMessage[(sizeof(rawMessage) -1 -i)];
	}

}



void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("starting");

  // clear message
  for (int i=0; i< sizeof(rawMessage);i++)
  {
	rawMessage[i]=0;
	message[i]=0;
  }

  // set the data rate for the SoftwareSerial port
  BTHM10.begin(38400);
  BTHM10.println("AT+ROLE0");
  delay(200);
  BTHM10.println("AT+POWE2");
  delay(200);

  // setup the servo
  steeringServo.attach(STEERING_SERVO_PIN);
  steeringServo.setMaxValue(STEERING_SERVO_RIGHT_MAX);
  steeringServo.setMinValue(STEERING_SERVO_LEFT_MAX);
  steeringServo.setTrim(STEERING_SERVO_TRIM);

  // setup the motor
  motor = MotorDriverTB6612FNG(MOTOR_1_IN_1, MOTOR_1_IN_2, MOTOR_1_PWM, MOTOR_1_STDBY);

}
/**
 * shifts all bytes one step left in array. [0] will become value of messageCRC
 */
void leftShiftMessageBytes ()
{
	for (int i=sizeof(rawMessage)-1; i>0 ;i--)
	{
		rawMessage[i] = rawMessage[i-1];
	}
	rawMessage[0] = messageCRC;
}


void debugMessages()
{
	char buffer[2];
	Serial.print("rawMessage: ");
	for (int i=sizeof(rawMessage)-1; i>= 0; i--)
	{
		sprintf(buffer, "%02X ", rawMessage[i]);
		Serial.print(buffer);
	}
	Serial.print("  rCRC: ");
	sprintf(buffer, "%02X ", messageCRC);
	Serial.print(buffer);
	Serial.print ("  cCRC: ");
	sprintf(buffer, "%02X ", CRC8.smbus(rawMessage, sizeof(rawMessage)));
	Serial.println(buffer);



	Serial.print("   Message: ");
	for (int i=sizeof(message)-1; i>= 0; i--)
	{
		sprintf(buffer, "%02X ", message[i]);
		Serial.print(buffer);
	}
	Serial.print("  rCRC: ");
	sprintf(buffer, "%02X ", messageCRC);
	Serial.print(buffer);
	Serial.print ("  cCRC: ");
	sprintf(buffer, "%02X ", CRC8.smbus(message, sizeof(message)));
	Serial.println(buffer);



}

void rightShiftMessageBytes ()
{
	for (int i=0; i < sizeof(rawMessage)-1 ;i++)
	{
		rawMessage[i] = rawMessage[i+1];
	}
	rawMessage[sizeof(rawMessage)-1] = messageCRC;
}

bool isTimeout()
{
	long timeDiff = millis() - protocolTimeout - messageReceivedTime;
//	Serial.print("timeoutCalc: ");
//	Serial.println(timeDiff);

	if (timeDiff > 0)
	{
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * this will normally occur if signal is lost and we ran into timeout. e.g. by out of range of sender - receiver
 */
void processEmergencyProtocolTimeout()
{
	// stop the motor (high impedance)
	motor.stop();

}

void loop() { // run over and over
	if (BTHM10.available() > 0)
	{
		// shift old bytes
		leftShiftMessageBytes();

		// read new byte
		messageCRC = (uint8_t) BTHM10.read();
		messageBytesRead++;

		// check if we have read enough bytes (more than protocol length)
		if ( messageBytesRead > sizeof(rawMessage))
		{


			swapMessage();
			//debugMessages();
			// calculate crc8
			if (messageCRC == CRC8.smbus(message, sizeof(message)))
			{
				// mark message as valid
				isMessageValid = true;
				//Serial.print("isMessageValid=");
				//Serial.println(isMessageValid);

				// update timestamp
				messageReceivedTime = millis();

				// reset BytesReadCounter
				messageBytesRead = 0;

				//Serial.write(message, sizeof(message));
				//Serial.println("CRC OK");
			}
			else
			{
				//Serial.println("CRC FAILURE");
			}
		}

	}


//	Serial.print("valid=");
//	Serial.println(isMessageValid);
	if (isMessageValid)
	{
		// we have a valid message
		if (isTimeout())
		{
			// set message as invalid
			Serial.println("timeout");
			isMessageValid = false;
			processEmergencyProtocolTimeout();
		}
		else
		{
			// not in timeout
			//Serial.write(message, sizeof(message));

//			Serial.print("servo: ");
//			char buffer[2];
//			sprintf(buffer, "%03d ", message[PROT_STICK_RX]);
//			Serial.print(buffer);
//
//			Serial.print(" motor: ");
//			sprintf(buffer, "%03d ", message[PROT_STICK_LY]);
//			Serial.println(buffer);

			// update steering servo
			Serial.print(F("in: "));
			Serial.print(message[PROT_STICK_RX],DEC);
			Serial.print(F("   "));
			steeringServo.enhancedWrite(message[PROT_STICK_RX]);
			//uint8_t steeringValue = map((int) message[PROT_STICK_RX], 0, 255, 0, 180);
			//steeringServo.write((int) message[PROT_STICK_RX]);
			//steeringServo.write(steeringValue);


			// update motor
			motor.movePWMTwoWay(message[PROT_STICK_LY], 0, 255);
			// invalidate message, because it has been processed
			isMessageValid = false;
		}
	}



}
