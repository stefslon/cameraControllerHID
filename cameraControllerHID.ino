/* 

	Pan/Tilt Camera Bracket Controller
	S.Slonevskiy
	June 2014
	
	Pan/Tilt bracket controls are done over USB raw HID protocol.
	
*/

#include "StepperMotor2.h"
#include <EEPROM.h>

// Setup motors for both axis
StepperMotor2 panAxis(16, 17, 18, 19, 5, 90, SM2_HALFSTEP);
StepperMotor2 tiltAxis(15, 14, 13, 12, 6, 90, SM2_HALFSTEP);

// These are the command ENUMs
#define CMD_RUN	        0x01
#define CMD_STEP		0x11
#define CMD_STOP		0xFF
#define CMD_GET_NAME    0x21
#define CMD_GET_NAME_C  0x22 
#define CMD_SET_NAME    0x23
#define CMD_SET_NAME_C  0x24
#define CMD_GET_PS      0x31
#define CMD_GET_PS_C    0x32
#define CMD_SET_PS      0x33
#define CMD_GO_PS       0x36
#define CMD_SET_POS     0x35
#define CMD_GET_POS     0x37
#define CMD_GET_POS_C   0x38
#define CMD_RECAL       0x50
#define CMD_PING 		0x99
#define POWER_LED       11

#define PRESET_MEM_START 50

// HID command structure (8 bytes)
// RawHID packets are always 4 bytes
struct rxCmd {
	byte cmd;        // 1 byte
	int payload[3];  // 3 x 2 bytes
	byte pad;        // 1 byte
};

void setup() {
	Serial.begin(9600);
	Serial.println("Started controller...");

	pinMode(POWER_LED,OUTPUT);
	digitalWrite(POWER_LED,HIGH);
}

rxCmd rxBuffer;
rxCmd txBuffer;
int rxNum;
int txNum;
int numChars;
int numPacks;
int presetNum;
byte memValue;
int panPosition;
int tiltPosition;
int ic; // generic counter

void loop() {
	panAxis.move();
	tiltAxis.move();

	rxNum = RawHID.recv(&rxBuffer, 0); // 0 timeout = do not wait
	if (rxNum>0) {
		// the computer sent a command go through a switchyard and take action
		switch (rxBuffer.cmd) {
			case CMD_PING:
				txBuffer.cmd = CMD_PING;
				txNum = RawHID.send(&txBuffer, 10);
				
				Serial.println("Command: CMD_PING ");
				break;

			case CMD_STEP:
				panAxis.step(rxBuffer.payload[0]);
				tiltAxis.step(rxBuffer.payload[1]);

				Serial.print("Command: STEP, ");
				Serial.print(rxBuffer.payload[0]);
				Serial.print(", ");
				Serial.println(rxBuffer.payload[1]);
				break;

			case CMD_RUN:
				panAxis.run(rxBuffer.payload[0]);
				tiltAxis.run(rxBuffer.payload[1]);

				Serial.print("Command: RUN, ");
				Serial.print(rxBuffer.payload[0]);
				Serial.print(", ");
				Serial.println(rxBuffer.payload[1]);
				break;

			case CMD_STOP:
				panAxis.stop();
				tiltAxis.stop();

				Serial.println("Command: STOP, no additional parameters.");
				break;

			case CMD_SET_PS:
				presetNum = rxBuffer.payload[0];  /* 0-3 */

				panPosition = panAxis.getPosition();
				tiltPosition = tiltAxis.getPosition();

				EEPROM.write(PRESET_MEM_START+presetNum*5,1);
				EEPROM.write(PRESET_MEM_START+presetNum*5+1,panPosition & 255);
				EEPROM.write(PRESET_MEM_START+presetNum*5+2,panPosition >> 8);
				EEPROM.write(PRESET_MEM_START+presetNum*5+3,tiltPosition & 255);
				EEPROM.write(PRESET_MEM_START+presetNum*5+4,tiltPosition >> 8);

				Serial.print("Command: CMD_SET_PS, ");
				Serial.print(rxBuffer.payload[0]);
				Serial.print(" with ");
				Serial.print(panPosition);
				Serial.print(", ");
				Serial.println(tiltPosition);
				break;

			case CMD_GO_PS:
				presetNum = rxBuffer.payload[0];  /* 0-3 */
				memValue = EEPROM.read(PRESET_MEM_START+presetNum*5);
				if (memValue==1)
				{
					panPosition = (int)(EEPROM.read(PRESET_MEM_START+presetNum*5+2))<<8 | (int)(EEPROM.read(PRESET_MEM_START+presetNum*5+1));
					tiltPosition = (int)(EEPROM.read(PRESET_MEM_START+presetNum*5+4))<<8 | (int)(EEPROM.read(PRESET_MEM_START+presetNum*5+3));
					panAxis.gotoPosition(panPosition,(unsigned int)rxBuffer.payload[1]);
					tiltAxis.gotoPosition(tiltPosition,(unsigned int)rxBuffer.payload[1]);
				}

				Serial.print("Command: CMD_GO_PS, ");
				Serial.print(rxBuffer.payload[0]);
				Serial.print(" lasting ");
				Serial.println((unsigned int)rxBuffer.payload[1]);
				break;

			case CMD_GET_POS:
				txBuffer.cmd = CMD_GET_POS_C;
				txBuffer.payload[0] = panAxis.getPosition();
				txBuffer.payload[1] = tiltAxis.getPosition();
				txBuffer.payload[2] = 0;

				Serial.println("Command: CMD_GET_POS ");

				txNum = RawHID.send(&txBuffer, 10);

				Serial.print("Sending ");
				Serial.print(txBuffer.payload[0]); Serial.print(" ");
				Serial.println(txBuffer.payload[1]);
				break;

			case CMD_SET_POS:
				panAxis.gotoPosition(rxBuffer.payload[0],(unsigned int)rxBuffer.payload[2]);
				tiltAxis.gotoPosition(rxBuffer.payload[1],(unsigned int)rxBuffer.payload[2]);

				Serial.print("Command: CMD_SET_POS, ");
				Serial.print(rxBuffer.payload[0]);
				Serial.print(", ");
				Serial.print(rxBuffer.payload[1]);
				Serial.print(", ");
				Serial.println((unsigned int)rxBuffer.payload[2]);
				break;

			case CMD_GET_NAME:
				txBuffer.cmd = CMD_GET_NAME_C;
				numChars = EEPROM.read(0);

				Serial.print("Command: GET_NAME, ");
				Serial.print(" in EEPROM ");
				Serial.println(numChars);

				numChars = min(numChars,30);
				numChars = max(numChars,0);
				numPacks = roundUp(numChars,3);

				txBuffer.payload[0] = numChars;
				txNum = RawHID.send(&txBuffer, 10);

				for (int ic=0; ic<numPacks; ic++)
				{
					txBuffer.payload[0] = EEPROM.read(ic*3+1);
					txBuffer.payload[1] = EEPROM.read(ic*3+2);
					txBuffer.payload[2] = EEPROM.read(ic*3+3);

					txNum = RawHID.send(&txBuffer, 10);

					Serial.print("Sending ");
					Serial.print((char)txBuffer.payload[0]);
					Serial.print((char)txBuffer.payload[1]);
					Serial.println((char)txBuffer.payload[2]);
				}
				break;

			case CMD_SET_NAME:
				numChars = rxBuffer.payload[0];
				numPacks = rxBuffer.payload[1];

				Serial.print("Command: SET_NAME, numChars = ");
				Serial.print(numChars);
				Serial.print(", numPacks = ");
				Serial.println(numPacks);

				EEPROM.write(0,numChars);

				// Respond with acknowledgement
				txBuffer.cmd = CMD_SET_NAME_C;
				txNum = RawHID.send(&txBuffer, 10);

				ic=0;
				while (ic<numPacks)
				{
					rxNum = RawHID.recv(&rxBuffer, 0);
					if (rxBuffer.cmd==CMD_SET_NAME_C) {
						EEPROM.write(ic*3+1,(char)rxBuffer.payload[0]);
						EEPROM.write(ic*3+2,(char)rxBuffer.payload[1]);
						EEPROM.write(ic*3+3,(char)rxBuffer.payload[2]);
						ic++;

						Serial.print("Receiving ");
						Serial.print((char)rxBuffer.payload[0]);
						Serial.print((char)rxBuffer.payload[1]);
						Serial.println((char)rxBuffer.payload[2]);
					}
				}

				break;

			case CMD_RECAL:
				panAxis.recal();
				tiltAxis.recal();

				Serial.println("Command: RECAL");
				break;

			default:
				Serial.print("Command: UNKNOWN = ");
				Serial.print(rxBuffer.cmd);
				Serial.print(", input parameter = ");
				Serial.println(rxBuffer.payload[0]);
				break;
		}
	}
}

int roundUp(int a, int b)
{
	int inter = a/b;  // this is round down
	if (inter*b<a) inter++;
	return inter;
}

