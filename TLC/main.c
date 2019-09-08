/*COMPSYS 303
 *Assignment 1
 *Simple Traffic Light Controller
 *Group 10 Submission
 *Authors:
 *	- Muhammad Azizul Islam
 *	- Babra Ajaz
 */

#include <system.h>
#include <altera_avalon_pio_regs.h> //lets us read and write operations to the pio (parallel input output) register
#include <stdio.h>
#include <stdlib.h>
#include <alt_types.h> // alt_u32 is a kind of alt_types
#include "sys/alt_alarm.h"
#include <sys/alt_irq.h>

//Initializing MACROs for Green LEDs
//corresponding to red yellow and green Lights
//in a normal traffic light
#define NS_G   0b00000001
#define NS_Y   0b00000010
#define NS_R   0b00000100
#define EW_G   0b00001000
#define EW_Y   0b00010000
#define EW_R   0b00100000

//additionally also initializing MACROs
//for green LEDs corresponding to the Pedestrian Crossing
// from the North South or East West side
#define NS_Ped 0b01000000
#define EW_Ped 0b10000000

//INIT LCD
#define ESC 27
#define CLEAR_LCD_STRING "[2J"

//Function declarations

void setMode(unsigned int val, FILE* lcd);
void mode1_simple_tlc();
void mode2_simple_tlc();
void configurableTLC(unsigned int uiSwitchValue);
void timeout_data_handler();
void camera_tlc();
void vehicleTimeout();
void changeLCD(FILE *lcd);
void NSEW_ped_handler();
unsigned int switchLogic(unsigned int previous, unsigned int current, unsigned int switchValue, unsigned int mode);

void timerHandle(unsigned int timeout);
void checkSwitch(unsigned int switchVal);
void TXUART(FILE* uartFp);
void RXUART(FILE* uartFp);
void setpedLED();

//State initializations for Traffic Light Controller
typedef enum States{
	RedRed , //1001000
	GreenRed,
	GreenRedP1,//pedestrian NS
	YellowRed,
	RedRed2,
	RedGreen,
	RedGreenP2,//pedestrian EW
	RedYellow
} States;

void changeState(enum States nextState);

/*PEDESTRIAN STATE MACHINE*/
//State Initializations for handling Pedestrian Requests
//for the North South Direction
typedef enum NSStates{
	IdleNS,
	NSPressed
} NSStates;

//State Initializations for handling Pedestrian
//requests for the East West Direction
typedef enum EWStates{
	IdleEW,
	EWPressed
} EWStates;

//Variables associated with the change of mode
//to be used in conjunction with all functionality
//as all states/functionality/advancement is
//dependent on this
volatile unsigned int modeState = 0;
unsigned int prevMode = 0;
unsigned int currentMode = 0;

//Global Variable initializations for the
//system to read and change according to
//the specified conditions
enum States currentState = RedRed;
enum NSStates currentNS = IdleNS;
enum EWStates currentEW = IdleEW;

/*initializing Pedestrian state
 *machine outputs and inputs */
unsigned int NSHandled = 0;
unsigned int EWHandled = 0;
unsigned int pedNS = 0;
unsigned int pedEW = 0;
//volatile variables because
//they are changed through ISRs
volatile unsigned int NSRaised = 0;
volatile unsigned int EWRaised = 0;

//initialized timeout Values
//to ones that were specified
//in the announcement
unsigned int t1 = 500;
unsigned int t2 = 6000;
unsigned int t3 = 2000;
unsigned int t4 = 500;
unsigned int t5 = 6000;
unsigned int t6 = 2000;

//initializing timers and
//flags associated with the usage
//of timers outside of the main
//this timer is for the operation of
//the main traffic light controller
//with pedestrian control
alt_alarm timer;
int timerStart = 0;
volatile int timeout = 0;

//variables used as flags to
//achieve MODE 3 functionality
unsigned int UARTCheck = 0;
unsigned int UARTComplete = 0;
unsigned int complete = 0;

//variables to compute if the
//data packet through UART is valid
unsigned int slash = 0;
unsigned int slashN = 0;
unsigned int slashR = 0;
unsigned int countSlash = 0;
unsigned int incorrectEnd = 0;
unsigned int incorrectPacket = 0;

//variable used to print different messages onto
//PUTTY, used in MODE 3 and 4
unsigned int activateUART = 0;
unsigned int welcome = 0;

//variables associated with MODE 4
//timer used to time 2 seconds after
//car has entered yellow state
//timer is also used to calculate the
//amount of time a vehicle spends in an
//intersection
alt_alarm camera_timer;
volatile unsigned int violationCount = 0;
volatile unsigned int cameraCount = 0;
volatile unsigned int interval = 1;
unsigned int vehicleTime = 0;
volatile unsigned int entryYellow = 0;
volatile unsigned int entryRed = 0;

//variable takes different values to
//associate different states of a car
//vehicleState = 0 means no car is at intersection
//vehicleState = 1 means a car just entered
//vehicleState = 2 means that a car has entered and is currently in the intersection
//vehicleState = 3 means that the car has left the intersection
//volatile as it changes in the button ISR
volatile int vehicleState = 0;


//Different ISRs that trigger upon different interactions with the
//traffic light controller

//TIMER 1
//used for timely operation of every state of the traffic light
//controller
alt_u32 timer_isr_function(void* context)
{
	//after the timer counts a specified amount of time
	//this flag timeout goes high for the state machine to
	//read and change states
	timeout = 1;

	//timer stops
	return 0;
}

//CAMERA_TIMER
//this camera is for mode 4 and achieves 2 functionalities
//measures 2 seconds upon entry into the YellowRed/RedY ellow
//state. Also times the amount of time the car spends in an
//intersection
alt_u32 camera_timer_isr(void* context)
{
	if (entryYellow == 1 || entryRed == 1){
		violationCount++;
	}
	cameraCount++;

	return interval;
}

//Button Handling for Pedestrians and Mode 4 car entering
//key 0 is for NS pedestrians
//key 1 is for EW pedestrians
//key 2 is for car entering and car leaving
void button_interrupts_function(void* context, alt_u32 id)
{
	int* temp = (int*) context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(KEYS_BASE);

	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(KEYS_BASE, 0);

	//Pedestrian handling and controlling state machine
	//variables for every modes greater than 1
	if (modeState > 1){
		if (*temp == 1){
			NSRaised = 1;
		}
		if (*temp == 2){
			EWRaised = 1;
		}
	}

	//changing the vehicle state (if it has entered or exited)
	//through presses of buttons
	if (modeState == 4){
		if (*temp == 4){
			vehicleState++;
		}
	}
}

int main(){

	//switch value initialization for mode switching
	unsigned int uiSwitchValue = 0;
	unsigned int previous = 0;
	unsigned int current = 0;
	unsigned int mode = 0;

	//initializing buttons
	int buttonValue = 1;
	void* context_going_to_be_passed = (void*) &buttonValue;
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(KEYS_BASE, 0);
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(KEYS_BASE, 0x7);
	alt_irq_register(KEYS_IRQ,context_going_to_be_passed,button_interrupts_function);

	//initializing the uart and lcd files
	FILE *lcd;
	FILE* uartFp;

	while(1)
	{

		uiSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SWITCHES_BASE);

		//configuring the switch values to get the right mode
		mode = switchLogic(previous, current, uiSwitchValue, mode);
		//actually setting that mode from teh value we get from the switch logic
		setMode(mode, lcd);

		//turing on some red leds for when pedestrians press the respective NS/EW buttons
		setpedLED();

		//configuring the pedestrian state machine only if the mode is not 1
		if (modeState > 1){
			NSEW_ped_handler();
		}

		//calling different functionality upon the setting of different modes
		if (modeState == 0){
			//keeping the states in RedRed
			currentState = RedRed;
			IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_R);
			IOWR_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE, 0);
		}
		if (modeState == 1){
			mode1_simple_tlc();
		} else if (modeState == 2){
			mode2_simple_tlc();
		} else if (modeState == 3){
			configurableTLC(uiSwitchValue);
			RXUART(uartFp);
			TXUART(uartFp);
		} else if (modeState == 4){
			vehicleTimeout();
			camera_tlc(uiSwitchValue);
			TXUART(uartFp);
			RXUART(uartFp);
		}

	}
	return 0;
}

//controls the mode logic and computes each mode with the switches being high or low
unsigned int switchLogic(unsigned int previous, unsigned int current, unsigned int switchValue, unsigned int mode){

	previous = current;
	current = switchValue; //assigns it as the value of the current

	if (current != previous){ //checks if current is not equal to previous
		if (current > previous){
			mode = current - previous;
		}
		else if (current == 0) {
			mode = current;
		}
	}
	else if (current == 0) {
		mode = current;
	}
	return mode;
}

//when pedestrians press the respective direction buttons, this function switches on
//red LEDs
void setpedLED(){
	if(modeState > 1){
		if (NSRaised == 1 && EWRaised == 1) {
			IOWR_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE, 3);
		}
		else if (NSRaised == 1){
			IOWR_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE, 1);
		}
		else if (EWRaised == 1){
			IOWR_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE, 2);
		}
		else if (NSRaised == 0 || EWRaised == 0){
			IOWR_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE, 0);
		}
	}
}

//receives new timeout values from the UART
void RXUART(FILE* uartFp){
	char tempByte;
	char temp[300];
	char completeString[300];
	int count = 0;


	while (UARTCheck == 1){
		uartFp = fopen(UART_NAME, "r+");
		if (uartFp != NULL){
			if (welcome == 1){
				fprintf(uartFp, "\r\n\e[1m%s\e[0m\r\n", "Please put the switch down");
				fprintf(uartFp,"%s\r\n", "Enter the desired timeout values in a valid packet format");
				fprintf(uartFp, "\e[4m%s\e[0m\r\n", "Valid Format t1,t2,t3,t4,t5,t6 followed with a slash r slash n");
				fprintf(uartFp, "%s\r\n", "Please note \r\n - t values can be upto 4 digit numbers");
				fprintf(uartFp, "%s\r\n", " - slash n is necessary but slash r is optional");
				welcome = 0;
			}
			tempByte = fgetc(uartFp);
			fprintf(uartFp,"%c", tempByte);

			if (tempByte == '\n') {
				slashN = 1;
				temp[count] = '\0';
				strcpy(completeString, temp);
				timeout_data_handler(completeString);
				count = 0;
				slash = 0;
				countSlash = 0;
				incorrectEnd = 0;
			}
			else if (slashR == 1 && countSlash == 1 && tempByte != '\n' ){
				incorrectEnd = 1;

			}

			else if (tempByte == '\r') {
				printf("%c",tempByte);
				slash = 0;
				slashR = 1;
				countSlash = 1;
			}
			else if (slash != 1 && countSlash != 1) {
				temp[count] = tempByte;
				count++;
			}
			if (incorrectPacket == 1){
				fprintf(uartFp, "\r\n\n\x1B[31m%s\e[0m\r\n", "Incorrect Packet, please enter a 'valid' packet again ");
				fprintf(uartFp, "\e[4m%s\e[0m\r\n", "Valid Format t1,t2,t3,t4,t5,t6 followed with a slash r slash n");
				fprintf(uartFp, "%s\r\n", "Please note \r\n - t values can be upto 4 digit numbers");
				fprintf(uartFp, "%s\r\n", " - slash n is necessary but slash r is optional");
				incorrectPacket = 0;
			}
		}
		fclose(uartFp);
	}
}

//transmitts camera values to through UART
void TXUART(FILE* uartFp){
	uartFp = fopen(UART_NAME, "r+");
	if (uartFp != NULL) // check if the UART is open successfully
	{
		if (activateUART == 1){
			fprintf(uartFp, "%s\r\n\n", "Camera Activated");
			activateUART = 0;
		}
		if (activateUART == 2){
			fprintf(uartFp, "%s\r\n", "Vehicle left");
			fprintf(uartFp, "VehicleTime = %d ms\r\n\n", vehicleTime);
			vehicleState = 0;
			activateUART = 0;
			vehicleTime = 0;
			cameraCount = 0;
		}
		if (activateUART == 3){
			fprintf(uartFp, "\r\n%s\r\n", "Vehicle Entered");
			fprintf(uartFp, "%s\r\n", "Snapshot Taken, you jumped a red light!!!");
			activateUART = 0;
		}
		if (activateUART == 4){
			fprintf(uartFp, "%s\r\n\n", "Snapshot Taken, you have exceeded the time limit");
			entryYellow = 0;
			activateUART = 0;
		}
		if (activateUART == 5){
			fprintf(uartFp, "\r\n%s\r\n", "Vehicle Entered");
			if (entryYellow == 1){
				fprintf(uartFp, "%s\r\n", "Camera Activated");
			}
			activateUART = 0;
		}
		if (activateUART == 6){
			fprintf(uartFp, "\e[91m%s\e[0m\r\n\r", "Potential Hazard, intersection under maintenance");
			activateUART = 0;
			complete = 1;
		}
		if (activateUART == 7){
			fprintf(uartFp, "\r\n\n\e[32m%s\e[0m\r\n", "Time updated Successfully");
			activateUART = 0;
		}
	}
	fclose(uartFp);
}

//changing modes on the LCD screen
void changeLCD(FILE *lcd){
	lcd = fopen(LCD_NAME, "w");
	if(lcd != NULL)
	{
		fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
		fprintf(lcd, "MODE: %d\n", modeState);
		//fprintf(lcd, "NS: %c EW: %c\n", NS, EW);
	}
	fclose(lcd);
}

//takes logic from the switch logic function and sets mode for the system
void setMode(unsigned int val, FILE* lcd) {

	if (currentState == RedRed || currentState == RedRed2){
		if (val == 0) {
			modeState = val;
			if (modeState != prevMode) {
				changeLCD(lcd);
			}
		}
		else if (val == 1) {
			modeState = val;
			if (modeState != prevMode) {
				changeLCD(lcd);
			}
		}
		else if (val == 2) {
			modeState = val;
			if (modeState != prevMode) {
				changeLCD(lcd);
			}
		}
		else if (val == 4) {
			modeState = 3;
			if (modeState != prevMode) {
				changeLCD(lcd);
			}
		}
		else if (val == 8) {
			modeState = 4;
			if (modeState != prevMode) {
				changeLCD(lcd);
			}
		}
	}
	prevMode = modeState;
}

//pedestrian state machine logic
void NSEW_ped_handler(){
	if(currentNS == IdleNS){
		NSHandled = 0;
		if (NSRaised == 1){
			pedNS = 1;
			currentNS = NSPressed;
		}
	} else if (currentNS == NSPressed){
		if (NSHandled == 1){
			pedNS = 0;
			currentNS = IdleNS;
		}
	}

	if (currentEW == IdleEW){
		EWHandled = 0;
		if (EWRaised == 1){
			pedEW = 1;
			currentEW = EWPressed;
		}
	} else if (currentEW == EWPressed){

		if(EWHandled == 1){
			pedEW = 0;
			currentEW = IdleEW;
		}
	}
}

//mode 1 staet machine with a simple traffic controller
void mode1_simple_tlc(){

	resetTimeOut();
	if (currentState == RedRed){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_R);
		timerHandle(t1);
		if (timeout == 1){
			changeState(GreenRed);
		}
	} else if (currentState == GreenRed){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_G + EW_R);
		timerHandle(t2);
		if (timeout == 1){
			changeState(YellowRed);
		}
	} else if (currentState == YellowRed){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_Y + EW_R);
		timerHandle(t3);
		if (timeout == 1){
			changeState(RedRed2);
		}
	} else if (currentState == RedRed2){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_R);

		timerHandle(t4);
		if (timeout == 1){
			changeState(RedGreen);
		}
	} else if (currentState == RedGreen){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_G);
		timerHandle(t5);
		if (timeout == 1){
			changeState(RedYellow);
		}
	} else if (currentState == RedYellow){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_Y);
		timerHandle(t5);
		if (timeout == 1){
			changeState(RedRed);
		}
	}
}

//mode 2 with simple state machine and pedestrian buttons
void mode2_simple_tlc(){

	resetTimeOut();
	if (currentState == RedRed){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_R);
		timerHandle(t1);

		if (timeout == 1 && pedNS == 0){
			changeState(GreenRed);
		}
		else if ( timeout == 1 && pedNS == 1){
			NSRaised = 0;
			changeState(GreenRedP1);
		}
	} else if (currentState == GreenRed){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_G + EW_R);
		timerHandle(t2);

		if (timeout == 1){
			changeState(YellowRed);
		}
	} else if (currentState == YellowRed){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_Y + EW_R);
		timerHandle(t3);

		if (timeout == 1){
			changeState(RedRed2);
		}
	} else if (currentState == RedRed2){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_R);
		timerHandle(t4);
		if (timeout == 1 && pedEW == 0){
			changeState(RedGreen);
		} else if ( timeout == 1 && pedEW == 1){
			EWRaised = 0;
			changeState(RedGreenP2);
		}
	} else if (currentState == RedGreen){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_G);
		timerHandle(t5);
		if (timeout == 1){
			changeState(RedYellow);
		}
	} else if (currentState == RedYellow){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_Y);
		timerHandle(t6);
		if (timeout == 1){
			changeState(RedRed);
		}
	} else if (currentState == GreenRedP1){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_G + EW_R + NS_Ped);
		timerHandle(t2);
		if (timeout == 1){
			NSHandled = 1;
			changeState(YellowRed);
		}
	} else if (currentState == RedGreenP2){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_G + EW_Ped);
		timerHandle(t5);
		if (timeout == 1){
			changeState(RedYellow);
			EWHandled = 1;
		}
	}
}

//mode 3 with simple state machine and pedestrian buttons and configurable timeout values
void configurableTLC(unsigned int uiSwitchValue){

	if (currentState == RedRed){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_R);
		timerHandle(t1);
		checkSwitch(uiSwitchValue);

		if (timeout == 1 && pedNS == 0 && UARTCheck == 0){
			changeState(GreenRed);
			UARTComplete = 0;
		} else if (timeout == 1 && pedNS == 1 && UARTCheck == 0){
			changeState(GreenRedP1);
			NSRaised = 0;
			UARTComplete = 0;
		}
	} else if (currentState == GreenRed){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_G + EW_R);
		timerHandle(t2);

		if (timeout == 1){
			changeState(YellowRed);
		}
	} else if (currentState == YellowRed){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_Y + EW_R);
		timerHandle(t3);
		if (timeout == 1){
			changeState(RedRed2);
		}
	} else if (currentState == RedRed2){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_R);
		timerHandle(t4);
		checkSwitch(uiSwitchValue);

		if (timeout == 1 && pedEW == 0 && UARTCheck == 0){
			changeState(RedGreen);
			UARTComplete = 0;
		} else if ( timeout == 1 && pedEW == 1 && UARTCheck == 0){
			EWRaised = 0;
			changeState(RedGreenP2);
			UARTComplete = 0;
		}

	} else if (currentState == RedGreen){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_G);
		timerHandle(t5);

		if (timeout == 1){
			changeState(RedYellow);
		}
	} else if (currentState == RedYellow){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_Y);
		timerHandle(t6);

		if (timeout == 1){
			changeState(RedRed);
		}
	} else if (currentState == GreenRedP1){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_G + EW_R + NS_Ped);
		timerHandle(t2);

		if (timeout == 1){
			changeState(YellowRed);
			NSHandled = 1;
		}
	} else if (currentState == RedGreenP2){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_G + EW_Ped);
		timerHandle(t5);
		if (timeout == 1){
			changeState(RedYellow);
			EWHandled = 1;
		}
	}
}

//vehicle entrance and exit logic
void vehicleTimeout(){

	int t_c = 0;
	void* camera_timerContext = (void*) &t_c;

	if (vehicleState == 1){
		alt_alarm_start(&camera_timer, interval, camera_timer_isr, camera_timerContext);
		vehicleState = 2;
		if (currentState != GreenRed && currentState != RedGreen && currentState != GreenRedP1 && currentState != RedGreenP2){
			activateUART = 1;
		}
		if (currentState == RedRed || currentState == RedRed2){
			activateUART = 3;
			entryRed = 1;
		}
		if (currentState == YellowRed || currentState == RedYellow){
			activateUART = 5;
			entryYellow = 1;
		}
		if (currentState == GreenRed || currentState == RedGreen || currentState == GreenRedP1 || currentState == RedGreenP2){
			activateUART = 5;
		}

	}

	if (vehicleState == 2){
		if (currentState == RedRed || currentState == RedRed2){
			entryRed = 1;
		}
	}

	if (entryYellow == 1){
		if (violationCount * interval > 2000){
			activateUART = 4;
		}
	}

	if (currentState == RedRed || currentState == RedRed2){
		if (complete == 0){
			if (violationCount * interval == 5000){
				activateUART = 6;
			}
		}
	}

	if (vehicleState == 3){
		alt_alarm_stop(&camera_timer);
		vehicleTime = cameraCount*interval;
		activateUART = 2;
		violationCount = 0;
		entryRed = 0;
		complete = 0;
	}

}

//mode 4 functionality with with simple TLC, pedestrian buttons. configurable timeout values and violation camera
void camera_tlc(unsigned int uiSwitchValue){

	if (currentState == RedRed){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_R);
		timerHandle(t1);
		checkSwitch(uiSwitchValue);

		if (vehicleState == 0 || vehicleState == 3){
			if (timeout == 1 && pedNS == 0 && UARTCheck == 0){
				changeState(GreenRed);
				UARTComplete = 0;
			} else if (timeout == 1 && pedNS == 1 && UARTCheck == 0){
				NSRaised = 0;
				changeState(GreenRedP1);
				UARTComplete = 0;
			}
		}

	} else if (currentState == GreenRed){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_G + EW_R);
		timerHandle(t2);
		if (timeout == 1){
			changeState(YellowRed);
		}
	} else if (currentState == YellowRed){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_Y + EW_R);
		timerHandle(t3);
		if (timeout == 1){
			changeState(RedRed2);
		}

	} else if (currentState == RedRed2){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_R);
		timerHandle(t4);
		checkSwitch(uiSwitchValue);

		if (vehicleState == 0 || vehicleState == 3){
			if (timeout == 1 && pedEW == 0 && UARTCheck == 0){
				changeState(RedGreen);
				UARTComplete = 0;
			} else if ( timeout == 1 && pedEW == 1 && UARTCheck == 0){
				EWRaised = 0;
				changeState(RedGreenP2);
				UARTComplete = 0;
			}
		}
	} else if (currentState == RedGreen){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_G);
		timerHandle(t5);
		if (timeout == 1){
			changeState(RedYellow);
		}
	} else if (currentState == RedYellow){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_Y);
		timerHandle(t6);
		if (timeout == 1){
			changeState(RedRed);
		}
	}
	else if (currentState == GreenRedP1){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_G + EW_R + NS_Ped);
		timerHandle(t2);
		if (timeout == 1){
			changeState(YellowRed);
			NSHandled = 1;
		}
	} else if (currentState == RedGreenP2){
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, NS_R + EW_G + EW_Ped);
		timerHandle(t5);
		if (timeout == 1){
			changeState(RedYellow);
			EWHandled = 1;
		}
	}

}

//timeout logic for configurable timeout values
void timeout_data_handler(char completeString[300]) {
	int packetIndex = 0;
	int j = 0;
	char t1str[10];
	char t2str[10];
	char t3str[10];
	char t4str[10];
	char t5str[10];
	char t6str[10];

	if (incorrectEnd == 1) {
		incorrectPacket = 1;
	}
	if (incorrectPacket == 0){
		for (int i = 0; i < 300; i++) {
			if (completeString[i] == ',') {
				packetIndex++;
				j = 0;
			}
			else if (packetIndex > 5){
				incorrectPacket = 1;
				break;
			}
			else if (packetIndex == 0) {

				if (completeString[i] < 48 || completeString[i] > 57){
					incorrectPacket = 1;
					break;
				} else {
					t1str[j] = completeString[i];
					j++;
					if (j > 4){

						incorrectPacket = 1;
						break;
					}
				}

			}
			else if (packetIndex == 1) {
				if (completeString[i] < 48 || completeString[i] > 57){
					incorrectPacket = 1;
					break;
				} else {
					t2str[j] = completeString[i];
					j++;
					if (j > 4){
						incorrectPacket = 1;

						break;
					}
				}

			}
			else if (packetIndex == 2) {
				if (completeString[i] < 48 || completeString[i] > 57){
					incorrectPacket = 1;
					break;
				} else {
					t3str[j] = completeString[i];
					j++;
					if (j > 4){
						incorrectPacket = 1;

						break;
					}
				}
			}
			else if (packetIndex == 3) {
				if (completeString[i] < 48 || completeString[i] > 57){
					incorrectPacket = 1;
					break;
				} else {
					t4str[j] = completeString[i];
					j++;
					if (j > 4){

						incorrectPacket = 1;
						break;
					}
				}
			}
			else if (packetIndex == 4) {
				if (completeString[i] < 48 || completeString[i] > 57){
					incorrectPacket = 1;
					break;
				} else {
					t5str[j] = completeString[i];
					j++;
					if (j > 4){

						incorrectPacket = 1;
						break;
					}
				}

			}
			else if (packetIndex == 5) {
				if (completeString[i] !='\0') {
					if (completeString[i] < 48 || completeString[i] > 57){
						incorrectPacket = 1;
						break;
					}
				}
				t6str[j] = completeString[i];

				printf("j = %d, completeString[i] = %d\n", j, completeString[i]);
				if (j > 4){

					incorrectPacket = 1;
					break;
				}
				if (completeString[i] != '\0'){
					j++;
				} else {
					t1 = atoi(t1str);
					t2 = atoi(t2str);
					t3 = atoi(t3str);
					t4 = atoi(t4str);
					t5 = atoi(t5str);
					t6 = atoi(t6str);
					printf("t1 = %d,t2 = %d,t3 = %d,t4 = %d,t5 = %d,t6 = %d\n", t1,t2,t3,t4,t5,t6);
					UARTCheck = 0;
					UARTComplete = 1;
					activateUART = 7;
					break;
				}

			}
		}
	}
}

// if you with to return to mode 1/2 it will reset the timeout values to the assignemtn predefined 500ms, 2s and 6s
//for red, yellow and green respectively
void resetTimeOut(){
	t1 = 500;
	t2 = 6000;
	t3 = 2000;
	t4 = 500;
	t5 = 6000;
	t6 = 2000;
}

//change state logic to be used by every state machine
void changeState(enum States nextState){
	currentState = nextState;
	timeout = 0;
	timerStart = 0;
}

//starting/stopping the main timer
void timerHandle(unsigned int timeout){
	//implement after changeState is fully integrated
	int t = 0;
	void* timerContext = (void*) &t;
	if (timerStart == 0){
		alt_alarm_start(&timer, timeout, timer_isr_function, timerContext);
		timerStart = 1;
	}
}

//chekcing if the switch is active to receive configarable timeout values
void checkSwitch(unsigned int switchVal){
	if (switchVal == 20 || switchVal == 24){
		if (UARTComplete == 0){

			UARTCheck = 1;
			welcome = 1;
		}
	}
}
