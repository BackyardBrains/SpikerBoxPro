/* --COPYRIGHT--
 *
 *  Backyard Brains 2015
 *  Build 1.02
 *
 *  This version has power rail detection and it responds on message about power state.
 *  It works with active HIGH RGB LED
 *  It has power OFF timer
 *  Stanislav Mircic 26 Sep 2016
 * ======== main.c ========
 */
#include <string.h>

#include "driverlib.h"

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"
#include "USB_API/USB_HID_API/UsbHid.h"
#include "USB_app/usbConstructs.h"

/*
 * NOTE: Modify hal.h to select a specific evaluation board and customize for
 * your own board.
 */
#include "hal.h"


//================ Parameters ===================
#define FIRMWARE_VERSION "1.05"  // firmware version. Try to keep it to 4 characters
#define HARDWARE_TYPE "MUSCLESB" // hardware type/product. Try not to go over 8 characters (MUSCLESB, NEURONSB)
#define HARDWARE_VERSION "1.0"  // hardware version. Try to keep it to 4 characters
#define COMMAND_RESPONSE_LENGTH 35  //16 is just the delimiters etc.
#define DEBOUNCE_TIME 2000
#define MAX_SAMPLE_RATE "10000" //this will be sent to Host when host asks for maximal ratings
#define MAX_NUMBER_OF_CHANNELS "2" //this will be sent to Host when host asks for maximal ratings

//operation modes
#define OPERATION_MODE_DEFAULT 0
#define OPERATION_MODE_BNC 1
#define OPERATION_MODE_FIVE_DIGITAL 2
#define OPERATION_MODE_ARDUINO_BOARD 3
#define OPERATION_MODE_MORE_ANALOG 4

#define TEN_K_SAMPLE_RATE 1600
#define HALF_SAMPLE_RATE 3200
#define POWER_SAVE_MODE_PERIOD 65000


#define LOW_RAIL_VOLTAGE_FIRST_HIGH 840
#define LOW_RAIL_VOLTAGE_FIRST_LOW 822 //2.65V @3.3V
#define LOW_RAIL_VOLTAGE_SECOND_HIGH 700
#define LOW_RAIL_VOLTAGE_SECOND_LOW 682 //2.2V @3.3V
#define BATERY_DEATH_VOLTAGE_HIGH 530
#define BATERY_DEATH_VOLTAGE_LOW 490 //1V @2.6


#define POWER_MODE_SOLID_GREEN 0
#define POWER_MODE_SOLID_RED 1
#define POWER_MODE_BLINKING_RED 2
#define POWER_MODE_LEDS_OFF 3
int powerMode = 0;

#define GREEN_LED BIT0
#define BLUE_LED BIT1
#define RED_LED BIT1

#define RELAY_OUTPUT BIT7

#define POWER_ENABLE BIT0
#define POWER_RAIL_MEASUREMENT_PIN BIT0

#define IO1 BIT2
#define IO2 BIT3
#define IO3 BIT4
#define IO4 BIT5
#define IO5 BIT6
//p5.1 = A9 is used to correct Vcc/2 offset
#define VCCTWO BIT1

int correctionVccOverTwo = 0;
unsigned int tempCorrectionVariable = 0;

int lowRailVoltageDetected = 0;
unsigned int blinkingLowVoltageTimer = 0;
#define BLINKING_LOW_VOLTAGE_TIMER_MAX_VALUE 5000


unsigned int blinkingBoardDetectionTimer = 0;
#define BLINKING_BOARD_DETECTION_TIMER_MAX_VALUE 2500
//===============================================

// Global flags set by events
volatile uint8_t bHIDDataReceived_event = FALSE; // Indicates data has been rx'ed
                                              // without an open rx operation
volatile uint8_t bHIDDataSent_event = FALSE;
// Application globals
uint16_t w;
volatile uint16_t rounds = 0;
char outString[32];
char pakOutString[16];

char *commandDeimiter = ";";
char *parameterDeimiter = ":";



#define MEGA_DATA_LENGTH 2048//992
#define RECEIVE_BUFFER_LENGTH 62

uint8_t tempSendBuffer[62];
uint8_t circularBuffer[MEGA_DATA_LENGTH];

unsigned int tempADCresult;
unsigned int head;
unsigned int tail;
unsigned int difference;
unsigned int generalIndexVar;


unsigned int tempIndex = 0;//used for parsing config parameters etc.
unsigned int counterd = 0;//debug counter

unsigned int weHaveDataToSend = 0;//complete data available in buffer - flag
unsigned int numberOfChannels = 2;//current number of channels

//debouncer variables
//used for events in normal mode and reaction timer
unsigned int debounceTimer1 = 0;
unsigned int debounceTimer2 = 0;
unsigned int debounceTimer3 = 0;
unsigned int debounceTimer4 = 0;
unsigned int debounceTimer5 = 0;
unsigned int eventEnabled1 = 1;
unsigned int eventEnabled2 = 1;
unsigned int eventEnabled3 = 1;
unsigned int eventEnabled4 = 1;
unsigned int eventEnabled5 = 1;

//if watchdog timer for battery is enabled (default 1 - enabled)
unsigned int saveBatteryTimerEnabled = 1;
//5 seconds indication blue LED timer max value
#define SAVE_BATTERY_LED_COUNTER_MAX_VALUE 50000
//5 seconds indication blue LED timer
unsigned int saveBatteryLEDIndicatorCounter = 0;
#define POWER_WATCH_DOG_TIMER_MAX_VALUE 18000000 //1sec = 10000 30min = 18000000
long powerWatchDogTimerCounter = 0;
unsigned int resetPowerWatchDogTimerCounter = 0;
#define THRESHOLD_FOR_POWER_SAVING 574 //(1.85/3.3)*1023
unsigned int weAreInPowerSaveMode = 0;
unsigned int counterForBlinkingLedPowerSave = 0;

//flag to start BSL
unsigned int enterTheBSL = 0;

//flag that lock circular buffer
//(used to block adding samples while adding messages)
unsigned int circularBufferLocked = 0;

//changes when we detect board
unsigned int operationMode = 0;

//used to detect changing in voltage on board detection
int lastEncoderVoltage = 0;
//used to detect changing in voltage on board detection
int currentEncoderVoltage = 0;
//measure time after voltage changes on board detection input
unsigned int debounceEncoderTimer = 0;
//flag that is active when board detection voltage stabilize
unsigned int encoderEnabled = 0;

//enable sampling of analog channels
//if zero sample timer will work but we will not
//collect any analog data
unsigned int sampleData = 0;


#define BOARD_DETECTION_TIMER_MAX_VALUE 15000 //1.5 sec


void defaultSetupADC();
void setupPeriodicTimer();
void sendStringWithEscapeSequence(char * stringToSend);
void executeCommand(char * command);
void setupOperationMode(void);

void main (void)
{

    // Set up clocks/IOs.  initPorts()/initClocks() will need to be customized
    // for your application, but MCLK should be between 4-25MHz.  Using the
    // DCO/FLL for MCLK is recommended, instead of the crystal.  For examples 
    // of these functions, see the complete USB code examples.  Also see the 
    // Programmer's Guide for discussion on clocks/power.
    WDT_A_hold(WDT_A_BASE); // Stop watchdog timer
    
    // Minimum Vcore setting required for the USB API is PMM_CORE_LEVEL_2 .
#ifndef DRIVERLIB_LEGACY_MODE
    PMM_setVCore(PMM_CORE_LEVEL_2);
#else
    PMM_setVCore(PMM_BASE, PMM_CORE_LEVEL_2);
#endif
    
       initPorts();            // Config GPIOS for low-power (output low)
       initClocks(16000000);   // Config clocks. MCLK=SMCLK=FLL=8MHz; ACLK=REFO=32kHz
       USB_setup(TRUE, TRUE);  // Init USB & events; if a host is present, connect
       operationMode = OPERATION_MODE_DEFAULT;
       defaultSetupADC();
       setupOperationMode();   //setup GPIO
       setupPeriodicTimer();   //setup periodic timer for sampling

       counterd = 0;
       enterTheBSL = 0;

       //setup buffers variables for ADC sampling
       head = 0;
       tail = 0;
       difference = 0;
       generalIndexVar = 0;
       //prepare state for USB
       bHIDDataSent_event = TRUE;


       //LED diode (BIT0, BIT1) and relay (BIT7)
       P4SEL = 0;//digital I/O
       P4DIR = GREEN_LED + BLUE_LED + RELAY_OUTPUT;
       P4OUT = 0;//active LOW RGB


       //Enable disable powersupply on P1.0
       //Power down blinking LED on P1.1
	   P1SEL = 0;//digital I/O
	   P1DIR = RED_LED + POWER_ENABLE;
	   P1OUT =  POWER_ENABLE;




       //set A9 for Vcc/2 reference (for offset correction)
	   //set A8 for measurement of power rail
       REFCTL0 &= ~REFON;//turn off ref. function
       P5SEL |= VCCTWO +POWER_RAIL_MEASUREMENT_PIN;

       if(P6IN & IO2)
       {
    	   saveBatteryTimerEnabled = 0;
       }
       if(saveBatteryTimerEnabled==0)
       {
    	   saveBatteryLEDIndicatorCounter = SAVE_BATTERY_LED_COUNTER_MAX_VALUE;
    	   //no action will be taken to check activity on analog inputs
    	   powerWatchDogTimerCounter = 0;
       }
       else
       {

    	   powerWatchDogTimerCounter = POWER_WATCH_DOG_TIMER_MAX_VALUE;
       }



       __enable_interrupt();  // Enable interrupts globally

       while (1)
       {

    	   if(weAreInPowerSaveMode==1)
    	   {
    		   while(1)
    		   {
    			   __no_operation();
    		   }

    	   }
    	   if(enterTheBSL)//if we should update firmware
    	   {
    		   	   USB_disconnect(); //disconnect from USB and enable BSL to enumerate again
    		   	   __disable_interrupt(); // Ensure no application interrupts fire during BSL
    		   	   ((void (*)())0x1000)(); // This sends execution to the BSL.
    	   }


           switch (USB_connectionState())
           {
			   // This case is executed while your device is connected to the USB
			   // host, enumerated, and communication is active. Never enter LPM3/4/5
			   // in this mode; the MCU must be active or LPM0 during USB communication
               case ST_ENUM_ACTIVE:

            	   //--------- receiving
            	   if(bHIDDataReceived_event)
            	   {


            		   // Holds the new  string
					   char receivedString[RECEIVE_BUFFER_LENGTH] = "";
					   char * command1;
					   char * command2;
					   char * command3;

					   // Add bytes in USB buffer to the string
					   hidReceiveDataInBuffer((uint8_t*)receivedString,
						   RECEIVE_BUFFER_LENGTH,
						   HID0_INTFNUM);
					   command1 = strtok(receivedString, commandDeimiter);
					   command2 = strtok(NULL, commandDeimiter);
					   command3 = strtok(NULL, commandDeimiter);
					   executeCommand(command1);
					   executeCommand(command2);
					   executeCommand(command3);

            		   bHIDDataReceived_event = FALSE;

            	   }

            	   if(difference > generalIndexVar)
            	   {
            		   while(difference > generalIndexVar && generalIndexVar<62)
            		   {
						   tempSendBuffer[generalIndexVar] = circularBuffer[tail];
						   generalIndexVar++;
						   tail++;
						   if(tail==MEGA_DATA_LENGTH)
						   {
							   tail = 0;
						   }
					   }
            	   }

            	   // ----------- sending
            	   if (difference > 61 && bHIDDataSent_event)
            	   {

            		   bHIDDataSent_event = FALSE;

            		   while(difference > generalIndexVar && generalIndexVar<62)
					   {
						   tempSendBuffer[generalIndexVar] = circularBuffer[tail];
						   generalIndexVar++;
						   tail++;
						   if(tail==MEGA_DATA_LENGTH)
						   {
							   tail = 0;
						   }
					   }
            		   generalIndexVar = 0;
            		   difference -=62;
            		   //103usec
					   if (hidSendDataInBackground((uint8_t*)tempSendBuffer,62,
													   HID0_INTFNUM,0))
					   {
							   // Operation probably still open; cancel it
							   USBHID_abortSend(&w,HID0_INTFNUM);
							   break;
					   }
            	   }

                   break;

               // These cases are executed while your device is disconnected from
               // the host (meaning, not enumerated); enumerated but suspended
               // by the host, or connected to a powered hub without a USB host
               // present.
               case ST_PHYS_DISCONNECTED:// physically disconnected from the host
               case ST_ENUM_SUSPENDED:// connected/enumerated, but suspended
               case ST_PHYS_CONNECTED_NOENUM_SUSP:// connected, enum started, but host unresponsive

            	   // In this example, for all of these states we enter LPM3. If
            	   // the host performs a "USB resume" from suspend, the CPU will
            	   // automatically wake. Other events can also wake the CPU, if
            	   // their event handlers in eventHandlers.c are configured to return TRUE.
                  // __bis_SR_register(LPM3_bits + GIE);
                  // _NOP();

            	  // TA0CCTL0 &= ~CCIE;//stop timer -> this will stop sampling -> this will stop stream
            	   //prepare buffer for next connection
            	   sampleData = 0;
            	  // P4OUT &= ~(RED_LED); //commented because to new 3 color LED
            	   head = 0;
            	   tail = 0;
            	   difference = 0;
				   //setup flag in case somebody pull the USB plug while it was sending
            	   bHIDDataSent_event = TRUE;
                   break;

               // The default is executed for the momentary state
               // ST_ENUM_IN_PROGRESS.  Usually, this state only last a few
               // seconds.  Be sure not to enter LPM3 in this state; USB
               // communication is taking place here, and therefore the mode must
               // be LPM0 or active-CPU.
               case ST_ENUM_IN_PROGRESS:
               default:;
           }


       }  //while(1)
   } //main()


//
// Setup operation mode inputs outputs and state
//
void setupOperationMode(void)
{
	__disable_interrupt();
	switch(operationMode)
	{
		case OPERATION_MODE_BNC:
			TA0CCR0 = TEN_K_SAMPLE_RATE;
			numberOfChannels = 2;
			P6SEL = BIT0+BIT1+BIT7;//analog inputs
			P6DIR = 0;//select all as inputs
			P6REN = ~(BIT0+BIT1+BIT7);
			P6OUT = 0;//put output register to zero
			P4OUT |= RELAY_OUTPUT;
			//default setup of ADC, redefines part of Port 6 pins
			//defaultSetupADC();
		break;
		case OPERATION_MODE_FIVE_DIGITAL:
		case OPERATION_MODE_ARDUINO_BOARD:
			TA0CCR0 = TEN_K_SAMPLE_RATE;
			numberOfChannels = 2;
			P6SEL = BIT0+BIT1+BIT7;//select analog inputs
			//set all to inputs
			P6DIR = 0;
			P6REN = ~(BIT0+BIT1+BIT7);
			P6OUT = 0;//put output register to zero

			P4OUT &= ~(RELAY_OUTPUT);
			//default setup of ADC, redefines part of Port 6 pins
			//defaultSetupADC();
		break;
		case OPERATION_MODE_MORE_ANALOG:
			TA0CCR0 = HALF_SAMPLE_RATE;
			numberOfChannels = 4;
			//make 4 analog inputs and additional for encoder
			P6SEL = BIT0+BIT1+IO4+IO5 +BIT7;
			P6REN = ~(BIT0+BIT1+IO4+IO5 +BIT7);
			P6OUT = 0;//put output register to zero
			P4OUT &= ~(RELAY_OUTPUT);

		break;
		case OPERATION_MODE_DEFAULT:
			TA0CCR0 = TEN_K_SAMPLE_RATE;
			numberOfChannels = 2;
			P6SEL = BIT0+BIT1+BIT7;//select all pins as digital I/O
			P6DIR = 0;//select all as inputs
			P6REN = ~(BIT0+BIT1+BIT7);
			P6OUT = 0;//put output register to zero

			P4OUT &= ~(RELAY_OUTPUT);
			//default setup of ADC, redefines part of Port 6 pins
			//defaultSetupADC();
		break;
		default:
			TA0CCR0 = TEN_K_SAMPLE_RATE;
			numberOfChannels = 2;
			P6SEL = BIT0+BIT1+BIT7;//select all pins as digital I/O
			P6DIR = 0;//select all as inputs
			P6REN = ~(BIT0+BIT1+BIT7);
			P6OUT = 0;//put output register to zero

			P4OUT &= ~(RELAY_OUTPUT);
			//default setup of ADC, redefines part of Port 6 pins
			//defaultSetupADC();

		break;
	}

	__enable_interrupt();
}


//
// Respond to a command. Send response if necessary
//
void executeCommand(char * command)
{
	if(command == NULL)
	{
		return;
	}
   char * parameter = strtok(command, parameterDeimiter);
   if (!(strcmp(parameter, "h"))){//stop
	 //  TA0CCTL0 &= ~CCIE;
	   sampleData = 0;
	  // P4OUT &= ~(RED_LED); //commented because to new 3 color LED
	   return;
   }
   else if (!(strcmp(parameter, "?"))){//get parameters of MSP


	   char responseString[COMMAND_RESPONSE_LENGTH] = "";

	   strcat(responseString, "FWV:");
	   strcat(responseString, FIRMWARE_VERSION );
	   strcat(responseString, ";");

	   strcat(responseString, "HWT:");
	   strcat(responseString, HARDWARE_TYPE );
	   strcat(responseString, ";");

	   strcat(responseString, "HWV:");
	   strcat(responseString, HARDWARE_VERSION);
	   strcat(responseString, ";");

	   sendStringWithEscapeSequence(responseString);

	   return;
   }
   else if (!(strcmp(parameter, "V"))){//check if power rail is ON or OFF
	   	   char responseString[COMMAND_RESPONSE_LENGTH] = "";

	   	   if(powerMode == POWER_MODE_LEDS_OFF)//if we are below 1V than 9V rail is off
	   	   {
	   		   strcat(responseString, "PWR:0;");
	   	   }
	   	   else
	   	   {
	   		strcat(responseString, "PWR:1;");
	   	   }

	   	   sendStringWithEscapeSequence(responseString);

	   	   return;
   }
   else if (!(strcmp(parameter, "max"))){//get parameters of MSP


   	   char responseString[COMMAND_RESPONSE_LENGTH] = "";

   	   strcat(responseString, "MSF:");
   	   strcat(responseString, MAX_SAMPLE_RATE );
   	   strcat(responseString, ";");

   	   strcat(responseString, "MNC:");
   	   strcat(responseString, MAX_NUMBER_OF_CHANNELS );
   	   strcat(responseString, ";");

   	   sendStringWithEscapeSequence(responseString);

   	   return;
      }
   else if (!(strcmp(parameter, "start"))){//start sampling
	   //TA0CCTL0 = CCIE;
	   sampleData = 1;
	  // P4OUT |= RED_LED; //commented because to new 3 color LED
	   return;
   }
   else if (!(strcmp(parameter, "board"))){//get board type
   	   //TA0CCTL0 = CCIE;
	   switch(operationMode)
	   	{
	   		case OPERATION_MODE_BNC:
	   			sendStringWithEscapeSequence("BRD:1;");
	   		break;
	   		case OPERATION_MODE_FIVE_DIGITAL:
	   			sendStringWithEscapeSequence("BRD:3;");
	   		break;
	   		case OPERATION_MODE_ARDUINO_BOARD:
				sendStringWithEscapeSequence("BRD:4;");
			break;
	   		case OPERATION_MODE_MORE_ANALOG:
	   			sendStringWithEscapeSequence("BRD:1;");
	   		break;
	   		case OPERATION_MODE_DEFAULT:
				sendStringWithEscapeSequence("BRD:0;");
			break;
	   	}
   	   return;
      }
   else if (!(strcmp(parameter, "s"))){//sample rate

	   return;
   }
   else if (!(strcmp(parameter, "conf s"))){//sample rate

	   return;
   }
   else if (!(strcmp(parameter, "c"))){//number of channels

	   return;
   }
   else if (!(strcmp(parameter, "update")))
   {
	   // enter the BSL flag. It will enter in the main loop
	   enterTheBSL = 1;
   }
}


void sendStringWithEscapeSequence(char * stringToSend)
{
	char responseBufferWithEscape[COMMAND_RESPONSE_LENGTH+12] = "";

	//Start of escape sequence
	responseBufferWithEscape[0] = 255;
	responseBufferWithEscape[1] = 255;
	responseBufferWithEscape[2] = 1;
	responseBufferWithEscape[3] = 1;
	responseBufferWithEscape[4] = 128;
	responseBufferWithEscape[5] = 255;


	int i = 0;
	//copy string that we want to send
	while(stringToSend[i]!=0)
	{
		responseBufferWithEscape[i+6] = stringToSend[i]; //copy character
		i = i+1;
	}

	//End of escape sequence
	responseBufferWithEscape[i+6] = 255;
	responseBufferWithEscape[i+7] = 255;
	responseBufferWithEscape[i+8] = 1;
	responseBufferWithEscape[i+9] = 1;
	responseBufferWithEscape[i+10] = 129;
	responseBufferWithEscape[i+11] = 255;

	int length= i+12;

	//check if sampling timer is turned ON
	int weAreSendingSamples = TA0CCTL0 & CCIE;

	circularBufferLocked = 1;
		//now put it to output buffer/s
		for(i=0;i<length;i++)
		{

					circularBuffer[head++] = responseBufferWithEscape[i];
					difference++;
					if(head>=MEGA_DATA_LENGTH)
					{
						head = 0;
					}
		}

		//if we are not sending data than add zeros up to one full frame = 62
		if(sampleData==0)
		{
			for(i=length;i<62;i++)
			{
				circularBuffer[head++] = 0;
				difference++;
				if(head>=MEGA_DATA_LENGTH)
				{
					head = 0;
				}
			}
			difference = 62;
		}
		circularBufferLocked = 0;
}



   /*
    * ======== UNMI_ISR ========
    */
   #if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
   #pragma vector = UNMI_VECTOR
   __interrupt void UNMI_ISR (void)
   #elif defined(__GNUC__) && (__MSP430__)
   void __attribute__ ((interrupt(UNMI_VECTOR))) UNMI_ISR (void)
   #else
   #error Compiler not found!
   #endif
   {
       switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG ))
       {
           case SYSUNIV_NONE:
               __no_operation();
               break;
           case SYSUNIV_NMIIFG:
               __no_operation();
               break;
           case SYSUNIV_OFIFG:
   #ifndef DRIVERLIB_LEGACY_MODE
               UCS_clearFaultFlag(UCS_XT2OFFG);
               UCS_clearFaultFlag(UCS_DCOFFG);
               SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
   #else
               UCS_clearFaultFlag(UCS_BASE, UCS_XT2OFFG);
               UCS_clearFaultFlag(UCS_BASE, UCS_DCOFFG);
               SFR_clearInterrupt(SFR_BASE, SFR_OSCILLATOR_FAULT_INTERRUPT);
   #endif
               break;
           case SYSUNIV_ACCVIFG:
               __no_operation();
               break;
           case SYSUNIV_BUSIFG:
               // If the CPU accesses USB memory while the USB module is
               // suspended, a "bus error" can occur.  This generates an NMI.  If
               // USB is automatically disconnecting in your software, set a
               // breakpoint here and see if execution hits it.  See the
               // Programmer's Guide for more information.
               SYSBERRIV = 0; //clear bus error flag
               USB_disable(); //Disable
       }
}
//Released_Version_4_20_00

//----------- PERIODIC TIMER ---------------------------

void setupPeriodicTimer()
{
	TA0CCR0 = TEN_K_SAMPLE_RATE;//32768=1sec;
	TA0CTL = TASSEL_2+MC_1+TACLR; // ACLK, count to CCR0 then roll, clear TAR
	TA0CCTL0 = CCIE;
}


#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not found!
#endif
{

	if(weAreInPowerSaveMode==1)
	{

		counterForBlinkingLedPowerSave++;
		if(counterForBlinkingLedPowerSave<2200)
		{
			P1OUT &=  ~(RED_LED);
		}
		else if(counterForBlinkingLedPowerSave<2201)
		{
			P1OUT |=  RED_LED;
		}
		else if(counterForBlinkingLedPowerSave<2209)
		{
			P1OUT &=  ~RED_LED;
		}
		else if(counterForBlinkingLedPowerSave<2210)
		{
			P1OUT |=  RED_LED;
		}
		else
		{
			P1OUT &=  ~RED_LED;
			counterForBlinkingLedPowerSave = 0;
		}


		return;
	}
	//P4OUT ^= BIT1;

	//P4OUT ^= RELAY_OUTPUT;
	ADC12CTL0 |= ADC12ENC + ADC12SC;
    //__bic_SR_register_on_exit(LPM3_bits);   // Exit LPM
}

//--------------------- ADC -----------------------------------------------

void defaultSetupADC()
{
	//P4OUT ^= RELAY_OUTPUT;

   //select recording inputs and board detection input(A7)
   //as analog inputs
   P6SEL |= BIT0+BIT1+BIT7;

   // ADC configuration
   //ADC12SHT02 Sampling time 16 cycles,
   //ADC12ON  ADC12 on
   ADC12CTL0 = ADC12SHT01 + ADC12ON +ADC12MSC;

   //ADC12CSTARTADD_0 start conversation address 0;
   //ADC12SHP - SAMPCON signal is sourced from the sampling timer.
   //ADC12CONSEQ_1 Sequence-of-channels (no automatic repeat)
   ADC12CTL1 = ADC12CSTARTADD_0 + ADC12SHP + ADC12CONSEQ_1;

   ADC12CTL2 = ADC12RES_1;//resolution to 10 bits
   // Use A0 as input to register 0
   ADC12MCTL0 = ADC12INCH_0;//recording channel
   ADC12MCTL1 = ADC12INCH_1;//recording channel
   ADC12MCTL2 = ADC12INCH_5;//recording channel
   ADC12MCTL3 = ADC12INCH_6;//recording channel
   ADC12MCTL4 = ADC12INCH_7;//board detection input
   ADC12MCTL5 = ADC12INCH_8;//power rail masurement pin
   ADC12MCTL6 = ADC12INCH_9 +ADC12EOS;
   //ADC12IE = 0x02;//enable interrupt on ADC12IFG2 bit

   ADC12IE = ADC12IE6;//trigger interrupt after conversation of A2 -------- it was ADC12IE5 before adding power rail measurement
   ADC12CTL0 &= ~ADC12SC;
 //  ADC12CTL0 |= ADC12ENC; // Enable conversion


}

#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12ISR (void)
#else
#error Compiler not found!
#endif
{


	//calculate offset correction
	tempADCresult = ADC12MEM6;
	//tempADCresult = 512;//fake measurement debug line remove this !!!!!!!!!!!!!!!!!!!!!
	correctionVccOverTwo = tempADCresult-512;
	tempCorrectionVariable = 1023+correctionVccOverTwo;

	// ------------------------------- CHECK THE POWER RAIL VOLTAGE -------------------------------
	tempADCresult = ADC12MEM5;


    //detect voltage level and set mode (with histeresis)
	if(tempADCresult>=LOW_RAIL_VOLTAGE_FIRST_HIGH)   //Vcc > X > LOW_RAIL_VOLTAGE_FIRST_HIGH
	{
		powerMode = POWER_MODE_SOLID_GREEN;

	}
	else
	{
		if(tempADCresult>=LOW_RAIL_VOLTAGE_FIRST_LOW) //LOW_RAIL_VOLTAGE_FIRST_HIGH > X > LOW_RAIL_VOLTAGE_FIRST_LOW
		{
				if(powerMode == POWER_MODE_SOLID_GREEN)
				{
					//do nothing it is in POWER_MODE_SOLID_GREEN
				}
				else
				{
					powerMode = POWER_MODE_SOLID_RED;
				}

		}
		else
		{
			if(tempADCresult>=LOW_RAIL_VOLTAGE_SECOND_HIGH)  //LOW_RAIL_VOLTAGE_FIRST_LOW > X > LOW_RAIL_VOLTAGE_SECOND_HIGH
			{
				powerMode = POWER_MODE_SOLID_RED;
			}
			else
			{
				if(tempADCresult>=LOW_RAIL_VOLTAGE_SECOND_LOW)  //LOW_RAIL_VOLTAGE_SECOND_HIGH > X > LOW_RAIL_VOLTAGE_SECOND_LOW
				{
						if(powerMode == POWER_MODE_SOLID_RED)
						{
							//do nothing it is in POWER_MODE_SOLID_GREEN
						}
						else
						{
							powerMode = POWER_MODE_BLINKING_RED;
						}
				}
				else
				{
					if(tempADCresult>=BATERY_DEATH_VOLTAGE_HIGH) //LOW_RAIL_VOLTAGE_SECOND_LOW > X > BATERY_DEATH_VOLTAGE_HIGH
					{
						powerMode = POWER_MODE_BLINKING_RED;
					}
					else
					{

						if(tempADCresult>=BATERY_DEATH_VOLTAGE_LOW)  //BATERY_DEATH_VOLTAGE_HIGH > X > BATERY_DEATH_VOLTAGE_LOW
						{
								if(powerMode == POWER_MODE_BLINKING_RED)
								{
									//do nothing it is in POWER_MODE_SOLID_GREEN
								}
								else
								{
									powerMode = POWER_MODE_LEDS_OFF;
								}
						}
						else                                     //BATERY_DEATH_VOLTAGE_LOW > X
						{
							powerMode = POWER_MODE_LEDS_OFF;
						}
					}

				}
			}

		}

	}

	//set LEDs according to power mode
	switch(powerMode)
	{
		case POWER_MODE_SOLID_GREEN:
			P4OUT |= GREEN_LED;
			P1OUT &=  ~(RED_LED);
			P1OUT |= POWER_ENABLE;
		break;
		case POWER_MODE_SOLID_RED:
			P4OUT &= ~(GREEN_LED);
			P1OUT |=  RED_LED;
			P1OUT |= POWER_ENABLE;
		break;
		case POWER_MODE_BLINKING_RED:
			P1OUT |= POWER_ENABLE;
			if(blinkingLowVoltageTimer>0)
			{
				blinkingLowVoltageTimer = blinkingLowVoltageTimer -1;
			}
			else
			{
				blinkingLowVoltageTimer = BLINKING_LOW_VOLTAGE_TIMER_MAX_VALUE;

				P1OUT ^=  RED_LED;//blinking red
				P4OUT &=  ~(GREEN_LED);
			}
		break;
		case POWER_MODE_LEDS_OFF:
			P1OUT &= ~(POWER_ENABLE);
			P1OUT &=  ~(RED_LED);
			P4OUT &=  ~(GREEN_LED);
		break;
	}





	//--------------------------------------------------------------------------------------------


	//
	// ------------------- BOARD DETECTION -----------------------------

	currentEncoderVoltage = ADC12MEM4;

	if(debounceEncoderTimer>0)
	{
		debounceEncoderTimer = debounceEncoderTimer -1;

		if(blinkingBoardDetectionTimer>0)
		{
			blinkingBoardDetectionTimer = blinkingBoardDetectionTimer -1;
			//turn off all except blue LED
			P1OUT &=  ~(RED_LED);
			P4OUT &=  ~(GREEN_LED);

		}
		else
		{
			blinkingBoardDetectionTimer = BLINKING_BOARD_DETECTION_TIMER_MAX_VALUE;
			P4OUT ^=  BLUE_LED;//blink blue LED
			//turn off all except blue led
			P1OUT &=  ~(RED_LED);
			P4OUT &=  ~(GREEN_LED);
		}
	}
	else
	{
		P4OUT &=  ~(BLUE_LED);
		//if board detection voltage is changing
		if((currentEncoderVoltage - lastEncoderVoltage)>100 || (lastEncoderVoltage - currentEncoderVoltage)>100)
		{
			debounceEncoderTimer = BOARD_DETECTION_TIMER_MAX_VALUE;
		}
	}

	if(debounceEncoderTimer==0)
	{
		if(currentEncoderVoltage < 155 )
		{
			//default
			if(operationMode != OPERATION_MODE_DEFAULT)
			{
				operationMode = OPERATION_MODE_DEFAULT;
				sendStringWithEscapeSequence("BRD:0;");
				setupOperationMode();

			}

		}
		else if((currentEncoderVoltage >= 155) && (currentEncoderVoltage < 310))
		{
			//first board BNC
			if(operationMode != OPERATION_MODE_MORE_ANALOG)
			{
				operationMode = OPERATION_MODE_MORE_ANALOG;
				sendStringWithEscapeSequence("BRD:1;");
				setupOperationMode();

			}

		}
		else if((currentEncoderVoltage >= 310) && (currentEncoderVoltage < 465))
		{
			//second board - Reaction timer

			if(operationMode != OPERATION_MODE_BNC)
			{
				operationMode = OPERATION_MODE_BNC;
				sendStringWithEscapeSequence("BRD:3;");
				setupOperationMode();

			}

		}
		else if((currentEncoderVoltage >= 465) && (currentEncoderVoltage < 620))
		{
			//third board - dev board
			if(operationMode != OPERATION_MODE_FIVE_DIGITAL)
			{
				operationMode = OPERATION_MODE_FIVE_DIGITAL;
				sendStringWithEscapeSequence("BRD:2;");
				setupOperationMode();

			}


		}
		else if((currentEncoderVoltage >= 620) && (currentEncoderVoltage < 775))
		{
			//forth board
			if(operationMode != OPERATION_MODE_ARDUINO_BOARD)
			{
				operationMode = OPERATION_MODE_ARDUINO_BOARD;
				sendStringWithEscapeSequence("BRD:4;");
				setupOperationMode();

			}
		}
		else if((currentEncoderVoltage >= 775) && (currentEncoderVoltage < 930))
		{
			//fifth board

		}
		else if((currentEncoderVoltage >= 930))
		{
			//sixth board

		}
	}

	lastEncoderVoltage = currentEncoderVoltage;


	//------------------------ POWER SAVE INDICATION --------------------------------------------
		if(saveBatteryLEDIndicatorCounter>0)
		{
			saveBatteryLEDIndicatorCounter--;
			if (saveBatteryLEDIndicatorCounter==0)
			{
				P4OUT &=  ~(BLUE_LED);
			}
			else
			{
				P4OUT |=  BLUE_LED;
				P1OUT &=  ~(RED_LED);
				P4OUT &=  ~(GREEN_LED);
			}
		}


	//--------------------- BOARD EXECUTION -------------------------------


	switch(operationMode)
		{

			case OPERATION_MODE_DEFAULT:
			case OPERATION_MODE_FIVE_DIGITAL:
			case OPERATION_MODE_ARDUINO_BOARD:
				//two additional digital inputs

				//================= EVENT 4 code ======================

				if(debounceTimer4>0)
				{
					debounceTimer4 = debounceTimer4 -1;
				}
				else
				{
					if(eventEnabled4>0)
					{
							if(P6IN & IO4)
							{
									eventEnabled4 = 0;
									debounceTimer4 = DEBOUNCE_TIME;
									sendStringWithEscapeSequence("EVNT:4;");
							}
					}
					else
					{
						if(!(P6IN & IO4))
						{
							eventEnabled4 = 1;
						}

					}
				}

				//================= EVENT 5 code ======================

				if(debounceTimer5>0)
				{
					debounceTimer5 = debounceTimer5 -1;
				}
				else
				{
					if(eventEnabled5>0)
					{
							if(P6IN & IO5)
							{
									eventEnabled5 = 0;
									debounceTimer5 = DEBOUNCE_TIME;
									sendStringWithEscapeSequence("EVNT:5;");
							}
					}
					else
					{
						if(!(P6IN & IO5))
						{
							eventEnabled5 = 1;
						}

					}
				}


			case OPERATION_MODE_BNC:
			case OPERATION_MODE_MORE_ANALOG:
			default:
				//============== event 1 =================

					if(debounceTimer1>0)
					{
						debounceTimer1 = debounceTimer1 -1;
					}
					else
					{
						if(eventEnabled1>0)
						{
								if(P6IN & IO1)
								{
										eventEnabled1 = 0;
										debounceTimer1 = DEBOUNCE_TIME;
										sendStringWithEscapeSequence("EVNT:1;");
								}
						}
						else
						{
							if(!(P6IN & IO1))
							{
								eventEnabled1 = 1;
							}
						}
					}

					//================= EVENT 2 code ======================

					if(debounceTimer2>0)
					{
						debounceTimer2 = debounceTimer2 -1;
					}
					else
					{
						if(eventEnabled2>0)
						{
								if(P6IN & IO2)
								{
										eventEnabled2 = 0;
										debounceTimer2 = DEBOUNCE_TIME;
										sendStringWithEscapeSequence("EVNT:2;");
								}
						}
						else
						{
							if(!(P6IN & IO2))
							{
								eventEnabled2 = 1;
							}

						}
					}

					//================= EVENT 3 code ======================

					if(debounceTimer3>0)
					{
						debounceTimer3 = debounceTimer3 -1;
					}
					else
					{
						if(eventEnabled3>0)
						{
								if(P6IN & IO3)
								{
										eventEnabled3 = 0;
										debounceTimer3 = DEBOUNCE_TIME;
										sendStringWithEscapeSequence("EVNT:3;");
								}
						}
						else
						{
							if(!(P6IN & IO3))
							{
								eventEnabled3 = 1;
							}

						}
					}

		}

	//========== ADC code ======================

	if(circularBufferLocked ==0)
	{
		    resetPowerWatchDogTimerCounter = 0;
			tempIndex = head;//remember position of begining of frame to put flag bit
			tempADCresult = ADC12MEM0;
			//correct DC offset
			if((int)tempADCresult<correctionVccOverTwo)
			{
				tempADCresult = 0;
			}
			else if(tempCorrectionVariable<tempADCresult)
			{
					tempADCresult = 1023;
			}
			else
			{
				tempADCresult = tempADCresult - correctionVccOverTwo;
			}

			if(powerWatchDogTimerCounter>0)
			{
				if(tempADCresult>THRESHOLD_FOR_POWER_SAVING)
				{
					resetPowerWatchDogTimerCounter = 1;
				}
			}


			if(sampleData == 1)
			{
				circularBuffer[head++] = (0x7u & (tempADCresult>>7));
				difference++;
				if(head==MEGA_DATA_LENGTH)
				{
					head = 0;
				}
				circularBuffer[head++] = (0x7Fu & tempADCresult);
				difference++;
				if(head==MEGA_DATA_LENGTH)
				{
					head = 0;
				}
			}



			tempADCresult = ADC12MEM1;

			//correct DC offset
			if((int)tempADCresult<correctionVccOverTwo)
			{
				tempADCresult = 0;
			}
			else if(tempCorrectionVariable<tempADCresult)
			{
					tempADCresult = 1023;
			}
			else
			{
				tempADCresult = tempADCresult - correctionVccOverTwo;
			}



			if(powerWatchDogTimerCounter>0)
			{
				if(tempADCresult>THRESHOLD_FOR_POWER_SAVING)
				{
					resetPowerWatchDogTimerCounter = 1;
				}
			}

			if(sampleData == 1)
			{
				circularBuffer[head++] = (0x7u & (tempADCresult>>7));
				difference++;
				if(head==MEGA_DATA_LENGTH)
				{
					head = 0;
				}
				circularBuffer[head++] = (0x7Fu & tempADCresult);
				difference++;
				if(head==MEGA_DATA_LENGTH)
				{
					head = 0;
				}
			}
			if(numberOfChannels>2)
			{
				tempADCresult = ADC12MEM2;

				//correct DC offset
				if((int)tempADCresult<correctionVccOverTwo)
				{
					tempADCresult = 0;
				}
				else if(tempCorrectionVariable<tempADCresult)
				{
						tempADCresult = 1023;
				}
				else
				{
					tempADCresult = tempADCresult - correctionVccOverTwo;
				}


				if(powerWatchDogTimerCounter>0)
				{
					if(tempADCresult>THRESHOLD_FOR_POWER_SAVING)
					{
						resetPowerWatchDogTimerCounter = 1;
					}
				}

				if(sampleData == 1)
				{
					circularBuffer[head++] = (0x7u & (tempADCresult>>7));
					difference++;
					if(head==MEGA_DATA_LENGTH)
					{
						head = 0;
					}
					circularBuffer[head++] = (0x7Fu & tempADCresult);
					difference++;
					if(head==MEGA_DATA_LENGTH)
					{
						head = 0;
					}
				}

				tempADCresult = ADC12MEM3;

				//correct DC offset
				if((int)tempADCresult<correctionVccOverTwo)
				{
					tempADCresult = 0;
				}
				else if(tempCorrectionVariable<tempADCresult)
				{
						tempADCresult = 1023;
				}
				else
				{
					tempADCresult = tempADCresult - correctionVccOverTwo;
				}

				if(powerWatchDogTimerCounter>0)
				{
					if(tempADCresult>THRESHOLD_FOR_POWER_SAVING)
					{
						resetPowerWatchDogTimerCounter = 1;
					}
				}

				if(sampleData == 1)
				{


					circularBuffer[head++] = (0x7u & (tempADCresult>>7));
					difference++;
					if(head==MEGA_DATA_LENGTH)
					{
						head = 0;
					}
					circularBuffer[head++] = (0x7Fu & tempADCresult);
					difference++;
					if(head==MEGA_DATA_LENGTH)
					{
						head = 0;
					}
				}

			}

			circularBuffer[tempIndex] |= BIT7;//put flag for begining of frame

			if(resetPowerWatchDogTimerCounter==1)
			{
				powerWatchDogTimerCounter = POWER_WATCH_DOG_TIMER_MAX_VALUE;
			}




	}
	else
	{
		tempADCresult = ADC12MEM0;
		tempADCresult = ADC12MEM1;
	}
	if(powerWatchDogTimerCounter>0)
	{
		powerWatchDogTimerCounter--;
		if(powerWatchDogTimerCounter==0)
		{
			//put MSP to sleep, disable voltage regulator
			P1OUT &=  ~(POWER_ENABLE);
			P1OUT &=  ~(RED_LED);
			P4OUT &=  ~(GREEN_LED);
			P4OUT &=  ~(BLUE_LED);
			USB_disconnect();
			USB_disable();

			//disable ADC
			ADC12CTL0 &=~ADC12ENC;
			ADC12CTL0 &=~(REFON + ADC12ON);
			ADC12CTL0 =0;

			TA0CCR0 = POWER_SAVE_MODE_PERIOD;
			TA0CTL = TASSEL_2+MC_1+TACLR+ID1; // ACLK, count to CCR0 then roll, clear TAR
			weAreInPowerSaveMode = 1;
			//__disable_interrupt();
			//enter low power mode
			//  PMMCTL0_H = PMMPW_H;                      // open PMM
			//  PMMCTL0_L |= PMMREGOFF;                   // set Flag to enter LPM4.5 with LPM4 request
			//  LPM4;
			//  __no_operation();// now enter LPM4.5
			return;
		}
	}
//Uncomment this when not using repeat of sequence
	ADC12CTL0 &= ~ADC12SC;


}
