/*****************************************************************************
 *   EE2024 Assignment 2, done by Wong Kok Woo and Andrew Yeo
 ******************************************************************************/
//ee2024
#include "stdio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_timer.h"
#include <string.h>

#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "led7seg.h"
#include "light.h"
#include "temp.h"

// ##################################### //
// ########  Flags Definitions  ######## //
// ##################################### //

#define CAT 0
#define ACTIVE 1
#define PS 2
#define NO 3
#define FP 4
#define STOP 999
#define ON 6
#define OFF 7
#define START_LED_7_SEG 0
#define START_16_LED 0
#define ON_ALL_16_LED 17

// ##################################### //
// ######  Variable Definitions   ###### //
// ##################################### //

volatile uint32_t msTicks; // counter for 1ms SysTicks

const int PERIOD_MS_4000 = 4000;
const int PERIOD_MS_1000 = 1000;
const int PERIOD_MS_500 = 500;
const int PERIOD_MS_250 = 250;

const char *MESSAGE_LOADING = "LOADING...\n\r";
const char *MESSAGE_PLEASE_WAIT = "PLEASE WAIT\n\r";
const char *MESSAGE_I_WATCH = "I-WATCH\n\r";
const char *MESSAGE_ELECTRONIC = "Electronic\n\r";
const char *MESSAGE_TAG = "Tag\n\r";
const char *MESSAGE_CONFIGURATION = "Configuration\n\r";
const char *MESSAGE_AND = "and\n\r";
const char *MESSAGE_TESTING_MODE = "Testing Mode\n\r";
const char *MESSAGE_SETTINGS = "Settings\n\r";
const char *MESSAGE_RUN_RGB = "Run RGB\n\r";
const char *MESSAGE_RUN_16LED = "Run 16 Led\n\r";
const char *MESSAGE_RUN_7SEG = "Run 7seg\n\r";
const char *CAT_MODE = "*CAT MODE*\n\r";
const char *ACTIVE_PS = "*ACTIVE-PS*\n\r";
const char *LUX_PS = "Lux: PS\n\r";
const char *TA_PS = "TA: PS";
const char *TB_PS = "TB: PS";
const char *X_PS = "X: PS";
const char *Y_PS = "Y: PS";
const char *Z_PS = "Z: PS";
const char *ACTIVE_NO = "*ACTIVE-NO*";
const char *ACTIVE_FP = "*ACTIVE-FP*";

static uint8_t MODE_TOGGLE;
static uint8_t JOYSTICK_STATE;
static int BLINK_BLUE = ON;
static int BLINK_RED = STOP;
static int led7Seg = STOP;
static int led16 = STOP;
static int mode = PS;
static int GET_STATUS = OFF;
static int light;
static uint32_t msTicksCurr;
static int sec;
static int min;
static int hrs;
static float tempA;
static float tempB;
static int8_t x;
static int8_t y;
static int8_t z;
static int32_t xoff;
static int32_t yoff;
static int32_t zoff;

// #################################### //
// #####  Function Declarations   ##### //
// #################################### //

static void init_UART(void);
static void init_ssp(void);
static void init_i2c(void);
static void init_GPIO(void);

void EINT3_IRQHandler(void);
void EINT0_IRQHandler(void);
void SysTick_Handler(void);

void get_Time(void);

void off_led7seg(void);
void off_RGB(void);
void off_Peripherals();

void invert_Segment_Display(int seg_counter);

int displayAnimation_CAT(void);
void displayOled_CAT(void);
void displayOled_Active_PS(void);
void displayOled_Sensors(void);
void clearOled_Sensors(void);

void read_Sensor(void);

void print_Results(void);
void displayCAT_UART(void);
void displayACTIVE_UART(void);

int cat_Mode(void);
int active_Mode(void);

// ########################### //
// ##### Implementations ##### //
// ########################### //

static void init_UART(void) {
	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;

	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);

	UART_Init(LPC_UART3, &uartCfg);
	UART_TxCmd(LPC_UART3, ENABLE);
}

static void init_ssp(void) {
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */

	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void) {
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void) {
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;

	//SW4
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1 << 31, 0);

	//RGB Blue
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 26;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0, 1 << 26, 1);

	//RGB Red
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, (1 << 0), 1);

	//Light sensor interrupt
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 5;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1 << 5, 0);
	LPC_GPIOINT ->IO2IntEnF |= 1 << 5;

	//SW3
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 2 << 10, 0);
	LPC_SC ->EXTMODE |= 1 << 0;
	LPC_SC ->EXTPOLAR |= 0 << 0;

	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);
	LPC_SC ->EXTINT |= 1 << 0;
	NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT0_IRQn);

}

void EINT3_IRQHandler(void) {
//	int lightValue;

	// Determine whether GPIO Interrupt P2.5 has occurred
	if ((LPC_GPIOINT ->IO2IntStatF >> 5) & 0x1) {

		light = light_read();

		//Power saver mode
		if (light < 50) {
			mode = PS;
			//Normal operation mode
		} else if (light >= 50 && light <= 900) {
			mode = NO;
			//Full performance mode
		} else if (light > 900) {
			mode = FP;

		}
		light_clearIrqStatus();
		// Clear GPIO Interrupt P2.5
		LPC_GPIOINT ->IO2IntClr = 1 << 5;

	}
}

void EINT0_IRQHandler(void) {

	GET_STATUS = ON;
	LPC_SC ->EXTINT |= 1 << 0;
	NVIC_ClearPendingIRQ(EINT0_IRQn);

}

void SysTick_Handler(void) {
	msTicks++;
}

uint32_t getMsTick(void) {
	return msTicks;
}

void get_Time(void) {
	//numOfMs represent the number of milli seconds that have passed since the program started
	int numOfMs;
	numOfMs = msTicks - msTicksCurr;
	sec = numOfMs / 1000;
	if (sec % 60 != 0) {
		min = sec / 60;
		sec = sec % 60;

	} else {
		min = sec / 60;
		sec = 0;
	}

	if (min % 60 != 0) {
		hrs = min / 60;
		min = min % 60;

	} else {
		hrs = min / 60;
		min = 0;
	}

}

xvoid off_led7seg(void) {
	led7seg_setChar('+', FALSE);
}

void off_RGB(void) {

	GPIO_ClearValue(0, (1 << 26));
	GPIO_ClearValue(2, (1 << 0));
}

void off_Peripherals() {
	//Switch off peripherals
	off_led7seg();
	off_RGB();
	pca9532_setLeds(0x0000, 0xFFFF);
	oled_clearScreen(OLED_COLOR_BLACK);
}

void invert_Segment_Display(int seg_counter) {
	switch (seg_counter) {
	case 0:
		led7seg_setChar(0x24, TRUE);
		break;
	case 1:
		led7seg_setChar(0x7D, TRUE);
		break;
	case 2:
		led7seg_setChar(0xE0, TRUE);
		break;
	case 3:
		led7seg_setChar(0x70, TRUE);
		break;
	case 4:
		led7seg_setChar(0x39, TRUE);
		break;
	case 5:
		led7seg_setChar(0x32, TRUE);
		break;
	}
}

int displayAnimation_CAT(void) {
	char string[16] = { };
	int loadBar = 4;
	int counter = 0;
	int last = 0;
	int flag = 0;

	uint32_t tickCounter = 0;

	oled_clearScreen(OLED_COLOR_BLACK);
	sprintf(string, MESSAGE_LOADING);
	oled_putString(0, 16, (uint8_t *) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(string, MESSAGE_PLEASE_WAIT);
	oled_putString(0, 24, (uint8_t *) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);

	while (1) {
		MODE_TOGGLE = (GPIO_ReadValue(1) >> 31) & 0x01;
		if ((msTicks - tickCounter >= PERIOD_MS_250) && (loadBar <= 60)) {
			oled_circle(loadBar, 40, 2, OLED_COLOR_WHITE);
			tickCounter = msTicks;
			loadBar = loadBar + 8;
		}
		if (loadBar > 60) {
			oled_circle(12, 40, 2, OLED_COLOR_BLACK);
			oled_circle(20, 40, 2, OLED_COLOR_BLACK);
			oled_circle(28, 40, 2, OLED_COLOR_BLACK);
			oled_circle(36, 40, 2, OLED_COLOR_BLACK);
			oled_circle(44, 40, 2, OLED_COLOR_BLACK);
			oled_circle(52, 40, 2, OLED_COLOR_BLACK);
			oled_circle(60, 40, 2, OLED_COLOR_BLACK);
			loadBar = 4;
			counter++;
		}
		if (counter == 2) {
			break;
		}

		//check for SW4
		if (last == 1 && MODE_TOGGLE == 0) {
			tickCounter = msTicks;
			flag = 1;
			return flag;
		}
		last = MODE_TOGGLE;
	}
	return 0;
}

void displayOled_CAT(void) {
	char string[16] = { };
	oled_clearScreen(OLED_COLOR_BLACK);
	sprintf(string, MESSAGE_I_WATCH);
	oled_putString(0, 0, (uint8_t *) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(string, MESSAGE_ELECTRONIC);
	oled_putString(0, 8, (uint8_t *) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(string, MESSAGE_TAG);
	oled_putString(0, 16, (uint8_t *) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(string, MESSAGE_CONFIGURATION);
	oled_putString(0, 24, (uint8_t *) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(string, MESSAGE_AND);
	oled_putString(0, 32, (uint8_t *) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(string, MESSAGE_TESTING_MODE);
	oled_putString(0, 40, (uint8_t *) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
}

void displayOled_Active_PS(void) {
	char string[16] = { };

	oled_clearScreen(OLED_COLOR_BLACK);
	sprintf(string, ACTIVE_PS);
	oled_putString(0, 0, (uint8_t*) string, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	sprintf(string, LUX_PS);
	oled_putString(0, 8, (uint8_t*) string, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	sprintf(string, TA_PS);
	oled_putString(0, 16, (uint8_t*) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(string, TB_PS);
	oled_putString(0, 24, (uint8_t*) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(string, X_PS);
	oled_putString(0, 32, (uint8_t*) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(string, Y_PS);
	oled_putString(0, 40, (uint8_t*) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(string, Z_PS);
	oled_putString(0, 48, (uint8_t*) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
}

void read_Sensor(void) {

	//read light sensor
	light = light_read();

	//read temp sensor
	tempA = temp_read();
	tempB = temp_read();

	//read acc sensor
	acc_read(&x, &y, &z);
	x = x + xoff;
	y = y + yoff;
	z = z + zoff;
}

void displayOled_Sensors(void) {
	char string[16] = { };
	sprintf(string, "Lux: %d\n\r", light);
	oled_putString(0, 8, (uint8_t*) string, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	sprintf(string, "TA: %.1fdeg\n\r", tempA / 10.0);
	oled_putString(0, 16, (uint8_t*) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(string, "TB: %.1fdeg\n\r", tempB / 10.0);
	oled_putString(0, 24, (uint8_t*) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(string, "X: %d\n\r", x);
	oled_putString(0, 32, (uint8_t*) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(string, "Y: %d\n\r", y);
	oled_putString(0, 40, (uint8_t*) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(string, "Z: %d\n\r", z);
	oled_putString(0, 48, (uint8_t*) string, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
}

void clearOled_Sensors(void) {
	char string[16] = { };
	sprintf(string, "Lux: %d\n\r", light);
	oled_putString(0, 8, (uint8_t*) string, OLED_COLOR_BLACK, OLED_COLOR_BLACK);
	sprintf(string, "TA: %.1fdeg\n\r", tempA / 10.0);
	oled_putString(0, 16, (uint8_t*) string, OLED_COLOR_BLACK,
			OLED_COLOR_BLACK);
	sprintf(string, "TB: %.1fdeg\n\r", tempB / 10.0);
	oled_putString(0, 24, (uint8_t*) string, OLED_COLOR_BLACK,
			OLED_COLOR_BLACK);
	sprintf(string, "X: %d\n\r", x);
	oled_putString(0, 32, (uint8_t*) string, OLED_COLOR_BLACK,
			OLED_COLOR_BLACK);
	sprintf(string, "Y: %d\n\r", y);
	oled_putString(0, 40, (uint8_t*) string, OLED_COLOR_BLACK,
			OLED_COLOR_BLACK);
	sprintf(string, "Z: %d\n\r", z);
	oled_putString(0, 48, (uint8_t*) string, OLED_COLOR_BLACK,
			OLED_COLOR_BLACK);
}

void print_Results(void) {
	char result[100] = { };
	get_Time();
	sprintf(result, "[%dhr %dmin %dsec] L%d_TA%.1f_TB%.1f_AX%d_AY%d_AZ%d\r\n",
			hrs, min, sec, light, tempA / 10.0, tempB / 10.0, x, y, z);
	UART_Send(LPC_UART3, (uint8_t *) result, strlen(result), BLOCKING);

}

void displayCAT_UART(void) {
	char msg[100] = { };
	//Entering CAT-MODE in UART
	get_Time();
	sprintf(msg, "[%dhr %dmin %dsec] You are now in |CAT-MODE|\r\n", hrs, min,
			sec);
	UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg), BLOCKING);

}

void displayACTIVE_UART(void) {
	char msg[100] = { };
	//Entering ACTIVE-MODE in UART
	get_Time();
	sprintf(msg, "[%dhr %dmin %dsec] You are now in |ACTIVE-MODE|\r\n", hrs,
			min, sec);
	UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg), BLOCKING);

}

void calibrate_acc(void) {
	// Assume base board in zero-g position when reading first value.
	acc_read(&x, &y, &z);
	xoff = 0 - x;
	yoff = 0 - y;
	zoff = 64 - z;

}

int cat_Mode(void) {

	//initialize all variables
	char string[16] = { };

	int flag = 0;
	int counter = 0;
	int last = 0;

	uint16_t ledOnMask = 0x0000;
	uint16_t bitIncreament = 0x0001;
	uint16_t ledOffMask = 0xFFFF;
	uint32_t tickCounter = 0;

	//Switch off peripherals
	off_Peripherals();

	//Display animation (Loading Bar), if SW4 pushed switch to active
	flag = displayAnimation_CAT();

	//Entering CAT-MODE in UART
	displayCAT_UART();
	if (flag == 1) {
		return flag;
	}

	//Display CAT mode
	displayOled_CAT();

	while (1) {

		MODE_TOGGLE = (GPIO_ReadValue(1) >> 31) & 0x01;

		//BLINK BLUE//////////////////////////////////////////
		if (msTicks - tickCounter >= PERIOD_MS_500 && BLINK_BLUE == ON) {
			GPIO_SetValue(0, (1 << 26));
			tickCounter = msTicks;
			BLINK_BLUE = OFF;
		}

		if (msTicks - tickCounter >= PERIOD_MS_500 && BLINK_BLUE == OFF) {
			GPIO_ClearValue(0, (1 << 26));
			tickCounter = msTicks;
			BLINK_BLUE = ON;
			counter++;
		}
		///////////////////////////////////////////////////////

		//BLINK BLUE 4 TIMES
		if (counter == 4) {
			BLINK_BLUE = STOP;
			BLINK_RED = ON;
			counter = 0;
		}

		//BLINK RED////////////////////////////////////////////
		if (msTicks - tickCounter >= PERIOD_MS_500 && BLINK_RED == ON) {
			GPIO_SetValue(2, (1 << 0));
			tickCounter = msTicks;
			BLINK_RED = OFF;
		}

		if (msTicks - tickCounter >= PERIOD_MS_500 && BLINK_RED == OFF) {
			GPIO_ClearValue(2, (1 << 0));
			tickCounter = msTicks;
			BLINK_RED = ON;
			counter++;
		}
		////////////////////////////////////////////////////////

		//BLINK RED 4 TIMES AND STAY
		if (counter == 4) {
			counter = 0;
			GPIO_SetValue(2, (1 << 0));   //RED STAY
			led7Seg = START_LED_7_SEG;
			BLINK_BLUE = STOP;
			BLINK_RED = STOP;
		}

		//LED7 - COUNT TILL 5 USING INVERT///////////////////////////////////
		if ((msTicks - tickCounter >= PERIOD_MS_1000) && led7Seg <= 6) {
			invert_Segment_Display(led7Seg);
			led7Seg++;
			tickCounter = msTicks;
			if (led7Seg == 6) {
				led16 = START_16_LED;
			}
		}
		/////////////////////////////////////////////////////////

		//16LED - ON ALL////////////////////////////////////////////
		if ((msTicks - tickCounter >= PERIOD_MS_250) && led16 <= ON_ALL_16_LED) {
			pca9532_setLeds(ledOnMask, ledOffMask);
			ledOnMask = ledOnMask << 1;
			ledOnMask = ledOnMask | bitIncreament;
			led16++;
			tickCounter = msTicks;
			if (led16 == ON_ALL_16_LED) {
				//Reset global values
				BLINK_BLUE = ON;
				BLINK_RED = STOP;
				led7Seg = STOP;
				led16 = STOP;
				break;
			}
		}
		///////////////////////////////////////////////////////////////

		//check for SW4
		if (last == 1 && MODE_TOGGLE == 0) {
			flag = 1;
			counter = 0;
			BLINK_BLUE = ON;
			BLINK_RED = STOP;
			led7Seg = STOP;
			led16 = STOP;
			tickCounter = msTicks;
			return flag;
		}

		last = MODE_TOGGLE;
	}

	//Display header *CAT_MODE*
	oled_clearScreen(OLED_COLOR_BLACK);
	sprintf(string, CAT_MODE);
	oled_putString(0, 0, (uint8_t*) string, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	//DISPLAY SENSORS RESULT ON OLED
	while (1) {
		JOYSTICK_STATE = joystick_read();

		MODE_TOGGLE = (GPIO_ReadValue(1) >> 31) & 0x01;

		read_Sensor();
		displayOled_Sensors();

		//check for SW4
		if (last == 1 && MODE_TOGGLE == 0) {
			flag = 1;
			return flag;
		}
		last = MODE_TOGGLE;

		//calibrate acc
		if ((JOYSTICK_STATE & JOYSTICK_CENTER) != 0) {
			calibrate_acc();
		}

	}

	return 0;
}

int active_Mode(void) {
	int flag;
	int last = 0;

	char string[16] = { };
	char result[100] = { };

	GET_STATUS = OFF;

	uint8_t MODE_TOGGLE = 1;
	uint32_t tickCounter = 0;

	off_led7seg();
	led7seg_setChar(0x28, TRUE);
	off_RGB();
	oled_clearScreen(OLED_COLOR_BLACK);

	uint16_t bitIncreament = 0x0001;
	uint16_t ledOffMask = 0xFFFF;
	uint16_t currentLedState = pca9532_getLedState(FALSE);

	displayACTIVE_UART();

	while (1) {
		MODE_TOGGLE = (GPIO_ReadValue(1) >> 31) & 0x01;
		JOYSTICK_STATE = joystick_read();

		//Power Saver behavior in Active Mode/////////////////////////////
		if (mode == PS) {
			last = 0;
			oled_clearScreen(OLED_COLOR_BLACK);
			light_setHiThreshold(50);
			light_setLoThreshold(0);
			displayOled_Active_PS();

			while (mode == PS) {
				JOYSTICK_STATE = joystick_read();

				MODE_TOGGLE = (GPIO_ReadValue(1) >> 31) & 0x01;
				if (msTicks - tickCounter >= 250) {
					pca9532_setLeds(currentLedState, ledOffMask);
					currentLedState = currentLedState >> 1;
					tickCounter = msTicks;
					if (currentLedState == 0x7FFF) {
						get_Time();
						sprintf(result,
								"[%dhr %dmin %dsec] Satellite Communication Suspended.\n\r",
								hrs, min, sec);
						UART_Send(LPC_UART3, (uint8_t *) result, strlen(result),
								BLOCKING);
					}
				}

				//calibrate acc
				if ((JOYSTICK_STATE & JOYSTICK_CENTER) != 0) {
					calibrate_acc();
				}

				//check for SW3
				if (GET_STATUS == ON) {

					clearOled_Sensors();
					read_Sensor();
					displayOled_Sensors();
					GET_STATUS = OFF;
					print_Results();
				}

				//check for SW4
				if (last == 1 && MODE_TOGGLE == 0) {
					flag = 0;
					return flag;
				}
				last = MODE_TOGGLE;
			}
		}
		///////////////////////////////////////////////////////////////////////

		//Normal Operation behavior in Active Mode/////////////////////////////
		if (mode == NO) {
			last = 0;
			light_setHiThreshold(900);
			light_setLoThreshold(50);
			oled_clearScreen(OLED_COLOR_BLACK);
			sprintf(string, ACTIVE_NO);
			oled_putString(0, 0, (uint8_t*) string, OLED_COLOR_WHITE,
					OLED_COLOR_BLACK);
			MODE_TOGGLE = (GPIO_ReadValue(1) >> 31) & 0x01;
			while (mode == NO) {
				JOYSTICK_STATE = joystick_read();
				MODE_TOGGLE = (GPIO_ReadValue(1) >> 31) & 0x01;
				if (msTicks - tickCounter >= 4000) {
					tickCounter = msTicks;
					read_Sensor();
					displayOled_Sensors();
				}

				//calibrate acc
				if ((JOYSTICK_STATE & JOYSTICK_CENTER) != 0) {
					calibrate_acc();
				}

				//check for SW3
				if (GET_STATUS == ON) {
					read_Sensor();
					print_Results();
					GET_STATUS = OFF;
				}

				//check for SW4
				if (last == 1 && MODE_TOGGLE == 0) {
					flag = 0;
					return flag;
				}
				last = MODE_TOGGLE;
			}
		}
		//////////////////////////////////////////////////////////////////////////

		//Full Performance behavior in Active Mode/////////////////////////////
		if (mode == FP) {
			light_setLoThreshold(900);
			currentLedState = pca9532_getLedState(FALSE);
			pca9532_setLeds(currentLedState, ledOffMask);
			oled_clearScreen(OLED_COLOR_BLACK);
			sprintf(string, ACTIVE_FP);
			oled_putString(0, 0, (uint8_t*) string, OLED_COLOR_WHITE,
					OLED_COLOR_BLACK);
			while (mode == FP) {
				JOYSTICK_STATE = joystick_read();
				MODE_TOGGLE = (GPIO_ReadValue(1) >> 31) & 0x01;
				if ((currentLedState != 0xFFFF)
						&& (msTicks - tickCounter >= 250)) {
					currentLedState = currentLedState << 1;
					currentLedState = currentLedState | bitIncreament;
					pca9532_setLeds(currentLedState, ledOffMask);
					tickCounter = msTicks;
					if (currentLedState == 0x7FFF) {
						get_Time();
						sprintf(result,
								"[%dhr %dmin %dsec] Satellite Communication Link Established.\n\r",
								hrs, min, sec);
						UART_Send(LPC_UART3, (uint8_t *) result, strlen(result),
								BLOCKING);
					}
				}

				if (currentLedState == 0XFFFF
						&& (msTicks - tickCounter >= 4000)) {

					read_Sensor();
					displayOled_Sensors();
					print_Results();
					tickCounter = msTicks;
				}

				//calibrate acc
				if ((JOYSTICK_STATE & JOYSTICK_CENTER) != 0) {
					calibrate_acc();
				}

				//check for SW3
				if (GET_STATUS == ON) {
					read_Sensor();
					print_Results();
					GET_STATUS = OFF;
				}

				//check for SW4
				if (last == 1 && MODE_TOGGLE == 0) {
					flag = 0;
					return flag;
				}

				last = MODE_TOGGLE;

			}
		}
		//////////////////////////////////////////////////////////////////////////

		//calibrate acc
		if ((JOYSTICK_STATE & JOYSTICK_CENTER) != 0) {
			calibrate_acc();
		}

		//check for SW3
		if (GET_STATUS == ON) {
			read_Sensor();
			print_Results();
			GET_STATUS = OFF;
		}

		//check for SW4
		if (last == 1 && MODE_TOGGLE == 0) {
			flag = 0;
			return flag;
		}

		last = MODE_TOGGLE;
	}
}

void welcome_Message(void) {
	char msg[100] = { };
	//Welcome message ON UART
	sprintf(msg, "************************************************\r\n");
	UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg), BLOCKING);
	sprintf(msg, "EE2024 ASSIGNMENT 2 BY WONG KOK WOO & ANDREW YEO\r\n");
	UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg), BLOCKING);
	sprintf(msg, "************************************************\r\n");
	UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg), BLOCKING);
	sprintf(msg, "\r\n");
	UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg), BLOCKING);
	sprintf(msg, "\r\n");
	UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg), BLOCKING);
	sprintf(msg,
			"--------------------------WELCOME TO I-WATCH---------------------------\r\n");
	UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg), BLOCKING);
	sprintf(msg,
			"(Initiative for Wildlife Analysis, Telemetry and Conservation of Habitat)\r\n");
	UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg), BLOCKING);

	sprintf(msg, "\r\n");
	UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg), BLOCKING);
	msTicksCurr = msTicks;
	get_Time();
	sprintf(msg,
			"[%dhr %dmin %dsec] <------ shows the time since the program started\r\n",
			hrs, min, sec);
	UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg), BLOCKING);

}

int main(void) {

	if (SysTick_Config(SystemCoreClock / 1000)) {
		while (1)
			;  // Capture error
	}

//initialization////////////////////////
	init_i2c();
	init_ssp();
	init_GPIO();
	init_UART();
	pca9532_init();
	joystick_init();
	acc_init();
	oled_init();
	led7seg_init();
	temp_init(&getMsTick);

	// Assume base board in zero-g position when reading first value.
	acc_read(&x, &y, &z);
	xoff = 0 - x;
	yoff = 0 - y;
	zoff = 64 - z;

	light_enable();
	light_setRange(LIGHT_RANGE_1000);
	light_setWidth(LIGHT_WIDTH_16BITS);
	light_setIrqInCycles(LIGHT_CYCLE_1);
	light_clearIrqStatus();
/////////////////////////////////////////

	off_Peripherals();

//when flag == 0 goes to cat-mode
//when flag == 1 goes to active-mode
	int flag;

	while (1) {

		MODE_TOGGLE = (GPIO_ReadValue(1) >> 31) & 0x01;

		if (MODE_TOGGLE == 0) {
			welcome_Message();
			flag = CAT;
			while (1) {

				//CAT MODE
				if (flag == CAT) {
					int flagValue = cat_Mode();
					flag = flagValue;
				}
				//ACTIVE MODE
				if (flag == ACTIVE) {
					int flagValue = active_Mode();
					flag = flagValue;

				}

			}

		}
	}
}
void check_failed(uint8_t *file, uint32_t line) {
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
		;
}
