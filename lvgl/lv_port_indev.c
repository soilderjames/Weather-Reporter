/**
 * @file lv_port_indev_templ.c
 *
 */

/*Copy this file as "lv_port_indev.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_indev.h"
#include "lvgl.h"
#include "main.h"
/*********************
 *      DEFINES
 *********************/
SPI_HandleTypeDef hspi2;
#define TCS1  HAL_GPIO_WritePin(TCS_GPIO_Port,TCS_Pin,GPIO_PIN_SET)
#define TCS0  HAL_GPIO_WritePin(TCS_GPIO_Port,TCS_Pin,GPIO_PIN_RESET)
/**********************
 *      TYPEDEFS
 **********************/
float xa = -0.066;
float xb = 257.4;
float ya = 0.09;
float yb = -18;
/**********************
 *  STATIC PROTOTYPES
 **********************/

static void touchpad_init(void);
static void touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
static bool touchpad_is_pressed(void);
static void touchpad_get_xy(lv_coord_t *x, lv_coord_t *y);

/**********************
 *  STATIC VARIABLES
 **********************/
lv_indev_t *indev_touchpad;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_indev_init(void) {
	/**
	 * Here you will find example implementation of input devices supported by LittelvGL:
	 *  - Touchpad
	 *  - Mouse (with cursor support)
	 *  - Keypad (supports GUI usage only with key)
	 *  - Encoder (supports GUI usage only with: left, right, push)
	 *  - Button (external buttons to press points on the screen)
	 *
	 *  The `..._read()` function are only examples.
	 *  You should shape them according to your hardware
	 */

	static lv_indev_drv_t indev_drv;

	/*------------------
	 * Touchpad
	 * -----------------*/

	/*Initialize your touchpad if you have*/
	touchpad_init();

	/*Register a touchpad input device*/
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = touchpad_read;
	indev_touchpad = lv_indev_drv_register(&indev_drv);

}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*------------------
 * Touchpad
 * -----------------*/

/*Initialize your touchpad*/
static void touchpad_init(void) {
	uint8_t cmd;
	TCS0;
	cmd = 0x80;
	HAL_SPI_Transmit(&hspi2, &cmd, 1, 1000);
	cmd = 0x00;
	HAL_SPI_Transmit(&hspi2, &cmd, 1, 1000);
	cmd = 0x00;
	HAL_SPI_Transmit(&hspi2, &cmd, 1, 1000);
	TCS1;
}

/*Will be called by the library to read the touchpad*/
static void touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
	static lv_coord_t last_x = 0;
	static lv_coord_t last_y = 0;

	/*Save the pressed coordinates and the state*/
	if (touchpad_is_pressed()) {
		touchpad_get_xy(&last_x, &last_y);
		data->state = LV_INDEV_STATE_PR;
	} else {
		data->state = LV_INDEV_STATE_REL;
	}

	/*Set the last pressed coordinates*/
	data->point.x = last_x;
	data->point.y = last_y;
}

/*Return true is the touchpad is pressed*/
static bool touchpad_is_pressed(void) {
	/*Your code comes here*/
	if (HAL_GPIO_ReadPin(TIRQ_GPIO_Port, TIRQ_Pin) == GPIO_PIN_RESET) {
		return true;
	} else {
		return false;
	}
}

/*Get the x and y coordinates if the touchpad is pressed*/
static void touchpad_get_xy(lv_coord_t *x, lv_coord_t *y) {
	/*Your code comes here*/
	uint16_t LSB, MSB = 0;
	uint8_t cmd[2] = { 0xD0, 0x90 };
	uint8_t none = 0x00;
	uint8_t signal = 0;
	int raw[2][4] = { 0 };
	int i, j = 0;
	int len = 4; //数组长度
	int tempx = 0; //交换数据存储
	int tempy = 0;
	int rawys = 0;
	int rawxs = 0;
	int error = 100;
	for (i = 0; i < len; i++) {
		TCS0;
		HAL_SPI_Transmit(&hspi2, &cmd[0], 1, 10);
		HAL_SPI_TransmitReceive(&hspi2, &none, &signal, sizeof(signal), 10);
		MSB = signal;
		HAL_SPI_TransmitReceive(&hspi2, &none, &signal, sizeof(signal), 10);
		LSB = signal;
		TCS1;
		raw[0][i] = ((MSB << 8) | (LSB)) >> 3;
		TCS0;
		HAL_SPI_Transmit(&hspi2, &cmd[1], 1, 10);
		HAL_SPI_TransmitReceive(&hspi2, &none, &signal, sizeof(signal), 10);
		MSB = signal;
		HAL_SPI_TransmitReceive(&hspi2, &none, &signal, sizeof(signal), 10);
		LSB = signal;
		TCS1;
		raw[1][i] = ((MSB << 8) | (LSB)) >> 3;
	}
	for (i = 0; i < len; i++) {
		for (j = 0; j < len - 1; j++) {
			if (raw[0][j] > raw[0][j + 1])
				tempx = raw[0][j];
			raw[0][j] = raw[0][j + 1];
			raw[0][j + 1] = tempx;
			if (raw[1][j] > raw[1][j + 1])
				tempy = raw[1][j];
			raw[1][j] = raw[1][j + 1];
			raw[1][j + 1] = tempy;
		}
	}
	if (raw[0][2] - raw[0][1] > error || raw[1][2] - raw[1][1] > error||raw[1][1]==0||raw[1][2]==0||raw[0][1]==0||raw[0][2]==0) {
		return;
	}
	rawys = raw[1][1] + raw[1][2];
	rawxs = raw[0][1] + raw[0][2];
	(*x) = 240 - (rawxs * xa / 2 + xb);
	(*y) = 320 - (rawys * ya / 2 + yb);
}

#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif
