/**
* \file
*
* \brief Example of usage of the maXTouch component with USART
*
* This example shows how to receive touch data from a maXTouch device
* using the maXTouch component, and display them in a terminal window by using
* the USART driver.
*
* Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
*
* \asf_license_start
*
* \page License
*
* Subject to your compliance with these terms, you may use Microchip
* software and any derivatives exclusively with Microchip products.
* It is your responsibility to comply with third party license terms applicable
* to your use of third party software (including open source software) that
* may accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
* AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
* LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
* SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
* ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
* RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*
* \asf_license_stop
*
*/

/**
* \mainpage
*
* \section intro Introduction
* This simple example reads data from the maXTouch device and sends it over
* USART as ASCII formatted text.
*
* \section files Main files:
* - example_usart.c: maXTouch component USART example file
* - conf_mxt.h: configuration of the maXTouch component
* - conf_board.h: configuration of board
* - conf_clock.h: configuration of system clock
* - conf_example.h: configuration of example
* - conf_sleepmgr.h: configuration of sleep manager
* - conf_twim.h: configuration of TWI driver
* - conf_usart_serial.h: configuration of USART driver
*
* \section apiinfo maXTouch low level component API
* The maXTouch component API can be found \ref mxt_group "here".
*
* \section deviceinfo Device Info
* All UC3 and Xmega devices with a TWI module can be used with this component
*
* \section exampledescription Description of the example
* This example will read data from the connected maXTouch explained board
* over TWI. This data is then processed and sent over a USART data line
* to the board controller. The board controller will create a USB CDC class
* object on the host computer and repeat the incoming USART data from the
* main controller to the host. On the host this object should appear as a
* serial port object (COMx on windows, /dev/ttyxxx on your chosen Linux flavour).
*
* Connect a terminal application to the serial port object with the settings
* Baud: 57600
* Data bits: 8-bit
* Stop bits: 1 bit
* Parity: None
*
* \section compinfo Compilation Info
* This software was written for the GNU GCC and IAR for AVR.
* Other compilers may or may not work.
*
* \section contactinfo Contact Information
* For further information, visit
* <A href="http://www.atmel.com/">Atmel</A>.\n
*/
/*
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
*/

#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "tfont.h"
#include "conf_board.h"
#include "conf_example.h"
#include "conf_uart_serial.h"
#include "soneca.h"
#include "digital521.h"
#include "termometro.h"
#include "ar.h"

/************************************************************************/
/* LCD + TOUCH                                                          */
/************************************************************************/
#define MAX_ENTRIES        3

struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;

/************************************************************************/
/* RTOS                                                                  */
/************************************************************************/
#define TASK_MXT_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_MXT_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_LCD_STACK_SIZE            (4*1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY        (tskIDLE_PRIORITY)

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)

/** The conversion data is done flag */
volatile bool g_is_conversion_done = false;

/** The conversion data value */
volatile uint32_t g_ul_value = 20;

/* Canal do sensor de temperatura */
#define AFEC_CHANNEL_TEMP_SENSOR 0

typedef struct {
	uint x;
	uint y;
} touchData;

QueueHandle_t xQueueTouch;
QueueHandle_t xQueueTem;
QueueHandle_t xQueuePWM;

SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t xSemaphore2;

#define PIO_PWM_0 PIOA
#define ID_PIO_PWM_0 ID_PIOA
#define MASK_PIN_PWM_0 (1 << 0)

/** PWM frequency in Hz */
#define PWM_FREQUENCY      1000
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0

/** PWM channel instance for LEDs */
pwm_channel_t g_pwm_channel_led;

volatile uint16_t duty = 0;

#define BUT1_PIO_ID			  ID_PIOD
#define BUT1_PIO			  PIOD
#define BUT1_PIN			  28u
#define BUT1_PIN_MASK		  (1 << BUT1_PIN)
#define BUT1_DEBOUNCING_VALUE 79

#define BUT2_PIO_ID			  ID_PIOC
#define BUT2_PIO			  PIOC
#define BUT2_PIN			  31u
#define BUT2_PIN_MASK		  (1 << BUT2_PIN)
#define BUT2_DEBOUNCING_VALUE 79

#define BUT3_PIO_ID			  ID_PIOA
#define BUT3_PIO			  PIOA
#define BUT3_PIN			  19u
#define BUT3_PIN_MASK		  (1 << BUT3_PIN)
#define BUT3_DEBOUNCING_VALUE 79

/************************************************************************/
/* RTOS hooks                                                           */
/************************************************************************/
/**
* \brief Called if stack overflow during execution
*/
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/**
* \brief This function is called by FreeRTOS idle task
*/
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
* \brief This function is called by FreeRTOS each tick
*/
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* init                                                                 */
/************************************************************************/

static void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
}

/**
* \brief Set maXTouch configuration
*
* This function writes a set of predefined, optimal maXTouch configuration data
* to the maXTouch Xplained Pro.
*
* \param device Pointer to mxt_device struct
*/
static void mxt_init(struct mxt_device *device)
{
	enum status_code status;

	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x0d, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00,
		0x00, 0x32, 0x19
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8B, 0x00, 0x00, 0x0E, 0x08, 0x00, 0x80,
		0x32, 0x05, 0x02, 0x0A, 0x03, 0x03, 0x20,
		0x02, 0x0F, 0x0F, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x18, 0x18, 0x20, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02,
		0x02
	};

	/* T46 configuration object data */
	uint8_t t46_object[] = {
		0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03,
		0x00, 0x00
	};
	
	/* T56 configuration object data */
	uint8_t t56_object[] = {
		0x02, 0x00, 0x01, 0x18, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* TWI configuration */
	twihs_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_TWI_ADDRESS,
	};

	status = (enum status_code)twihs_master_setup(MAXTOUCH_TWI_INTERFACE, &twi_opt);
	Assert(status == STATUS_OK);

	/* Initialize the maXTouch device */
	status = mxt_init_device(device, MAXTOUCH_TWI_INTERFACE,
	MAXTOUCH_TWI_ADDRESS, MAXTOUCH_XPRO_CHG_PIO);
	Assert(status == STATUS_OK);

	/* Issue soft reset of maXTouch device by writing a non-zero value to
	* the reset register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_COMMANDPROCESSOR_T6, 0)
	+ MXT_GEN_COMMANDPROCESSOR_RESET, 0x01);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Write data to configuration registers in T7 configuration object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 0, 0x20);
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 1, 0x10);
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x4b);
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 3, 0x84);

	/* Write predefined configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_SPT_CTE_CONFIGURATION_T46, 0), &t46_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_PROCI_SHIELDLESS_T56, 0), &t56_object);

	/* Issue recalibration command to maXTouch device by writing a non-zero
	* value to the calibrate register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_COMMANDPROCESSOR_T6, 0)
	+ MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x01);
}

/************************************************************************/
/* DRAW                                                             */
/************************************************************************/

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}
}

void draw_sonec(void){
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(0,84,ILI9488_LCD_WIDTH,ILI9488_LCD_HEIGHT);
	ili9488_draw_pixmap(ILI9488_LCD_WIDTH-(soneca.width+1),0,soneca.width, soneca.height+2, soneca.data);
}

void draw_ar(void){
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_pixmap(0,300,ar.width,ar.height+2,ar.data);
}

void draw_screen(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
}

void draw_termo(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_pixmap(0,90,termometro.width, termometro.height+2, termometro.data);
}

void draw_potencia(uint32_t potencia){
	char buffer[4];
	sprintf(buffer, "%3d", potencia);
	font_draw_text(&digital52, buffer, ar.width + 5, 305, 1);
	font_draw_text(&digital52, "%", ar.width + 125,305, 1);
}

void draw_pwm(int32_t pwm){
	char buffer[5];
	sprintf(buffer, "%3d", pwm);
	font_draw_text(&digital52, buffer, termometro.width + 5, 145, 1);
	font_draw_text(&digital52, "PWM", termometro.width + 125, 145, 1);
}

void draw_temp(uint32_t temperatura){
	char buffer[4];
	sprintf(buffer, "%3d", temperatura);
	
	// Limpar tela e escrever temperatura
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(termometro.width + 5,95, termometro.width + 175 ,95+49);
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	
	font_draw_text(&digital52, buffer, termometro.width + 5, 95, 1);
	font_draw_text(&digital52, "C", termometro.width + 125, 95, 1);
	
	// Temperatura Gráficamente
	if (temperatura < 10) ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLUE));
	else if (temperatura < 20) ili9488_set_foreground_color(COLOR_CONVERT(COLOR_GREEN));
	else if (temperatura < 30) ili9488_set_foreground_color(COLOR_CONVERT(COLOR_YELLOW));
	else if (temperatura < 35) ili9488_set_foreground_color(COLOR_CONVERT(COLOR_TOMATO));
	else ili9488_set_foreground_color(COLOR_CONVERT(COLOR_RED));
	ili9488_draw_filled_rectangle(termometro.width + 176,95, termometro.width + 226 , 95 + 49);
}


/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

uint32_t convert_axis_system_x(uint32_t touch_y) {
	// entrada: 4096 - 0 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
	// entrada: 0 - 4096 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_HEIGHT*touch_x/4096;
}

void PWM0_init(uint channel, uint duty){
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM0);

	/* Disable PWM channels for LEDs */
	pwm_channel_disable(PWM0, PIN_PWM_LED0_CHANNEL);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_peripheral_hz()
	};
	
	pwm_init(PWM0, &clock_setting);

	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	g_pwm_channel_led.alignment = PWM_ALIGN_CENTER;
	/* Output waveform starts at a low level */
	g_pwm_channel_led.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = duty;
	g_pwm_channel_led.channel = channel;
	pwm_channel_init(PWM0, &g_pwm_channel_led);
	
	/* Enable PWM channels for LEDs */
	pwm_channel_enable(PWM0, channel);
}

void mxt_handler(struct mxt_device *device, uint *x, uint *y)
{
	/* USART tx buffer initialized to 0 */
	uint8_t i = 0; /* Iterator */

	/* Temporary touch event data struct */
	struct mxt_touch_event touch_event;
	
	/* first touch only */
	uint first = 0;

	/* Collect touch events and put the data in a string,
	* maximum 2 events at the time */
	do {

		/* Read next next touch event in the queue, discard if read fails */
		if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
			continue;
		}
		
		/************************************************************************/
		/* Envia dados via fila RTOS                                            */
		/************************************************************************/
		if(first == 0 ){
			*x = convert_axis_system_x(touch_event.y);
			*y = convert_axis_system_y(touch_event.x);
			first = 1;
		}
		
		i++;

		/* Check if there is still messages in the queue and
		* if we have reached the maximum numbers of events */
	} while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));
}

void AFEC_callback_t(void){
	g_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	xQueueSendFromISR( xQueueTem, &g_ul_value, 0);
}

static void config_ADC_TEMP(void){
	/*************************************
	* Ativa e configura AFEC
	*************************************/
	/* Ativa AFEC - 0 */
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_0,	AFEC_callback_t, 6);

	/*** Configuracao específica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	down to 0.
	*/
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);

	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
}

static int32_t convert_adc_to_temp(int32_t ADC_value){
	return(ADC_value);
}

void button_Callback(void) {
	xSemaphoreGiveFromISR(xSemaphore, 0);
}

void button_Callback2(void) {
	xSemaphoreGiveFromISR(xSemaphore2, 0);
}

void io_init(void) {

	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIN_MASK,PIO_PULLUP | PIO_DEBOUNCE);

	pmc_enable_periph_clk(BUT2_PIO_ID);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIN_MASK, PIO_IT_FALL_EDGE, button_Callback);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIN_MASK, PIO_IT_FALL_EDGE, button_Callback2);
	
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 6);
	
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 6);
	
	pio_enable_interrupt(BUT2_PIO, BUT2_PIN_MASK);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIN_MASK);
}

/************************************************************************/
/* tasks                                                                */
/************************************************************************/

void task_mxt(void){
	
	struct mxt_device device; /* Device data container */
	mxt_init(&device);       	/* Initialize the mXT touch device */
	touchData touch;          /* touch queue data type*/
	
	while (true) {
		/* Check for any pending messages and run message handler if any
		* message is found in the queue */
		if (mxt_is_message_pending(&device)) {
			mxt_handler(&device, &touch.x, &touch.y);
			xQueueSend( xQueueTouch, &touch, 0);           /* send mesage to queue */
		}
		vTaskDelay(100);
	}
}

void task_afec(void){
	
	config_ADC_TEMP();
	printf("\n-Task Afec Configurado");
	
	for (;;){
		afec_start_software_conversion(AFEC0);
		vTaskDelay(1000);
	}
}

void task_lcd(void){
	xQueueTouch = xQueueCreate( 10, sizeof( touchData ) );
	xQueueTem =	xQueueCreate( 10, sizeof( uint32_t ) );
	
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphore2 = xSemaphoreCreateBinary();
	
	
	uint32_t t = 25;
	int32_t pot = 0;
	int32_t d_cicle = 0;
	
	
	configure_lcd();
	
	io_init();
	
	draw_screen();
	draw_sonec();
	draw_termo();
	draw_ar();
	
	// Escreve HH:MM no LCD
	font_draw_text(&digital52, "17:40", 0, 0, 1);
	
	
	// Linha
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	ili9488_draw_filled_rectangle(0,80,ILI9488_LCD_WIDTH-1,82);
	
	while (true) {

		if (xQueueReceive( xQueueTem, &t, ( TickType_t )  100 / portTICK_PERIOD_MS)) {
			t = convert_adc_to_temp(t)/100;
						
			if (t > 50) t = 50;
			else if (t <= 0) t = 0;
			
			draw_temp(t);
			
			
			printf("\n %d", pot);
			
			d_cicle = pot;
			
			draw_pwm(pot);
			
			xQueueSend(xQueuePWM, &d_cicle,0);
		}
		if (xSemaphoreTake(xSemaphore2, ( TickType_t )  100 / portTICK_PERIOD_MS)) {
			pot += 5;
			if (pot >= 100) pot = 100;
			
			draw_potencia(pot);
		}
		if (xSemaphoreTake(xSemaphore, ( TickType_t )  100 / portTICK_PERIOD_MS)) {
			pot -= 5;
			if (pot < 0) pot = 0;
			draw_potencia(pot);
		}
		
	

	
	}
}

void task_pwm(void){
	xQueuePWM = xQueueCreate( 10, sizeof( int32_t ) );
		
	pmc_enable_periph_clk(ID_PIO_PWM_0);
	pio_set_peripheral(PIO_PWM_0, PIO_PERIPH_A, MASK_PIN_PWM_0 );
	
	uint duty = 0;
	PWM0_init(0, duty);
	
	while (true) {
		if (xQueueReceive( xQueuePWM, &duty, ( TickType_t ) 100 / portTICK_PERIOD_MS)) {
			pwm_channel_update_duty(PWM0, &g_pwm_channel_led, 100 - duty);
		}
	}
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void)
{
	/* Initialize the USART configuration struct */
	const usart_serial_options_t usart_serial_options = {
		.baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength   = USART_SERIAL_CHAR_LENGTH,
		.paritytype   = USART_SERIAL_PARITY,
		.stopbits     = USART_SERIAL_STOP_BIT
	};

	sysclk_init(); /* Initialize system clocks */
	board_init();  /* Initialize board */
	
	/* Initialize stdio on USART */
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);
	printf("LIGOU");
	
	/* Create task to handler touch */
	if (xTaskCreate(task_mxt, "mxt", TASK_MXT_STACK_SIZE, NULL, TASK_MXT_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to handler LCD */
	if (xTaskCreate(task_lcd, "lcd", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to handler LCD */
	if (xTaskCreate(task_afec, "afec", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test afec task\r\n");
	}

	
	if (xTaskCreate(task_pwm, "pwm", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test adc task\r\n");
		}/**/

	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){

	}
	return 0;
	}
