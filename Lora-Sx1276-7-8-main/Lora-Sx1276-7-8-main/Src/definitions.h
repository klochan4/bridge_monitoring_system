
#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "stm32f1xx_hal.h"

extern int irq1counter;
extern int irq2counter;

extern int sw1_flag;
extern int sw3_flag;
extern int sw4_flag;

extern int accelerometer_activity_flag;

extern char interruptSource;
extern UART_HandleTypeDef huart1;


#define SECONDARY_UNIT_TEST_BOARD 0


#define CENTRAL_UNIT 0


#if CENTRAL_UNIT == 1
#define SECONDARY_UNIT 0
#else
#define SECONDARY_UNIT 1



#if CENTRAL_UNIT == 1
#define HAS_ESP32 1
#endif

#if SECONDARY_UNIT == 1
#define HAS_ADXL345 1
#endif


#define USE_SLEEP_MODE 1

#define RECEIVING_ADDRESS_CHECK 1
#define EXTERNAL_INTERRUPTS 1

#define USE_ADXL345_INT1 1
#define USE_ADXL345_INT2 1

#define LORA_LIB_VINOD 0

#define FFT 0

#define DEBUG_HW 1
#define DEBUG_FFT 1 
#define DEBUG_SW 1
#define DEBUG_MEASURE 1

#define USE_DMA 1
#define USE_LORA 1

#define USE_CLOUD_IF_POSSIBLE 1

#define TEST_CODE 1

#define MEASURE_X_AXIS 1
#define MEASURE_Y_AXIS 1
#define MEASURE_Z_AXIS 1

#if MEASURE_X_AXIS == 0 || MEASURE_Y_AXIS == 0 || MEASURE_Z_AXIS == 0
#define CLOUD 0
#else
#define CLOUD USE_CLOUD_IF_POSSIBLE
#endif


//use value from datasheet
/*
frequency bandwidth bits			uA
3200 			1600 			1111 			140
1600 			800 			1110 			90
800 			400 			1101 			140
400 			200 			1100 			140
200 			100 			1011 			140
100 			50 				1010 			140
50 				25 				1001 			90
25 				12.5 			1000 			60
12.5 			6.25		  0111 			50
6.25 			3.13 			0110 			45
3.13 			1.56 			0101 			40
1.56 			0.78 			0100 			34
0.78 			0.39 			0011 			23
0.39 			0.20 			0010 			23
0.20 			0.10 			0001 			23
0.10 			0.05 			0000 			23
*/

#define SAMPLING_FREQUENCY 200.0


#define TIME_BETWEEN_MEASUREMENTS_MS 2000

#if SECONDARY_UNIT_TEST_BOARD == 1
	#define USE_LORA 0
	


#endif

////////////////////////////////////////
//definition of lora packet header

#define LORA_PACKET_DESTINATION_ADDRESS_INDEX 0
#define LORA_PACKET_DESTINATION_ADDRESS_BYTE_LENGTH 2

#define LORA_PACKET_SOURCE_ADDRESS_INDEX (LORA_PACKET_DESTINATION_ADDRESS_INDEX + LORA_PACKET_DESTINATION_ADDRESS_BYTE_LENGTH)
#define LORA_PACKET_SOURCE_ADDRESS_BYTE_LENGTH 2

#define LORA_PACKET_SESSION_ID_INDEX (LORA_PACKET_SOURCE_ADDRESS_INDEX + LORA_PACKET_SOURCE_ADDRESS_BYTE_LENGTH)
#define LORA_PACKET_SESSION_ID_BYTE_LENGTH 1

#define LORA_PACKET_PACKET_ID_INDEX (LORA_PACKET_SESSION_ID_INDEX + LORA_PACKET_SESSION_ID_BYTE_LENGTH)
#define LORA_PACKET_PACKET_ID_BYTE_LENGTH 1
/*
#define LORA_PACKET_TOTAL_PACKETS_INDEX (LORA_PACKET_PACKET_ID_INDEX + LORA_PACKET_PACKET_ID_BYTE_LENGTH)
#define LORA_PACKET_TOTAL_PACKETS_BYTE_LENGTH 1 
*/

#define LORA_PACKET_PACKET_TYPE_INDEX (LORA_PACKET_PACKET_ID_INDEX + LORA_PACKET_PACKET_ID_BYTE_LENGTH)
#define LORA_PACKET_PACKET_TYPE_BYTE_LENGTH 1 

#define LORA_PACKET_AXIS_INDEX (LORA_PACKET_PACKET_TYPE_INDEX + LORA_PACKET_PACKET_TYPE_BYTE_LENGTH)
#define LORA_PACKET_AXIS_BYTE_LENGTH 1


#define HEADER_SIZE (LORA_PACKET_AXIS_INDEX + LORA_PACKET_AXIS_BYTE_LENGTH)

#define LORA_PACKET_TYPE_DATA_PACKET 0
#define LORA_PACKET_TYPE_END_SESSION 1
#define LORA_PACKET_TYPE_TEST_PACKET 2
#define LORA_PACKET_TYPE_ACK 3
#define LORA_PACKET_START_TEST_SESSION 4


#define TOTAL_COUNT_OF_TEST_PACKETS_IN_SESSION 10
#define LORA_PACKET_TOTAL_TRANSMISSIONS_BEFORE_FAIL 3

#define TIME_FOR_RETRANSMISSION_LOWER_BORDER_MS 0
#define TIME_FOR_RETRANSMISSION_UPPER_BORDER_MS 1000

#define TIME_FOR_ACK_MS 1000


#define TOTAL_NUMBER_OF_TEST_SESSIONS 256
#define NOTIFY_LED_ON_TIME 100

#define ESP_ON_WAIT_TIME 3000
#define ESP_PROCESS_AXIS_DATA_WAIT_TIME 2000 * (SAMPLE_SIZE / 256)
#define ESP_SEND_DATA_TO_CLOUD_WAIT_TIME 30000



#define UART_BUFFER_SIZE 100

////////////////////////////////////////
//test stats

#define SUCCESSFULLY_TRANSMITTED_TEST_STATS_ID 0
#define ATTEMPT_TEST_STATS_ID 1


////////////////////////////////////////


#define CU_LORA_ADDRESS 0x0000
#define SU_LORA_ADDRESS 0x0001


////////////////////////////////////////

#define ESP_SS_PIN    GPIO_PIN_2
#define ESP_SS_PORT    GPIOA

#define ESP_EN_PIN    GPIO_PIN_1
#define ESP_EN_PORT    GPIOA

#define LED_RED_PIN    GPIO_PIN_8
#define LED_RED_PORT    GPIOB

#define LED_GREEN1_PIN    GPIO_PIN_7
#define LED_GREEN1_PORT    GPIOB

#define LED_GREEN2_PIN    GPIO_PIN_9
#define LED_GREEN2_PORT    GPIOB

#define LED_GREEN3_PIN    GPIO_PIN_6
#define LED_GREEN3_PORT    GPIOB


#warning po zmene zmenit aj nastavenia externych preruseni a mapovanie v subore srm32f1xx_it.c

#define SWITCH1_PIN    GPIO_PIN_5
#define SWITCH1_PORT    GPIOB

#define SWITCH3_PIN    GPIO_PIN_4
#define SWITCH3_PORT    GPIOB

#define SWITCH4_PIN    GPIO_PIN_3
#define SWITCH4_PORT    GPIOB

#define ADXL345_TO_MCU_INT1_PIN    GPIO_PIN_11
#define ADXL345_TO_MCU_INT1_PORT    GPIOB

#define ADXL345_TO_MCU_INT2_PIN    GPIO_PIN_12
#define ADXL345_TO_MCU_INT2_PORT    GPIOB

///////////////////////////////////////////

#define SAMPLE_SIZE 256
#define DMA_SIZE SAMPLE_SIZE * sizeof(short)
#define LORA_PACKET_LENGTH 45

//////////////////////////////////////////

#define USER_ACTION_LED_PORT LED_GREEN2_PORT
#define USER_ACTION_LED_PIN LED_GREEN2_PIN

#define TX_LED_PORT LED_GREEN3_PORT
#define TX_LED_PIN LED_GREEN3_PIN

#define RX_LED_PORT LED_RED_PORT
#define RX_LED_PIN LED_RED_PIN

#define ENABLE_TX_LED() HAL_GPIO_WritePin(TX_LED_PORT, TX_LED_PIN, GPIO_PIN_SET)
#define DISABLE_TX_LED() HAL_GPIO_WritePin(TX_LED_PORT, TX_LED_PIN, GPIO_PIN_RESET)

#define ENABLE_RX_LED() HAL_GPIO_WritePin(RX_LED_PORT, RX_LED_PIN, GPIO_PIN_SET)
#define DISABLE_RX_LED() HAL_GPIO_WritePin(RX_LED_PORT, RX_LED_PIN, GPIO_PIN_RESET)

#define ENABLE_USER_ACTION_LED() HAL_GPIO_WritePin(USER_ACTION_LED_PORT, USER_ACTION_LED_PIN, GPIO_PIN_SET)
#define DISABLE_USER_ACTION_LED() HAL_GPIO_WritePin(USER_ACTION_LED_PORT, USER_ACTION_LED_PIN, GPIO_PIN_RESET)


#define UART_SIZE 128


#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define LORA_RX_DEBUG 0


#endif
