#include "main.h"
#include "lora.h"
#include "..\Src\ADXL345.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <complex.h>
#include <stdlib.h>
#include "arm_math.h"
#include "arm_const_structs.h"
#include "..\Src\definitions.h"


#warning TODO: distance tests
#warning TODO: power consumption tests
#warning TODO: general test the SU on steel rod
#warning TODO: general test the devices in terrain, measure data, analyze it
#warning TODO: start writing diploma thesis
#warning TODO: better communication protocol between stm and esp
#warning TODO: create seed for random funct

#warning ked sa zaznamena prerusenie na aktivity pri adxl, vypnut prerusenia pre ten pin, spravit potrebny pocet vzoriek
#warning odoslat ich a az potom znovu zapnut prerusenia
#warning ak sa bud zastavi tick alebo ulozi do sleep rezimu, tak nie je mozne flashnut ani cez 2wire jtag
#warning interrupt z adxl sa nastartuje az vtedy, ako sa privedie externa LOG H na pin


/*
Use of the 3200 Hz and 1600 Hz output data rates is only recommended with SPI communication rates greater than or equal to
2 MHz. The 800 Hz output data rate is recommended only for
communication speeds greater than or equal to 400 kHz, and
the remaining data rates scale proportionally. For example, the
minimum recommended communication speed for a 200 Hz output
data rate is 100 kHz. Operation at an output data rate above the
recommended maximum may result in undesirable effects on the
acceleration data, including missing samples or additional noise

*/

/*
Overrun
The overrun bit is set when new data replaces unread data. The
precise operation of the overrun function depends on the FIFO
mode. In bypass mode, the overrun bit is set when new data
replaces unread data in the DATAX, DATAY, and DATAZ registers
(Address 0x32 to Address 0x37). In all other modes, the overrun
bit is set when FIFO is filled. The overrun bit is automatically
cleared when the contents of FIFO are read.

FIFO
The ADXL345 contains patent pending technology for an
embedded 32-level FIFO that can be used to minimize host
processor burden. This buffer has four modes: bypass, FIFO,
stream, and trigger (see Table 19). Each mode is selected by the
settings of the FIFO

Trigger Mode
In trigger mode, FIFO accumulates samples, holding the latest
32 samples from measurements of the x-, y-, and z-axes. After
a trigger event occurs and an interrupt is sent to the INT1 or
INT2 pin (determined by the trigger bit in the FIFO_CTL register),
FIFO keeps the last n samples (where n is the value specified by
the samples bits in the FIFO_CTL register) and then operates in
FIFO mode, collecting new samples only when FIFO is not full.
A delay of at least 5 탎 should be present between the trigger event
occurring and the start of reading data from the FIFO to allow
the FIFO to discard and retain the necessary samples. Additional
trigger events cannot be recognized until the trigger mode is
reset. To reset the trigger mode, set the device to bypass mode
and then set the device back to trigger mode. Note that the
FIFO data should be read first because placing the device into
bypass mode clears FIFO. 

stream mode
FIFO holds the last 32 data values. When FIFO is full,
the oldest data is overwritten with newer data.

In stream mode, data from measurements of the x-, y-, and z-axes
are stored in FIFO. When the number of samples in FIFO equals
the level specified in the samples bits of the FIFO_CTL register
(Address 0x38), the watermark interrupt is set. FIFO continues
accumulating samples and holds the latest 32 samples from measurements of the x-, y-, and z-axes, discarding older data as new
data arrives. The watermark interrupt continues occurring until the
number of samples in FIFO is less than the value stored in the
samples bits of the FIFO_CTL register.



RETRIEVING DATA FROM FIFO
The FIFO data is read through the DATAX, DATAY, and DATAZ
registers (Address 0x32 to Address 0x37). When the FIFO is in
FIFO, stream, or trigger mode, reads to the DATAX, DATAY, and
DATAZ registers read data stored in the FIFO. Each time data is
read from the FIFO, the oldest x-, y-, and z-axes data are placed
into the DATAX, DATAY, and DATAZ registers.
If a single-byte read operation is performed, the remaining bytes of
data for the current FIFO sample are lost. Therefore, all axes of
interest should be read in a burst (or multiple-byte) read operation.
To ensure that the FIFO has completely popped (that is, that new
data has completely moved into the DATAX, DATAY, and DATAZ
registers), there must be at least 5 탎 between the end of reading
the data registers and the start of a new read of the FIFO or a read
of the FIFO_STATUS register (Address 0x39). The end of reading
a data register is signified by the transition from Register 0x37 to
Register 0x38 or by the CS pin going high.
For SPI operation at 1.6 MHz or less, the register addressing
portion of the transmission is a sufficient delay to ensure that the
FIFO has completely popped. For SPI operation greater than 1.6
MHz, it is necessary to deassert the CS pin to ensure a total
delay of 5 탎; otherwise, the delay is not sufficient. The total delay
necessary for 5 MHz operation is at most 3.4 탎. This is not a
concern when using I2C mode because the communication rate is
low enough to ensure a sufficient delay between FIFO reads.




Register 0x39뾈IFO_STATUS (Read Only)
Table 39. Register 0x39
D7 D6 D5 D4 D3 D2 D1 D0
FIFO_TRIG 0 Entries
FIFO_TRIG Bit
A 1 in the FIFO_TRIG bit corresponds to a trigger event occurring,
and a 0 means that a FIFO trigger event has not occurred.
Entries Bits
These bits report how many data values are stored in FIFO. Access
to collect the data from FIFO is provided through the DATAX,
DATAY, and DATAZ registers. FIFO reads must be done in burst or
multiple-byte mode because each FIFO level is cleared after any
read (single- or multiple-byte) of FIFO. FIFO stores a maximum of
32 entries, which equates to a maximum of 33 entries available at
any given time because an additional entry is available at the output
filter of the device.





Register 0x32 to Register 0x37뾆ATAX0,
DATAX1, DATAY0, DATAY1, DATAZ0, DATAZ1
(Read Only)
These six bytes (Register 0x32 to Register 0x37) are eight bits
each and hold the output data for each axis. Register 0x32 and
Register 0x33 hold the output data for the x-axis, Register 0x34 and
Register 0x35 hold the output data for the y-axis, and Register 0x36
and Register 0x37 hold the output data for the z-axis. The output
data is twos complement, with DATAx0 as the least significant byte
and DATAx1 as the most significant byte, where x represent X,
Y, or Z. The DATA_FORMAT register (Address 0x31) controls the
format of the data. It is recommended that a multiple-byte read of all
registers be performed to prevent a change in data between reads
of sequential registers.


*/


UART_HandleTypeDef huart1;      							// UART1 structure variable
static void MX_USART1_UART_Init(void);

uint8_t data_rec[6];
uint8_t chipid=0;
int16_t x,y,z;
float xg, yg, zg;
char x_char[3], y_char[3], z_char[3];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);


char msg[UART_SIZE];
uint16_t count = 0;


//little endian
void setHeaderField(uint8_t* header, int index, int byteSize, int value){
	int i;
	
	for(i = index; i < (byteSize + index); i++){
		header[i] = value & 0xFF;
		value >>= 8;
	}
}


int lastSessionId = 0;

int generateSessionId(){
	int thisSessionId;
	do{
		thisSessionId = rand();
	}while(lastSessionId == thisSessionId);
	
	lastSessionId = thisSessionId;
	return lastSessionId;
}


#define SAMPLE_DATA_TYPE short

#if MEASURE_X_AXIS == 1
SAMPLE_DATA_TYPE x_axis_samples[SAMPLE_SIZE];
#endif

#if MEASURE_Y_AXIS == 1
SAMPLE_DATA_TYPE y_axis_samples[SAMPLE_SIZE];
#endif

#if MEASURE_Z_AXIS == 1
SAMPLE_DATA_TYPE z_axis_samples[SAMPLE_SIZE];
#endif


bool seedSet = false;

void setSeedForRandFunct(){
	if(!seedSet){
		srand(HAL_GetTick());
		seedSet = true;
	}
}

void generateGenericLoraHeader(uint8_t* packet, int destinationAddress, int sourceAddress, int packetType, int sessionId, int packetId){
	setHeaderField(packet, LORA_PACKET_DESTINATION_ADDRESS_INDEX, LORA_PACKET_DESTINATION_ADDRESS_BYTE_LENGTH, destinationAddress);
	setHeaderField(packet, LORA_PACKET_SOURCE_ADDRESS_INDEX, LORA_PACKET_SOURCE_ADDRESS_BYTE_LENGTH, sourceAddress);
	setHeaderField(packet, LORA_PACKET_PACKET_TYPE_INDEX, LORA_PACKET_PACKET_TYPE_BYTE_LENGTH, packetType);
	setHeaderField(packet, LORA_PACKET_SESSION_ID_INDEX, LORA_PACKET_SESSION_ID_BYTE_LENGTH, sessionId);
	setHeaderField(packet, LORA_PACKET_PACKET_ID_INDEX, LORA_PACKET_PACKET_ID_BYTE_LENGTH, packetId);
}

void generateDataLoraHeader(uint8_t* packet, int destinationAddress, int sourceAddress, int sessionId, int packetId, int packetType, int axis){
	generateGenericLoraHeader(packet, destinationAddress, sourceAddress, packetType, sessionId, packetId);
	setHeaderField(packet, LORA_PACKET_AXIS_INDEX, LORA_PACKET_AXIS_BYTE_LENGTH, axis);
}


lora_pins_t lora_pins;												// Structure variable for lora pins
lora_t lora;																	// Structure variable for lora

SPI_HandleTypeDef hspi1;        							// SPI1  structure variable
SPI_HandleTypeDef hspi2;        							// SPI2  structure variable
UART_HandleTypeDef huart1;      							// UART1 structure variable



void printArray(int8_t* array, int len){
		for(int i = 0; i < len; i++){
				sprintf(msg,"%d, ", array[i]);
				HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
		}
		printLine("");
}



void sendPacket(uint8_t* packet, int packetSize){
	
	#if USE_LORA == 0
		Error_Handler("Lora is disabled");
	#endif
	
	#if USE_LORA == 1
	
	int ret = lora_begin_packet(&lora);
	
	lora_tx(&lora, packet, packetSize);
	ret = lora_end_packet(&lora);
	
	#endif
	
}

int getValueFromPositionInArray(uint8_t* array, int position, int byteLength){
	int value = 0;
	
	for(int i = 0; i < byteLength; i++){
		value <<= 8;
		value += array[position + i];
	}
	
	return value;
}



bool checkDestinationAddress(uint8_t* packet, int destAddressForCheck){
	for(int i = LORA_PACKET_DESTINATION_ADDRESS_INDEX; i < LORA_PACKET_DESTINATION_ADDRESS_BYTE_LENGTH; i++){
		if(packet[i] != (destAddressForCheck & 0xFF)) return false;
		
		destAddressForCheck >>= 8;
	}
	
	return true;
}

int remainingTransmissions;

bool sendPacketWithAck(uint8_t* packet, uint8_t packetSize, int* packetId, int addressOfThisDevice){
	
	int generatedTimeForRetransmission = 0;
	
	for(remainingTransmissions = LORA_PACKET_TOTAL_TRANSMISSIONS_BEFORE_FAIL; 
			remainingTransmissions > 0; remainingTransmissions--){
				
		#if DEBUG_SW == 1
		sprintf(msg,"Sending packet %d\r\n", *packetId);
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
	#endif

	ENABLE_TX_LED();
	sendPacket(packet, packetSize);
	DISABLE_TX_LED();
	
	
	uint8_t lora_receive_buffer[LORA_PACKET_LENGTH + 1];

	int timerForAckRecv = 0;
	bool ackReceived = false;
				

	while(timerForAckRecv < TIME_FOR_ACK_MS && !ackReceived){
		

		if(lora_prasePacket(&lora)){
			uint16_t lora_received_length = 0;
			
			ENABLE_RX_LED();
			
			int packetType = -1;
			
			bool correctAddress = false;
			while(lora_available(&lora)){
				lora_receive_buffer[lora_received_length++] = lora_read(&lora);
				
				if(lora_received_length == (LORA_PACKET_DESTINATION_ADDRESS_INDEX + LORA_PACKET_DESTINATION_ADDRESS_BYTE_LENGTH)){
					correctAddress = checkDestinationAddress(lora_receive_buffer, addressOfThisDevice);
					
					if(!correctAddress) break;
						
				}
			}
			
			DISABLE_RX_LED();
			
			if(correctAddress){
				
				packetType = getValueFromPositionInArray(lora_receive_buffer, 
					LORA_PACKET_PACKET_TYPE_INDEX, LORA_PACKET_PACKET_TYPE_BYTE_LENGTH);
			
				if(packetType == LORA_PACKET_TYPE_ACK){
					ackReceived = true;
				}
					
			}
	}
	timerForAckRecv++;
	HAL_Delay(1);
	

}
	
	if(!ackReceived && timerForAckRecv >= TIME_FOR_ACK_MS){
		#if DEBUG_SW == 1
		printLine("ACK not received within specified timeframe");
	#endif
	
		if(remainingTransmissions == 1){
		#if DEBUG_SW == 1
		Error_Handler("Transmission of this packet was unsuccessful");
			return false;
	#endif
	}
		
	}else{
		break;
	}
	
	
	generatedTimeForRetransmission = (rand() % 
	(TIME_FOR_RETRANSMISSION_UPPER_BORDER_MS - TIME_FOR_RETRANSMISSION_LOWER_BORDER_MS + 1)) 
	+ TIME_FOR_RETRANSMISSION_LOWER_BORDER_MS;	
	
	HAL_Delay(generatedTimeForRetransmission);
	
	sprintf(msg, "generatedTimeForRetransmission: %d", generatedTimeForRetransmission);
		printLine(msg);		
	
			}
	
	return true;
}

bool sendAllDataForSpecificAxis(uint8_t* packet, int axisId, uint8_t* axisData, int currentSessionId, int* packetId){
	char msg[UART_BUFFER_SIZE];
	
	int samplesBufferIndex = 0;
	int payloadSize;
	int packetSize;
	
	
	int byte_sampleSize = SAMPLE_SIZE * sizeof(SAMPLE_DATA_TYPE);
	
	while(samplesBufferIndex < byte_sampleSize){
		
	generateDataLoraHeader(packet, CU_LORA_ADDRESS, SU_LORA_ADDRESS, currentSessionId, *packetId, LORA_PACKET_TYPE_DATA_PACKET, axisId);
	
	if(samplesBufferIndex + (LORA_PACKET_LENGTH - HEADER_SIZE) < byte_sampleSize){
		payloadSize = LORA_PACKET_LENGTH - HEADER_SIZE;
	}else{
		payloadSize = byte_sampleSize - samplesBufferIndex;
	}
	
	if(payloadSize > LORA_PACKET_LENGTH - HEADER_SIZE){
		//an error occured
		
		
		sprintf(msg, "Payload size: %d", payloadSize);
		printLine(msg);
		Error_Handler("Payload size is greater than allowed size, session is abrupted");
		return false;
	}
	
	#warning here may be a problem with address allignment:
	memcpy(packet + HEADER_SIZE, axisData + samplesBufferIndex, payloadSize);
	
	samplesBufferIndex += payloadSize;
	packetSize = HEADER_SIZE + payloadSize;
	
	
	if(!sendPacketWithAck(packet, packetSize, packetId, SU_LORA_ADDRESS)) return false;
	
	(*packetId)++;
	}
	
	return true;
}


bool sendAllDataThroughLora(){
	uint8_t packet[LORA_PACKET_LENGTH];
	
	int currentSessionId = generateSessionId();
	int packetId = 0;
	
	#if MEASURE_X_AXIS == 1
	#if DEBUG_SW == 1
		printLine("sending x axis:");
	#endif
	
	if(!sendAllDataForSpecificAxis(packet, X_AXIS, (uint8_t*)x_axis_samples, currentSessionId, &packetId)){
		Error_Handler("Unable to send data for X axis");
		return false;
	}
	
	#endif
	
	
	#if MEASURE_Y_AXIS == 1
	#if DEBUG_SW == 1
		printLine("sending y axis:");
	#endif
	
	if(!sendAllDataForSpecificAxis(packet, Y_AXIS, (uint8_t*)y_axis_samples, currentSessionId, &packetId)){
	Error_Handler("Unable to send data for Y axis");
		return false;
	}
	#endif
	
	#if MEASURE_Z_AXIS == 1
	#if DEBUG_SW == 1
		printLine("sending z axis:");
	#endif
	
	if(!sendAllDataForSpecificAxis(packet, Z_AXIS, (uint8_t*)z_axis_samples, currentSessionId, &packetId)){
		Error_Handler("Unable to send data for Z axis");
		return false;
	}
	#endif
	
	//generateLoraHeader(packet, CU_LORA_ADDRESS, SU_LORA_ADDRESS, currentSessionId, -1, LORA_PACKET_TYPE_END_SESSION, -1);
	generateGenericLoraHeader(packet, CU_LORA_ADDRESS, SU_LORA_ADDRESS, LORA_PACKET_TYPE_END_SESSION, currentSessionId, -1);
	
	ENABLE_TX_LED();
	sendPacket(packet, HEADER_SIZE);
	DISABLE_TX_LED();
	
	return true;
}


#if CENTRAL_UNIT == 1
static void ESP_GPIO_Init(void);

typedef struct {
	int pin;
	void * port;
} esp_gpio_t;

typedef struct {
	/*esp_gpio_t reset;
	esp_gpio_t dio0;*/
	esp_gpio_t nss;
	void * spi;
} esp_pins_t;

typedef struct {
	esp_pins_t * pin;
} esp_t;

void ESP_SPI_write(esp_t * module, uint8_t addr, uint8_t cmd);
extern void ESP_write_data(esp_t * module, uint8_t* data, uint16_t size);

esp_pins_t esp_pins;
esp_t esp;


#endif


/*
#define NPT 64 /* Number of sampling points*/
//uint32_t lBufInArray[NPT];
//uint32_t lBufOutArray[NPT/2]; /* FFT output array*/
//uint32_t lBufMagArray[NPT/2]; /* Amplitude of each harmonic*/


/*
void GetPowerMag(void)
{
    signed short lX,lY;
    float X,Y,Mag;
    unsigned short i;
	
    for(i=0; i<NPT/2; i++)
    {
        lX = (lBufOutArray[i] << 16) >> 16;
        lY = (lBufOutArray[i] >> 16);

        //divide by 32768 and multiply by 65536 in order to comply with the floating point number calculation rules
        X = NPT * ((float)lX) / 32768;
        Y = NPT * ((float)lY) / 32768;
        Mag = sqrt(X * X + Y * Y)*1.0/ NPT;
        if(i == 0)    
            lBufMagArray[i] = (unsigned long)(Mag * 32768);
        else
            lBufMagArray[i] = (unsigned long)(Mag * 65536);
    }
}
*/


//#define TEST_LENGTH_SAMPLES 2048



/* -------------------------------------------------------------------
* External Input and Output buffer Declarations for FFT Bin Example
* ------------------------------------------------------------------- */
//float32_t testInput_f32_10khz[SAMPLE_SIZE];
static float32_t testOutput[SAMPLE_SIZE / 2];

/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */

uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

/* Reference index at which max energy of bin ocuurs */
uint32_t testIndex = 0;


void accelerometer_get_samples(){
	#if (MEASURE_X_AXIS && MEASURE_Y_AXIS && MEASURE_Z_AXIS) == 0
		return;
	#endif
	
	//double xyz[3] = {0, 0, 0};
	short x, y, z;	
	
	
	ADXL345_setInterrupt(ADXL345_INT_WATERMARK_BIT, 1); //enable watermark interrupt
	
	//#if FFT == 0
	for(int i = 0; i < SAMPLE_SIZE; i++){
		if(ADXL345_getInterfaceState())
		{
			// Accelerometer Readings 
			ADXL345_readAccel(&x, &y, &z);
			
			#if MEASURE_X_AXIS == 1
			x_axis_samples[i] = x;
			#endif
			
			#if MEASURE_Y_AXIS == 1
			y_axis_samples[i] = y;
			#endif
			
			#if MEASURE_Z_AXIS == 1
			z_axis_samples[i] = z;
			#endif
			
			
			//ADXL345_readAccel(&x, &y, &z);
			//ADXL345_get_Gxyz(xyz);
			//HAL_Delay(SAMPLING_PERIOD_MS);
			
			
			if(i < SAMPLE_SIZE - 1)
			if((ADXL345_getInterruptSourceWithInt(ADXL345_INT_WATERMARK_BIT)) == 0){
				//SLEEP until watermark interrupt
				
				HAL_SuspendTick();
				HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
				HAL_ResumeTick();
				
				ADXL345_getInterruptSource();
				
				/*
				interruptSource = ADXL345_getInterruptSource();
				int mask = 128;
			
				sprintf(msg,"interrupt sources: \r\n");
				
				int i;
				for(i = 0; i < 8; i++){
				
					if(interruptSource & mask) sprintf(msg + strlen(msg),"1");
					else sprintf(msg + strlen(msg),"0");
						
					mask >>= 1;
				}
				
				sprintf(msg + strlen(msg),"\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
				*/
				
			}
			
			
			
		}
	}
	ADXL345_setInterrupt(ADXL345_INT_WATERMARK_BIT, 0); //disable watermark interrupt
	
	//#endif
	/*
	#if FFT == 1
	
	for(int i = 0; i < SAMPLE_SIZE; i += 2){
		if(ADXL345_getInterfaceState())
		{
			// Accelerometer Readings 
			ADXL345_readAccel(&x, &y, &z);
			
			
			//uklada sa ako komplexne cislo
			x_axis_samples[i] = x;
			x_axis_samples[i + 1] = 0.0f;
			
			y_axis_samples[i] = y;
			y_axis_samples[i + 1] = 0.0f;
			
			z_axis_samples[i] = z;
			z_axis_samples[i + 1] = 0.0f;
			
			
			
			//ADXL345_readAccel(&x, &y, &z);
			//ADXL345_get_Gxyz(xyz);
			HAL_Delay(SAMPLING_PERIOD_MS);
			
		}
	}
	
	#endif
	*/
}

void print(const char* message){
	sprintf(msg,message);
	HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
}

void printLine(const char* message){
	sprintf(msg,message);
	sprintf(msg + strlen(message), "\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
}

int rx_expectedPacketId = 0;

int x_axis_byte_length = 0;
int y_axis_byte_length = 0;
int z_axis_byte_length = 0;

void printArray_short(short* array, int len){
		for(int i = 0; i < len; i++){
				sprintf(msg,"%d, ", array[i]);
				HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
		}
		printLine("");
}

void copyDataFromPacketToMemory(uint8_t* packet, int packetLength){
int axisId = getValueFromPositionInArray(packet, 
			LORA_PACKET_AXIS_INDEX, LORA_PACKET_AXIS_BYTE_LENGTH);
	int payloadSize = packetLength - HEADER_SIZE;
	
	switch(axisId){
		
		#if MEASURE_X_AXIS == 1
		case X_AXIS:
			printLine("copying x axis to mem");
			memcpy((uint8_t*)x_axis_samples + x_axis_byte_length, packet + HEADER_SIZE, payloadSize);
		x_axis_byte_length += payloadSize;
		break;
		#endif
		
		#if MEASURE_Y_AXIS == 1
		case Y_AXIS:
			printLine("copying y axis to mem");
			memcpy((uint8_t*)y_axis_samples + y_axis_byte_length, packet + HEADER_SIZE, payloadSize);
		y_axis_byte_length += payloadSize;
		break;
		#endif
		
		
		#if MEASURE_Z_AXIS == 1
		case Z_AXIS:
			printLine("copying z axis to mem");
			memcpy((uint8_t*)z_axis_samples + z_axis_byte_length, packet + HEADER_SIZE, payloadSize);
		z_axis_byte_length += payloadSize;
		break;
		#endif
	}
}

bool checkReceivedPacketHeader(uint8_t* packet){
	
	#warning check session id
	
	/*
	printLine("receivedPacketData");
	printArray((int8_t*)packet + HEADER_SIZE, packetLength - HEADER_SIZE);
	*/

	return true;
}



void sendAck(int currentSessionId, int packetId){
	uint8_t packet[LORA_PACKET_LENGTH];
						
	generateGenericLoraHeader(packet, SU_LORA_ADDRESS, CU_LORA_ADDRESS, LORA_PACKET_TYPE_ACK, currentSessionId, packetId);
	
	ENABLE_TX_LED();
	sendPacket(packet, LORA_PACKET_LENGTH);
	DISABLE_TX_LED();
}



int testSessionId = 0;
int testSessionsStats[TOTAL_NUMBER_OF_TEST_SESSIONS][2];

#define FLOAT_SAMPLES_SIZE SAMPLE_SIZE * 2
float32_t samples[FLOAT_SAMPLES_SIZE]; //vynasobene 2 aby sa spravilo miesto pre imaginarnu zlozku
	//alebo potom pouzit realnu fft


void ADXL345_disableInterrupts(void){
	#if HAS_ADXL345 == 1
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
#if USE_ADXL345_INT2
	
	
	
	GPIO_InitStruct.Pin = ADXL345_TO_MCU_INT2_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADXL345_TO_MCU_INT2_PORT, &GPIO_InitStruct);

#endif

#if USE_ADXL345_INT1

GPIO_InitStruct.Pin = ADXL345_TO_MCU_INT1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADXL345_TO_MCU_INT1_PORT, &GPIO_InitStruct);

#endif


#if USE_ADXL345_INT1 || USE_ADXL345_INT2
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0); //2 int pins of accelerometer
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	
#endif

#endif
}

void ADXL345_enableInterrupts(void){
	#if HAS_ADXL345 == 1
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
#if USE_ADXL345_INT2
	
	
	
	GPIO_InitStruct.Pin = ADXL345_TO_MCU_INT2_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
			
GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(ADXL345_TO_MCU_INT2_PORT, &GPIO_InitStruct);

#endif

#if USE_ADXL345_INT1

GPIO_InitStruct.Pin = ADXL345_TO_MCU_INT1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ADXL345_TO_MCU_INT1_PORT, &GPIO_InitStruct);

#endif


#if USE_ADXL345_INT1 || USE_ADXL345_INT2
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0); //2 int pins of accelerometer
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	
#endif

#endif
}

#warning ak sa nezmestia data do ram, presunut ich do global 
int irq2counter = 0;
int irq1counter = 0;
int accelerometer_activity_flag = 0;
int sw1_flag = 0;
int sw3_flag = 0;
int sw4_flag = 0;

char interruptSource = 0;

int checkPacketId(uint8_t* packet){
	
	int packetId = getValueFromPositionInArray(packet, 
			LORA_PACKET_PACKET_ID_INDEX, LORA_PACKET_PACKET_ID_BYTE_LENGTH);
	
	
	if(packetId < rx_expectedPacketId){
		sprintf(msg,"Packet %d already received\r\n", packetId);
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
		return 2;
	}
	
		if(rx_expectedPacketId != packetId){
				
			
			sprintf(msg,"Packet id: %d\r\n", packetId);
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
			
			sprintf(msg,"rx_expectedPacketId: %d\r\n", rx_expectedPacketId);
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
			
			Error_Handler("Packet with unexpected packet id was received");
				
			rx_expectedPacketId = 0;
			return 0;
			}
		
			return 1;
}


// Main Function
int main(void)
{
	HAL_Init();																	// HAL library Initialization
  SystemClock_Config();												// System clock Initialization

	MX_GPIO_Init();															// GPIO Pins Initialization

	MX_SPI1_Init();															// SPI Communication Initialization
	MX_USART1_UART_Init(); 											// UART1 Communication Initialization

	printLine("UART initialized");
	
	
	
	#if CENTRAL_UNIT == 1
	ESP_GPIO_Init();
	
	esp_pins.nss.port   = ESP_SS_PORT;				// NSS pin to which port is connected
	esp_pins.nss.pin    = ESP_SS_PIN;					// NSS pin to which pin is connected
	esp_pins.spi  			= &hspi1;
	esp.pin = &esp_pins;	
	
  #endif
	
	#if SECONDARY_UNIT == 1
	
		//ADXL345_init();
		
		ADXL345_SPI_Init();	
		
		
	//ADXL345_powerOn();
	
	ADXL345_writeTo(ADXL345_POWER_CTL, 0);	// Wakeup     
	ADXL345_setRangeSetting(2); //+-2g

	ADXL345_setFullResBit(1);
	
	ADXL345_disableLowPower();
	//ADXL345_setBypassMode();
	ADXL345_setStreamMode();
	
	ADXL345_setRate(SAMPLING_FREQUENCY);	
	double r = ADXL345_getRate();
	sprintf(msg, "rate: %lf", r);
	printLine(msg);

	ADXL345_setInterrupt(255, 0); //clear all interrupts
	ADXL345_setActivityThreshold(5); //125mg acceleration will trigger interrupt

	ADXL345_setInterruptMapping( ADXL345_INT_WATERMARK_BIT,   ADXL345_INT1_PIN );
	
	ADXL345_setImportantInterruptMapping(0,0, 0, 2, 0); //activity to INT2 pin
	ADXL345_setInterrupt(ADXL345_INT_ACTIVITY_BIT, 1); //enable activity interrupt
	
	ADXL345_writeTo(ADXL345_POWER_CTL, 16);	// Auto_Sleep
	ADXL345_writeTo(ADXL345_POWER_CTL, 8); //measure
	
	ADXL345_setActivityX(1); //activity of X axis will be triggered
	ADXL345_enableInterrupts(); //sets pins of MCU
	
	ADXL345_getInterruptSource(); //this needs to be here, because some interrupt 
	//has already been trigerred and it needs to be read which one, so that it can be repeated later again
	

	#endif
	
	
	#if USE_LORA == 1
	//lora_pins.dio0.port  = LORA_DIO0_PORT;       
	//lora_pins.dio0.pin   = LORA_DIO0_PIN;
	lora_pins.nss.port   = LORA_SS_PORT;				// NSS pin to which port is connected
	lora_pins.nss.pin    = LORA_SS_PIN;					// NSS pin to which pin is connected
	lora_pins.reset.port = LORA_RESET_PORT;			// RESET pin to which port is connected
	lora_pins.reset.pin  = LORA_RESET_PIN;			// RESET pin to which pin is connected
	lora_pins.spi  			 = &hspi1;
	
	lora.pin = &lora_pins;											
	
	#if LORA_LIB_VINOD == 1
	//lora.frequency = FREQ_433MHZ;								// 433MHZ Frequency 
	//lora.frequency = FREQ_865MHZ;								// 865MHZ Frequency
	//lora.frequency = FREQ_866MHZ;								// 866MHZ Frequency
	lora.frequency = FREQ_867MHZ;								// 867MHZ Frequency
	#endif
	
	
	printLine("Configuring LoRa module");
	
	#if LORA_LIB_VINOD == 1
	while(lora_init(&lora)){										// Initialize the lora module
		printLine("LoRa Failed");
		HAL_Delay(1000);
		
		HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GREEN1_PORT, LED_GREEN1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GREEN2_PORT, LED_GREEN2_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GREEN3_PORT, LED_GREEN3_PIN, GPIO_PIN_SET);
		HAL_Delay(NOTIFY_LED_ON_TIME);
		HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GREEN1_PORT, LED_GREEN1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GREEN2_PORT, LED_GREEN2_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GREEN3_PORT, LED_GREEN3_PIN, GPIO_PIN_RESET);
		HAL_Delay(NOTIFY_LED_ON_TIME);
	}
	
	#else
	
	while(!lora_init_not_vinod(&lora)) // Initialize the lora module
  {
     printLine("LoRa Failed");
		HAL_Delay(1000);
		
		HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GREEN1_PORT, LED_GREEN1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GREEN2_PORT, LED_GREEN2_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GREEN3_PORT, LED_GREEN3_PIN, GPIO_PIN_SET);
		HAL_Delay(NOTIFY_LED_ON_TIME);
		HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GREEN1_PORT, LED_GREEN1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GREEN2_PORT, LED_GREEN2_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GREEN3_PORT, LED_GREEN3_PIN, GPIO_PIN_RESET);
		HAL_Delay(NOTIFY_LED_ON_TIME);
  }
	#endif
	printLine("Done configuring LoRa module");
	
	
	#if CENTRAL_UNIT == 1
	HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_SET);
	
	#warning enabled or disabled???
	printLine("ESP32 enabled");
	#endif


	#endif

	
		HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GREEN1_PORT, LED_GREEN1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GREEN2_PORT, LED_GREEN2_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GREEN3_PORT, LED_GREEN3_PIN, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GREEN1_PORT, LED_GREEN1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GREEN2_PORT, LED_GREEN2_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GREEN3_PORT, LED_GREEN3_PIN, GPIO_PIN_RESET);
		
		printLine("Waiting for 1 second");
		HAL_Delay(1000);
		/*
		#if SECONDARY_UNIT == 1
		
		#if EXTERNAL_INTERRUPTS == 1
		ADXL345_disableInterrupts();
		ADXL345_enableInterrupts();
		#endif
		
		#endif
		*/
		
	while (1)																		// Inifinite Loop
  {
		
		/*
		sprintf(msg,"irq2 counter %d\r\n", irq2counter);
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
			
		//interruptSource = ADXL345_getInterruptSource();

		
		byte im = 1;
		sprintf(msg,"im %d: %d\r\n", im, ADXL345_getInterruptMapping(im));
		im++;
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
		
		sprintf(msg,"im %d: %d\r\n", im, ADXL345_getInterruptMapping(im));
		
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
		im++;
		
		sprintf(msg,"im %d: %d\r\n", im, ADXL345_getInterruptMapping(im));
		
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
		im++;
		sprintf(msg,"im %d: %d\r\n", im, ADXL345_getInterruptMapping(im));
		
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
			im++;
			sprintf(msg,"im %d: %d\r\n", im, ADXL345_getInterruptMapping(im));
		
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
			im++;
			sprintf(msg,"im %d: %d\r\n", im, ADXL345_getInterruptMapping(im));
		
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
			im++;
			sprintf(msg,"im %d: %d\r\n", im, ADXL345_getInterruptMapping(im));
		
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
			im++;
			sprintf(msg,"im %d: %d\r\n", im, ADXL345_getInterruptMapping(im));
		
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
		
		
		
		interruptSource = ADXL345_getInterruptSource();
		int mask = 128;
	
		sprintf(msg,"interrupt sources: \r\n");
		
		int i;
		for(i = 0; i < 8; i++){
		
			if(interruptSource & mask) sprintf(msg + strlen(msg),"1");
			else sprintf(msg + strlen(msg),"0");
				
			mask >>= 1;
		}
		
		sprintf(msg + strlen(msg),"\r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
		*/
		
		
		// Initially make the NSS esp pin HIGH
	
		#if SECONDARY_UNIT == 1
	
		#if EXTERNAL_INTERRUPTS == 1
			HAL_Delay(1);
			
			 
			sprintf(msg,"irq1 counter %d\r\n", irq1counter);
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
			
		
		#endif
	
		/*
			sprintf(msg,"process interface funct: %d\r\n", ADXL345_processInterfaceFunctionality());
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
		*/
		
		#if TEST_CODE == 1
		#if USE_LORA == 1
		
		
		if(HAL_GPIO_ReadPin(SWITCH1_PORT, SWITCH1_PIN) == GPIO_PIN_SET){
			setSeedForRandFunct();
			
			ENABLE_USER_ACTION_LED();
			HAL_Delay(NOTIFY_LED_ON_TIME);
			DISABLE_USER_ACTION_LED();
			
			uint8_t packet[LORA_PACKET_LENGTH];
		
		int successfullyTransmitted = 0;
		int packetId = 0;
		
			generateGenericLoraHeader(packet, CU_LORA_ADDRESS, SU_LORA_ADDRESS, 
										LORA_PACKET_START_TEST_SESSION, testSessionId, packetId);
			


			if(sendPacketWithAck(packet, HEADER_SIZE, &packetId, SU_LORA_ADDRESS)){
				testSessionsStats[testSessionId][ATTEMPT_TEST_STATS_ID] = 0;
				
				for(packetId = 0; packetId < TOTAL_COUNT_OF_TEST_PACKETS_IN_SESSION; packetId++){
					generateGenericLoraHeader(packet, CU_LORA_ADDRESS, SU_LORA_ADDRESS, 
											LORA_PACKET_TYPE_TEST_PACKET, testSessionId, packetId);
				
					testSessionsStats[testSessionId][ATTEMPT_TEST_STATS_ID] += LORA_PACKET_TOTAL_TRANSMISSIONS_BEFORE_FAIL - remainingTransmissions;
					
					if(sendPacketWithAck(packet, LORA_PACKET_LENGTH, &packetId, SU_LORA_ADDRESS)) successfullyTransmitted++;
					
				}
				
			}else{
					printLine("Unable to initiate LoRa test session");
				}
			
		testSessionsStats[testSessionId][SUCCESSFULLY_TRANSMITTED_TEST_STATS_ID] = successfullyTransmitted;
		testSessionId++;
		
		}
		
		if(HAL_GPIO_ReadPin(SWITCH3_PORT, SWITCH3_PIN) == GPIO_PIN_SET){
			
			ENABLE_USER_ACTION_LED();
			HAL_Delay(NOTIFY_LED_ON_TIME);
			DISABLE_USER_ACTION_LED();
			
			printLine("===============================");
			printLine("LoRa test statistics:");
			
			int totalAcksReceived = 0;
			int totalRetransmissions = 0;
			
			int testSessionToPrint;
			for(testSessionToPrint = 0; testSessionToPrint < testSessionId; testSessionToPrint++){
				sprintf(msg,"Test session id: %d, ACKs received: %d/%d, retransmissions: %d/%d\r\n", 
				testSessionToPrint,
				testSessionsStats[testSessionToPrint][SUCCESSFULLY_TRANSMITTED_TEST_STATS_ID], TOTAL_COUNT_OF_TEST_PACKETS_IN_SESSION,
				testSessionsStats[testSessionToPrint][ATTEMPT_TEST_STATS_ID],
				TOTAL_COUNT_OF_TEST_PACKETS_IN_SESSION * LORA_PACKET_TOTAL_TRANSMISSIONS_BEFORE_FAIL - TOTAL_COUNT_OF_TEST_PACKETS_IN_SESSION
				);
				HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
				
				
				totalAcksReceived += testSessionsStats[testSessionToPrint][SUCCESSFULLY_TRANSMITTED_TEST_STATS_ID];
				totalRetransmissions += testSessionsStats[testSessionToPrint][ATTEMPT_TEST_STATS_ID];
				
			}
			
			int totalSessions = testSessionToPrint;
			
			sprintf(msg,"Total  sessions: %d, total ACKs received: %d/%d, total retransmissions: %d/%d\r\n", 
				totalSessions,
				totalAcksReceived, TOTAL_COUNT_OF_TEST_PACKETS_IN_SESSION * totalSessions,
				totalRetransmissions,
			  (TOTAL_COUNT_OF_TEST_PACKETS_IN_SESSION * LORA_PACKET_TOTAL_TRANSMISSIONS_BEFORE_FAIL 
					- TOTAL_COUNT_OF_TEST_PACKETS_IN_SESSION) * totalSessions
				);
				HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);

			printLine("===============================");
		
		}
		
		#endif
		#else //test code == 0
		
		/* Check the functinality of spi or i2c*/
		if(ADXL345_processInterfaceFunctionality() == 0){
		//block for measuring samples
			#if USE_SLEEP_MODE == 1
			
			
			

			if(!accelerometer_activity_flag){
			//clear any interrupt flags from internals of ADXL so that new interrupts can be generated, and hence wake up MCU
				ADXL345_getInterruptSource();
				printLine("entering sleep mode");
				HAL_SuspendTick();
				HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
				HAL_ResumeTick();
				
				printLine("exiting sleep mode");
			}
			
			
			/*
			interruptSource = ADXL345_getInterruptSource();
		int mask = 128;
	
		sprintf(msg,"interrupt sources: \r\n");
		
		int i;
		for(i = 0; i < 8; i++){
		
			if(interruptSource & mask) sprintf(msg + strlen(msg),"1");
			else sprintf(msg + strlen(msg),"0");
				
			mask >>= 1;
		}
		
		sprintf(msg + strlen(msg),"\r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
			*/
		
		
		#endif
			
			if(sw1_flag){
				sw1_flag = 0;
				printLine("SW1 pressed");
			}
			
			if(sw3_flag){
				sw3_flag = 0;
				printLine("SW3 pressed");
			}
			
			if(sw4_flag){
				sw4_flag = 0;
				printLine("SW4 pressed");
			}
			
		if(accelerometer_activity_flag){
			
		accelerometer_activity_flag = 0;
		printLine("ACC AD");

		#if DEBUG_SW == 1
		printLine("Getting samples");
		#endif
		
		accelerometer_get_samples();
	
		#if DEBUG_SW == 1
			printLine("Measuring done");
		#endif
		
		#if DEBUG_MEASURE == 1
		
		printLine("Printing samples");
		
		#if MEASURE_X_AXIS == 1
		printLine("x: ");
		printArray_short(x_axis_samples, SAMPLE_SIZE);
		#endif
		
		#if MEASURE_Y_AXIS == 1
		printLine("y: ");
		printArray_short(y_axis_samples, SAMPLE_SIZE);
		#endif
		
		#if MEASURE_Z_AXIS == 1
		printLine("z: ");
		printArray_short(z_axis_samples, SAMPLE_SIZE);
		#endif

		printLine("");
		#endif

			}
	}else{
			Error_Handler("Communication with accelerometer failed");
			continue;
		}

		#if FFT == 1
		
		sprintf(msg,"Performing FFT\r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
		
		#warning vypisovat velkost stacku a momentalnu zaplnenost
		#warning parne indexy na vstupe realna zlozka
		#warning neparne indexy na vstupe imaginarna zlozka
		#warning fft kniznica https://github.com/EMBEDONIX/STM32F103-ADC-VCP/blob/master/Drivers/CMSIS/DSP_Lib/Examples/arm_fft_bin_example/ARM/arm_fft_bin_example_f32.c
		
	
	int i;
	for(i = 0; i < FLOAT_SAMPLES_SIZE; i += 2){
		samples[i] = (float)x_axis_samples[i];
		samples[i + 1] = 0;
	}

		arm_status status;
		float32_t maxValue;
		status = ARM_MATH_SUCCESS;

		/* Process the data through the CFFT/CIFFT module */
		#if SAMPLE_SIZE == 2048
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, samples, ifftFlag, doBitReverse);
		#endif
#if SAMPLE_SIZE == 1024
		arm_cfft_f32(&arm_cfft_sR_f32_len512, samples, ifftFlag, doBitReverse);
		#endif
		
		#if SAMPLE_SIZE == 512
		arm_cfft_f32(&arm_cfft_sR_f32_len256, samples, ifftFlag, doBitReverse);
		#endif
		
		#if SAMPLE_SIZE == 256
		arm_cfft_f32(&arm_cfft_sR_f32_len128, samples, ifftFlag, doBitReverse);
		#endif
		
		#if SAMPLE_SIZE == 128
		arm_cfft_f32(&arm_cfft_sR_f32_len64, samples, ifftFlag, doBitReverse);
		#endif


		
		/* Process the data through the Complex Magnitude Module for
		calculating the magnitude at each bin */
		
		arm_cmplx_mag_f32(samples, testOutput, SAMPLE_SIZE / 2);


		//Calculates maxValue and returns corresponding BIN value
		arm_max_f32(testOutput, SAMPLE_SIZE / 2, &maxValue, &testIndex);

		sprintf(msg,"max value: %f \r\n", maxValue);
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);

  /* ----------------------------------------------------------------------
  ** Loop here if the signals fail the PASS check.
  ** This denotes a test failure
  ** ------------------------------------------------------------------- */

  if( status != ARM_MATH_SUCCESS)
  {
		Error_Handler("FFT was not computed successfully\r\n");
		while(1);
  }

	printLine("OUTPUT MAGNITUDES (absolute values of complex numbers after FFT): ");
	
	for(int i = 0; i < SAMPLE_SIZE / 2; i++){
		sprintf(msg,"%f, ", testOutput[i]);
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
	}
	
	
	/*
sprintf(msg,"\r\nFFT: \r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);

	
	for(int i = 0; i < SAMPLE_SIZE; i += 2){
		//sprintf(msg,"%f + %fi, ", samples[i], samples[i + 1]);
		sprintf(msg,"%f, ", samples[i]);
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
	}
	
	*/
	
	
	/*
	for(int i = 0; i < SAMPLE_SIZE; i += 2){
		//sprintf(msg,"%f + %fi, ", samples[i], samples[i + 1]);
		sprintf(msg,"%fi, ", samples[i + 1]);
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
	}
	*/
	printLine("");
  
		#endif


			#if USE_LORA == 1
				#if DEBUG_SW == 1
					printLine("Send data through LoRa");
				#endif
				
				if(!sendAllDataThroughLora()){
					Error_Handler("Unable to successfully send data through LoRa");
				}
				
			#endif
			
				
			#if USE_SLEEP_MODE == 0
				HAL_Delay(TIME_BETWEEN_MEASUREMENTS_MS);
			#endif
				
		#endif
		
				#endif
		
		#if CENTRAL_UNIT == 1
		
			#if USE_LORA == 1
			uint8_t lora_receive_buffer[LORA_PACKET_LENGTH + 1];

			int ret = 0;
				ret = lora_prasePacket(&lora);
		
		if(ret){
			
			
			uint16_t lora_received_length = 0;
			HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_RESET);
		
			#if LORA_RX_DEBUG == 1
		
		printLine("ESP disabled");
		
			HAL_GPIO_WritePin(LED_GREEN1_PORT, LED_GREEN1_PIN, GPIO_PIN_SET);
			HAL_Delay(NOTIFY_LED_ON_TIME);
			HAL_GPIO_WritePin(LED_GREEN1_PORT, LED_GREEN1_PIN, GPIO_PIN_RESET);
			HAL_Delay(NOTIFY_LED_ON_TIME);
			#endif
			
			ENABLE_RX_LED();
			
			int packetType = -1;
			bool sessionSuccessful = true;
			
			bool correctAddress = false;
			while(lora_available(&lora)){
				lora_receive_buffer[lora_received_length++] = lora_read(&lora);
				
				if(lora_received_length == (LORA_PACKET_DESTINATION_ADDRESS_INDEX + LORA_PACKET_DESTINATION_ADDRESS_BYTE_LENGTH)){
					correctAddress = checkDestinationAddress(lora_receive_buffer, CU_LORA_ADDRESS);
					
					#if RECEIVING_ADDRESS_CHECK == 1
						if(!correctAddress) break;
					#endif
						
				}
			}
			
			DISABLE_RX_LED();
			/*
			printLine("received packet: ");
			printArray((int8_t*)lora_receive_buffer, lora_received_length);
			
			*/
			
			if(correctAddress || RECEIVING_ADDRESS_CHECK == 0){
			#if RECEIVING_ADDRESS_CHECK == 0
				printLine("RECEIVING_ADDRESS_CHECK skipped");
			#endif
				
				packetType = getValueFromPositionInArray(lora_receive_buffer, 
					LORA_PACKET_PACKET_TYPE_INDEX, LORA_PACKET_PACKET_TYPE_BYTE_LENGTH);
			
				int currentSessionId = getValueFromPositionInArray(lora_receive_buffer, 
							LORA_PACKET_SESSION_ID_INDEX, LORA_PACKET_SESSION_ID_BYTE_LENGTH);
						
						int packetId = getValueFromPositionInArray(lora_receive_buffer, 
							LORA_PACKET_PACKET_ID_INDEX, LORA_PACKET_PACKET_ID_BYTE_LENGTH);
				
						#warning add timeout from receiving last packet if session has not ended
						
				
				if(packetType == LORA_PACKET_TYPE_DATA_PACKET){
					if(checkReceivedPacketHeader(lora_receive_buffer)){
						int packetReceived = checkPacketId(lora_receive_buffer);
							if( packetReceived == 1 || packetReceived == 2){
								if(packetReceived == 1){
									copyDataFromPacketToMemory(lora_receive_buffer, lora_received_length);
									rx_expectedPacketId++;
								}
								
								sessionSuccessful = true;
							}else{
								sessionSuccessful = false;
							}
					
					}else{
						sessionSuccessful = false;
					}
					
					if(sessionSuccessful){
						#if DEBUG_SW == 1
						printLine("packet successful");
						#endif
						
						sendAck(currentSessionId, packetId);
					}
				}
				
				if(packetType == LORA_PACKET_START_TEST_SESSION){
					printLine("New test session");
					sendAck(currentSessionId, packetId);
				}
				
				if(packetType == LORA_PACKET_TYPE_TEST_PACKET){
					
					if(checkReceivedPacketHeader(lora_receive_buffer)){
						
						#if DEBUG_SW == 1
						printLine("TP 1");
						#endif
					
						sendAck(currentSessionId, packetId);
					}
					
				}
				
				if(packetType == LORA_PACKET_TYPE_END_SESSION){
				rx_expectedPacketId = 0;
					
					#if MEASURE_X_AXIS == 1
					printLine("received x axis:");
					for(int i = 0; i < x_axis_byte_length / sizeof(SAMPLE_DATA_TYPE); i++){
							sprintf(msg,"%d, ", x_axis_samples[i]);
							HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
					}
					printLine("");
					
					#endif
					
					#if MEASURE_Y_AXIS == 1
					printLine("received y axis:");
					for(int i = 0; i < y_axis_byte_length / sizeof(SAMPLE_DATA_TYPE); i++){
							sprintf(msg,"%d, ", y_axis_samples[i]);
							HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
					}
					printLine("");
					
					#endif
					
					#if MEASURE_Z_AXIS == 1
					printLine("received z axis:");
					for(int i = 0; i < z_axis_byte_length / sizeof(SAMPLE_DATA_TYPE); i++){
							sprintf(msg,"%d, ", z_axis_samples[i]);
							HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
					}
					#endif
					
					printLine("");
					x_axis_byte_length = 0;
					y_axis_byte_length = 0;
					z_axis_byte_length = 0;

				}
				
			}
			
			
			#if LORA_RX_DEBUG == 1
			sprintf(msg,"received data length: %d\r\n", lora_received_length);
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
			sprintf(msg,"received data binary:\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
			
			
			
			for(int j = 0; j < lora_received_length; j++){
				sprintf(msg,"%d, ", buf[j]);
				HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
			}
			
			
			sprintf(msg,"\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
			
			
			sprintf(msg,"received string: %s\r\n",buf);
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
			
			#endif
			
			//count++;
			#if CLOUD == 1
			
				if(packetType == LORA_PACKET_TYPE_END_SESSION && sessionSuccessful){
					HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_SET);
					HAL_Delay(ESP_ON_WAIT_TIME);	
					
					sprintf(msg,"Sending to esp32 %d\r\n",count);
					HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);

					
					#if USE_DMA == 1
					printLine("Sending x axis to esp:");
					
					ESP_write_data(&esp, (uint8_t*)x_axis_samples, DMA_SIZE);
					HAL_Delay(ESP_PROCESS_AXIS_DATA_WAIT_TIME);
					
					printLine("Sending y axis to esp:");
					ESP_write_data(&esp, (uint8_t*)y_axis_samples, DMA_SIZE);
					HAL_Delay(ESP_PROCESS_AXIS_DATA_WAIT_TIME);
					
					printLine("Sending z axis to esp:");
					ESP_write_data(&esp, (uint8_t*)z_axis_samples, DMA_SIZE);
					
					
					#endif
					#if USE_DMA == 0
						ESP_SPI_write(&esp, 0, 100);
					#endif
					
					HAL_Delay(ESP_SEND_DATA_TO_CLOUD_WAIT_TIME);
					
					
					HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_RESET);
					HAL_Delay(1);
				}
			#endif
			
		}
		
		#endif
		
		HAL_Delay(1);
		
		#endif
  }
}
// END OF Main


#if CENTRAL_UNIT == 1

void ESP_SPI_write(esp_t * module, uint8_t addr, uint8_t cmd){

	HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_RESET);
  HAL_SPI_Transmit(module->pin->spi, &addr, 1, 1000);
	while (HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(module->pin->spi, &cmd, 1, 1000);
	while (HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_SET);
}

void ESP_write_data(esp_t * module, uint8_t* data, uint16_t size){

	HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(module->pin->spi, data, size, 1000);
	
	while (HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_SET);
}

#endif

void SystemClock_Config(void)
{
	// Initialize the system clock for delay and all
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler("HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK");
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler("HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK");
  }
}

static void MX_SPI1_Init(void)
{
	// Initialize SPI1 for lora communication and esp comm
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
	
	#warning skusit zvysit rychlost
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler("HAL_SPI_Init(&hspi1) != HAL_OK");
  }
}


#if SECONDARY_UNIT == 1

static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
	
	#warning adxl345 max rychlost 1.6mhz podla datasheetu pri citani z FIFO, aby sa zabezpecil 5us delay
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler("HAL_SPI_Init(&hspi2) != HAL_OK");
  }

}

#endif


static void MX_USART1_UART_Init(void)
{
	// UART1 for Print statements in the serial terminal with baud rate of 9600
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler("HAL_UART_Init(&huart1) != HAL_OK");
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	
	#if HAS_ESP32
	//disable esp
	HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_RESET);
	
	#endif
	
	
	// Intialize GPIOB - B11 for reset pin of lora
  GPIO_InitStruct.Pin = LORA_RESET_PIN;   
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_RESET_PORT, &GPIO_InitStruct);
	
	// Initilize GPIOA - A3 for NSS pin of lora
	GPIO_InitStruct.Pin = LORA_SS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_SS_PORT, &GPIO_InitStruct);	
	
	// Initilize GPIOA - B6 for NSS pin of led
	GPIO_InitStruct.Pin = LED_RED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_PORT, &GPIO_InitStruct);	
	
	
	GPIO_InitStruct.Pin = LED_GREEN1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN1_PORT, &GPIO_InitStruct);	
	
	GPIO_InitStruct.Pin = LED_GREEN2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN2_PORT, &GPIO_InitStruct);	
	
	
	GPIO_InitStruct.Pin = LED_GREEN3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN3_PORT, &GPIO_InitStruct);	
	
	#if CENTRAL_UNIT == 1
	
	GPIO_InitStruct.Pin = ESP_EN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESP_EN_PORT, &GPIO_InitStruct);	
	
	#endif
	


#if EXTERNAL_INTERRUPTS == 1

	GPIO_InitStruct.Pin = SWITCH1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWITCH1_PORT, &GPIO_InitStruct);	
	
	GPIO_InitStruct.Pin = SWITCH3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWITCH3_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = SWITCH4_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWITCH4_PORT, &GPIO_InitStruct);
	
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0); //pb3 switch4
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0); //pb4 switch3
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0); //pb5 - 9 switch5
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		

#endif

	#if EXTERNAL_INTERRUPTS == 0
	
	GPIO_InitStruct.Pin = SWITCH1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SWITCH1_PORT, &GPIO_InitStruct);	
	
	GPIO_InitStruct.Pin = SWITCH3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SWITCH3_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = SWITCH4_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SWITCH4_PORT, &GPIO_InitStruct);
	
	#endif
	
	// Initially make the NSS lora pin HIGH
	HAL_GPIO_WritePin(LORA_SS_PORT, LORA_SS_PIN, GPIO_PIN_SET);
	
}

#if CENTRAL_UNIT == 1

static void ESP_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Initilize GPIOA - A0 for NSS pin of esp
	GPIO_InitStruct.Pin = ESP_SS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESP_SS_PORT, &GPIO_InitStruct);	
	
	// Initially make the NSS esp pin HIGH
	HAL_GPIO_WritePin(ESP_SS_PORT, ESP_SS_PIN, GPIO_PIN_SET);
}

#endif

void Error_Handler(const char* message)
{
	
	sprintf(msg, "ERROR: ");
	sprintf(msg + strlen(msg), message);
	
	printLine(msg);
	
	//HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
		
}




#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{ 
}
#endif 
