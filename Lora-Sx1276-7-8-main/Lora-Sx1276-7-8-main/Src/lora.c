/* This Driver is suitable for SX1276/7/8 Lora module
 * Author: Vinod Kumar from Vinod Embedded
 * Goto: vinodembedded.wordpress.com for detailed explanation of the 
 * lora driver
 */

#include "lora.h"

uint8_t lora_read_reg(lora_t * module, uint8_t addr) {
	uint8_t txByte = addr & 0x7f;
	uint8_t rxByte = 0x00;
	HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(module->pin->spi, &txByte, 1, 1000);
	while (HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	
	HAL_SPI_Receive(module->pin->spi,&rxByte, 1, 1000);
	while(HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_SET);
	return rxByte;
}

void lora_write_reg(lora_t * module, uint8_t addr, uint8_t cmd){
	uint8_t add = addr | 0x80;
  HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_RESET);
  HAL_SPI_Transmit(module->pin->spi, &add, 1, 1000);
	while (HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(module->pin->spi, &cmd, 1, 1000);
	while (HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_SET);
}

uint8_t lora_begin_packet(lora_t * module){
	//int ret;
	if ((lora_read_reg(module, REG_OP_MODE) & MODE_TX) == MODE_TX) {
    return 1;
  }
	lora_write_reg(module, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
	lora_write_reg(module, REG_MODEM_CONFIG_1, 0x72);
	lora_write_reg(module, REG_FIFO_ADDR_PTR, 0);
  lora_write_reg(module, REG_PAYLOAD_LENGTH, 0);
	return 0;
}

void lora_tx(lora_t * module, uint8_t * buf, uint8_t size){
	int currentLength = lora_read_reg(module, REG_PAYLOAD_LENGTH);
  if ((currentLength + size > MAX_PKT_LENGTH)){
    size = MAX_PKT_LENGTH - currentLength;
  }

  for (int i = 0; i < size; i++) {
    lora_write_reg(module, REG_FIFO, buf[i]);
  }
  lora_write_reg(module, REG_PAYLOAD_LENGTH, currentLength + size);
}

uint8_t lora_end_packet(lora_t * module){
	uint8_t timeout = 100;
	lora_write_reg(module, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  while((lora_read_reg(module,REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
		if(--timeout==0){
			HAL_Delay(1);
			return 1;
		}
  }
  lora_write_reg(module, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	return 0;
}


uint8_t packetIndex;

int lora_prasePacket(lora_t * module){
	int packetLength = 0, irqFlags; //,ret;
	irqFlags = lora_read_reg(module, REG_IRQ_FLAGS);
	//ret = lora_read_reg(module, REG_MODEM_CONFIG_1);
	lora_write_reg(module, REG_MODEM_CONFIG_1, 0x72);
	
	lora_write_reg(module, REG_IRQ_FLAGS, irqFlags);

	if((irqFlags & IRQ_RX_DONE_MASK) && ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)) {
		packetLength = lora_read_reg(module, REG_RX_NB_BYTES);
		lora_write_reg(module, REG_FIFO_ADDR_PTR, lora_read_reg(module, REG_FIFO_RX_CURRENT_ADDR));
		lora_write_reg(module, REG_OP_MODE, 0x81);
		packetIndex = 0;
	}
	else if((lora_read_reg(module, REG_OP_MODE)) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)){
		lora_write_reg(module, REG_FIFO_ADDR_PTR, 0);
		lora_write_reg(module, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
	}
	if((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK)== IRQ_PAYLOAD_CRC_ERROR_MASK){
		return -1;
	}
	return packetLength;
}



uint8_t lora_available(lora_t * module){
	return (lora_read_reg(module, REG_RX_NB_BYTES) - packetIndex);
}




uint8_t lora_read(lora_t * module){
	if(!lora_available(module))
		return 0;
	packetIndex++;
	return lora_read_reg(module, REG_FIFO);
}




#if LORA_LIB_VINOD == 1


#include <string.h>



uint8_t lora_init(lora_t * module){
	uint8_t ret;
	HAL_GPIO_WritePin(module->pin->reset.port, module->pin->reset.pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(module->pin->reset.port, module->pin->reset.pin, GPIO_PIN_SET);
  HAL_Delay(10);
	
	ret = lora_read_reg(module, REG_VERSION);
	if(ret != 0x12){
		return 1;
	}
	lora_write_reg(module, REG_OP_MODE, (MODE_LONG_RANGE_MODE | MODE_SLEEP));
	//lora_write_reg(module, REG_FRF_MSB, 0x6C);
	//lora_write_reg(module, REG_FRF_MID, 0x40);
	//lora_write_reg(module, REG_FRF_LSB, 0x00);
	lora_set_frequency(module, FREQUENCY[module->frequency]);
	lora_write_reg(module, REG_FIFO_TX_BASE_ADDR, 0);
	lora_write_reg(module, REG_FIFO_RX_BASE_ADDR, 0);
	ret = lora_read_reg(module, REG_LNA);
	lora_write_reg(module, REG_LNA, ret | 0x03);
	lora_write_reg(module, REG_MODEM_CONFIG_3, 0x04);
	lora_write_reg(module, REG_PA_CONFIG, 0x8f);
	lora_write_reg(module, REG_OP_MODE, (MODE_LONG_RANGE_MODE | MODE_STDBY));
	return 0;
}


void lora_set_frequency(lora_t * module, uint64_t freq){
	uint64_t frf = ((uint64_t)freq << 19) / 32000000;
  lora_write_reg(module, REG_FRF_MSB, (uint8_t)(frf >> 16));
  lora_write_reg(module,REG_FRF_MID, (uint8_t)(frf >> 8));
  lora_write_reg(module,REG_FRF_LSB, (uint8_t)(frf >> 0));
	
}

/*   End of File
 * Author  : Vinod Kumar
 * Website : vinodembedded.wordpress.com
 */

#endif

#if LORA_LIB_VINOD == 0

// default header mode is explicit header mode
uint8_t _headerMode;
// for implicit header mode
uint8_t _payloadLength;

bool config(lora_t * module)
{
	int ret;
	ret = lora_read_reg(module, REG_VERSION);
	if(ret != 0x12){
		return 1;
	}
	lora_write_reg(module, REG_OP_MODE, (MODE_LONG_RANGE_MODE | MODE_SLEEP));
	
	// RF frequency 868.5M
  setFrequency(module, 868500000);  
	
	lora_write_reg(module, REG_FIFO_TX_BASE_ADDR, 0);
	lora_write_reg(module, REG_FIFO_RX_BASE_ADDR, 0);
	ret = lora_read_reg(module, REG_LNA);
	lora_write_reg(module, REG_LNA, ret | 0x03);
	lora_write_reg(module, REG_MODEM_CONFIG_3, 0x04);
	lora_write_reg(module, REG_PA_CONFIG, 0x8f);
	lora_write_reg(module, REG_OP_MODE, (MODE_LONG_RANGE_MODE | MODE_STDBY));
	
	
	
  // In setting mode, RF module should turn to sleep mode
  // low frequency mode£¬sleep mode
  //SPIWriteReg(LR_RegOpMode,LR_Mode_SLEEP|LORA_FREQUENCY_BAND);  
	lora_write_reg(module, LR_RegOpMode,LR_Mode_SLEEP|LORA_FREQUENCY_BAND);
  // wait for steady
  //delay(5);    
	HAL_Delay(5);	

  // external Crystal
  //SPIWriteReg(LR_RegTCXO,LR_EXT_CRYSTAL|LR_REGTCXO_RESERVED); 
	lora_write_reg(module, LR_RegTCXO,LR_EXT_CRYSTAL|LR_REGTCXO_RESERVED);
	
  // set lora mode
  //SPIWriteReg(LR_RegOpMode,LR_LongRangeMode_LORA|LORA_FREQUENCY_BAND);
	lora_write_reg(module, LR_RegOpMode,LR_LongRangeMode_LORA|LORA_FREQUENCY_BAND);
	
  // RF frequency 868.5M
  setFrequency(module, 868500000);  
  // max power ,20dB
  setTxPower(module, 0x0f);
  // close ocp
  //SPIWriteReg(LR_RegOcp,LR_OCPON_OFF|0x0B); 
	lora_write_reg(module, LR_RegOcp,LR_OCPON_OFF|0x0B);
	
  // enable LNA
  //SPIWriteReg(LR_RegLna,LR_LNAGAIN_G1|LR_LNABOOSTHF_1);   
	lora_write_reg(module, LR_RegLna,LR_LNAGAIN_G1|LR_LNABOOSTHF_1);
  
  _headerMode=LR_EXPLICIT_HEADER_MODE;
  
  // bandwidth = 62.5Hz, spreading factor=9,
  // coding rate = 4/5,implict header mode 
  setHeaderMode(module, _headerMode);
  setRFpara(module, LR_BW_62p5k,LR_CODINGRATE_1p25,LR_SPREADING_FACTOR_9,LR_PAYLOAD_CRC_ON);
  
	// LNA
  //SPIWriteReg(LR_RegModemConfig3,LR_LOWDATARATEOPTIMIZE_DISABLED);  
  lora_write_reg(module, LR_RegModemConfig3,LR_LOWDATARATEOPTIMIZE_DISABLED);
	
	/////////////////////////////
	
	// max rx time out
  setRxTimeOut(module, 0x3ff);
  // preamble 12+4.25 bytes   
  setPreambleLen(module, 12);

  // 20dBm on PA_BOOST pin
  //SPIWriteReg(LR_RegPADAC,LR_REGPADAC_RESERVED|LR_20DB_OUTPUT_ON);  
	lora_write_reg(module, LR_RegPADAC,LR_REGPADAC_RESERVED|LR_20DB_OUTPUT_ON);
	
	
   // no hopping
  //SPIWriteReg(LR_RegHopPeriod,0x00);     
	lora_write_reg(module, LR_RegHopPeriod,0x00);

////////////////////////////////////////////

  // DIO5=ModeReady,DIO4=CadDetected
  //SPIWriteReg(LR_RegDIOMAPPING2,LR_DIO4_CADDETECTED|LR_DIO5_MODEREADY);  
	lora_write_reg(module, LR_RegDIOMAPPING2,LR_DIO4_CADDETECTED|LR_DIO5_MODEREADY);

   // standby mode
  //SPIWriteReg(LR_RegOpMode,LR_Mode_STBY|LORA_FREQUENCY_BAND); 
	lora_write_reg(module, LR_RegOpMode,LR_Mode_STBY|LORA_FREQUENCY_BAND);
  
	
	
	
  // default payload length is 10 bytes in implicit mode
  //setPayloadLength(module, 10);

return 0;

}


bool setFrequency(lora_t * module, uint32_t freq)
{
  uint32_t frf;
  uint32_t temp1;
  uint32_t temp2;
  uint8_t reg[3];

  // Frf(23:0)=frequency/(XOSC/2^19)
  
  temp1=freq/1000000;
  temp2=LORA_XOSC/1000000;
  frf=temp1*524288/temp2;

  temp1=freq%1000000/1000;
  temp2=LORA_XOSC/1000;
  frf=frf+temp1*524288/temp2;

  temp1=freq%1000;
  temp2=LORA_XOSC;
  frf=frf+temp1*524288/temp2;
  
  reg[0]=frf>>16&0xff;
  reg[1]=frf>>8&0xff;
  reg[2]=frf&0xff;
  
	/*
  SPIWriteReg(LR_RegFrMsb,reg[0]);
  SPIWriteReg(LR_RegFrMid,reg[1]);
  SPIWriteReg(LR_RegFrLsb,reg[2]);  
	*/
	
	lora_write_reg(module, LR_RegFrMsb,reg[0]);
	lora_write_reg(module, LR_RegFrMid,reg[1]);
	lora_write_reg(module, LR_RegFrLsb,reg[2]);
  
  // read if the value has been in register
	//lora_read_reg(lora_t * module, uint8_t addr)
  if((reg[0]!=lora_read_reg(module, LR_RegFrMsb))||(reg[1]!=lora_read_reg(module, LR_RegFrMid))||(reg[2]!=lora_read_reg(module, LR_RegFrLsb)))
    return false;
}

bool setRFpara(lora_t * module, uint8_t BW,uint8_t CR,uint8_t SF,uint8_t payloadCRC)
{
  // check if the data is correct
  if(((BW&0x0f)!=0)||((BW>>8)>0x09))
    return false;
  if(((CR&0xf1)!=0)||((CR>>1)>0x04)||((CR>>1)<0x00))
    return false;
  if(((SF&0x0f)!=0)||((SF>>4)>12)||((SF>>4)<6))
    return false;
  if((payloadCRC&0xfb)!=0)
    return false;
  
  uint8_t temp;
  //SF=6 must be use in implicit header mode,and have some special setting
  if(SF==LR_SPREADING_FACTOR_6)   
  {
    _headerMode=LR_IMPLICIT_HEADER_MODE;
    //SPIWriteReg(LR_RegModemConfig1,BW|CR|LR_IMPLICIT_HEADER_MODE);
		lora_write_reg(module, LR_RegModemConfig1,BW|CR|LR_IMPLICIT_HEADER_MODE);
		
    //temp=SPIReadReg(LR_RegModemConfig2);
		temp = lora_read_reg(module, LR_RegModemConfig2);
    temp=temp&0x03;
    //SPIWriteReg(LR_RegModemConfig2,SF|payloadCRC|temp); 
		lora_write_reg(module, LR_RegModemConfig2,SF|payloadCRC|temp);
		
    // according to datasheet
    //temp = SPIReadReg(0x31);
		temp = lora_read_reg(module, 0x31);
    temp &= 0xF8;
    temp |= 0x05;
		/*
    SPIWriteReg(0x31,temp);
    SPIWriteReg(0x37,0x0C); 
		*/
		
		lora_write_reg(module, 0x31,temp);
		lora_write_reg(module, 0x37,0x0C);
  }
  else
  {
    //temp=SPIReadReg(LR_RegModemConfig2);
		temp = lora_read_reg(module, LR_RegModemConfig2);
		
    temp=temp&0x03;
		/*
    SPIWriteReg(LR_RegModemConfig1,BW|CR|_headerMode);
    SPIWriteReg(LR_RegModemConfig2,SF|payloadCRC|temp);
		*/
		lora_write_reg(module, LR_RegModemConfig1,BW|CR|_headerMode);
		lora_write_reg(module, LR_RegModemConfig2,SF|payloadCRC|temp);
		}
  return true;
}


bool setPreambleLen(lora_t * module, uint16_t length)
{
  // preamble length is 6~65535
  if(length<6)
    return false;
	
  //SPIWriteReg(LR_RegPreambleMsb,length>>8);
	lora_write_reg(module, LR_RegPreambleMsb,length>>8);
	
   // the actual preamble len is length+4.25
  //SPIWriteReg(LR_RegPreambleLsb,length&0xff);     
	lora_write_reg(module, LR_RegPreambleLsb,length&0xff);
}


bool setHeaderMode(lora_t * module, uint8_t mode)
{
  if(_headerMode>0x01)
    return false;
  _headerMode=mode;

  uint8_t temp;
  // avoid overload the other setting
  //temp=SPIReadReg(LR_RegModemConfig1);
	temp = lora_read_reg(module, LR_RegModemConfig1);
	
  temp=temp&0xfe;
  //SPIWriteReg(LR_RegModemConfig1,temp|mode);
	lora_write_reg(module, LR_RegModemConfig1,temp|mode);
}

// in implict header mode, the payload length is fix len
// need to set payload length first in this mode
bool setPayloadLength(lora_t * module, uint8_t len)
{
  _payloadLength=len;
  //SPIWriteReg(LR_RegPayloadLength,len);
	lora_write_reg(module, LR_RegPayloadLength,len);
	
}


bool setTxPower(lora_t * module, uint8_t power)
{
  if(power>0x0f)
    return false;
  //SPIWriteReg(LR_RegPaConfig,LR_PASELECT_PA_POOST|0x70|power);
	lora_write_reg(module, LR_RegPaConfig,LR_PASELECT_PA_POOST|0x70|power);
	
}
// only valid in rx single mode
bool setRxTimeOut(lora_t * module, uint16_t symbTimeOut)
{
  //rxtimeout=symbTimeOut*(2^SF*BW)
  if((symbTimeOut==0)||(symbTimeOut>0x3ff))
    return false;

  uint8_t temp;
  //temp=SPIReadReg(LR_RegModemConfig2);
	temp = lora_read_reg(module, LR_RegModemConfig2);
	
  temp=temp&0xfc;
	/*
  SPIWriteReg(LR_RegModemConfig2,temp|(symbTimeOut>>8&0x03));
  SPIWriteReg(LR_RegSymbTimeoutLsb,symbTimeOut&0xff); */
	lora_write_reg(module, LR_RegModemConfig2,temp|(symbTimeOut>>8&0x03));
	lora_write_reg(module, LR_RegSymbTimeoutLsb,symbTimeOut&0xff);
}

// RSSI[dBm]=-137+rssi value
uint8_t readRSSI(lora_t * module, uint8_t mode)
{
  if(!mode) //read current rssi
  {
    //return SPIReadReg(LR_RegRssiValue);
		return lora_read_reg(module, LR_RegRssiValue);
  }
  else      // read rssi of last packet received
    //return SPIReadReg(LR_RegPktRssiValue);
	return lora_read_reg(module, LR_RegPktRssiValue);
}


bool lora_init_not_vinod(lora_t* module)
{
  // reset lora
  powerOnReset();
  // Set RF parameter,like frequency,data rate etc
  config(module);

  return true;
}


void powerOnReset()
{

	HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
	HAL_Delay(20);
}





#endif
