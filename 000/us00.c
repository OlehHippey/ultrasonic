/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2017 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Nuvoton Technoledge Corp. 
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//  Date   : Apr/21/2017
//***********************************************************************************************************

//***********************************************************************************************************
//  File Function: N76E003 Timer0/1 Mode1 demo code
//***********************************************************************************************************
#include "N76E003.h"
#include "Common.h"
#include "Delay.h"
#include "Timer01.h"
#include "SFR_Macro.h"
#include "Function_define.h"

//*****************  The Following is in define in Fucntion_define.h  ***************************
//****** Always include Function_define.h call the define you want, detail see main(void) *******
//***********************************************************************************************
#if 0
//#define		TIMER0_MODE0_ENABLE		TMOD&=0x0F
//#define		TIMER0_MODE1_ENABLE		TMOD&=0x0F;TMOD|=0x10
//#define		TIMER0_MODE2_ENABLE		TMOD&=0x0F;TMOD|=0x20
//#define		TIMER0_MODE3_ENABLE		TMOD&=0x0F;TMOD|=0x3F

//#define		TIMER1_MODE0_ENABLE		TMOD&=0xF0
//#define		TIMER1_MODE1_ENABLE		TMOD&=0xF0;TMOD|=0x01
//#define		TIMER1_MODE2_ENABLE		TMOD&=0xF0;TMOD|=0x02
//#define		TIMER1_MODE3_ENABLE		TMOD&=0xF0;TMOD|=0xF3
#endif



#define LED_TX_PIN P00
#define LED_TX_PIN_SET set_P00
#define LED_TX_PIN_CLR clr_P00

#define LED_STS_PIN P01
#define LED_STS_SET set_P01
#define LED_STS_CLR clr_P01

#define LED_LVL_PIN P11
#define LED_LVL_PIN_SET set_P11
#define LED_LVL_PIN_CLR clr_P11


#define RF_TX_PIN P14
#define RF_TX_PIN_SET set_P14
#define RF_TX_PIN_CLR clr_P14

#define TRIG_PIN P17
#define TRIG_PIN_SET set_P17
#define TRIG_PIN_CLR clr_P17
#define ECHO_PIN() P30

#define RF_UZ_PIN P04
#define RF_UZ_SET set_P04
#define RF_UZ_CLR clr_P04

#define RF_TM_PIN P15
#define RF_TM_SET set_P15
#define RF_TM_CLR clr_P15


#define DS18B20_PIN() P05
#define DS18B20_PIN_SET set_P05
#define DS18B20_PIN_CLR clr_P05

#define ADC_CHANNEL 0 // ADC channel 0 (P1.2)
#define VDD_VOLTAGE 3300 // The supply voltage in millivolts (e.g., 3.3V)

//========================== VARIABLES =========================================
volatile bit trig_flag = 0;
volatile bit presence = 0;
volatile bit led_on = 0;

volatile unsigned int start_time = 0;
volatile unsigned int end_time = 0;
volatile unsigned int distance = 0;
volatile unsigned int cnt_sec = 0;

float outTemperature = 0.0;
float supplyVoltage = 0.0;
double Bandgap_Voltage = 0.0; 
double Bandgap_Value = 0.0;
double VDD_Voltage = 0.0;

volatile unsigned int batteryValue = 0;
volatile unsigned int temperatureValue = 0;
     
const unsigned char tmr_clbr=194;
const unsigned char poopADDR[3]={20,22}; //

//------------------------------------------------------------
double  Bandgap_Voltage,VDD_Voltage;			//please always use "double" mode for this
unsigned  char xdata ADCdataH[5], ADCdataL[5];
int ADCsumH=0, ADCsumL=0;
unsigned char ADCavgH,ADCavgL;

//========================== BASIC FUNCTIONS ===================================
unsigned char Get_Low_Byte(unsigned int word) {return word - 0x100 * (word / 0x100);}
//------------------------------------------------------------------------------
unsigned char Get_High_Byte(unsigned int word) {return word / 0x100;}
//------------------------------------------------------------------------------
unsigned char set_Bit(char num_val, char num_bit) {return num_val = num_val | (1 << num_bit);}
//------------------------------------------------------------------------------
unsigned char clr_Bit(char num_val, char num_bit) {return  num_val = num_val & ~(1 << num_bit);}
//------------------------------------------------------------------------------
unsigned char tgl_Bit(char num_val, char num_bit) {return num_val = num_val ^ (1 << num_bit);}
//------------------------------------------------------------------------------
unsigned char get_Bit(char num_val, char num_bit) {return num_val = (num_val >> num_bit) & 1; }
//------------------------------------------------------------------------------
unsigned char upd_Bit(char num_val, char num_bit, char num_shift) {
  if (get_Bit(num_val, num_bit)) return (set_Bit(num_val, num_bit+num_shift));
     else return (clr_Bit(num_val, num_bit+num_shift));
}
//------------------------------------------------------------------------------
unsigned char makeIntToHex(unsigned char dec_symbol){
  if (dec_symbol<10) return(dec_symbol+0x30); else
  if (dec_symbol>9 && dec_symbol<16)return(dec_symbol+0x37); else return(0);
}
//========================= MCU FUNCTIONS =====================================
void Init_MCU(){ //Init processes
    clr_EA; // Disable global interrupts during ADC initialization
    
    P00_Quasi_Mode;       //LED TX     
    P01_Quasi_Mode;       //LED STATUS
    P11_Quasi_Mode;       //LED LEVEL   
     
    P04_Quasi_Mode;       //SONAR POWER  
    P15_Quasi_Mode;       //DALLAS POWER
     
    P05_PushPull_Mode;    // Dallas pin
    P14_PushPull_Mode;    // Radio pin config
    P17_Quasi_Mode;       // Set TRIG_PIN (P17) as Quasi-bidirectional mode
    P30_Input_Mode;       // Set ECHO_PIN (P30) as Input mode
  
//    ADCM = ADC_CLOCK_P12_DIV_16; // Set ADC clock divider (adjust as needed)
//    ADCCON1 = (ADCCON1 & 0xCF) | 0x00; // Select channel 0 (P1.2) as ADC input
     
    TIMER0_MODE1_ENABLE;  // TMOD = 0x01;     // Timer0 mode 1 (16-bit timer)
    set_ET0;              // enable Timer0 interruptTH0 = 0;
    TH0 = 0; 
    TL0 = 0;              // set counter  
    set_EA;               // EA = 1; Enable global interrupt
    set_ET0;              // ET0 = 1;Enable Timer0 interrupt
    set_TR0;             // TR0 = 1;   Start Timer0
    
    RF_UZ_CLR;
    RF_TM_CLR;      
}
//-----------------------------------------------------------------------------
void delay_rf(unsigned int ms) {
   uint16_t dly_us;
   while(ms--) {
     dly_us=800   
     while(dly_us--) {
        _nop_();
     }
  }
}
//-----------------------------------------------------------------------------
void delay_us(unsigned int us) {
    while(us--) {
        _nop_();
        _nop_();
        _nop_();
        _nop_();
    }
}
//-----------------------------------------------------------------------------
void delay_ms(unsigned int ms) {
    while(ms--) {
        delay_us(1000);
    }
}
//========================= SONAR FUNCTIONS ===================================
/*
void Trigger_Pulse_10us(){
    RST_ECHO=0;   UZI_ECHO=0;   RST_ECHO=1;
    UZI_TRIGGER = 1;
    delay_us(10); //10
    UZI_TRIGGER = 0;
} */
//-----------------------------------------------------------------------------
void reset_Echolot(){
   TRIG_PIN_SET;           // Generate 10us pulse on TRIG_PIN
   delay_us(10);
   TRIG_PIN_CLR;
   trig_flag = 0;
        
   while (ECHO_PIN() == 0);  // Wait for the start of echo pulse
   start_time = TH0;
   start_time = (start_time << 8) | TL0;
        
   while (ECHO_PIN() == 1);  // Wait for the end of echo pulse
   end_time = TH0;
   end_time = (end_time << 8) | TL0;
        
   distance = ((end_time - start_time) * 34) / 200;  // Calculate distance in centimeters (speed of sound is 343 m/s)
}
//========================== RADIO FUNCTIONS =======================================
unsigned long  make1527Code(uint8_t packType){
  if (packType==0) return  0x123456; else return 0x123457;
}
//----------------------------------------------------------------------------------
void sendEV1527Code(uint8_t codeEvType) {
    unsigned char i = 0;
    unsigned long ev1527Code = make1527Code(codeEvType);  
    const unsigned char tmrEVset=4; //350
    LED_TX_PIN_SET;     
    //Send start bit
    RF_TX_PIN_SET;  //RF_TX_PIN = 1; Set the RF transmitter pin HIGH for a '1' bit
    delay_rf(tmrEVset);
    RF_TX_PIN_CLR;  //RF_TX_PIN = 0; 
    delay_rf(31*tmrEVset);     
    for (i = 0; i < 24; i++) {
        if (ev1527Code & (1UL << i)) {
            RF_TX_PIN_SET;  //RF_TX_PIN = 1; Set the RF transmitter pin HIGH for a '1' bit
            delay_rf(3*tmrEVset);
            RF_TX_PIN_CLR;  //RF_TX_PIN = 0; 
            delay_rf(tmrEVset);
        } else {
            RF_TX_PIN_SET;  //RF_TX_PIN = 1;  Set the RF transmitter pin HIGH for a '0' bit
            delay_rf(tmrEVset);
            RF_TX_PIN_CLR;  //RF_TX_PIN = 0;
            delay_rf(3*tmrEVset);
        }
    }
    LED_TX_PIN_CLR;
}
//======================== ONE WIRE FUNCTIONS ================================================
bit ds18b20_reset() {
    DS18B20_PIN_CLR;
    delay_us(480);
    DS18B20_PIN_SET;
    delay_us(60);
    presence = DS18B20_PIN();
    delay_us(420);
    return presence;
}
//---------------------------------------------------------------------------------------------
void ds18b20_write_byte(unsigned char dataTemperature) {
    unsigned char i;
    for (i = 0; i < 8; i++) {
        DS18B20_PIN_CLR;
        _nop_();
        if (dataTemperature & 0x01) DS18B20_PIN_SET; else DS18B20_PIN_CLR;
     //   DS18B20_PIN = dataTemperature & 0x01;
        delay_us(60);
        DS18B20_PIN_SET;
        dataTemperature >>= 1;
    }
}
//---------------------------------------------------------------------------------------------
unsigned char ds18b20_read_byte() {
    unsigned char i, dataTemperature = 0;
    for (i = 0; i < 8; i++) {
        DS18B20_PIN_CLR;
        _nop_();
        DS18B20_PIN_SET;
        delay_us(6);
        dataTemperature >>= 1;
        if (DS18B20_PIN()) {
            dataTemperature |= 0x80;
        }
        delay_us(60);
    }
    return dataTemperature;
}
//---------------------------------------------------------------------------------------------
float read_ds18b20_temperature() {
    unsigned char lsb, msb;
    float temperature;
    LED_LVL_PIN_SET;  
    ds18b20_reset();
    ds18b20_write_byte(0xCC); // Skip ROM command
    ds18b20_write_byte(0x44); // Convert temperature command
    delay_ms(750); // DS18B20 conversion time

    ds18b20_reset();
    ds18b20_write_byte(0xCC); // Skip ROM command
    ds18b20_write_byte(0xBE); // Read Scratchpad command
    lsb = ds18b20_read_byte();
    msb = ds18b20_read_byte();
    temperature = ((msb << 8) | lsb) * 0.0625;
    LED_LVL_PIN_CLR;   
    return temperature;
}

//======================= ADC Functions ======================================================
void READ_BANDGAP() { 
     UINT8 BandgapHigh,BandgapLow; 
     set_IAPEN; // Enable IAPEN 
     IAPAL = 0x0C; 
     IAPAH = 0x00; 
     IAPCN = 0x04; 
     set_IAPGO; // Trig set IAPGO 
     BandgapHigh = IAPFD; 
     IAPAL = 0x0d;
     IAPAH = 0x00; 
     IAPCN = 0x04; 
     set_IAPGO; // Trig set IAPGO 
     BandgapLow = IAPFD; 
     BandgapLow = BandgapLow&0x0F; 
     clr_IAPEN; // Disable IAPEN 
     Bandgap_Value = (BandgapHigh<<4)+BandgapLow; 
     Bandgap_Voltage = 3072/(0x1000/Bandgap_Value); 
}
//---------------------------------------------------------------------------------------------------------
void ADC_Bypass (void){  // The first three times convert should be bypass
   unsigned char ozc; 
   for (ozc=0;ozc<0x03;ozc++){
     clr_ADCF; 
     set_ADCS; 
     while(ADCF == 0); 
   } 
}
//----------------------------------------------------------------------------------------------------------
 
void get_Voltage(){
    double bgvalue;
     READ_BANDGAP();
     Enable_ADC_BandGap;
     ADC_Bypass();
     clr_ADCF;
     set_ADCS;
     while(ADCF == 0); 
     bgvalue = (ADCRH<<4) + ADCRL;
     VDD_Voltage = (0XFFF/bgvalue)*Bandgap_Voltage; 
 //    printf (“\n Bandgap voltage = %e”, Bandgap_Voltage); 
 //    printf (“\n VDD voltage = %e”, VDD_Voltage);
}
 

void READ_BANDGAP_CUSTOM(){
  UINT8 BandgapHigh,BandgapLow,BandgapMark;
  double Bandgap_Value,Bandgap_Voltage_Temp;
  set_IAPEN;
  IAPCN = READ_UID;
  IAPAL = 0x0d;
  IAPAH = 0x00;
  set_IAPGO;
  BandgapLow = IAPFD;
  BandgapMark = BandgapLow&0xF0;
  if (BandgapMark==0x80){
	BandgapLow = BandgapLow&0x0F;
	IAPAL = 0x0C;
	IAPAH = 0x00;
	set_IAPGO;
	BandgapHigh = IAPFD;
	Bandgap_Value = (BandgapHigh<<4)+BandgapLow;
	Bandgap_Voltage_Temp = Bandgap_Value*3/4;
	Bandgap_Voltage = Bandgap_Voltage_Temp - 33;			//the actually banggap voltage value is similar this value.
  }
  if (BandgapMark==0x00){
	BandgapLow = BandgapLow&0x0F;
	IAPAL = 0x0C;
	IAPAH = 0x00;
	set_IAPGO;
	BandgapHigh = IAPFD;
	Bandgap_Value = (BandgapHigh<<4)+BandgapLow;
	Bandgap_Voltage= Bandgap_Value*3/4;
  }
  if (BandgapMark==0x90){
	IAPAL = 0x0E;
	IAPAH = 0x00;
	set_IAPGO;
	BandgapHigh = IAPFD;
	IAPAL = 0x0F;
	IAPAH = 0x00;
	set_IAPGO;
	BandgapLow = IAPFD;
	BandgapLow = BandgapLow&0x0F;
	Bandgap_Value = (BandgapHigh<<4)+BandgapLow;
	Bandgap_Voltage= Bandgap_Value*3/4;
  }
  clr_IAPEN;
//			printf ("\n BG High = %bX",BandgapHigh); 
//			printf ("\n BG Low = %bX",BandgapLow); 
//			printf ("\n BG ROMMAP = %e",Bandgap_Voltage); 
}
//----------------------------------------------------------------------------
void get_MCU_Voltage(){
    double bgvalue;
    unsigned int i;
    Enable_ADC_BandGap;
    CKDIV = 0x02;	// IMPORTANT!! Modify system clock to 4MHz ,then add the ADC sampling clock base to add the sampling timing.
    for(i=0;i<5;i++){ // All following ADC detect timing is 200uS run under 4MHz.
  	  clr_ADCF;
	  set_ADCS;																
	  while(ADCF == 0);
	  ADCdataH[i] = ADCRH;
	  ADCdataL[i] = ADCRL;
	}		
	CKDIV = 0x00;	// After ADC sampling, modify system clock back to 16MHz to run next code.
	Disable_ADC;
     for(i=2;i<5;i++){// use the last 3 times data to make average 
		ADCsumH = ADCsumH + ADCdataH[i];
		ADCsumL = ADCsumL + ADCdataL[i];
	}				
	ADCavgH = ADCsumH/3;
	ADCavgL = ADCsumL/3;
	bgvalue = (ADCavgH<<4) + ADCavgL;
	VDD_Voltage = (0x1000/bgvalue)*Bandgap_Voltage;
	printf ("\n VDD voltage = %e", VDD_Voltage); 
	//Timer0_Delay1ms(500);
	ADCsumH = 0;
	ADCsumL = 0;    
}

//-----------------------------------------------------------------------------------------------------------
void readADCdata(unsigned char adcChannelNumber){
   unsigned int adc_value=0;  
   unsigned char ADCdataH=0;
   unsigned char ADCdataL=0;
   //READ ADC CHANNELS
   switch (adcChannelNumber){   
     case 0:  Enable_ADC_AIN0;  break;   
     case 1:  Enable_ADC_AIN1;  break;
     case 2:  Enable_ADC_AIN2;  break;
     case 3:  Enable_ADC_AIN3;  break;
     case 4:  Enable_ADC_AIN4;  break;   
     case 5:  Enable_ADC_AIN5;  break;  
     case 6:  Enable_ADC_AIN6;  break;
     case 7:  Enable_ADC_AIN7;  break;          
     default: Enable_ADC_AIN0;  break;
   }  
   clr_ADCF;
   set_ADCS;                                
   while(ADCF == 0);
   ADCdataH = ADCRH;
   ADCdataL = ADCRL;
   adc_value=ADCRH << 2 | ADCRL;
   if (adcChannelNumber==0){
      supplyVoltage = (float)adc_value * VDD_VOLTAGE / 1023.0; // Calculate supply voltage in millivolts
      batteryValue=(unsigned int)(1000*supplyVoltage);
   } else {
      temperatureValue=adc_value;
   }
   
   Disable_ADC;  
}
//=============================== LED INFORMATION ===========================================
void ledProc(){
   if (led_on==0){
      led_on=1;
      LED_STS_SET;  
   } else {
      led_on=0;
      LED_STS_CLR; 
   } 
}
//==================================== Power-down Mode Using WKT  ===========================
void sleepPoop(void){
/*
     // Enter power-down mode with wake-up by WKT (Wake-up Timer)
  set_PCON_PD; // Enter power-down mode with WKT wake-up
  set_EWKTEN; // Enable WKT (Wake-up Timer)
  clr_EA; // Disable global interrupts during power-down
  set_WKTF; // Clear the WKT interrupt flag
  clr_WKCON_WKTF; // Clear the WKT flag
  clr_WKCON_WKTR; // Start the WKT (Wake-up Timer)
*/     
}
//================================ UART Mode Usage ===========================================
void SendRsString(void){
  unsigned char poopLOG[22]={'L','=',0,0,0,0,';','T','=',0,0,0,0,';','V','=',0,0,0,0,0x0D,0x0A};  
  uint8_t i, i_high, i_low;
  uint16_t i_temp_h, i_temp_l;
  //Distance
  i_temp_h=distance/0x100;         i_temp_l=distance-0x100*i_temp_h;
  i_high=Get_High_Byte(i_temp_h);  i_low=Get_Low_Byte(i_temp_h);      
  poopLOG[2]=makeIntToHex(i_high); poopLOG[3]=makeIntToHex(i_low);  
  i_high=Get_High_Byte(i_temp_l);  i_low=Get_Low_Byte(i_temp_l);      
  poopLOG[4]=makeIntToHex(i_high); poopLOG[5]=makeIntToHex(i_low);     
  //Temperature
  i_temp_h=temperatureValue/0x100; i_temp_l=temperatureValue-0x100*i_temp_h;
  i_high=Get_High_Byte(i_temp_h);  i_low=Get_Low_Byte(i_temp_h);      
  poopLOG[9]=makeIntToHex(i_high); poopLOG[10]=makeIntToHex(i_low);  
  i_high=Get_High_Byte(i_temp_l);  i_low=Get_Low_Byte(i_temp_l);      
  poopLOG[11]=makeIntToHex(i_high); poopLOG[12]=makeIntToHex(i_low);
  //Battery
  i_temp_h=batteryValue/0x100;     i_temp_l=batteryValue-0x100*i_temp_h;
  i_high=Get_High_Byte(i_temp_h);  i_low=Get_Low_Byte(i_temp_h);      
  poopLOG[16]=makeIntToHex(i_high); poopLOG[17]=makeIntToHex(i_low);  
  i_high=Get_High_Byte(i_temp_l);  i_low=Get_Low_Byte(i_temp_l);      
  poopLOG[18]=makeIntToHex(i_high); poopLOG[19]=makeIntToHex(i_low);     
       
       
  for (i = 0; i < 22; i++){
     Send_Data_To_UART0(poopLOG[i]); // delay_us(10);
  }
  delay_ms(1);
}
/************************************************************************************************************
*    TIMER 0 interrupt subroutine
************************************************************************************************************/
void Timer0_ISR (void) interrupt 1  //interrupt address is 0x000B
{
    TH0 = 0;
    TL0 = 0;
    trig_flag = 1;
    if (cnt_sec<50) cnt_sec++; 
}
/************************************************************************************************************
*    Main function 
************************************************************************************************************/
void main (void){
    uint8_t i=0;
    Init_MCU();
    READ_BANDGAP_CUSTOM();
   //READ_BANDGAP();
    InitialUART0_Timer1(9600); 
    while(1){
      if (cnt_sec>=50){
         cnt_sec=0;   
         ledProc();  
        // reset_Echolot();
        // readADCdata(0);
        // readADCdata(1);
         get_MCU_Voltage(); 
         outTemperature = read_ds18b20_temperature();  
         for (i=0; i<8; i++) sendEV1527Code(1);
//         if (test_key_IN()==0)
          SendRsString();
      }
      delay_ms(10);
    };
}

