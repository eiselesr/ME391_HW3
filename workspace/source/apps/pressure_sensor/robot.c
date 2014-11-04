/***********************************************************************************

      WIRELESS ROBOTIC SYSTEM FILE --- LOAD ONTO CC2530 RFB 743

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include <hal_lcd.h>
#include <hal_led.h>
#include <hal_joystick.h>
#include <hal_assert.h>
#include <hal_board.h>
#include <hal_int.h>
#include "hal_mcu.h"
#include "hal_button.h"
#include "hal_rf.h"
#include "util_lcd.h"
#include "basic_rf.h"


/***********************************************************************************
* CONSTANTS
*/
// Application parameters
#define RF_CHANNEL                25      // 2.4 GHz RF channel Can choose 11-26
// Hakan: 17

// BasicRF address definitions
#define PAN_ID                0x2007
#define SWITCH_ADDR           0x2520
#define LIGHT_ADDR            0xBEEF

//#define ROBOT_ADDR             0xFEED
//#define DONGLE_ADDR             0xBABE

#define ROBOT_ADDR      SWITCH_ADDR
#define DONGLE_ADDR     LIGHT_ADDR



#define APP_PAYLOAD_LENGTH        105
#define LIGHT_TOGGLE_CMD          0

#define INIT_COMM_CMD 1
#define ACK 23

// Application states
#define IDLE                      0
#define SEND_CMD                  1

// ADC MASKS
#define ADC_AIN5            0x05     // single ended P0_5
#define ADC_AIN6            0x06     // single ended P0_6
#define ADC_AIN7            0x07     // single ended P0_7
#define ADC_VDD_3           0x0F     // (vdd/3)
#define ADC_10_BIT          0x20     // 256 decimation rate 


//STATES AND OTHER DATA DEFINES

#define ADC 0
#define SPI 1
#define CS P1_4 // CHIP SELECT FOR PRESSURE CENSOR

#define a0_MSB 0x88 //Coefficient addresses (Read built in)
#define a0_LSB 0x8A
#define b1_MSB 0x8C
#define b1_LSB 0x8E
#define b2_MSB 0x90
#define b2_LSB 0x92
#define c12_MSB 0x94
#define c12_LSB 0x96

#define Padc_MSB 0x80  //Sensor addresses (Read built in)
#define Padc_LSB 0x82
#define Tadc_MSB 0x84
#define Tadc_LSB 0x86


/***********************************************************************************
* LOCAL VARIABLES
*/
unsigned char spiTxBuffer[10];
unsigned char spiRxBuffer[20];  
static uint8 pTxData[APP_PAYLOAD_LENGTH];
static uint8 pRxData[APP_PAYLOAD_LENGTH];

static basicRfCfg_t basicRfConfig;

int a0;
int b1;
int b2;
int c12;
int value; // Value in which ADC conversion is stored.
int counter = 0;


/***********************************************************************************
* LOCAL FUNCTIONS
*/

static void appADC();
static void basicRfSetUp();
static void configurePressure();
static void readPressure();
_Pragma("vector=0x4B") __near_func __interrupt void LIGHTUP(void);


void main(void)
{
  CLKCONCMD &= 0x00;//Set system clock to 32MHZ => set bit 6 to 1 => [-1-- ----]
  while(CLKCONSTA&0x40);//waiting for clock to become stable
  //-------------------------------------------------------------------------
  // TEST
  //-------------------------------------------------------------------------
  P1SEL &= ~ 0x01;  
  P1DIR |= 0x01;
  P1_0=1;
  //-------------------------------------------------------------------------
  // CONFIGURE ADC 5, 6, and 7
  //-------------------------------------------------------------------------
  P0SEL |=  0xE0;//SET P0_7, 6, 5 to peripheral [111-  ----]
  P0DIR &= ~0xE0;//SET P0_7, 6, 5 to input [000-  ----]
  APCFG |= 0xE0;// Analog Peripheral I/O enable [111- ----] pg.85, 134  
  
//--------------------------------------------------------------------------
  // CONFIGURE SPI (USART 1 ALT 2)
  //--------------------------------------------------------------------------
  PERCFG |= 0x02; //SET USART 1 I/O location TO ALTERNATIVE 2 => set bit 1 to 1: [---- --1-] - Family pg. 85
  U1CSR &= ~0xA0; //Set USART 1 TO SPI MODE and Set USART 1 SPI TO MASTER MODE[0-0- ----]
  
  //-----------------------------------------------------
  // CONFIGURE SPI PERIPHERALS
  //-----------------------------------------------------  
  P1SEL |=0xE0; //set P1_5, P1_6, P1_7 are peripherals [111- ----]
  P1SEL &= ~0x1F;  //P1_4(CS),3(SDN),2(GROUND),1(VDD) are GP I/O (SSN) [---0 0000]
  P1DIR = 0x7F; // SET MO, C, CS, VDD, GND, SDN to output [0111 1111]
  //P1DIR &=~0x80; //SET MI to input[0--- ----]
  //-----------------------------------------------------
  // CONFIGURE SPI BAUD RATE
  //-----------------------------------------------------   
  U1BAUD = 0x3B;//BAUD_M= 59  //0x00;// BAUD_M = 0
  U1GCR |= 0x06;//BAUD_E = 6  //0x11;// BAUD_E = 17
  //-----------------------------------------------------
  // CONFIGURE SPI POLARITY, DATA TRANSFER, AND BIT ORDER
  //-----------------------------------------------------   
  //CPHA = 0 means:
  //Data is output on MOSI when SCK goes from CPOL inverted to CPOL, and data input
  //is sampled on MISO when SCK goes from CPOL to CPOL inverted.
  //CPOL = 0 => Clock polarity is negative
  U1GCR &= ~0xC0; //U1GCR.CPOL = U1GCR.CPHA = 0 [00-- ----] - familiy pg. 163
  U1GCR |=0x20;// U1GCR.ORDER = 1=> MSB first  [--1- ----]
  
  //-----------------------------------------------------
  // CONFIGURE TIMER 1
  //-----------------------------------------------------
  PERCFG &= ~0x40; //[-0-- ----]  (TIMER 1 ALT 1)
  P0SEL |= 0x18; //[---1 1---] (CH 1 and CH 2 to peripheral)
  P1DIR |= 0x18; //[---1 1---] (CH1 and CH2 to output)
  //------------------------
  // CONFIGURE 800Hz Timer
  //------------------------
  //Set divisor to 32 => clock speed is 1 MHz, Sets timer to up-down mode (Pg 114) (Family)
  T1CTL |= 0x0B; //[---- 1-11]
  T1CTL &= ~0x04; //[---- -0--]
  //T1CC0 = 625. Need 800hz trigger frq => (1MHZ timer/count)=800 => count = 1250.
  //In up-down mode, the timer counts up to a number, then counts down to zero to trigger interrupt.
  //We want the high number to be 1250/2 => 650.
  T1CC0H = 0x02;  
  T1CC0L = 0x71;
  //------------------------
  // Set-up PWM
  //------------------------
  //Set each channel to set-up, clear-down and compare mode
  T1CCTL1 |= 0x1C;  //[---1 11--]
  T1CCTL1 &= ~0x23;  //[--0- --00]
  T1CCTL2 |= 0x1C;  //[---1 11--]
  T1CCTL2 &= ~0x23;  //[--0- --00]
  IEN0 |= 0x80; //[1--- ----] Allow interrupts
  IEN1 |= 0x02; //[---- --1-] Enable Timer 1 interrupt
    

  
  

  //configurePressure();
  //-------------------------------------------------------------------------
  // RF STUFF
  //-------------------------------------------------------------------------
  basicRfSetUp();
  
  // Initalise board peripherals
  halBoardInit();
  halJoystickInit();
  
  
  if(halRfInit()==FAILED) {
    HAL_ASSERT(FALSE);
  }
  
  halMcuWaitMs(350);  
  
  //begins waiting for msg
  
  while(TRUE){
    basicRfReceiveOn();
    while(!basicRfPacketIsReady());
    
    if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0) {
      if(pRxData[0] == INIT_COMM_CMD) {
        
        //turn receive off -- send ack to dongle
        basicRfReceiveOff();
        pTxData[0] = ACK;  //Read as 0x17 (23)
        pTxData[1] = 0x14; //These mean nothing... Just cool that it works =)
        pTxData[2]=0x15;
        basicRfSendPacket(DONGLE_ADDR, pTxData, 105);
      }
    }
  }
}
  // appADC();
//----------------------------------
//
//    SEND ADC VALUES 
//
//---------------------------------
static void appADC()
{
  //I think we should move the Receive out of this function.  This function should be called after the receive command has been received.
  basicRfReceiveOn();
  
  //  //----------------------------
//  // Start conversion
//  //----------------------------
//  ADCCON3 = ADC_10_BIT | ADC_AIN5; //Use internal voltage reference at 256 decimation rate(10bit)[0010 ----]
//  while(!(ADCCON1 & 0x80)); //WAIT UNTIL END OF CONVERSION
//  //----------------------------
//  // READ conversion
//  //----------------------------
//  value = ADCL; //ADCL is the ADC low bits
//  value |= (((unsigned int)ADCH) << 8); //ADCH is ADC high bits
//  //wait after sending this value 
  
  while (TRUE) {
    //wait until this is true 
    while(!basicRfPacketIsReady());
    
    if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0) {
      //turn receive off -- send adc info to dongle
        basicRfReceiveOff();
        pTxData[0] = value;
        basicRfSendPacket(DONGLE_ADDR, pTxData, 105);
        
      }
    }
}
 //-----------------------------------------
// CONFIGURE AND INITIALIZE BASIC RF 
//
//--------------------------------------------

static void basicRfSetUp()
{
  // Config basicRF
  basicRfConfig.panId = PAN_ID;
  basicRfConfig.channel = RF_CHANNEL;
  basicRfConfig.ackRequest = TRUE;
  
  
#ifdef SECURITY_CCM
  basicRfConfig.securityKey = key;
#endif
  
  basicRfConfig.myAddr = ROBOT_ADDR;
  
  // Initialize BasicRF
  
  if(basicRfInit(&basicRfConfig)==FAILED) {
    HAL_ASSERT(FALSE);
  }
}

//-----------------------------------------
// CONFIGURE PRESSURE SENSOR AND OBTAIN COEFFICIENTS
//-----------------------------------------
static void configurePressure()
{
  CS = 1;
  //Set VDD high, GND low, SDN low
  P1_1=1; //VDD
  P1_2=0; //GND
  P1_3=1; //SDN
  
  //Read coefficients
  spiTxBuffer[0] = a0_MSB;
  spiTxBuffer[1] = a0_LSB;
  spiTxBuffer[2] = b1_MSB;
  spiTxBuffer[3] = b1_LSB;
  spiTxBuffer[4] = b2_MSB;
  spiTxBuffer[5] = b2_LSB;
  spiTxBuffer[6] = c12_MSB;
  spiTxBuffer[7] = c12_LSB;
  spiTxBuffer[8] = 0x00;
  
  CS = 0;
  
  //for(int i=0; i<9; i++)
  int i = 0;
  while(i<9)
  {
    
    U1TX_BYTE = 0;
    U1DBUF = spiTxBuffer[i];//(a0_MSB + 2*i);
    while(!U1TX_BYTE);    
    
    spiRxBuffer[i*2] = U1DBUF;
    //spiRxBuffer[i] = U1DBUF;
    
    U1TX_BYTE = 0; //NOT SURE IF THIS IS NEEDED OR NOT
    U1DBUF = 0x00;
    while (!U1TX_BYTE);    
    spiRxBuffer[i*2+1] = U1DBUF;
    
    i++;
    //i = i%9;
    
    
  }
  CS = 1;
  
  a0 = (spiRxBuffer[0]<<8) + spiRxBuffer[1];
  b1 = (spiRxBuffer[2]<<8) + spiRxBuffer[3];
  b2 = (spiRxBuffer[4]<<8) + spiRxBuffer[5];
  c12 = (spiRxBuffer[6]<<8) + spiRxBuffer[7];
}

//-----------------------------------------
// TIMER INTERRUPT
//-----------------------------------------
_Pragma("vector=0x4B") __near_func __interrupt void LIGHTUP(void)
{
  //P1_0 = (P1_0 == 0);
  counter = counter+1;
  
  if(counter>=160){
    P0_0 = !P0_0; //LED1
    counter=0;
  }
}

//----------------------------------------
//
//
//      END OF WORKING CODE 
//
//--------------------------------------

/***********************************************************************************
* @fn          appSwitch
*
* @brief       Application code for switch application. Puts MCU in
*              endless loop to wait for commands from from switch
*
* @param       basicRfConfig - file scope variable. Basic RF configuration data
*              pTxData - file scope variable. Pointer to buffer for TX
*              payload
*              appState - file scope variable. Holds application state
*
* @return      none
*/
//static void appSwitch() -- SEND 
//{
//  halLcdWriteLine(HAL_LCD_LINE_1, "Switch");
//  halLcdWriteLine(HAL_LCD_LINE_2, "Joystick Push");
//  halLcdWriteLine(HAL_LCD_LINE_3, "Send Command");
//#ifdef ASSY_EXP4618_CC2420
//  halLcdClearLine(1);
//  halLcdWriteSymbol(HAL_LCD_SYMBOL_TX, 1);
//#endif
//  
//  pTxData[0] = LIGHT_TOGGLE_CMD;
//  
//  // Initialize BasicRF
//  basicRfConfig.myAddr = SWITCH_ADDR;
//  if(basicRfInit(&basicRfConfig)==FAILED) {
//    HAL_ASSERT(FALSE);
//  }
//  
//  // Keep Receiver off when not needed to save power
//  basicRfReceiveOff();
//  
//  // Main loop
//  while (TRUE) {
//    if( halJoystickPushed() ) {
//      
//      basicRfSendPacket(LIGHT_ADDR, pTxData, APP_PAYLOAD_LENGTH);
//      
//      // Put MCU to sleep. It will wake up on joystick interrupt
//      halIntOff();
//      halMcuSetLowPowerMode(HAL_MCU_LPM_3); // Will turn on global
//      // interrupt enable
//      halIntOn();
//      
//    }
//  }
//}


/***********************************************************************************
* @fn          appLight
*
* @brief       Application code for light application. Puts MCU in endless
*              loop waiting for user input from joystick.
*
* @param       basicRfConfig - file scope variable. Basic RF configuration data
*              pRxData - file scope variable. Pointer to buffer for RX data
*
* @return      none
*/
//static void appLight() --- RECEIVE 
//{
//  // Initialize BasicRF
//  basicRfConfig.myAddr = LIGHT_ADDR;
//  if(basicRfInit(&basicRfConfig)==FAILED) {
//    HAL_ASSERT(FALSE);
//  }
//  basicRfReceiveOn();
//  
//  // Main loop
//  while (TRUE) {
//    while(!basicRfPacketIsReady());
//    
//    if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0) {
//      if(pRxData[0] == LIGHT_TOGGLE_CMD) {
//        halLedToggle(1);
//      }
//    }
//  }
//}



/***********************************************************************************
* Interrupts
*/

/***********************************************************************************
_Pragma("vector=0x4B") __near_func __interrupt void LIGHTUP(void);
_Pragma("vector=0x4B") __near_func __interrupt void LIGHTUP(void)
{
//P1_0 = (P1_0 == 0);
counter = counter+1;

if(counter>=freqcount){
P1_0 = (P1_0 == 0); //LED1
counter=0;
  }

if(timerrunning)//timer has started
{
timercount+=1;//Increment timercount by 1 (representing 1ms)
  }
}
*/




/***********************************************************************************
* @fn          main
*
* @brief       This is the main entry of the "Light Switch" application.
*              After the application modes are chosen the switch can
*              send toggle commands to a light device.
*
* @param       basicRfConfig - file scope variable. Basic RF configuration
*              data
*              appState - file scope variable. Holds application state
*
* @return      none
*/




//  basicRfReceiveOn();
//  
//  // Main loop
//  while (TRUE) {
//    while(!basicRfPacketIsReady());//we want WRS to sit her until mc1 initializes com
//    
//    if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0) {
//      if(pRxData[0] == LIGHT_TOGGLE_CMD) {
//        halLedToggle(1);
//      }
//    }
//  }