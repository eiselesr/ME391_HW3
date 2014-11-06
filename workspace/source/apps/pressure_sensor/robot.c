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
#define INIT_CONTDATA_CMD 2
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
#define START 0x24

enum{
  
  INITIAL,
  SENDING,
  WAITING,
  READY
    
}states;


/***********************************************************************************
* LOCAL VARIABLES
*/
unsigned char spiTxBuffer[APP_PAYLOAD_LENGTH];//10
unsigned char spiRxBuffer[APP_PAYLOAD_LENGTH];  //20
static uint8 pTxData[APP_PAYLOAD_LENGTH];
static uint8 pRxData[APP_PAYLOAD_LENGTH];
static uint8 coeffBuf[APP_PAYLOAD_LENGTH];

uint8 status;

static basicRfCfg_t basicRfConfig;

int a0;
int b1;
int b2;
int c12;
double a0dec;
double b1dec;
double b2dec;
double c12dec;
int value; // Value in which ADC conversion is stored.
int counter = 0;
int spiPkg;
int adcPkg;
int dongleActive = 0;
int state = 0;
int sendPacket = 0;


/***********************************************************************************
* LOCAL FUNCTIONS
*/

static void appADC();
static void basicRfSetUp();
static void configurePressure();
static void readPressure();
uint8 sendPressure();

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
  P0SEL |= 0x10; //[---1 1---] (CH 1 and CH 2 to peripheral)
  P0DIR |= 0x10; //[---1 1---] (CH1 and CH2 to output)
  P0SEL |= 0x08;
  P0DIR |= 0x08;
  //------------------------
  // CONFIGURE 800Hz Timer
  //------------------------
  //Set divisor to 32 => clock speed is 1 MHz, Sets timer to up-down mode (Pg 114) (Family)
  T1CTL |= 0x0B; //[---- 1-11]
  T1CTL &= ~0x04; //[---- -0--]
  //T1CC0 = 625. Need 800hz trigger frq => (1MHZ timer/count)=800 => count = 1250.
  //In up-down mode, the timer counts up to a number, then counts down to zero to trigger interrupt.
  //We want the high number to be 1250/2 => 650. 
  T1CC0L = 0x71;
  T1CC0H = 0x02; 
  //------------------------
  // Set-up PWM
  //------------------------
  //Test PWM.  CH1 MAX, CH2 MIN
  T1CC1H = 0x00;
  T1CC1L = 0x64;
  T1CC2H = 0x00;
  T1CC2L = 0x64;
  //Set each channel to set-up, clear-down and compare mode
  T1CCTL1 |= 0x1C;  //[---1 11--]
  T1CCTL1 &= ~0x63;  //[--0- --00]
  T1CCTL2 |= 0x1C;  //[---1 11--]
  T1CCTL2 &= ~0x63;  //[-00- --00]
  //  IEN0 |= 0x80; //[1--- ----] Allow interrupts
  //  IEN1 |= 0x02; //[---- --1-] Enable Timer 1 interrupt
  
  //  //To Test Timer (5Hz)
  P0SEL &= ~0x01;
  P0DIR |= 0x01;
  
  
  //-------------------------------------------------------------------------
  // RF STUFF
  //-------------------------------------------------------------------------
  basicRfSetUp();
  
  // Initalise board peripherals
  // halBoardInit();  
  
  if(halRfInit()==FAILED) {
    HAL_ASSERT(FALSE);
  }
  
  halMcuWaitMs(350);  //Why are we waiting here?
  
  //begins waiting for msg
  configurePressure();
  //sendPressure Coefficients
  //status = basicRfSendPacket(DONGLE_ADDR, spiRxBuffer, 105);
  //Wait for Handshake
  
  
  while(TRUE){
    // 
    
    switch(state)
    {
      //uninitialized
    case 0: 
      basicRfReceiveOn();
      while(!basicRfPacketIsReady());
      if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0) {
        if(pRxData[0] == INIT_COMM_CMD) {        
//          //turn receive off -- send ack to dongle
//          basicRfReceiveOff();
//          pTxData[0] = ACK;  //Read as 0x17 (23)
//          pTxData[1] = 0x14; //These mean nothing... Just cool that it works =)
//          pTxData[2]=0x15;
//          basicRfSendPacket(DONGLE_ADDR, pTxData, 105);
//          basicRfReceiveOn();
          state = 1; //Ready State
        }
      }
      break;
    case 1:
      while(!basicRfPacketIsReady());
      
      if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0) {
        if(pRxData[0] == INIT_CONTDATA_CMD){
          state = 2;
          configurePressure();// populates spiTxBuffer
          basicRfSendPacket(DONGLE_ADDR, coeffBuf, 105);
          //IEN1 |= 0x02; //[---- --1-] Enable Timer 1 interrupt <- Probably won't use
        }
        //if UP
        //if DOWN
        //if LEFT
        //if RIGHT
      }      
      break;
      
    case 2:
      while(!basicRfPacketIsReady() && (sendPacket==0));
      if(basicRfPacketIsReady())
      {
        if(basicRfReceive(pRxData, APP_PAYLOAD_LENGTH, NULL)>0) {
          //if UP
          //if DOWN
          //if LEFT
          //if RIGHT
        }
      }
      if(sendPacket ==1)
      {
        readPressure();
        if(sendPressure()==FAILED){
          state = 1;//Send pressure, go back to ready state if fail to send
          //IEN1 &= ~0x02; //[---- --1-] Disable Timer 1 interrupt <- Probably won't use
        }
        sendPacket = 0;
      }
      break; 
    }
    
    //
    
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
  coeffBuf[0] = 'C'; //Coefficient header
  coeffBuf[1] = 0; //Packet Number
  coeffBuf[2] = 1; //Packet Number
  for(int i=0; i<9; i++)
  {
    
    U1TX_BYTE = 0;
    U1DBUF = spiTxBuffer[i];
    while(!U1TX_BYTE);    
    
    U1TX_BYTE = 0; 
    U1DBUF = 0x00;
    while (!U1TX_BYTE);    
    coeffBuf[i+3] = U1DBUF;    
  }  
  CS = 1;
  
  //a0 = (spiRxBuffer[0]<<8) + spiRxBuffer[1];
  //b1 = (spiRxBuffer[2]<<8) + spiRxBuffer[3];
  //b2 = (spiRxBuffer[4]<<8) + spiRxBuffer[5];
  //c12 = (spiRxBuffer[6]<<8) + spiRxBuffer[7];
  //a0dec=(double)a0/8;
  //b1dec=(double)b1/8192;
  //b2dec = (double)b2/16384;
  //c12dec = (double)c12/16777216;  
}

//-----------------------------------------
// READ PRESSURE AND TEMPERATURE
//-----------------------------------------
static void readPressure()
{
  CS = 1;
  //Set VDD high, GND low, SDN low
  P1_1=1; //VDD
  P1_2=0; //GND
  P1_3=1; //SDN
  
  
  CS=0;
  //Start Measuring Pressure
  U1TX_BYTE = 0;
  U1DBUF = START;
  while(!U1TX_BYTE);    
  
  U1TX_BYTE = 0; 
  U1DBUF = 0x00;
  while (!U1TX_BYTE);    
  CS=1;
  
  halMcuWaitMs(3);//Wait for conversion
  
  //Read coefficients
  spiTxBuffer[0] = Padc_MSB;
  spiTxBuffer[1] = Padc_LSB;
  spiTxBuffer[2] = Tadc_MSB;
  spiTxBuffer[3] = Tadc_LSB;
  spiTxBuffer[4] = 0x00;
  
  CS = 0;
  for(int i=0; i<5; i++)
  {
    
    U1TX_BYTE = 0;
    U1DBUF = spiTxBuffer[i];
    while(!U1TX_BYTE);    
    
    U1TX_BYTE = 0; 
    U1DBUF = 0x00;
    while (!U1TX_BYTE);    
    spiRxBuffer[i] = U1DBUF;
    
  }
  CS = 1;
  
}
//-----------------------------------------
// Send PRESSURE AND TEMPERATURE
//-----------------------------------------
uint8 sendPressure()
{  
  uint8 status = FAILED;// FAILED = 1
  int i = 0;
  while(i<5)
  {
    pTxData[0] = 'P'; //P => Pressure data
    pTxData[1] = spiPkg/8;
    pTxData[2] = spiPkg%8;
    pTxData[3] = spiTxBuffer[0];
    pTxData[4] = spiTxBuffer[1];
    pTxData[5] = spiTxBuffer[2];
    pTxData[6] = spiTxBuffer[3];
    status = basicRfSendPacket(DONGLE_ADDR, pTxData, APP_PAYLOAD_LENGTH);
    spiPkg++;
    i++;
    if(status == SUCCESS){
      break;
    }
  }
  return status;  
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
    sendPacket = 1;
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