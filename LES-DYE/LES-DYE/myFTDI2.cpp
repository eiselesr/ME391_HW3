#include "myFTDI2.h"
#include "ftd2xx.h"
#include <iostream>
#include <string>

//#include <thread>

using namespace std;

myFTDI2::~myFTDI2(void)
{
}

//int getPulseWidths(int index){
//
//	int val = newBuffer[index];
//	return val;
//}

void myFTDI2::WritePWM(char keyVal){

	
	switch(keyVal){
	case 'a': 
		keyVal++;
	case 'b':
		keyVal++;
	case 'k':
		keyVal++;
	case 's':
		keyVal++;
		break;
	}

	TxBuffer[0] = keyVal;


	ftStatus = FT_Write(ftHandle, TxBuffer, sizeof(TxBuffer), &BytesWritten);

	if(ftStatus==FT_OK){
		cout << "write successful" << endl;
	}
}

void myFTDI2::Write(char keyVal){

	TxBuffer[0] = keyVal;

	ftStatus = FT_Write(ftHandle, TxBuffer, sizeof(TxBuffer), &BytesWritten);

	if(ftStatus==FT_OK){
		cout << "write successful" << endl;
	}
}


//void myFTDI2::Write(char * pwmBuffer, int size){
//
//
//	ftStatus = FT_Write(ftHandle, pwmBuffer, sizeof(pwmBuffer), &BytesWritten);
//
//	if(ftStatus==FT_OK){
//		cout << "write successful" << endl;
//	}
//}

//void myFTDI2::Write(int input){
//
//	TxBuffer[bufferIndex] = input;
//
//	ftStatus = FT_Write(ftHandle, TxBuffer, sizeof(TxBuffer), &BytesWritten);
//
//	if(ftStatus==FT_OK){
//		cout << "write successful" << endl;
//	}
//}


void myFTDI2::Close(){
	ftStatus = FT_Close(ftHandle);

}
void myFTDI2::FT_Initialize(){

	ftStatus = FT_Open(0, &ftHandle);

	if(ftStatus!=FT_OK){
		cout << "failed to open device"<<endl;	return;}

	ftStatus = FT_SetBaudRate(ftHandle, 28800);

	if(ftStatus!=FT_OK){
		cout << "failed to set Baud rate"<<endl;	}

	ftStatus = FT_SetDataCharacteristics(ftHandle,FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
	if(ftStatus!=FT_OK){
		cout << "failed to set data characteristics";	}


	//ftStatus = FT_SetTimeouts(ftHandle, 5000, 1000);
}
//void myFTDI2::ReadCoeffs(char * RxBuffer){
//
//	  a0 = (RxBuffer[0]<<8) + RxBuffer[1];
//			   a0dec=(double)a0/8;
//
//			  pressureCoeffs[0] = a0dec;
//
//  b1 = (RxBuffer[2]<<8) + RxBuffer[3];
// b1dec=(double)b1/8192;
//
//  pressureCoeffs[1] = b1dec;
//
//  b2 = (RxBuffer[4]<<8) + RxBuffer[5];
//   b2dec = (double)b2/16384;
//
//  pressureCoeffs[2] = b2dec;
//  
//
//  c12 = (RxBuffer[6]<<8) + RxBuffer[7];
//  c12dec = (double)c12/16777216; 
//
//  pressureCoeffs[3] = c12dec;
//
//}

void myFTDI2::Read(unsigned char *RxBuffer){

	int Ready = 0;

	while(Ready == 0)//waiting until UART/serial msg ready
	{
		FT_GetStatus(ftHandle, &RxBytes, &TxBytes, &EventDWord);
		if(RxBytes>104)
			{Ready=1;}
		else 
			return;
	}


	ftStatus =FT_Read(ftHandle, RxBuffer, RxBytes, &BytesReceived);

	if(ftStatus==FT_OK)
	{
		//if(BytesReceived == RxBytes){ 
			//cout << "read Pressure successful" << endl;}
	}
	else
	{	
		//FT_READ failed
		cout << "did not receive all bytes available" << endl;
	}

}

void myFTDI2::ReadBlock(unsigned char *RxBuffer){

	int Ready = 0;

	while(Ready == 0)//waiting until UART/serial msg ready
	{
		FT_GetStatus(ftHandle, &RxBytes, &TxBytes, &EventDWord);
		if(RxBytes>104)
			{Ready=1;}
	}


	ftStatus =FT_Read(ftHandle, RxBuffer, RxBytes, &BytesReceived);

	if(ftStatus==FT_OK)
	{
		if(BytesReceived == RxBytes){ 
			cout << "read PWM successful" << endl;}
	}
	else
	{	
		//FT_READ failed
		cout << "did not receive all bytes available" << endl;
	}

}



///TRANSFORM RECEIVED DATA
/*for (int i=0;i<BytesReceived;i++) {
newBuffer[i]=5*intRxBuffer[i];
cout << newBuffer[i];*/
//Write(newBuffer[i]);




//
//
//DWORD myFTDI2::ReadGYRO(){
//
//	FT_GetStatus(ftHandle,&RxBytes,&TxBytes,&EventDWord); 
//	if (RxBytes > 0) 	
//	{ 
//		ftStatus = FT_Read(ftHandle,RxBuffer,RxBytes,&BytesReceived); 
//		if (ftStatus == FT_OK) 
//		{ 
//			// FT_Read OK 
//			return RxBytes;
//		} 
//		else
//		{ 
//			// FT_Read Failed 
//			cout << "did not receive all bytes available" << endl;
//		}
//	}
//
//}



