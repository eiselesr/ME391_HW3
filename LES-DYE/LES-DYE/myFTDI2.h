#pragma once
#include "ftd2xx.h"
#include "myFTDI2.h"
#include <iostream>
#include <string>
class myFTDI2
{

	
public:
	
	DWORD BytesWritten;
	DWORD EventDWord;
	DWORD RxBytes;
	DWORD TxBytes;
	DWORD BytesReceived;

	//DWORD RxBuffer[35];
//	unsigned char RxBuffer[35];
	//int newBuffer[35];

	FT_HANDLE ftHandle;
	FT_STATUS ftStatus;

	unsigned char TxBuffer[1];
//	DWORD TxBuffer[1];

	int numDevices;
	int bufferIndex;
	int inputSize;
	
	int numBytesWritten;
	int numBytes;

public:
	
	myFTDI2(void):ftHandle(), bufferIndex(0), inputSize(1){};
	
	~myFTDI2(void);

	void FT_Initialize();
	void Write(char);
	void Write(int);
	void WritePWM(char);
	void Write(char * pwmBuffer, int size);
//void Read();
	void Read(unsigned char *RxBuffer);
	void ReadBlock(unsigned char *RxBuffer);
	void Close();
	void FT_Stall();
	int getPulseWidths();
	DWORD ReadGYRO();

//};
};

