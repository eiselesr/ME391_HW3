
#include "ftd2xx.h"
#include "myFTDI2.h"
#include <iostream>
#include <string>
//#include <stdlib.h>


using namespace std;

int main(){

	myFTDI2 ftdi;
	ftdi.FT_Initialize();
	
	char keyVal = 0;

	///----------
	// MAIN LOOP
	//----------
	
	while(true){
	
		cout << "Enter Input" << endl;
		cin >> keyVal;
		
		//THIS IS HOW WE CAN PACKAGE THE INFO BEFORE HAND
		//ie take advantage of faster processer to reduce load on microcontroller -- maybe will speed up 

		switch(keyVal){
			case 'a':
			//toggle LED 
				ftdi.Write(keyVal);
				break;
			
			case 's':
			//toggle LED
				ftdi.Write(keyVal);
				break;

			case 'w':
				ftdi.Write(keyVal);
				//sleep(30);
				ftdi.Read();
				break;

			case 'i':
				cout << "Increase blinking frequency of LED1" << endl;
				ftdi.Write(keyVal);
				break;

			case 'd':
				cout << "Decrease blinking frequency of LED1" << endl;
				ftdi.Write(keyVal);
				break;

			case 'x':
				ftdi.Write(keyVal);
				cout<< "Timer Stopped" << endl;
				//then read in time
				break;
		
			case 'c':
				ftdi.Write(keyVal);
				cout << "Timer Started" << endl;
				break;

			default:
				cout << "Please enter a valid option" << endl;
				break;
		}
	

	}//end of main loop
	
	ftdi.Close();
	
}
