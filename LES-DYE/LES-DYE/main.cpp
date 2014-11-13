//#include <stdlib.h>
#include "ftd2xx.h"
#include "myFTDI2.h"
#include <fstream>
#include <iostream>
#include <string>
#include "conio.h"
//#define _USE_MATH_DEFINES
#include <math.h>

#define M_PI       3.14159265358979323846
#define UP_ARROW    72
#define LEFT_ARROW  75
#define DOWN_ARROW  80
#define RIGHT_ARROW 77

using namespace std;
static double pressureCoeffs[4];
static short rawPressCoeffs[8];
static short analog1[25];
static short analog2[25];
static short analog3[25];

int a0;
int b1;
int b2;
int c12;
int Padc_MSB;
int Padc_LSB;
int Tadc_MSB;
int Tadc_LSB;
double Padc;
double Tadc;
double a0dec;
double b1dec;
double b2dec;
double c12dec;
double pcomp;
double pressure;
char key; 

ofstream myfile;

int main(){
	char keyVal;
	short PWM_1;
	short PWM_2;
	unsigned char RxBuffer[105];
	//unsigned char TxBuffer[105];
	myFTDI2 ftdi;
	ftdi.FT_Initialize();

	while(!kbhit());

	cout << "Enter input" << endl;
	cin >> keyVal;

	//Initialize WRS communication
	ftdi.Write('a');

	ftdi.ReadBlock(RxBuffer);
	
	ftdi.Write('Z');//acknowledge


	myfile.open("C:\\Users\\youngem1\\Documents\\Visual Studio 2010\\Projects\\LES-DYE\\ADCData.txt");
	
	//myfile.open("C:\\Users\\RASL\\Desktop\\ADCData.txt");
	myfile.open("C:\\Users\\youngem1\\Documents\\Visual Studio 2010\\Projects\\LES-DYE\\PressureData.txt");
	//myfile.open("C:\\Users\\RASL\\Desktop\\PressureData.txt");


	myfile.open("C:\\Users\\youngem1\\Documents\\Visual Studio 2010\\Projects\\LES-DYE\\PressureCoefficients.txt", ios::app);
	//myfile.open("C:\\Users\\RASL\\Desktop\\PressureCoefficients.txt");

	myfile << "Pressure Coefficient Values" << endl;

	//header
	myfile << " a0: \t  b1: \t" << " b2: \t c12 \t"<<endl;

	//STORING PRESSURE COEFFICIENTS 
	a0 = (RxBuffer[3]<<8) + RxBuffer[4];
	a0dec=(double)a0/8;
	pressureCoeffs[0] = a0dec;
	myfile<< a0dec << "\t ";

	b1 = (RxBuffer[5]<<8) + RxBuffer[6];
	if(b1>4)
	{
		b1=(b1-65536);
	}
	b1dec=(double)b1/8192;

	pressureCoeffs[1] = b1dec;
	myfile<< b1dec<< "\t ";

	b2 = (RxBuffer[7]<<8) + RxBuffer[8];
	if(b2>2)
	{
		b2=b2-65536;
	}
	b2dec = (double)b2/16384;
	pressureCoeffs[2] = b2dec;
	myfile << b2dec<< "\t ";

	c12 = (RxBuffer[9]<<8) + RxBuffer[10];
	c12dec = (double)c12/16777216; 
	pressureCoeffs[3] = c12dec;
	myfile << c12dec << endl;

	myfile.close();
	//ftdi.Write('K');//

	///MANUAL PUT REAL COEFFS IN 
	//a0dec = 3176.13;
	  
//b1dec = -1.63293;
//b2dec =	-0.638916; 
//c12dec 	=	 	 	 0.00157201;


	//CLEAR BUFFER

	while(1)
	{

		ftdi.Read(RxBuffer); //RETURNS IF LESS THAN 105 IN BUFFER WITH AN EMPTY BUFFER


		char header = RxBuffer[0];//this is how all logic is done>?

		if(RxBuffer[0] == 'P')
		{

			//myfile.open("C:\\Users\\RASL\\Desktop\\PressureData.txt", ios::app);
			myfile.open("C:\\Users\\youngem1\\Documents\\Visual Studio 2010\\Projects\\LES-DYE\\PressureData.txt", ios::app);

			int packageNum = (RxBuffer[1]<<8) + RxBuffer[2];
			myfile << " Package #: " << packageNum;
			cout<<"Read Pressure Successfully"<<endl;
			cout<<"Pkg Num: " << packageNum << endl;

			Padc = (RxBuffer[3]<<8) + RxBuffer[4];
			Padc=Padc/64;
			Tadc = (RxBuffer[5]<<8) + RxBuffer[6];
			Tadc=Tadc/64;

			pcomp=a0dec+(b1dec+c12dec*Tadc)*Padc+(b2dec*Tadc);
			pressure=pcomp*(115-50)/1023+50;//pcomp is zero so... 

			myfile << " Pressure: " << pressure;
			cout << " Pressure: " << pressure<<endl;

			//4 bytes pressure data 
			//Padc_MSB = RxBuffer[3];
			//myfile << " Padc_MSB: " << Padc_MSB;
			//Padc_LSB = RxBuffer[4];
			//myfile << " Padc_LSB: " << Padc_LSB;
			//Tadc_MSB = RxBuffer[5];
			//myfile << " Tadc_MSB: " << Tadc_MSB;
			//Tadc_LSB = RxBuffer[6];
			//myfile << " Tadc_LSB: " << Tadc_LSB;

			//int RSSI = RxBuffer[102];
			//myfile << " RSSI: " << RSSI << endl;
			//cout<<" RSSI: " << RSSI <<endl;
			int RSSI = RxBuffer[104]-256;
			myfile << " RSSI: " << RSSI << endl;
			cout<<" RSSI: " << RSSI << endl;
 

			myfile.close();

			//CLEAR RX BUFFER
			RxBuffer[0] = NULL;

		}

		else if(RxBuffer[0] == 'A')
		{
			myfile.open("C:\\Users\\youngem1\\Documents\\Visual Studio 2010\\Projects\\LES-DYE\\ADCData.txt", ios::app);
			
			//myfile.open("C:\\Users\\RASL\\Desktop\\ADCData.txt", ios::app);
			int packageNum = (RxBuffer[1]<<8) + RxBuffer[2];
			myfile << " Package #: " << packageNum;
			
			cout<<"Read ADC Successfully"<< endl;
			cout<<"Pkg Num: " << packageNum << endl;

			//ADC data 
			for(int i=0;i<25;i++) 
			{
				analog1[i] = RxBuffer[3+i];
				myfile<< " Analog 1: " << analog1[i];
				analog2[i] = RxBuffer[28+i];
				myfile<< " Analog 2: " << analog2[i];
				analog3[i] = RxBuffer[53+i];
				myfile<< " Analog 3: " << analog3[i] << endl;
			}

			int RSSI = RxBuffer[104]-256;
			myfile << " RSSI: " << RSSI << endl;
			//calc pressure 
			myfile.close();
			RxBuffer[0] = NULL;

		}//end of reading ADC value 

		if(kbhit())
		{
			//stop collecting data
			key = getch();
			if(key==-32)
			{
				key = getch();
				switch(key)
				{
				case UP_ARROW:
					cout << "UP ARROW" << endl;
					ftdi.Write(key);
					//ftdi.ReadBlock(RxBuffer); 
					break;
				case DOWN_ARROW:
					cout << "DOWN ARROW" << endl;
					ftdi.Write(key);
					//ftdi.ReadBlock(RxBuffer); 
					break;
				case LEFT_ARROW:
					cout << "LEFT_ARROW" << endl;
					ftdi.Write(key);
					//ftdi.ReadBlock(RxBuffer); 
					break;
				case RIGHT_ARROW:
					cout << "RIGHT_ARROW" << endl;
					ftdi.Write(key);
					//ftdi.ReadBlock(RxBuffer); 
					break;
				}

				//PWM_1 = (short)RxBuffer[1]*256 + (short)RxBuffer[2];
				//PWM_2= (short)RxBuffer[3]*256 + (short)RxBuffer[4];
				//cout << PWM_1 << endl;
				//cout << PWM_2 << endl;
				key = 0;
			}
		}//end if(kbhit)

		//readDataFromWRS

	}//end of main while loop
}

//int main(){
////--------------------------------
//// DECLARE VARIABLES
////--------------------------------
//	unsigned char RxBuffer[35];
//	myFTDI2 ftdi;
//	ftdi.FT_Initialize();
//	short x_bias = 0;
//	short y_bias = 0;
//	short z_bias = 0;
//
//	int IDnum;
//	short xL;
//	short xH;
//	short x;
//	short yL;
//	short yH;
//	short y;
//	short zL;
//	short zH;
//	short z;
//	short PW1;
//	short PW2;
//	char keyVal;
//	short PW3;
//	short PW4;
//	double xdoub;
//	double ydoub;
//	short PW_1H;
//	short PW_1L;
//	short PW_2H;
//	short PW_2L;
//	short PW_4H;
//	short PW_4L;
//	short PW_3L;
//	short PW_3H;
//	unsigned char PWBuffer[8];
//	short roll_Scaled;///M_PI;
//	short pitch_Scaled;
//	double roll;
//	double pitch;
//	ofstream myfile;
////--------------------------------
//// BEGIN PROGRAM
////--------------------------------
//	while(1)
//	{
//		//keyVal = getch();
//		cout << "Enter input" << endl;
//		cin >> keyVal;
//
//		switch(keyVal){
//		case 'a':
//			//Initialize Sensor communication
//			ftdi.Write(keyVal);
//			keyVal =0;
//			break;
//
//		case 'b': 
//			//Bias sensor
//			ftdi.Write(keyVal);
//			ftdi.Read(RxBuffer);
//			xL=(short)RxBuffer[1];
//			xH=(short)RxBuffer[2]<<8;
//			x_bias = xH + xL;
//			yL=(short)RxBuffer[3];
//			yH=(short)RxBuffer[4]<<8;
//			y_bias = yH + yL;
//			zL=(short)RxBuffer[5];
//			zH=(short)RxBuffer[6]<<8;
//			z_bias = zH + zL;
//			keyVal =0;
//			break;
//		case 's':	
//			//Stop the aquisition
//			ftdi.Write(keyVal);
//			keyVal =0;
//			break;
//
//		case 'k':
//			//Start the aquisition
//			myfile.open("C:\\Users\\youngem1\\Documents\\Visual Studio 2010\\Projects\\LES-DYE\\HW2Data.txt");
//			do{	
//				ftdi.Write(keyVal);
//				//-----------------------------------
//				//     GET PACKAGE
//				//     COMBINE HI AND LOW BITS
//				//-----------------------------------
//				ftdi.Read(RxBuffer);
//				IDnum = RxBuffer[0];
//				xL=(short)RxBuffer[1];
//				xH=(short)RxBuffer[2]<<8;
//				x = xH + xL - x_bias;
//				yL=(short)RxBuffer[3];
//				yH=(short)RxBuffer[4]<<8;
//				y = yH + yL - y_bias;
//				zL=(short)RxBuffer[5];
//				zH=(short)RxBuffer[6]<<8;				
//				z = zH + zL - z_bias;
//				cout<<"Packet Number: "<<IDnum<<endl;
//				cout<<"x: "<<x<<endl;
//				cout<<"y: "<<y<<endl;
//				cout<<"z: "<<z<<endl;
//
//				myfile<<" Packet Number: "<<IDnum;
//				myfile<<" x: "<<x;
//				myfile<<" y: "<<y;
//				myfile<<" z: "<<z << endl;
//
//				//-------------------------------------------------
//				//     Make sure x and y vales are bounded
//				//-------------------------------------------------
//				if(x>=16384)
//				{
//					x= 16384;
//				}
//				if(x<=-16384)
//				{
//					x= -16384;
//				}
//				xdoub = (double) x;
//				roll = asin(xdoub/16384);
//
//				if(y>=16384)
//				{
//					y= 16384;
//				}
//				if(y<=-16384)
//				{
//					y= -16384;
//				}
//				ydoub = (double) y;
//				pitch = asin(ydoub/16384);
//
//				roll_Scaled = (short)((roll*1999)/M_PI);
//		
//				pitch_Scaled = (short) ((pitch*1999)/M_PI);
//				//-------------------------------------------------
//				//     Set&Send PWM X
//				//-------------------------------------------------
//
//				if(roll_Scaled>=0)
//				{
//					PW1 = 999 - roll_Scaled;
//					PW2 = 999;
//				}
//				else
//				{
//					PW1 = 999;
//					PW2 = 999 - abs(roll_Scaled);
//				}
//
//				PW_1H =(PW1/256);
//				PW_1L = (PW1%256);
//				PW_2H = (PW2/256);
//				PW_2L = (PW2%256);
//
//
//
//				ftdi.WritePWM((char)PW_1H);
//				ftdi.WritePWM((char)PW_1L);
//				//Sleep(50);
//				ftdi.WritePWM((char)PW_2H);
//				ftdi.WritePWM((char)PW_2L);
//
//				//-------------------------------------------------
//				//     Set&Send PWM Y
//				//-------------------------------------------------
//				if(pitch_Scaled>=0)
//				{
//					PW3 = 999 - pitch_Scaled;
//					PW4 = 999;
//				}
//				else
//				{
//					PW3 = 999;
//					PW4 = 999 - abs(pitch_Scaled);
//				}
//
//				PW_3H = PW3/256;
//				PW_3L = PW3%256;
//				PW_4H = PW4/256;
//				PW_4L = PW4%256;
//
//
//				ftdi.WritePWM((char)PW_3H);
//				ftdi.WritePWM((char)PW_3L);
//				ftdi.WritePWM((char)PW_4H);
//				ftdi.WritePWM((char)PW_4L);
//
//			}while(!kbhit()); //CHECK TO SEE IF THE MODE HAS CHANGED
//
//			myfile.close();
//			keyVal =0;
//			break;
//		default:
//			cout << "Please enter a valid option" << endl;
//			//for debugging... 
//			//ftdi.Write(keyVal);
//			break;
//		}//END SWITCH
//	}//END WHILE
//}//ENDMAIN