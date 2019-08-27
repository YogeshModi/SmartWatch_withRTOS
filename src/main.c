#include <stdint.h>
#include <string.h>
#include <math.h>
#include "BSP.h"
#include "icons.h"
#include "UART0.h"
#include "tm4c123gh6pm.h"
#include "CortexM.h"
#include "os.h"
#include "AP.h" 
#include "AP_lab6.h"
#include "eDisk.h"
#include "eFile.h"

#define THREADFREQ 1000   // frequency in Hz of round robin scheduler
#define SECTOR_SIZE 1024

uint32_t sqrt32(uint32_t s);
//---------------- Global variables shared between tasks ----------------
uint32_t Time;              // elasped time in 100 ms units
uint32_t Steps= 0;             // number of steps counted
uint32_t Magnitude;         // will not overflow (3*1,023^2 = 3,139,587)
uint32_t EWMA;              // https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
int32_t TemperatureData;    // 0.1C

char buff[100],tString[100];
uint8_t totalSteps[SECTOR_SIZE];
uint8_t sBuf[SECTOR_SIZE] = "0";

// File
uint8_t n; 

unsigned short weatherI[1156];
unsigned short stepsI[1156];
unsigned short temperatureI[1156];
unsigned short backI[1156];

// semaphores
int32_t NewData;  // true when new numbers to display on top of LCD
int32_t LCDmutex; // exclusive access to LCD
int32_t I2Cmutex; // exclusive access to I2C
uint8_t leftRight, upDown, buttonPress; // Joystick
int32_t select,Back;
int32_t saveData;
void EnableInterrupts(void);  // Enable interrupts

enum plotstate{
  stepCount,
  weather_data,
  temperatureData,
	menu_display,
	clock,
};
enum plotstate PlotState = clock;

//---------------- Task1 measures acceleration ----------------
// Event thread run by OS in real time at 10 Hz
int32_t TakeAccelerationData;
uint32_t LostTask1Data;     // number of times that the FIFO was full when acceleration data was ready
uint16_t AccX, AccY, AccZ;  // returned by BSP as 10-bit numbers
#define ALPHA 128           // The degree of weighting decrease, a constant smoothing factor between 0 and 1,023. A higher ALPHA discounts older observations faster.
                            // basic step counting algorithm is based on a forum post from
                            // http://stackoverflow.com/questions/16392142/android-accelerometer-profiling/16539643#16539643
enum state{                 // the step counting algorithm cycles through four states
  LookingForMax,            // looking for a local maximum in current magnitude
  LookingForCross1,         // looking for current magnitude to cross average magnitude, minus a constant
  LookingForMin,            // looking for a local minimum in current magnitude
  LookingForCross2          // looking for current magnitude to cross average magnitude, plus a constant
};
enum state AlgorithmState = LookingForMax;
#define LOCALCOUNTTARGET 5  // The number of valid measured magnitudes needed to confirm a local min or local max.  Increase this number for longer strides or more frequent measurements.
#define AVGOVERSHOOT 25     // The amount above or below average a measurement must be to count as "crossing" the average.  Increase this number to reject increasingly hard shaking as steps.
// *********Task1*********
// Task1 collects data from accelerometer in real time
// Periodic main thread runs in real time at 10 Hz
// Inputs:  none
// Outputs: none
void Accelerometer_Task(void){uint32_t squared;
  // initialize the exponential weighted moving average filter
  BSP_Accelerometer_Input(&AccX, &AccY, &AccZ);
  Magnitude = sqrt32(AccX*AccX + AccY*AccY + AccZ*AccZ);
  EWMA = Magnitude;                // this is a guess; there are many options
  Steps = 0;
  LostTask1Data = 0;
  while(1){
    OS_Wait(&TakeAccelerationData); // signaled by OS every 100ms
    BSP_Accelerometer_Input(&AccX, &AccY, &AccZ);
    squared = AccX*AccX + AccY*AccY + AccZ*AccZ;
    if(OS_FIFO_Put(squared) == -1){  // makes Task2 run every 100ms
      LostTask1Data = LostTask1Data + 1;
    }
    Time++; // in 100ms units
  }
}
/* ****************************************** */
/*          End of Task1 Section              */
/* ****************************************** */

void tostring(char str[], int num)
{
    int i, rem, len = 0, n;
 
    n = num;
    while (n != 0)
    {
        len++;
        n /= 10;
    }
    for (i = 0; i < len; i++)
    {
        rem = num % 10;
        num = num / 10;
        str[len - (i + 1)] = rem + '0';
    }
    str[len] = '\0';
}

int toint(char str[])
{
    int len = strlen(str);
    int i, num = 0;
 
    for (i = 0; i < len; i++)
    {
        num = num + ((str[len - (i + 1)] - '0') * pow(10, i));
    }
 
   return num;
}

void testbuildbuff(char *inString,uint8_t bt[SECTOR_SIZE]) {
    uint32_t i = 0;

    while ((i < SECTOR_SIZE) && (inString[i] != 0)) {
        bt[i] = inString[i];
        i = i + 1;
    }

    while (i < SECTOR_SIZE) {
        bt[i] = 0xFF; // fill the remainder of the buffer with 0xFF
        i = i + 1;
    }
}
int bufToStr(uint8_t bu[SECTOR_SIZE])
{
	int i=0;
	if(bu[0] == 0xFF){
		tString[0] = 0;
		return 0;
		}
	while(bu[i] != 0xFF ){
		tString[i] = bu[i];
		i++;
	}
	
	tString[i] = '\0';
	
	return 0;
		
}

//---------------- Task2 calculates steps and plots data on LCD ----------------
// Main thread scheduled by OS round robin preemptive scheduler
// accepts data from accelerometer, calculates steps, plots on LCD
// If no data are lost, the main loop in Task2 runs exactly at 10 Hz, but not in real time

void stepCounting_Task(void){uint32_t data;
  uint32_t localMin;   // smallest measured magnitude since odd-numbered step detected
  uint32_t localMax;   // largest measured magnitude since even-numbered step detected
  uint32_t localCount; // number of measured magnitudes above local min or below local max
  localMin = 1024;
  localMax = 0;
  localCount = 0;
  while(1){
    data = OS_FIFO_Get();
    Magnitude = sqrt32(data);
    EWMA = (ALPHA*Magnitude + (1023 - ALPHA)*EWMA)/1024;
    if(AlgorithmState == LookingForMax){
      if(Magnitude > localMax){
        localMax = Magnitude;
        localCount = 0;
      } else{
        localCount = localCount + 1;
        if(localCount >= LOCALCOUNTTARGET){
          AlgorithmState = LookingForCross1;
        }
      }
    } else if(AlgorithmState == LookingForCross1){
      if(Magnitude > localMax){
      // somehow measured a very large magnitude
        localMax = Magnitude;
        localCount = 0;
        AlgorithmState = LookingForMax;
      } else if(Magnitude < (EWMA -  AVGOVERSHOOT)){
        // step detected
        Steps = Steps + 1;
        localMin = 1024;
        localCount = 0;
        AlgorithmState = LookingForMin;
      }
    } else if(AlgorithmState == LookingForMin){
      if(Magnitude < localMin){
        localMin = Magnitude;
        localCount = 0;
      } else{
        localCount = localCount + 1;
        if(localCount >= LOCALCOUNTTARGET){
          AlgorithmState = LookingForCross2;
        }
      }
    } else if(AlgorithmState == LookingForCross2){
      if(Magnitude < localMin){
      // somehow measured a very small magnitude
        localMin = Magnitude;
        localCount = 0;
        AlgorithmState = LookingForMin;
      } else if(Magnitude > (EWMA + AVGOVERSHOOT)){
        // step detected
        Steps = Steps + 1;
        localMax = 0;
        localCount = 0;
        AlgorithmState = LookingForMax;
      }
    }
		
	}
	
}
/* ****************************************** */
/*          End of Task2 Section              */
/* ******************************************/
int countTest = 0;
int m=0;
uint32_t prevSteps=0;
void File_Task(void){
	
	while(1){
		countTest++;
		
		OS_Wait(&saveData); //Signaled by OS every 2 Sec
		
		bufToStr(totalSteps);
		m = toint(tString);
		if(Steps == prevSteps+1)
			m++;
		prevSteps = Steps;
		tostring(buff,m);
		testbuildbuff(buff,sBuf);
		OS_File_Rewrite(n,sBuf);
		OS_File_Flush();
		
	}
}


//------------Task3 handles switch input, buzzer output-------
// *********Task3*********
// Main thread scheduled by OS round robin preemptive scheduler
// real-time task, signaled on touch
//   with bouncing, may also be called on release
// checks the switches, updates the mode, and outputs to the buzzer and LED
// Inputs:  none
// Outputs: none
uint8_t count=0;
void Switch1_task(void){
  uint8_t last;
	last = BSP_Button1_Input();
  while(1){
		OS_Wait(&select); // OS signals on touch
    if(last){     // Button1 was pressed 
     // BSP_Buzzer_Set(512);   // beep for 20ms
      //OS_Sleep(20);          
      //BSP_Buzzer_Set(0);
			if(PlotState == clock)
				PlotState = menu_display;
			else {
				if(upDown ==1 && leftRight==1){
					PlotState = stepCount;
				} else if(upDown ==1 && leftRight==0){
					PlotState = weather_data;
				} else if(upDown ==0 && leftRight==1){
					PlotState = temperatureData;
				} 
			}
			count++;
		}
		OS_Sleep(20);        // wait for bouncing to be over
    last = BSP_Button1_Input();		
		GPIO_PORTD_IM_R |= 0x40;          // Re-arm interrupt on PD6
		GPIO_PORTD_ICR_R = 0x40;           // Clear PD6 flag
  }
}

void Switch2_task(void){
  uint8_t last;
	last = BSP_Button2_Input();

  while(1){
		OS_Wait(&Back); // OS signals on touch
		//BSP_Buzzer_Set(512);   // beep for 20ms
		//OS_Sleep(20);          
		//BSP_Buzzer_Set(0);
    if(last){     // Button2 was pressed 
			if(PlotState == menu_display || PlotState == clock){
				PlotState = clock;
				count=0;
			}
			else
				PlotState = menu_display;
		}
		OS_Sleep(20);        // wait for bouncing to be over
		last = BSP_Button2_Input();
		GPIO_PORTD_IM_R |= 0x80;          // Re-arm interrupt on PD6
    GPIO_PORTD_ICR_R = 0x80;           // Clear PD6 flag
  }
}
/* ****************************************** */
/*          End of Task3 Section              */
/* ****************************************** */



//------------Task4 measures temperature-------
// *********Task4*********
// Main thread scheduled by OS round robin preemptive scheduler
// measures temperature
// Inputs:  none
// Outputs: none
void Temperature_task(void){int32_t voltData,tempData;
  int done;
  while(1){
    BSP_TempSensor_Start();
    done = 0;
    OS_Sleep(1000);    // waits about 1 sec
    while(done == 0){
      done = BSP_TempSensor_End(&voltData, &tempData);
    }
    TemperatureData = tempData/10000;
  }
}
/**********************************
Joystick Input Task
**********************************/

void joystick_Task(void)
{
	uint16_t x,y;;
	upDown = 1;
	leftRight = 1;
	
	while(1){
		BSP_Joystick_Input(&x,&y,&buttonPress);
		if(x>700)
		{
			leftRight = 0; // right
		}
		if(x<200)
		{
			leftRight = 1; //left
		}
		if(y<200)
		{
			upDown = 0; //down
		}
		if(y>700)
		{
			upDown = 1; //up
		}
	}
}


/**********************************
Menu Handel Task
***********************************/
void menu_init(void)
{
	BSP_LCD_DrawBitmap(15, 49, steps, 34, 34);
	BSP_LCD_DrawBitmap(79, 49, weather, 34, 34);
	BSP_LCD_DrawBitmap(15, 113, Temperature, 34, 34);
	BSP_LCD_DrawBitmap(79, 113, back, 34, 34);
}

void invert(const unsigned short* input, unsigned short* output)
{
	for(int i=0;i<1156;i++)
	{
		output[i] = BSP_LCD_SwapColor(input[i]);
	}
}

uint8_t clear_Flag1=0,clearFlag2 = 0;

void drawCircle(void){
	int x,y;
	for(x=13;x<=113;x++)
	{
		y = sqrt(2500 - pow((x-63),2)) + 63;
		BSP_LCD_DrawPixel(x,y,0xFFFF);
	}
	for(x=14;x<=113;x++)
	{
		y = 63 - sqrt(2500 - pow((x-63),2));
		BSP_LCD_DrawPixel(x,y,0xFFFF);
	}
}

void displayClock(){
	if(clearFlag2 <1){
		BSP_LCD_FillScreen(BSP_LCD_Color565(0, 0, 0));
		clearFlag2++;
	}
	drawCircle();
}

void displaySteps(){
	if(clearFlag2 <1){
		BSP_LCD_FillScreen(BSP_LCD_Color565(0, 0, 0));
		clearFlag2++;
	}
	uint8_t res;
	OS_File_Read(n,0,totalSteps);
	BSP_LCD_DrawString(2,  4, "Total Steps = ",  0xFFFF);
	//BSP_LCD_SetCursor(17, 4);
	//bufToStr(totalSteps,Total);
	BSP_LCD_DrawString(17,4,tString,0xFFFF);
	BSP_LCD_DrawString(2,  6, "Steps Today = ",  0xFFFF);
	BSP_LCD_SetCursor(17, 6);
	BSP_LCD_OutUDec(Steps,0xFFE0);
}

char degreeC[2];
void displayTemperature(){
	if(clearFlag2 <1){
		BSP_LCD_FillScreen(BSP_LCD_Color565(0, 0, 0));
		clearFlag2++;
	}
	BSP_LCD_DrawString(0,  6, "Temperature = ",  0xFFFF);
	BSP_LCD_SetCursor(14, 6);
	BSP_LCD_OutUFix2_1(TemperatureData,0xFFE0);
	BSP_LCD_DrawString(19,  6, degreeC ,  0xFFE0);
}
void displayWeather(){
	BSP_LCD_FillScreen(BSP_LCD_Color565(0, 255, 0));
}

void menu_Task(void)
{
	BSP_LCD_FillScreen(BSP_LCD_Color565(0, 0, 0));
	while(1){
		if(PlotState == menu_display){
			if(clear_Flag1 <1 ){
				BSP_LCD_FillScreen(BSP_LCD_Color565(0, 0, 0));
				clear_Flag1++;
				clearFlag2 = 0;
			}
			
			if(upDown ==1 && leftRight==1)
			{
					BSP_LCD_DrawBitmap(15, 49, stepsI, 34, 34);
					BSP_LCD_DrawBitmap(79, 49, weather, 34, 34);
					BSP_LCD_DrawBitmap(15, 113, Temperature, 34, 34);
					BSP_LCD_DrawBitmap(79, 113, back, 34, 34);
			}
			
			else if(upDown ==1 && leftRight==0 )
			{ 
					BSP_LCD_DrawBitmap(15, 49, steps, 34, 34);
					BSP_LCD_DrawBitmap(79, 49, weatherI, 34, 34);
					BSP_LCD_DrawBitmap(15, 113, Temperature, 34, 34);
					BSP_LCD_DrawBitmap(79, 113, back, 34, 34);
				
			}
			else if(upDown ==0 && leftRight==1 )
			{
					BSP_LCD_DrawBitmap(15, 49, steps, 34, 34);
					BSP_LCD_DrawBitmap(79, 49, weather, 34, 34);
					BSP_LCD_DrawBitmap(15, 113, temperatureI, 34, 34);
					BSP_LCD_DrawBitmap(79, 113, back, 34, 34);	
						
			}
			else
			{
				BSP_LCD_DrawBitmap(15, 49, steps, 34, 34);
				BSP_LCD_DrawBitmap(79, 49, weather, 34, 34);
				BSP_LCD_DrawBitmap(15, 113, Temperature, 34, 34);
				BSP_LCD_DrawBitmap(79, 113, backI, 34, 34);		
			}
		}
		else if(PlotState == clock)
		{
			displayClock();
			clear_Flag1 = 0;
		}
		else if(PlotState == stepCount )
		{
			displaySteps();
			clear_Flag1 = 0;
		}
		
		else if(PlotState == weather_data )
		{
			displayWeather();
			clear_Flag1=0;

		}
		
		else if(PlotState == temperatureData)
		{
			displayTemperature();
			clear_Flag1=0;
		}
	}

}

//---------------- Task7 dummy function ----------------
// *********Task7*********
// Main thread scheduled by OS round robin preemptive scheduler
// Task7 does nothing but never blocks or sleeps
// Inputs:  none
// Outputs: none
uint32_t Count7;
void idleTask(void){
  Count7 = 0;
  while(1){
    Count7++;
    WaitForInterrupt();
  }
}

// Test function: Copy a NULL-terminated 'inString' into the
// 'Buff' global variable with a maximum of 512 characters.
// Uninitialized characters are set to 0xFF.
// Inputs:  inString  pointer to NULL-terminated character string
// Outputs: none



int main(void)
{
	OS_Init();
	invert(weather,weatherI);
	invert(steps,stepsI);
	invert(Temperature,temperatureI);
	invert(back,backI);
	BSP_Clock_InitFastest();
	BSP_Joystick_Init();
	BSP_Buzzer_Init(0);
	BSP_LCD_Init();
	BSP_TempSensor_Init();
	BSP_Button1_Init();
	BSP_Button2_Init();
	BSP_Accelerometer_Init();
	OS_FIFO_Init();
	OS_EdgeTrigger_Init(&select,&Back, 3);
	OS_InitSemaphore(&NewData, 0);
	OS_InitSemaphore(&select,0); // signaled on touch button1
	OS_InitSemaphore(&Back,0); // signaled on touch button1
	OS_EdgeTrigger_Init(&select,&Back, 3);
	eDisk_Init(0);
	BSP_LCD_FillScreen(BSP_LCD_Color565(0, 0, 0));
	
	degreeC[0] = 248;
	degreeC[1] = 67;	
	
	//OS_File_Format();
	n = OS_File_New();
	
	//testbuildbuff("0",totalSteps);
	testbuildbuff("0",sBuf);
	OS_File_Read(n,0,totalSteps);
	//OS_File_Append(n,sBuf);
	displayClock();
	OS_AddThreads(&File_Task,3, &Accelerometer_Task,1, &stepCounting_Task,2, &Switch1_task,3,&Switch2_task,3, 
	              &joystick_Task,3,&Temperature_task,3, &menu_Task,3, &idleTask,4);
	OS_PeriodTrigger1_Init(&TakeAccelerationData,100); //every 100ms
	OS_PeriodTrigger0_Init(&saveData,1000); //every 2000ms
	OS_Launch(BSP_Clock_GetFreq()/THREADFREQ); // doesn't return, interrupts enabled in here
		

	
		return 0;
}

// Newton's method
// s is an integer
// sqrt(s) is an integer
uint32_t sqrt32(uint32_t s){
uint32_t t;   // t*t will become s
int n;             // loop counter
  t = s/16+1;      // initial guess
  for(n = 16; n; --n){ // will finish
    t = ((t*t+s)/t)/2;
  }
  return t;
}

