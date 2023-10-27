/* Final Project Code

Name: Antheus Aldridge
Student Number: 400339569
MacID: aldria1

*/

#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

void PortH0H1H2H3_Init(void){		//This enables Port H pins 0 to 3
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7; //activate the clock for Port H
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){}; //allow time for clock to stabilize
	GPIO_PORTH_DIR_R = 0b00001111; // Make PH0:PH3 inputs, reading if the button is pressed or not
	GPIO_PORTH_DEN_R = 0b00001111; // Enable PH0:PH3
return;
}

void PortM0_Init(void){		//This enables Port M pin 0
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; //activate the clock for Port M
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){}; //allow time for clock to stabilize
	GPIO_PORTM_DIR_R = 0b00000000; // Make PM0 input, reading if the button is pressed or not
	GPIO_PORTM_DEN_R = 0b00000001; // Enable PM0
return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

double getValues(uint8_t dataReady, uint16_t	dev, int status, double angle){	
	uint16_t Distance;
	
		//wait until the ToF sensor's data is ready
	  while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
			VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0;
	  
		//read the distsnce data values from ToF sensor
	  status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
    
	  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
		return Distance;
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;


int main(void) {
	char TxChar;
	int input = 0;
	int data_array [3];
	
	s:		//Start for goto once button is pressed
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH0H1H2H3_Init();
	PortM0_Init();
	
	
	//ToF Variables
	uint8_t sensorState=0, i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint8_t dataReady;
	double angle = 0;
	double distance;

		/* Those basic I2C read functions can be used to check your own I2C functions */
		status = VL53L1X_GetSensorId(dev, &wordData);

		// Booting ToF chip
		while(sensorState==0){
			status = VL53L1X_BootState(dev, &sensorState);
			SysTick_Wait10ms(10);
		}
		FlashAllLEDs();

		status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

		/* This function must to be called to initialize the sensor with the default setting  */
		status = VL53L1X_SensorInit(dev);
		Status_Check("SensorInit", status);


		/* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
		status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */ // Set distance mode to long

		status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
	
	// always wait for the transmission code from pc. if 's' recieved then send data		
		
		//wait for the right transmition initiation code
		while(1){
			input = UART_InChar();
			if (input == 's')
				break;
		}
		
		while(1)
		{
			if((GPIO_PORTM_DATA_R&0b00000001)==0)
			{
				break;
			}
		}
		
		// simulating the transmission of 10 measurements
		//The value can be changed for less slices
		for(int i = 0; i < 10; i++)
		{
			angle = 0;
			
			for(int i = 0; i < 16; i++) {
				
				// Get distance reading
				distance = getValues(dataReady, dev, status, angle);
				
				sprintf(printf_buffer,"%d,  %f, %f, \r\n", i, distance, angle);
				//send string to uart
				UART_printf(printf_buffer);
				angle+=22.5;
				
				for(int i=0; i<32; i++){					

					if((GPIO_PORTM_DATA_R&0b00000001)==0) // If button is pressed
					{
						goto s;
					}
					
					GPIO_PORTH_DATA_R = 0b00001001;
					SysTick_Wait10ms(1);
					GPIO_PORTH_DATA_R = 0b00000011;
					SysTick_Wait10ms(1);
					GPIO_PORTH_DATA_R = 0b00000110;
					SysTick_Wait10ms(1);
					GPIO_PORTH_DATA_R = 0b00001100;
					SysTick_Wait10ms(1);
					
				}
				
				FlashLED3(1);
				SysTick_Wait10ms(20);
			}
			
			// Rotate stepper motor 360 degrees counter clockwise
			for(int i=0; i<512; i++){					
					GPIO_PORTH_DATA_R = 0b00001100;
					SysTick_Wait10ms(1);
					GPIO_PORTH_DATA_R = 0b00000110;
					SysTick_Wait10ms(1);
					GPIO_PORTH_DATA_R = 0b00000011;
					SysTick_Wait10ms(1);
					GPIO_PORTH_DATA_R = 0b00001001;
					SysTick_Wait10ms(1);
			}
		}
		
		VL53L1X_StopRanging(dev);
}






