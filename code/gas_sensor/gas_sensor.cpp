/*******************************************************************************
*
* based on Robotics Cape i2c.c
*
*******************************************************************************/
 
#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"
#include "../../libraries/serial_ports/rc_i2c.c"
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stropts.h>
#include <time.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>

uint8_t addr = 0x04; // MiCS-6814 I2C address
enum{CO, NO2, NH3, C3H8, C4H10, CH4, H2, C2H5OH};		
uint8_t ADDR_USER_ADC_HN3 = 0x08; // register address to get the calibrated default value for NH3 channel (with no calibration, it should be the factory default value)
uint8_t ADDR_USER_ADC_CO = 0x0A; // register address to get the calibrated default value of CO channel 
uint8_t ADDR_USER_ADC_NO2 = 0x0C; // register address to get the calibrated default value of CO channel
uint8_t CH_VALUE_NH3 = 0x01; // register address to get the current value for NH3 channel
uint8_t CH_VALUE_CO = 0x02; // register address to get the current value for CO channel
uint8_t CH_VALUE_NO2 = 0x03; // register address to get the current value for NO2 channel
uint8_t Power_On[2] =  {0x11,0x01}; // to power on the heating resistance
uint8_t Power_Off[2] =  {0x11,0x00}; // to power off the heating resistance	
float CO_ppm = 0.0; // ppm value for CO
float NO2_ppm = 0.0; 
float NH3_ppm = 0.0; // ppm value for NH3
float C3H8_ppm = 0.0; 
float C4H10_ppm = 0.0;
float CH4_ppm = 0.0;
float H2_ppm = 0.0;
float C2H5OH_ppm = 0.0;

void ledOn()
{
  uint8_t led = 0x0A;
	uint8_t on = 0x01;
	uint8_t buffer[2] = {led, on}; 
	rc_i2c_send_bytes(1, 2, buffer);
}
    
void ledOff()
{
  uint8_t led = 0x0A;
	uint8_t off = 0x00;
	uint8_t buffer[2] = {led, off}; 
	rc_i2c_send_bytes(1, 2, buffer);
}

unsigned int get_addr_dta(uint8_t addr_reg, uint8_t __dta) // get default value for a specific resistance
{
	//char hex[10]; // useful for debugging: shows the value read in hexadecimal
	unsigned int dta = 0; // data read
	uint8_t raw[2]; // raw data
	
	// assemble the data to be sent
	uint8_t buffer[2] = {addr_reg,__dta}; 
	
	// send data
	rc_i2c_send_bytes(1, 2, buffer);

	// read 2 bytes
	rc_i2c_read_bytes_void(1, (uint8_t)2, raw); // NOTE: the rc_i2c_read_bytes function in rc_i2c.c in libraries has been modified 
								  //	   in order to not write anything to the sensor before the reading operation
	
	// raw data manipulation to get actual data
	dta = raw[0];
	dta <<= 8;
	dta += raw[1];
	
	// print the hexadecimal value of read data
	//sprintf(&hex, "%x", dta);
	//printf("bytes read : 0x%s",hex);
	
	return dta;
	
}

unsigned int get_addr_dta_void(uint8_t addr_reg) // get current value for a specific resistance
{
	
	//char hex[10]; // useful for debugging: shows the value read in hexadecimal
	unsigned int dta = 0; // data read
	uint8_t raw[2]; // raw data
	
	// send data
	rc_i2c_send_byte(1, addr_reg);
	
	// read 2 bytes
	rc_i2c_read_bytes_void(1, (uint8_t)2, raw); // NOTE: the rc_i2c_read_bytes function in rc_i2c.c in libraries has been modified
											//	   in order to not write anything to the sensor before the reading operation
	
	// raw data manipulation to get actual data	
	dta = raw[0];
	dta <<= 8;
	dta += raw[1];
	
	// print the hexadecimal value of read data
	//sprintf(&hex, "%x", dta);
	//printf("bytes read : 0x%s",hex);
	
	return dta; 
	
}

float calcGas(int gas) // get ppm value of specified gas
{
	ledOn();
	
	unsigned int a0_0 = get_addr_dta(0x06, ADDR_USER_ADC_HN3); //0x035C as default value of resistance
	unsigned int a0_1 = get_addr_dta(0x06, ADDR_USER_ADC_CO);   //0x03B6 as default value of resistance
	unsigned int a0_2 = get_addr_dta(0x06, ADDR_USER_ADC_NO2); // default value of resistance
	
	unsigned int an_0 = get_addr_dta_void(CH_VALUE_NH3); // current value
	unsigned int an_1 = get_addr_dta_void(CH_VALUE_CO);  // current value
	unsigned int an_2 = get_addr_dta_void(CH_VALUE_NO2); // current value

	float ratio0 = (float)an_0/(float)a0_0*(1023.0-a0_0)/(1023.0-an_0);
	float ratio1 = (float)an_1/(float)a0_1*(1023.0-a0_1)/(1023.0-an_1);
	float ratio2 = (float)an_2/(float)a0_2*(1023.0-a0_2)/(1023.0-an_2);
	 
	float c = 0;
	
	switch(gas)
	{
		case CO:
		{
			c = pow(ratio1, -1.179)*4.385;
			break;
		}
        case NO2:
        {
            c = pow(ratio2, 1.007)/6.855;  
            break;
        }
        case NH3:
        {
            c = pow(ratio0, -1.67)/1.47;  
            break;
        }
        case C3H8:  
        {
            c = pow(ratio0, -2.518)*570.164;
            break;
        }
        case C4H10:  
        {
            c = pow(ratio0, -2.138)*398.107;
            break;
        }
        case CH4:  
        {
            c = pow(ratio1, -4.363)*630.957;
            break;
        }
        case H2: 
        {
            c = pow(ratio1, -1.8)*0.73;
            break;
        }
        case C2H5OH:  
        {
            c = pow(ratio1, -1.552)*1.622;
            break;
        }
        default:
			break;
		
	}
	
	ledOff();
	
	return c;
}

int main(){

		/*VERSION 1 ONLY: (MiCS 6814 SHOULD BE VERSION 2)
		uint8_t Res_0 =  0x01; // write this command to request Res_0 value, i.e. the resistance value for the CH3 channel
		uint8_t Res_1 =  0x02; // write this command to request Res_1 value, i.e. the resistance value for the CO channel
		uint8_t Res_2 =  0x03; // write this command to request Res_2 value, i.e. the resistance value for the NO2 channel
		uint8_t Res0_0 =  0x11; // write this command to request Res0_0 value, i.e. the stored default resistance value for the CH3 channel
		uint8_t Res0_1 =  0x12; // write this command to request Res0_1 value, i.e. the stored default resistance value for the CO channel
		uint8_t Res0_2 =  0x13; // write this command to request Res0_2 value, i.e. the stored default resistance value for the NO2 channel
		uint8_t checksum = 0x00; // checksum for security reasons
		*/
	int res;
	time_t t = time(NULL);
	struct tm *tmp = gmtime(&t);
		
        // initialize hardware first
        if(rc_initialize()){
                fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
                return -1;
        }// this call is mandatory at the beginning of the program. avoids conflicts and handles exit calls
		
		// initialize i2c bus (bus 1. bus 0 is reserved for internal IMU and barometer)
		if ((res = rc_i2c_init(1, addr)) != 0)
        {
                fprintf(stderr, "ERROR: failed to initialize I2C: %d\n", res);
                return -1;
        }
		
		//printf("\n\n-- Invoking command :  i2cdetect -r 1\n\n");
		//system("i2cdetect -r 1");
		
		//powering on the heating resistance
		//printf("\n-- Powering the heating resistance --\n");
		if(rc_i2c_send_bytes(1, 2, Power_On) != 0)
			{
				printf("\nFail to power the heating resistance");
			}
		else{ printf("\nHeating resistance powered");}
		//int c;
		//printf("\n\nPress enter to continue: ");
		//do{
		//c = getchar(); 
		//putchar(c);}
		//while (c != '\n');		
		
		// acquire data
		while (rc_get_state() != EXITING){
			
			//unsigned int An_1 = get_addr_dta_void(CH_VALUE_CO);
			//unsigned int A0_1 = get_addr_dta(0x06, ADDR_USER_ADC_CO);
			//unsigned int An_0 = get_addr_dta_void(CH_VALUE_NH3);
			//unsigned int A0_0 = get_addr_dta(0x06, ADDR_USER_ADC_HN3);
			t = time(NULL);
			tmp = gmtime(&t);
			CO_ppm = calcGas(CO);
			//printf("\nVALUE FOR CO [detected]: %u", An_1);
			//printf("\nVALUE FOR CO [detected]: %f [float]", (float)An_1);
			//printf("\nVALUE FOR CO [default] : %u", A0_1);
			//printf("\nVALUE FOR CO [default] : %f [float]", (float)A0_1);
			printf("\nCO : %f ", CO_ppm);
			printf("%d:%d:%d [ppm]\n",(tmp->tm_hour+2)%24,tmp->tm_min, tmp->tm_sec);
			//sleep(1);
			NH3_ppm = calcGas(NH3);
			printf("NH3 : %f ", NH3_ppm);
			printf("%d:%d:%d [ppm]\n",(tmp->tm_hour+2)%24,tmp->tm_min,tmp->tm_sec);
			//printf("\nVALUE FOR NH3 [detected]: %u", An_0);
			//printf("\nVALUE FOR NH3 [detected]: %f [float]", (float)An_0);
			//printf("\nVALUE FOR NH3 [default] : %u", A0_0);
			//printf("\nVALUE FOR NH3 [default] : %f [float]", (float)A0_0);
			//sleep(1);
			NO2_ppm = calcGas(NO2); // CHECK THIS
			printf("NO2 : %f ", NO2_ppm);
			printf("%d:%d:%d [ppm]\n",(tmp->tm_hour+2)%24,tmp->tm_min,tmp->tm_sec);
			C3H8_ppm = calcGas(C3H8); // CHECK THIS
			printf("C3H8 : %f ", C3H8_ppm);
			printf("%d:%d:%d [ppm]\n",(tmp->tm_hour+2)%24, tmp->tm_min,tmp->tm_sec);
			//sleep(1);
			C4H10_ppm = calcGas(C4H10); // CHECK THIS
			printf("C4H10 : %f ", C4H10_ppm);
			printf("%d:%d:%d [ppm]\n",(tmp->tm_hour+2)%24,tmp->tm_min,tmp->tm_sec);
			//sleep(1);
			CH4_ppm = calcGas(CH4); // CHECK THIS
			printf("CH4 : %f ", CH4_ppm);
			printf("%d:%d:%d [ppm]\n",(tmp->tm_hour+2)%24,tmp->tm_min,tmp->tm_sec);
			//sleep(1);
			H2_ppm = calcGas(H2); // CHECK THIS
			printf("H2 : %f ", H2_ppm);
			printf("%d:%d:%d [ppm]\n",(tmp->tm_hour+2)%24,tmp->tm_min,tmp->tm_sec);
			//sleep(1);
			C2H5OH_ppm = calcGas(C2H5OH); // CHECK THIS
			printf("C2H5OH : %f ", C2H5OH_ppm);
			printf("%d:%d:%d [ppm]\n",(tmp->tm_hour+2)%24,tmp->tm_min,tmp->tm_sec);
			printf("\n\n--- End of Iteration --\n\n");
			fflush(stdout);
			rc_usleep(500000);
			//sleep(1);
			
		}
			
			// close all threads, objects and pending files. mandatory for conflicts avoidance
			rc_i2c_close(1);
			rc_cleanup(); //this call is mandatory at the end of the program. closes files, pointers and waits for threads to safely stop
			return 0;
}
