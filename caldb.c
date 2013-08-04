#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/*
 * Test bench to calculate INA226 calibration value
 */


main(int argc, char *argv[])
{
	uint32_t a107, rs107,power_lsb,current_lsb;
	uint8_t shunt_mv;
	uint16_t shunt_amps,ina226_cal;
	
	
	if(argc != 3){
		printf("Usage: caldb amps mv\n");
		exit(1);
	}
	
	shunt_mv = (uint8_t) atoi(argv[2]);
	shunt_amps = (uint16_t) atoi(argv[1]);
	
	printf("shunt_mv   = 0x%02X\n", shunt_mv);
	printf("shunt_amps = 0x%04X\n", shunt_amps);
	
	
	

    a107 = 10000000 * shunt_amps;
    rs107 = (shunt_mv*(10000000/1000))/shunt_amps;
    
	printf("a107   = %u\n", a107);
	printf("rs107 = %u\n", rs107);  

    current_lsb = a107 >> 15;
    power_lsb = 25 * current_lsb;
    
    
    printf("power_lsb   = %u\n", power_lsb);
	printf("current_lsb = %u\n", current_lsb);  
  
    
    ina226_cal = (uint16_t) ((512000000)/((current_lsb * rs107 )/1000));
	printf("Cal value: %X\n", ina226_cal);
	exit(0);
}
