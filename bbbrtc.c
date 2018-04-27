

#define RTC_SS_BASE 0x44e3e000L				// Memory base for the RTC registers


#define RTC_CURRENT_TIME_TM		0x00		// Offset to the current time registers

#define SECONDS_REG 			0x00
#define MINUTES_REG				0x04
#define HOURS_REG				0x08
#define DAYS_REG				0x0C
#define MONTHS_REG 				0x10
#define YEARS_REG				0x14


#define RTC_ALARM_TIME_TM		0x20		// Offset to the alarm time registers (wake time)

#define ALARM_SECONDS_REG 		0x20
#define ALARM_MINUTES_REG		0x24
#define ALARM_HOURS_REG			0x28
#define ALARM_DAYS_REG			0x2C
#define ALARM_MONTHS_REG 		0x30
#define ALARM_YEARS_REG			0x34


#define RTC_ALARM2_TIME_TM		0x80		// Offset to the alarm2 time registers (sleep time)

#define ALARM2_SECONDS_REG 		0x80
#define ALARM2_MINUTES_REG		0x84
#define ALARM2_HOURS_REG		0x88
#define ALARM2_DAYS_REG			0x8C
#define ALARM2_MONTHS_REG 		0x90
#define ALARM2_YEARS_REG		0x94

#define RTCALARMINT 			76			// RTC alarm interrupt

#define RTC_CTRL_REG			0x40
#define RTC_CTRL_STOP			(1<<0)		// 1=Stop the RTC

#define RTC_STATUS_REG			0x44
#define RTC_STATUS_ALARM		(1<<6)		// Indicates ALARM has happened
#define RTC_STATUS_RUN			(1<<1)		// 0=frozen, 1=running

#define RTC_INTERRUPTS_REG		0x48
#define RTC_INTERRUPTS_IT_ALARM2 (1<<4)		// Enable interrupt on ALARM2
#define RTC_INTERRUPTS_IT_ALARM	 (1<<3)		// Enable interrupt on ALARM
#define RTC_INTERRUPTS_IT_TIMER	 (1<<2)		// Enable interrupt on TIMER

#define RTC_SYSCONFIG			0x78
#define RTC_IRQWAKEEN			0x7c
#define RTC_IRQWAKEEN_ALARM		(1<<1)		// Allow wakeup generation on ALARM
#define RTC_IRQWAKEEN_TIMER		(1<<0)		// Allow wakeup generation on TIMER

#define RTC_PMIC				0x98
#define RTC_PMIC_PWN_ENABLE_EN	(1<<16)
#define RTC_PMIC_EXT_WAKE_POL	(1<<4)		// 1=active low
#define RTC_PMIC_EXT_WAKE_EN	(1<<0)		// 1=external wake enabled

#define ASSIGN_WORD(w,v) ((*( (unsigned long *) (&(w)) ) ) = (v) )

#include <time.h>		// Get linux time conversion API functions
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>			// strerror
#include <errno.h>
//#include <signal.h>
#include <fcntl.h>
//#include <ctype.h>
//#include <termios.h>
//#include <sys/types.h>
#include <sys/mman.h>

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

int   bcd2n(  int bcd ) {
	
	return ((bcd/16) * 10 ) + ( bcd & 15);
	
}

int n2bcd( int n ) {
	
	return (( n / 10 ) * 16 ) + (n % 10 );
	
}

void dump( unsigned char *base) {
	printf("Dumping RTC:\n");
	
	for(int i=0; i<=0x98;i+=4) {
		
		printf("%02.2x %08.8x\n", i ,  *((unsigned long *) (base+i)) );
		
	}
		
}

void set32reg( unsigned char *base , unsigned char off, unsigned val ) {
	
	*( (unsigned *) (base+off)) = val;
	
}

unsigned get32reg( unsigned char *base , unsigned char off) {
	
	return *( (unsigned *) (base+off));
	
}


int main(int argc, char **argv) {
    int fd;
    void *map_base, *virt_addr;
    unsigned long read_result, writeval;
    off_t target;
   
    target = RTC_SS_BASE;
	
	printf("Opening /dev/mem...");
    fflush(stdout);
     
    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
		
		fprintf(stderr,"Error opening /dev/mem:%s",strerror(errno));
		
	} else {
	
		printf("opened.\n");

		printf("Mappng in %p..." , target );
		fflush(stdout);
				
		map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target );
		
		if(map_base == MAP_FAILED ) {
			
			fprintf(stderr,"Error mmapping /dev/mem:%s",strerror(errno));
			
		} else {
			
			printf("mapped at address %p.\n", map_base);	
			fflush(stdout);

			unsigned char *base = (unsigned char *) map_base;
			
			printf("Value at address %d\n", *base );
			fflush(stdout);
			
			dump(base);

			// HERE IS THE BEEF
			
			printf("Turn off PMIC OFF bit...");
			
			int i2cset_ret=system("i2cset -f -y 0 0x24 0x0a 0x00");			// Turn off OFF bit
			
			if (i2cset_ret) {
				
				printf("failed with code %d.\n");
				
			} else {
			
				printf("off.\n");

				
				// We can not guaratee being able to access regs in the 15us not busy window under linux, so
				// instead we stop the whole RTC and the restart it when done.
				// We will drop some time each time, but not much. 
				
				
				printf("Stopping RTC...");
				fflush(stdout);
				
				set32reg( base , RTC_CTRL_REG ,  0x00);		// Write a 0 to bit 0 to freeze the RTC so we cna update regs
				
				printf("waiting for stop...");
				fflush(stdout);
				
				do {
					unsigned int wait_counter=0;
				
					while ( get32reg( base , RTC_STATUS_REG)  & RTC_STATUS_RUN ) wait_counter++;
				
					printf("took %u tries.\n",wait_counter);
					fflush(stdout); 
					
				} while (0);
				
				
				printf("Get current time ...\n");
					
				int secs   = bcd2n( base[ SECONDS_REG] );
				int mins   = bcd2n( base[ MINUTES_REG] );
				int hours  = bcd2n( base[ HOURS_REG ] );
				int days   = bcd2n( base[ DAYS_REG ] );
				int months = bcd2n( base[ MONTHS_REG ] );
				int years  = bcd2n( base[ YEARS_REG ] );

				printf("Bump forward 5 secs in the future......\n");
				
				secs += 5;
				
				if (secs >= 60 ) {
					
					secs %= 60;
					
					mins++;
					
					if (mins >= 60 ) {
						
						mins %=60; 
						
						hours++;
						
						if (hours >=24 ) {
							
							hours %=24;
							
							days++;
							
							// TODO: Make work last day of the month at midnight
						}
					}
				}
				/*
				
				printf("Set ALARM for wakeup......\n");
					
				base[ALARM_SECONDS_REG]  = n2bcd( secs );
				base[ALARM_MINUTES_REG]  = n2bcd( mins );
				base[ALARM_HOURS_REG]    = n2bcd( hours );
				base[ALARM_DAYS_REG]     = n2bcd( days );
				base[ALARM_MONTHS_REG]   = n2bcd( months );
				base[ALARM_YEARS_REG]    = n2bcd( years );
				
				printf("NOW ALARM\n");
				printf("%3.3x %3.3x\n" , base[ SECONDS_REG], base[ ALARM_SECONDS_REG] );
				printf("%3.3x %3.3x\n" , base[ MINUTES_REG], base[ ALARM_MINUTES_REG] );
				printf("%3.3x %3.3x\n" , base[ HOURS_REG], base[ ALARM_HOURS_REG] );
				printf("%3.3x %3.3x\n" , base[ DAYS_REG], base[ ALARM_DAYS_REG] );
				printf("%3.3x %3.3x\n" , base[ MONTHS_REG], base[ ALARM_MONTHS_REG] );
				printf("%3.3x %3.3x\n" , base[ YEARS_REG], base[ ALARM_YEARS_REG] );
			 
				printf(" Enable ALARM interrupt (necessary?)... \n");
			 
				set32reg( base , RTC_INTERRUPTS_REG , RTC_INTERRUPTS_IT_ALARM );
				
				printf("Enable PWR_ENABLE_EN to be controlled ON->OFF by ALARM2...\n");
				
				set32reg( base , RTC_PMIC , RTC_PMIC_PWN_ENABLE_EN );
				
				set32reg( base , RTC_IRQWAKEEN , RTC_IRQWAKEEN_ALARM );
				
				*/
				
				printf("Set ALARM2 for sleep......\n");
					
				base[ALARM2_SECONDS_REG]  = n2bcd( secs );
				base[ALARM2_MINUTES_REG]  = n2bcd( mins );
				base[ALARM2_HOURS_REG]    = n2bcd( hours );
				base[ALARM2_DAYS_REG]     = n2bcd( days );
				base[ALARM2_MONTHS_REG]   = n2bcd( months );
				base[ALARM2_YEARS_REG]    = n2bcd( years );
				
				printf("NOW ALARM2\n");
				printf("%3.3x %3.3x\n" , base[ SECONDS_REG], base[ ALARM2_SECONDS_REG] );
				printf("%3.3x %3.3x\n" , base[ MINUTES_REG], base[ ALARM2_MINUTES_REG] );
				printf("%3.3x %3.3x\n" , base[ HOURS_REG], base[ ALARM2_HOURS_REG] );
				printf("%3.3x %3.3x\n" , base[ DAYS_REG], base[ ALARM2_DAYS_REG] );
				printf("%3.3x %3.3x\n" , base[ MONTHS_REG], base[ ALARM2_MONTHS_REG] );
				printf("%3.3x %3.3x\n" , base[ YEARS_REG], base[ ALARM2_YEARS_REG] );
			 

				// bump forward an additional 5 seconds after sleep
			 
				secs += 5;
				
				if (secs >= 60 ) {
					
					secs %= 60;
					
					mins++;
					
					if (mins >= 60 ) {
						
						mins %=60; 
						
						hours++;
						
						if (hours >=24 ) {
							
							hours %=24;
							
							days++;
							
							// TODO: Make work last day of the month at midnight
						}
					}
				}
				
				
				printf("Set ALARM for wakeup......\n");
					
				base[ALARM_SECONDS_REG]  = n2bcd( secs );
				base[ALARM_MINUTES_REG]  = n2bcd( mins );
				base[ALARM_HOURS_REG]    = n2bcd( hours );
				base[ALARM_DAYS_REG]     = n2bcd( days );
				base[ALARM_MONTHS_REG]   = n2bcd( months );
				base[ALARM_YEARS_REG]    = n2bcd( years );
				
				printf("NOW ALARM\n");
				printf("%3.3x %3.3x\n" , base[ SECONDS_REG], base[ ALARM_SECONDS_REG] );
				printf("%3.3x %3.3x\n" , base[ MINUTES_REG], base[ ALARM_MINUTES_REG] );
				printf("%3.3x %3.3x\n" , base[ HOURS_REG], base[ ALARM_HOURS_REG] );
				printf("%3.3x %3.3x\n" , base[ DAYS_REG], base[ ALARM_DAYS_REG] ); 
				printf("%3.3x %3.3x\n" , base[ MONTHS_REG], base[ ALARM_MONTHS_REG] );
				printf("%3.3x %3.3x\n" , base[ YEARS_REG], base[ ALARM_YEARS_REG] );
				 

				printf("Enable WAKE...\n");
				set32reg( base , RTC_IRQWAKEEN , RTC_IRQWAKEEN_ALARM | RTC_IRQWAKEEN_TIMER);		// Necessary? Need timer also?

				printf("DONE setting Alarm times...\n");
				
				
				printf("Enable PWR_ENABLE_EN to be controlled ON->OFF by ALARM2, EXTERNAL WAKE enabled...\n");	
				//ASSIGN_WORD( base[RTC_PMIC] , RTC_PMIC_PWN_ENABLE_EN | RTC_PMIC_EXT_WAKE_EN  );
				
				set32reg( base , RTC_PMIC , RTC_PMIC_PWN_ENABLE_EN  );
				
				// Check with...
				//  devmem2 0x44e3e098
				
				// Clear the ALARM@ bit with 
				//  devmem2 0x44e3e044 w 0x82
					
				//printf("Enable Wake pin, invert polarity, PWR_ENABLE_EN to be controlled ON->OFF by ALARM2...\n");	
				//base[RTC_PMIC]=0x10011;				//Enable wakeup pin, invert polarity, POWER_ENABLE_EN
					
				printf(" Enable alarm interrupts... \n");
				set32reg( base , RTC_INTERRUPTS_REG  ,  RTC_INTERRUPTS_IT_ALARM2 | RTC_INTERRUPTS_IT_ALARM );	
				
				// Check with...
				// devmem2 0x44e3e048
				// should be 0x18
				
						 
				/*
				
				for(int i=0; i<15;i++) {
					printf("%d-%d\n",i, base[RTC_STATUS_REG] & RTC_STATUS_ALARM  );
				
					sleep(1); 
					
				} 
				*/
				
				dump(base);
				
				printf("Restarting RTC...");
				
				set32reg( base  , RTC_CTRL_REG ,  RTC_CTRL_STOP);		// Write a 1 to bit 0 to start the RTC so we can update regs
				
				printf( "waiting for it to start..." );
				
				do {
					
					unsigned int wait_counter=0;
				
					while ( !( get32reg( base , RTC_STATUS_REG)  & RTC_STATUS_RUN ) ) wait_counter++;
				
					printf("took %u tries.\n",wait_counter);
					fflush(stdout); 
					
				} while (0);
				
				
			}
			
			printf("unmap...");

			munmap(map_base, MAP_SIZE);
			
			printf("unmaped.\n");
			
		}
		
		printf("closing fd...");		
		
		close(fd);	
		
		printf("closed.\n");
		
	}

		
	// Now got to sleep!
	
//	system("devmem2 0x44e3e098 w 0x10011");				//Enable wakeup pin, invert polarity, POWER_ENABLE_EN
	 


	
//	system("i2cset -f -y 0 0x24 0x0a 0x080");			// Turn on OFF bit
	
//	system("i2cset -f -y 0 0x24 0x0b 0x63");			// PAssword to access 1e
	
	//printf("Sending SEQDWN+SEQUP...\n");
	//system("i2cset -f -y 0 0x24 0x1e 0x06");			// Set DWNSEQ bit to start shutdown sequence
	
	
	//system("i2cset -f -y 0 0x24 0x1e 0x02");			// Set DWNSEQ bit to start shutdown sequence

	printf("Waiting for turnoff via TIMER2 in 5 seconds!...\n");
	
	//__asm__ __volatile__ ("cpsie i"); /* Clear PRIMASK */
	
	
    return 0;
	
	/* Clear stuff
	
	#clear INT regs
	devmem2 0x44e3e048 W 0x00
	#status regs
	devmem2 0x44e3e044 W 0xC2
	#wake enable
	devmem2 0x44e3e07c W 0x00
	
	
	 
	 
	 */
}
