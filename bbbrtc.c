
/*

	bbbrtc.c
	
	A utility for accessing the Real Time Clock inside the AM355x processor on the Beagle Bone Black computer.
	(c)2018 Josh Levine. http://josh.com
	
*/


#define RTC_SS_BASE 0x44e3e000L				// Memory base for the RTC registers

// There are three different time "records" in the RTC that follow the same format

#define RTC_CURRENT_TIME_TM		0x00		// Offset to the current time registers

#define RTC_ALARM_TIME_TM		0x20		// Offset to the alarm time registers (wake time)

#define RTC_ALARM2_TIME_TM		0x80		// Offset to the alarm2 time registers (sleep time)


// Offsets into the fields of a time record

#define TM_SECONDS_OFF 			0x20
#define TM_MINUTES_OFF			0x24
#define TM_HOURS_OFF			0x28
#define TM_DAYS_OFF				0x2C
#define TM_MONTHS_OFF 			0x30
#define TM_YEARS_OFF			0x34


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


unsigned gettime( unsigned char *base, unsigned char time_offset ) {
	
	struct tm time;
	
	time.tm_sec   = get32reg( base , time_offset + TM_SECONDS_OFF );
	time.tm_min   = get32reg( base , time_offset + TM_MINUTES_OFF );
	time.tm_hour  = get32reg( base , time_offset + TM_HOURS_OFF );
	time.tm_mday  = get32reg( base , time_offset + TM_DAYS_OFF );
	time.tm_mon   = get32reg( base , time_offset + TM_MONTHS_OFF );
	time.tm_year  = get32reg( base , time_offset + TM_YEARS_OFF ) + 100;	// Assume we are in the 2000's not 1900's (RTC only has 2 year digits)
	
	return mktime( &time );
	
}


void settime( unsigned char *base, unsigned char time_offset , time_t newtime ) {
	
	struct tm *time = gmtime( &newtime );
		
	set32reg( base , time_offset + TM_SECONDS_OFF , time->tm_sec   );
	set32reg( base , time_offset + TM_MINUTES_OFF , time->tm_min);
	set32reg( base , time_offset + TM_HOURS_OFF , time->tm_hour );
	set32reg( base , time_offset + TM_DAYS_OFF , time->tm_mday   );
	set32reg( base , time_offset + TM_MONTHS_OFF , time->tm_mon);
	set32reg( base , time_offset + TM_YEARS_OFF , time->tm_year % 100) ;	//  (RTC only has 2 year digits)
		
}


void showhelp() {
	printf("Usage:\n");
	printf("   rtcbb (sleep|wake|now) (get|set <time>) \n");
	printf("Where:\n");
	printf("   set sets the specified value to <time>\n");
	printf("   get prints the specified value to stdout\n");
	printf("   time is in seconds since epoch, parsed leniently\n");
	printf("Notes:");
	printf("   wake should always be later than sleep\n");
	printf("   if you sleep without setting a wake then you must push the power button to wake\n");
	printf("   use 'bbbrtc dump` to dump the contents of all registers to stdout\n");
	printf("times:\n");
	printf("   always specified in seconds since the epoch Jan 1, 1970\n");
	printf("examples:\n");
	printf("   set the rtc to the current system clock time...\n");
	printf("      rtcbbb now set $(date +%%s)\n");
	printf("   set the system clock to the current rtc time...\n");
	printf("      date $(rtcbbb now get)\n");
	printf("   set to sleep 10 seconds from now...\n");
	printf("      rtcbbb sleep set $(date +%%s -d=\"$(rtcbbb now get)+(10 sec)\")\n");
	printf("   set to wake 6 months after we go to sleep ...\n");
	printf("      rtcbbb wake set $(date +%%s -d=\"$(rtcbbb sleep get)+(6 month)\")\n");
	printf("notes:\n");
	printf("   If you want to be able to wake from sleeping, you must turn off the 'off' bit\n");
	printf("   in the PMIC controller with this command before sleeping..\n");
	printf("   i2cset -f -y 0 0x24 0x0a 0x00\n");	
	printf("\n");
	printf("   The RTC doesn't know about timezones or DST, but as long as you are consistent\n");
	printf("   and set both all the times using the same base, then all relatives times should work.\n");
}

enum clock_choice_t  { CLOCK_NONE , NOW, SLEEP , WAKE };
enum action_choice_t { ACTION_NONE , GET , SET , DUMP };

// Returns the offset into the RTC base of the record to the selected clock

unsigned clock_choice_off( clock_choice_t clock_choice ) {
	
	switch (clock_choice) {
		
		case NOW:
			return RTC_CURRENT_TIME_TM;
			
		case SLEEP:
			return RTC_ALARM2_TIME_TM;
			
		case WAKE:
			return RTC_ALARM_TIME_TM;
		
		
	}
		
}

int main(int argc, char **argv) {
		
	clock_choice_t clock_choice;								// Pointer to the chosen clock record
	action_choice_t action_choice = ACTION_NONE;
	unsigned set_value;
	
	printf("argc=%d argv[1]=%s\n",argc,argv[1]);
	
	if (argc == 2 && !strcasecmp( argv[1] , "dump" ) ) {
		
		action_choice = DUMP;
		
	} else {
				
		if (argc==3 && !strcasecmp( argv[2] , "get")) {
			action_choice = GET;
		} else if (argc==4 &&  !strcasecmp( argv[1] , "set")) {
			action_choice = SET;
			set_value = (unsigned) strtoul( argv[3] , NULL , 10 );
		} 
		
		if (action_choice!= ACTION_NONE) {
			
			// Get target clock
			
			if (!strcasecmp( argv[1] , "now")) {
				clock_choice = NOW;
			} else if (!strcasecmp( argv[1] , "sleep")) {
				clock_choice = SLEEP;
			} else if (!strcasecmp( argv[1] , "wake" )) {
				clock_choice = WAKE;
			} else {
				printf("1st arg was '%s', must be either dump, now, sleep, or wake\n" , argv[2]);
				action_choice = ACTION_NONE;
			}								
		}			
	}
	
	
	if ( action_choice == ACTION_NONE ) {			// Bad params
	
			showhelp();
			
	} else {
		
		// HERE IS THE BEEF
		
		int fd;
				
		printf("Opening /dev/mem...");
		fflush(stdout);
		 
		if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
			
			fprintf(stderr,"Error opening /dev/mem:%s",strerror(errno));
			
		} else {
		
			printf("opened.\n");

			printf("Mappng in %p..." , RTC_SS_BASE );
			fflush(stdout);
			
			unsigned char *base;
					
			base = (unsigned char *) mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, RTC_SS_BASE );
			
			if( base == MAP_FAILED ) {
				
				fprintf(stderr,"Error mmapping /dev/mem:%s",strerror(errno));
				
			} else {
				
				printf("mapped at address %p.\n", base);	
				fflush(stdout);
				

				// We can not guarantee being able to access regs in the 15us not busy window under linux, so
				// instead we stop the whole RTC and the restart it when done.
				// We will drop some time each time, but not much. 
				
				
				printf("Stopping RTC...");
				fflush(stdout);
				
				set32reg( base , RTC_CTRL_REG ,  0x00);		// Write a 0 to bit 0 to freeze the RTC so we can update regs
				
				printf("waiting for stop...");
				fflush(stdout);
				
				do {
					unsigned int wait_counter=0;
				
					while ( get32reg( base , RTC_STATUS_REG)  & RTC_STATUS_RUN ) wait_counter++;
				
					printf("took %u tries.\n",wait_counter);
					fflush(stdout); 
					
				} while (0);


				
				switch (action_choice) {
					
					case DUMP:
						dump(base);
						break;
						
					case GET:
						gettime( base , clock_choice_off( clock_choice) );
						break;
						
					case SET:
						settime( base , clock_choice_off( clock_choice) , set_value );
											
						if ( clock_choice == SLEEP ) {
							
							printf("Enable PWR_ENABLE_EN to be controlled ON->OFF by ALARM2...\n");							
							set32reg( base , RTC_PMIC , RTC_PMIC_PWN_ENABLE_EN );
							printf( "enabled.\n");

							printf(" Enable ALARM2 interrupt bit... \n");							
							set32reg( base , RTC_INTERRUPTS_REG , get32reg( base , RTC_INTERRUPTS_REG ) |  RTC_INTERRUPTS_IT_ALARM2 );
							printf( "enabled.\n");
							
							
						} else if (clock_choice == WAKE ) {
							
							printf("Enable PWR_ENABLE_EN to be controlled OFF->ON by ALARM...\n");														
							set32reg( base , RTC_INTERRUPTS_REG , get32reg( base , RTC_INTERRUPTS_REG ) |  RTC_INTERRUPTS_IT_ALARM );
							printf( "enabled.\n");

							printf(" Enable ALARM interrupt bit... \n");							
							set32reg( base , RTC_INTERRUPTS_REG , get32reg( base , RTC_INTERRUPTS_REG ) |  RTC_INTERRUPTS_IT_ALARM );
							printf( "enabled.\n");

							printf(" Enable IRQ WAKE ENABLE bit... \n");													
							set32reg( base , RTC_IRQWAKEEN , RTC_IRQWAKEEN_ALARM );
							printf( "enabled.\n");							
							
						}
						
						break;
						
					default:
						printf("Huh? How'd that happen?\n");
						break;
						
				}
				
				
				printf("Restarting RTC...");
				
				set32reg( base  , RTC_CTRL_REG ,  RTC_CTRL_STOP);		// Write a 1 to bit 0 to start the RTC
				
				printf( "waiting for it to start..." );
				
				do {
					
					unsigned int wait_counter=0;
				
					while ( !( get32reg( base , RTC_STATUS_REG)  & RTC_STATUS_RUN ) ) wait_counter++;
				
					printf("took %u tries.\n",wait_counter);
					fflush(stdout); 
					
				} while (0);
				
							
				printf("unmaping memory block...");

				munmap( base, MAP_SIZE );
				
				printf("unmaped.\n");
				
			}
			
			printf("closing fd...");		
			
			close(fd);	
			
			printf("closed.\n");
			
		}
		
	}
	
	
	
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
