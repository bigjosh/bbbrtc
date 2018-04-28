
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
	fprintf( stderr,"Dumping RTC:\n");
	
	for(int i=0; i<=0x98;i+=4) {
		
		fprintf( stdout ,"%02.2x %08.8x\n", i ,  *((unsigned long *) (base+i)) );
		
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
	fprintf( stderr,"Usage:\n");
	fprintf( stderr,"   rtcbb dump\n");	
	fprintf( stderr,"   rtcbb (sleep|wake|now) <new_time> \n");
	fprintf( stderr,"Where:\n");
	fprintf( stderr,"   'bbbrtc dump` dumps the contents of all registers to stdout\n");	
	fprintf( stderr,"   if <new_time> is present and non-zero, then the specified time is set\n");
	fprintf( stderr,"   time is in seconds since epoch, parsed leniently\n");
	fprintf( stderr,"   prints existing or newly set the value of the specified time\n");
	fprintf( stderr,"Notes:");
	fprintf( stderr,"   wake should always be later than sleep or you will sleep forever\n");
	fprintf( stderr,"   if you sleep without setting a wake then you must push the power button to wake\n");
	fprintf( stderr,"times:\n");
	fprintf( stderr,"   always specified in seconds since the epoch Jan 1, 1970\n");
	fprintf( stderr,"examples:\n");
	fprintf( stderr,"   set the rtc to the current system clock time...\n");
	fprintf( stderr,"      rtcbbb now set $(date +%%s)\n");
	fprintf( stderr,"   set the system clock to the current rtc time...\n");
	fprintf( stderr,"      date $(rtcbbb now get)\n");
	fprintf( stderr,"   set to sleep 10 seconds from now...\n");
	fprintf( stderr,"      rtcbbb sleep set $(date +%%s -d=\"$(rtcbbb now get)+(10 sec)\")\n");
	fprintf( stderr,"   set to wake 6 months after we go to sleep ...\n");
	fprintf( stderr,"      rtcbbb wake set $(date +%%s -d=\"$(rtcbbb sleep get)+(6 month)\")\n");
	fprintf( stderr,"notes:\n");
	fprintf( stderr,"   If you want to be able to wake from sleeping, you must turn off the 'off' bit\n");
	fprintf( stderr,"   in the PMIC controller with this command before sleeping..\n");
	fprintf( stderr,"   i2cset -f -y 0 0x24 0x0a 0x00\n");	
	fprintf( stderr,"\n");
	fprintf( stderr,"   The RTC doesn't know about timezones or DST, but as long as you are consistent\n");
	fprintf( stderr,"   and set both all the times using the same base, then all relatives times should work.\n");
}


enum clock_choice_t  { NONE , NOW, SLEEP , WAKE };

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

const char *clock_choice_name( clock_choice_t clock_choice ) {
	
	switch (clock_choice) {
		
		case NONE:
			return "NONE";		
		
		case NOW:
			return "NOW";
			
		case SLEEP:
			return "SLEEP";
			
		case WAKE:
			return "WAKE";
		
		
	}
		
}

int main(int argc, char **argv) {
		
	clock_choice_t clock_choice=NONE;
	unsigned new_time=0;
	
	fprintf( stderr,"argc=%d argv[1]=%s\n",argc,argv[1]);
		
	if (argc >= 2) {

		if (!strcasecmp( argv[1] , "now")) {
			clock_choice = NOW;
		} else if (!strcasecmp( argv[1] , "sleep")) {
			clock_choice = SLEEP;
		} else if (!strcasecmp( argv[1] , "wake" )) {
			clock_choice = WAKE;
		} 
		
		if (argc>= 3 ) {
			
			new_time = (unsigned) strtoul( argv[3] , NULL , 10 );
			
		} 
		
	}
	
	
	if ( clock_choice == NONE ) {			// Bad params
	
			showhelp();
			
	} else {
		
		// HERE IS THE BEEF
		
		int fd;
				
		fprintf( stderr,"Opening /dev/mem...");
		fflush(stdout);
		 
		if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
			
			fprintf( stderr,"Error opening /dev/mem:%s",strerror(errno));
			
		} else {
		
			fprintf( stderr,"opened.\n");

			fprintf( stderr,"Mappng in %p..." , RTC_SS_BASE );
			fflush(stdout);
			
			unsigned char *base;
					
			base = (unsigned char *) mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, RTC_SS_BASE );
			
			if( base == MAP_FAILED ) {
				
				fprintf( stderr ,"Error mmapping /dev/mem:%s",strerror(errno));
				
			} else {
				
				fprintf( stderr,"mapped at address %p.\n", base);	
				fflush(stdout);
				

				// We can not guarantee being able to access regs in the 15us not busy window under linux, so
				// instead we stop the whole RTC and the restart it when done.
				// We will drop some time each time, but not much. 
				
				
				fprintf( stderr,"Stopping RTC...");
				fflush(stdout);
				
				set32reg( base , RTC_CTRL_REG ,  0x00);		// Write a 0 to bit 0 to freeze the RTC so we can update regs
				
				fprintf( stderr,"waiting for stop...");
				fflush(stdout);
				
				do {
					unsigned int wait_counter=0;
				
					while ( get32reg( base , RTC_STATUS_REG)  & RTC_STATUS_RUN ) wait_counter++;
				
					fprintf( stderr,"took %u tries.\n",wait_counter);
					fflush(stdout); 
					
				} while (0);
				
				
				if (new_time) {			// Are we setting a new time?
				
					fprintf( stderr,"Setting %s to %u...\n",clock_choice_name( clock_choice) , new_time );									
					settime( base , clock_choice_off( clock_choice) , new_time );
					fprintf( stderr, "set.\n");

					
					if ( clock_choice == SLEEP ) {
						
						fprintf( stderr,"Enable PWR_ENABLE_EN to be controlled ON->OFF by ALARM2...\n");							
						set32reg( base , RTC_PMIC , RTC_PMIC_PWN_ENABLE_EN );
						fprintf( stderr, "enabled.\n");

						fprintf( stderr," Enable ALARM2 interrupt bit... \n");							
						set32reg( base , RTC_INTERRUPTS_REG , get32reg( base , RTC_INTERRUPTS_REG ) |  RTC_INTERRUPTS_IT_ALARM2 );
						fprintf( stderr, "enabled.\n");
						
						
					} else if (clock_choice == WAKE ) {
						
						fprintf( stderr,"Enable PWR_ENABLE_EN to be controlled OFF->ON by ALARM...\n");														
						set32reg( base , RTC_INTERRUPTS_REG , get32reg( base , RTC_INTERRUPTS_REG ) |  RTC_INTERRUPTS_IT_ALARM );
						fprintf( stderr, "enabled.\n");

						fprintf( stderr," Enable ALARM interrupt bit... \n");							
						set32reg( base , RTC_INTERRUPTS_REG , get32reg( base , RTC_INTERRUPTS_REG ) |  RTC_INTERRUPTS_IT_ALARM );
						fprintf( stderr, "enabled.\n");

						fprintf( stderr," Enable IRQ WAKE ENABLE bit... \n");													
						set32reg( base , RTC_IRQWAKEEN , RTC_IRQWAKEEN_ALARM );
						fprintf( stderr, "enabled.\n");							
						
					}
					
				}

				fprintf( stderr,"Reading %s and printing to stdout...\n",clock_choice_name( clock_choice)  );									
				printf( "%u\n" , gettime( base , clock_choice_off( clock_choice) ) );
				fprintf( stderr, "done.\n");

				
				fprintf( stderr,"Restarting RTC...");
				
				set32reg( base  , RTC_CTRL_REG ,  RTC_CTRL_STOP);		// Write a 1 to bit 0 to start the RTC
				
				fprintf( stderr, "waiting for it to start..." );
				
				do {
					
					unsigned int wait_counter=0;
				
					while ( !( get32reg( base , RTC_STATUS_REG)  & RTC_STATUS_RUN ) ) wait_counter++;
				
					fprintf( stderr,"took %u tries.\n",wait_counter);
					fflush(stdout); 
					
				} while (0);
				
							
				fprintf( stderr,"unmaping memory block...");

				munmap( base, MAP_SIZE );
				
				fprintf( stderr,"unmaped.\n");
				
			}
			
			fprintf( stderr,"closing fd...");		
			
			close(fd);	
			
			fprintf( stderr,"closed.\n");
			
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
