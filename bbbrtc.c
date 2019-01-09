
/*

    bbbrtc.c

    A utility for accessing the Real Time Clock inside the AM355x processor on the Beagle Bone Black computer.
    (c)2018 Josh Levine. http://josh.com

*/

// Linux 4+ locks the registers we need.
// Here is the magic for unlocking them.
#define RTC_KICK0_REG 0x6c
#define RTC_KICK1_REG 0x70
#define KICK0_VALUE 0x83e70b13
#define KICK1_VALUE 0x95a4f1e0

#define RTC_SS_BASE 0x44e3e000L             // Memory base for the RTC registers

// There are three different time "records" in the RTC that follow the same format

#define RTC_CURRENT_TIME_TM     0x00        // Offset to the current time registers

#define RTC_ALARM_TIME_TM       0x20        // Offset to the alarm time registers (wake time)

#define RTC_ALARM2_TIME_TM      0x80        // Offset to the alarm2 time registers (sleep time)


// Offsets into the fields of a time record

#define TM_SECONDS_OFF          0x00
#define TM_MINUTES_OFF          0x04
#define TM_HOURS_OFF            0x08
#define TM_DAYS_OFF             0x0C
#define TM_MONTHS_OFF           0x10
#define TM_YEARS_OFF            0x14


#define RTCALARMINT             76          // RTC alarm interrupt

#define RTC_CTRL_REG            0x40
#define RTC_CTRL_STOP           (1<<0)      // 1=Stop the RTC

#define RTC_STATUS_REG          0x44
#define RTC_STATUS_ALARM        (1<<6)      // Indicates ALARM has happened
#define RTC_STATUS_RUN          (1<<1)      // 0=frozen, 1=running

#define RTC_INTERRUPTS_REG      0x48
#define RTC_INTERRUPTS_IT_ALARM2 (1<<4)     // Enable interrupt on ALARM2
#define RTC_INTERRUPTS_IT_ALARM  (1<<3)     // Enable interrupt on ALARM
#define RTC_INTERRUPTS_IT_TIMER  (1<<2)     // Enable interrupt on TIMER

#define RTC_REVISION            0x74        // Chip revision code
#define RTC_REVISION_MAGIC      0x4eb01106U // Expected value

#define RTC_SYSCONFIG           0x78
#define RTC_IRQWAKEEN           0x7c
#define RTC_IRQWAKEEN_ALARM     (1<<1)      // Allow wakeup generation on ALARM
#define RTC_IRQWAKEEN_TIMER     (1<<0)      // Allow wakeup generation on TIMER

#define RTC_PMIC                0x98
#define RTC_PMIC_PWN_ENABLE_EN  (1<<16)
#define RTC_PMIC_EXT_WAKE_POL   (1<<4)      // 1=active low
#define RTC_PMIC_EXT_WAKE_EN    (1<<0)      // 1=external wake enabled

#define ASSIGN_WORD(w,v) ((*( (unsigned long *) (&(w)) ) ) = (v) )

#include <time.h>       // Get linux time conversion API functions
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>         // strerror
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdarg.h>         // vargs for diagprint

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

// Unlock RTC registers in 4.x+ kernels
void unlockrtcregs( unsigned char *base ) {
    set32reg( base, RTC_KICK0_REG, KICK0_VALUE);
    set32reg( base, RTC_KICK1_REG, KICK1_VALUE);
}


// From: https://stackoverflow.com/questions/150543/forward-an-invocation-of-a-variadic-function-in-c

void diagprint( const char *fmt, ... ) {

    /* Declare a va_list type variable */
    va_list myargs;

    /* Initialize the va_list variable with the ... after fmt */

    va_start(myargs, fmt);

    //fprintf( stderr , "bbbrtc:");

    /* Forward the '...' to vprintf */
    vfprintf( stderr , fmt, myargs);

    //fprintf( stderr , "\n");


    /* Clean up the va_list */
    va_end(myargs);
}

// Directly print the BCD registers for debuging

void printtimeregs( unsigned char *base, unsigned char time_offset  ) {
    printf("sec   %3.3x\n" , get32reg( base , time_offset + TM_SECONDS_OFF ) );
    printf("min   %3.3x\n" , get32reg( base , time_offset + TM_MINUTES_OFF ) );
    printf("hour  %3.3x\n" , get32reg( base , time_offset + TM_HOURS_OFF ) );
    printf("day   %3.3x\n" , get32reg( base , time_offset + TM_DAYS_OFF ) );
    printf("month %3.3x\n" , get32reg( base , time_offset + TM_HOURS_OFF ) );
    printf("year  %3.3x\n" , get32reg( base , time_offset + TM_YEARS_OFF ) );

}


unsigned gettime( unsigned char *base, unsigned char time_offset ) {

    struct tm time;

    time.tm_sec   = bcd2n( get32reg( base , time_offset + TM_SECONDS_OFF ));
    time.tm_min   = bcd2n( get32reg( base , time_offset + TM_MINUTES_OFF ));
    time.tm_hour  = bcd2n( get32reg( base , time_offset + TM_HOURS_OFF ));
    time.tm_mday  = bcd2n( get32reg( base , time_offset + TM_DAYS_OFF ));
    time.tm_mon   = bcd2n( get32reg( base , time_offset + TM_MONTHS_OFF ));
    time.tm_year  = bcd2n( get32reg( base , time_offset + TM_YEARS_OFF ) );

    if (time.tm_year <70 ) {
            time.tm_year += 100;    // Assume we are in the 2000's not 1900's after 1970 (RTC only has 2 year digits). Breaks in 2070. Come find me and I'll fix it then.
    }

    time_t readtime = mktime(&time);

    //diagprint( "read:%s\n", ctime(&readtime) );

    return readtime;

}


void settime( unsigned char *base, unsigned char time_offset , time_t newtime ) {

    struct tm *time = gmtime( &newtime );

    //diagprint( "setting:%s\n", ctime(&newtime));

    //diagprint(" tmyear = %d\n " , time->tm_year  );

    set32reg( base , time_offset + TM_SECONDS_OFF , n2bcd(time->tm_sec    ));
    set32reg( base , time_offset + TM_MINUTES_OFF , n2bcd( time->tm_min   ));
    set32reg( base , time_offset + TM_HOURS_OFF   , n2bcd( time->tm_hour  ));
    set32reg( base , time_offset + TM_DAYS_OFF    , n2bcd( time->tm_mday  ));
    set32reg( base , time_offset + TM_MONTHS_OFF  , n2bcd( time->tm_mon   ));
    set32reg( base , time_offset + TM_YEARS_OFF   , n2bcd( time->tm_year % 100)) ;  //  (RTC only has 2 year digits)


}


void showhelp() {
    diagprint( "Usage:\n");
    diagprint( "   bbbrtc dump\n");
    diagprint( "   bbbrtc now <new_time>\n");
    diagprint( "   bbbrtc (sleep|wake) (<new_time>|never)\n");
    diagprint( "\n");
    diagprint( "Where:\n");
    diagprint( "   'bbbrtc dump` dumps the contents of all registers to stdout\n");
    diagprint( "   if <new_time> is present and non-zero, then the specified time is set\n");
	diagprint( "   if never is specified with sleep or wake, the event is disabled (clock value is left the same)\n");
    diagprint( "   time is in seconds since epoch, parsed leniently\n");
    diagprint( "   prints existing or newly set the value of the specified time\n");
    diagprint( "\n");
    diagprint( "Notes:");
    diagprint( "   wake should always be later than sleep or you will sleep for 100 years\n");
    diagprint( "   if you sleep without setting a wake then you must push the power button to wake\n");
    diagprint( "\n");
    diagprint( "times:\n");
    diagprint( "   always specified in seconds since the epoch Jan 1, 1970\n");
    diagprint( "\n");
    diagprint( "examples:\n");
    diagprint( "   set the rtc to the current system clock time...\n");
    diagprint( "      bbbrtc now $(date +%%s)\n");
    diagprint( "   set the system clock to the current rtc time...\n");
    diagprint( "      date -s @$(bbbrtc now)\n");
    diagprint( "   set to sleep 10 seconds from now...\n");
    diagprint( "      bbbrtc sleep $[$(rtcbbb now)+10])\n");
    diagprint( "   set to wake 1 day after we go to sleep ...\n");
    diagprint( "       bbbrtc wake $(date +%%s -d \"$(date -d \"@$(bbbrtc sleep)\") + 1 day\")\n");
    diagprint( "   start a bbbrtc party...\n");
    diagprint( "      bbbrtc now 946684770\n");
    diagprint( "\n");
    diagprint( "notes:\n");
    diagprint( "   If you want to be able to wake from sleeping, you must turn off the 'off' bit\n");
    diagprint( "   in the PMIC controller with this command before going to sleep..\n");
    diagprint( "   i2cset -f -y 0 0x24 0x0a 0x00\n");
    diagprint( "\n");
    diagprint( "   The RTC doesn't know about timezones or DST, so best to use UTC time with it,\n");
    diagprint( "   but as long as you are consistent and set both all the times using the same base,\n");
    diagprint( "   then all relatives times should work.\n");
    diagprint( "\n");
    diagprint( "   It looks like my Debian calls hwclock --systohc sometime during startup,\n");
    diagprint( "   even when Linux has no way of knowing what time it is (it is hardcoded to\n");
    diagprint( "   Jun15 2016). This sadly which clobbers the time stored in the RTC, which \n");
    diagprint( "   could otherwise be correct and used to set the linux clock.\n");

    /*

    Take a quick nap for testing...

    i2cset -f -y 0 0x24 0x0a 0x00
    sync
    bbbrtc now 100 2>/dev/null
    bbbrtc sleep 105 2>/dev/null
    bbbrtc wake 110 2>/dev/null

    */

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

int dumpflag=0;     // Are we doing a dump?

int main(int argc, char **argv) {

    clock_choice_t clock_choice=NONE;

    const char *new_time=NULL;			// new time specified?

    if (argc >= 2) {

        if (!strcasecmp( argv[1] , "now")) {
            clock_choice = NOW;
        } else if (!strcasecmp( argv[1] , "sleep")) {
            clock_choice = SLEEP;
        } else if (!strcasecmp( argv[1] , "wake" )) {
            clock_choice = WAKE;
        } else if (!strcasecmp( argv[1] , "dump" )) {
            dumpflag=1;
        }

        if (argc>= 3 ) {

            new_time = argv[2];

        }

    }


    if ( clock_choice == NONE && !dumpflag ) {          // Bad params

            showhelp();

    } else {

        // HERE IS THE BEEF

        int fd;

        diagprint( "Opening /dev/mem...");
        fflush(stdout);

        if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {

            diagprint( "Error opening /dev/mem:%s",strerror(errno));

        } else {

            diagprint( "opened.\n");

            diagprint( "Mappng in %p..." , RTC_SS_BASE );
            fflush(stdout);

            unsigned char *base;

            base = (unsigned char *) mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, RTC_SS_BASE );

            if( base == MAP_FAILED ) {

                fprintf( stderr ,"Error mmapping /dev/mem:%s",strerror(errno));

            } else {

                diagprint( "mapped at address %p.\n", base);
                fflush(stdout);


                // We can not guarantee being able to access regs in the 15us not busy window under linux, so
                // instead we stop the whole RTC and the restart it when done.
                // We will drop some time each time, but not much.


                diagprint( "Stopping RTC...");
                fflush(stdout);

                // Unlock registers
                unlockrtcregs( base );

                set32reg( base , RTC_CTRL_REG ,  0x00);     // Write a 0 to bit 0 to freeze the RTC so we can update regs

                diagprint( "waiting for stop...");
                fflush(stdout);

                do {
                    unsigned int wait_counter=0;

                    while ( get32reg( base , RTC_STATUS_REG)  & RTC_STATUS_RUN ) wait_counter++;

                    diagprint( "took %u tries.\n",wait_counter);
                    fflush(stdout);

                } while (0);

                diagprint("Checking RTC chip rev...");

                if ( get32reg( base , RTC_REVISION ) != RTC_REVISION_MAGIC ) {

                    diagprint("WARNING: The revision on the RTC is %x, should be %x\n" , get32reg( base , RTC_REVISION ) , RTC_REVISION_MAGIC  );
                    diagprint("This program is only expected to work on an AM355x processor!\n");
                } else {
                    diagprint("checks good.\n");
                }

                if (dumpflag) {

                    dump(base);

                } else {

                    //printtimeregs( base , clock_choice_off(clock_choice ) );

                    if (new_time) {         // Are we setting a new time?


          						if (!strcasecmp( new_time , "never") ) {


          							//printtimeregs( base , clock_choice_off(clock_choice ) );


          							if ( clock_choice == SLEEP ) {

          								diagprint( "Disable ALARM2 interrupt bit... \n");
          								set32reg( base , RTC_INTERRUPTS_REG , get32reg( base , RTC_INTERRUPTS_REG ) &  ~RTC_INTERRUPTS_IT_ALARM2 );
          								diagprint( "disabled.\n");


          							} else if (clock_choice == WAKE ) {

          								diagprint( "Disable ALARM interrupt bit... \n");
          								set32reg( base , RTC_INTERRUPTS_REG , get32reg( base , RTC_INTERRUPTS_REG ) &  !RTC_INTERRUPTS_IT_ALARM );
          								diagprint( "disabled.\n");

          								diagprint( "Disable IRQ WAKE ENABLE bit... \n");
          								set32reg( base , RTC_IRQWAKEEN , get32reg( base , RTC_IRQWAKEEN ) & ~RTC_IRQWAKEEN_ALARM );
          								diagprint( "disabled.\n");

          							}

          						} else {		// Set new time...

          							unsigned set_value = (unsigned) strtoul( new_time , NULL , 10 );

          							diagprint( "Setting %s to %u...",clock_choice_name( clock_choice) , set_value );
          							settime( base , clock_choice_off( clock_choice) , set_value );
          							diagprint( "set.\n");

          							//printtimeregs( base , clock_choice_off(clock_choice ) );


          							if ( clock_choice == SLEEP ) {

          								diagprint( "Enable PWR_ENABLE_EN to be controlled ON->OFF by ALARM2...\n");
          								set32reg( base , RTC_PMIC , RTC_PMIC_PWN_ENABLE_EN );
          								diagprint( "enabled.\n");

          								diagprint( "Enable ALARM2 interrupt bit... \n");
          								set32reg( base , RTC_INTERRUPTS_REG , get32reg( base , RTC_INTERRUPTS_REG ) |  RTC_INTERRUPTS_IT_ALARM2 );
          								diagprint( "enabled.\n");


          							} else if (clock_choice == WAKE ) {

          								diagprint( "Enable PWR_ENABLE_EN to be controlled OFF->ON by ALARM...\n");
          								set32reg( base , RTC_PMIC , RTC_PMIC_PWN_ENABLE_EN );
          								diagprint( "enabled.\n");

          								diagprint( "Enable ALARM interrupt bit... \n");
          								set32reg( base , RTC_INTERRUPTS_REG , get32reg( base , RTC_INTERRUPTS_REG ) |  RTC_INTERRUPTS_IT_ALARM );
          								diagprint( "enabled.\n");

          								diagprint( "Enable IRQ WAKE ENABLE bit... \n");
          								set32reg( base , RTC_IRQWAKEEN , RTC_IRQWAKEEN_ALARM );
          								diagprint( "enabled.\n");

          							} else {

          								diagprint("WARNING: never does not really make sense here! Nothing done!\n");

          							}
          						}

                    }

                    diagprint( "Reading %s and printing to stdout...\n",clock_choice_name( clock_choice)  );
                    printf( "%u\n" , gettime( base , clock_choice_off( clock_choice) ) );
                    diagprint( "done.\n");
                }

                diagprint( "Restarting RTC...");

                set32reg( base  , RTC_CTRL_REG ,  RTC_CTRL_STOP);       // Write a 1 to bit 0 to start the RTC

                diagprint( "waiting for it to start..." );

                do {

                    unsigned int wait_counter=0;

                    while ( !( get32reg( base , RTC_STATUS_REG)  & RTC_STATUS_RUN ) ) wait_counter++;

                    diagprint( "took %u tries.\n",wait_counter);
                    fflush(stdout);

                } while (0);


                diagprint( "Unmaping memory block...");

                munmap( base, MAP_SIZE );

                diagprint( "unmaped.\n");

            }

            diagprint( "Closing fd...");

            close(fd);

            diagprint( "closed.\n");

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
