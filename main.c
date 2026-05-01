#include <avr/io.h>
#include <stdint.h>
#include "twi.h"
#include "oledm.h"
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include "text.h"
#include "terminus8x16_var1.h"

// Wiring
//
//                    +--------+
// ZÖLD - PB5 - N/C  -|        |- VCC - PIROS
//        PB3 - N/C  -| Tiny85 |- SCL - PB2 - SÁRGA
//        PB4 - N/C  -|        |- N/C - PB1 - NARANCS
//      BARNA - GND  -|        |- SDA - PB0 - KÉK
//                    +--------+

#define OLED_CONNECTED 1

#define OLEDM_INIT oledm_basic_init
void button_state_changed_pb4(uint8_t new_state);
void chest_opened();
void wdt_init(void);
int my_itoa(int num, char* str, int base);
void get_elapsed_time_string(uint32_t seconds, char* time_text);
void start_timer0(uint8_t timer);
void update_display();

struct OLEDM display;

struct Text text;

// --- PB4 (button) ---
uint8_t pb4_state = 0;
volatile uint8_t pb4_prev_state = 0;
volatile uint8_t pb4_state_changed = 0;
volatile uint8_t pb4_event = 0;        // raw event happened (interrupt detected)
volatile uint8_t pb4_changed_to_on = 0;      // stable change after debounce
volatile uint8_t pb4_changed_to_off = 0; 


// --- PB1 (reed switch) ---
uint8_t pb1_state = 0;
volatile uint8_t pb1_prev_state = 0;
volatile uint8_t pb1_state_changed = 0;
volatile uint8_t pb1_event = 0;        // raw event happened (interrupt detected)
volatile uint8_t pb1_changed = 0;      // stable change after debounce

volatile uint8_t button_pressed = 0;

volatile uint8_t sleep_mode = SLEEP_MODE_PWR_DOWN;

volatile uint32_t seconds_elapsed = 0;  // counts seconds
volatile uint8_t drift_counter = 0;
volatile uint8_t watchdog_fired = 0;

volatile uint8_t event_display_off = 0;

volatile uint32_t last_open_times[] = {0,0,0};
volatile uint8_t screen_content = 0;

void christmassMessage(){
	oledm_clear(&display, 0x00);
	text.column = 0;
	text.row = 0;
	text_char(&text, 250);
	text_char(&text, 251);
	text_char(&text, 252);
	text_char(&text, '\n');
	text_char(&text, 253);
	text_char(&text, 254);
	text_char(&text, 255);
	text_str(&text, "  Boldog\n   Karácsonyt");
	text_char(&text, 250);
	text_char(&text, 251);
	text_char(&text, 252);
	text_str(&text, "\n      2025   ");
	text_char(&text, 253);
	text_char(&text, 254);
	text_char(&text, 255);
	screen_content = 2;
}

void love_message()
{
	oledm_clear(&display, 0x00);
	text.column = 0;
	text.row = 0;
	text_str(&text, "\n       ");
	text_char(&text, 200);
	text_char(&text, 201);
	text_char(&text, 202);
	text_char(&text, 203);
	text_str(&text, " Ibi\n  Dani ");
	text_char(&text, 204);
	text_char(&text, 205);
	text_char(&text, 206);
	text_char(&text, 207);
	screen_content = 3;
}

uint8_t counter = 0;


//drift: in 15 tick(watchdog interrupt) add 13 seconds.
//In every <DRIFT_MODIFICATION> from <DRIFT_TICK_COUNT> modify second by <DRIFT_MODIFICATION_DIRECTION>
//In ever 13 tick from 15 add 1 additional second
#define DRIFT_TICK_COUNT 15 //tick
#define DRIFT_MODIFICATION 13 //seconds
#define DRIFT_MODIFICATION_DIRECTION 1

volatile uint8_t timer_debounce_counter = 0;
volatile uint16_t timer_oled_counter = 0;
uint8_t timers[2];
#define DEBOUNCE_TICKS  3     // 3×10ms = 30ms debounce
#define OLED_TIMEOUT_TICKS 500  // 500×10ms = 5 seconds
#define TIMER_OLED 0
#define TIMER_DEBOUNCE 1
#define TIMER_ON(timer) timers[timer] = 1
#define TIMER_OFF(timer) timers[timer] = 0
#define IS_TIMER_ON(timer) timers[timer]
#define IS_ANY_TIMER_ON (timers[0] || timers[1])
#define RESET_OLED_TIMER timer_oled_counter = 0


#define EVENT_WATCHDOG_FIRED 0
#define EVENT_PB4_ON 1
#define EVENT_PB4_OFF 2
#define EVENT_PB1_ON 3
#define EVENT_PB1_OFF 4
#define EVENT_DIPLAY_OFF 5
#define EVENT_NUMBER 6

volatile uint8_t events[EVENT_NUMBER];

#define SET_EVENT(event) events[event] = 1 
#define CLEAR_EVENT(event) events[event] = 0
#define GET_EVENT(event) events[event]

void init_events()
{
	for (int i=0; i<EVENT_NUMBER; i++)
	{
		events[i] = 0;
	}
}

uint8_t is_active_event()
{
	for (int i=0; i<EVENT_NUMBER; i++)
	{
		if (events[i] == 1) return 1;
	} 
	return 0;
}

int main() {

	//Oled Init
#ifdef OLED_CONNECTED
	OLEDM_INIT(&display);
	text_init(&text, terminus8x16_var1, &display);
	oledm_start(&display);
	oledm_display_on(&display);
	
	update_display();
#endif
	
	start_timer0(TIMER_OLED);
	
    // --- PB4 input (button) ---
    DDRB &= ~(1 << PB4);
    PORTB |= (1 << PB4);  // pull-up enabled

    // --- PB1 input (reed switch) ---
    DDRB &= ~(1 << PB1);
    PORTB |= (1 << PB1);  // pull-up enabled

	// PB3 output - DEBUG LED
	DDRB |= (1 << PB3);
	PORTB &= ~(1 << PB3); // PB3 OFF


	GIMSK |= (1 << PCIE);                // Enable pin change interrupt system
	PCMSK |= (1 << PCINT4) | (1 << PCINT1); // Enable PCINT for PB4 and PB1
	
	wdt_init();
	
	// --- Initialize previous states ---
    pb4_state = pb4_prev_state = (PINB & (1 << PB4)) ? 1 : 0;
    pb1_state = pb1_prev_state = (PINB & (1 << PB1)) ? 1 : 0;

    sei(); // enable global interrupts


    while (1)
    {

		//PORTB &= ~(1 << PB3); //Debug LED 
		set_sleep_mode(sleep_mode);
		sleep_enable();
		sleep_cpu();
		sleep_disable();
		
		while(is_active_event())
		{
			if (GET_EVENT(EVENT_PB4_ON)) {
				CLEAR_EVENT(EVENT_PB4_ON);
				PORTB |= (1 << PB3);  //Debug LED
				button_state_changed_pb4(0);
			}
			if (GET_EVENT(EVENT_PB4_OFF)){
				CLEAR_EVENT(EVENT_PB4_OFF);
				PORTB &= ~(1 << PB3); //Debug LED
			}
			if (GET_EVENT(EVENT_PB1_OFF)) {
				CLEAR_EVENT(EVENT_PB1_OFF);
			}
			if (GET_EVENT(EVENT_PB1_ON)) {
				CLEAR_EVENT(EVENT_PB1_ON);
				chest_opened();
			}
			if (GET_EVENT(EVENT_DIPLAY_OFF)){
				CLEAR_EVENT(EVENT_DIPLAY_OFF);
#ifdef OLED_CONNECTED
				oledm_display_off(&display);
#endif
				screen_content = 0;
			}
		}
    }
}

void display_on()
{
	RESET_OLED_TIMER;
	start_timer0(TIMER_OLED);
#ifdef OLED_CONNECTED
	oledm_display_on(&display);
#endif	
}

// ============================
// PB4 button handler
// ============================
void button_state_changed_pb4(uint8_t pb4_state)
{

	if (pb4_state == 0) //pressed
	{
		if (screen_content == 1){
			christmassMessage();
			display_on();
		}
		else if (screen_content == 2){
			love_message();
			display_on();
		}
		else {
			update_display();
			display_on();
		}
	}
	else // released
	{
	}
}

// ============================
// PB1 reed switch handler
// ============================
void chest_opened()
{
	if (last_open_times[0] == 0 || (seconds_elapsed - last_open_times[0] > 60)) {
		last_open_times[2] = last_open_times[1];
		last_open_times[1] = last_open_times[0];
		last_open_times[0] = seconds_elapsed;
	}
	update_display();
#ifdef OLED_CONNECTED
	oledm_display_on(&display);
#endif	
	start_timer0(TIMER_OLED);
}

// ============================
// Pin Change Interrupt (PB1 & PB4)
// ============================
ISR(PCINT0_vect)
{
	// Disable pin change interrupts during debounce
	//GIMSK &= ~(1 << PCIE);

	// Check PB4
	if (((PINB & (1 << PB4)) ? 1 : 0) != pb4_prev_state) {
		pb4_event = 1;
	}

	// Check PB1
	if (((PINB & (1 << PB1)) ? 1 : 0) != pb1_prev_state) {
		pb1_event = 1;
	}

	// Start Timer0 CTC for 10 ms debounce (same for both)
	start_timer0(TIMER_DEBOUNCE);

}

void start_timer0(uint8_t timer)
{
	if (!IS_ANY_TIMER_ON){
		OCR0A = 155;                        // ~10 ms at 1 MHz, prescaler 64
		TCCR0A = (1 << WGM01);              // CTC mode
		TCCR0B = (1 << CS01) | (1 << CS00); // prescaler 64
		TIMSK |= (1 << OCIE0A);             // enable compare interrupt

		sleep_mode = SLEEP_MODE_IDLE;
	}
	TIMER_ON(timer);
}
// ============================
// Timer0 Compare A ISR (debounce for PB4 & PB1)
// ============================
ISR(TIM0_COMPA_vect)
{
	if (IS_TIMER_ON(TIMER_DEBOUNCE)) {
		if (++timer_debounce_counter >= DEBOUNCE_TICKS) {
			timer_debounce_counter = 0;
			TIMER_OFF(TIMER_DEBOUNCE);

			// Handle PB4
			if (pb4_event) {
				uint8_t new_state = (PINB & (1 << PB4)) ? 1 : 0;
				if (new_state) SET_EVENT(EVENT_PB4_OFF);
				else SET_EVENT(EVENT_PB4_ON);
				pb4_event = 0;
				pb4_prev_state = new_state;
			}

			// Handle PB1
			if (pb1_event) {
				uint8_t new_state = (PINB & (1 << PB1)) ? 1 : 0;
				if (new_state) SET_EVENT(EVENT_PB1_OFF);
				else SET_EVENT(EVENT_PB1_ON);
				pb1_event = 0;
				pb1_prev_state = new_state;
			}

			// Re-enable pin change interrupts
			GIMSK |= (1 << PCIE);
		}
	}

	if (IS_TIMER_ON(TIMER_OLED)) {
		if (++timer_oled_counter >= OLED_TIMEOUT_TICKS) {
			SET_EVENT(EVENT_DIPLAY_OFF);
			timer_oled_counter = 0;
			TIMER_OFF(TIMER_OLED);
		}
	}
	if (!IS_ANY_TIMER_ON) {
		TIMSK &= ~(1 << OCIE0A);   // disable timer interrupt
		TCCR0B = 0;                // stop timer
		sleep_mode = SLEEP_MODE_PWR_DOWN;
	}

}

void wdt_init(void)
{
    wdt_reset();

    // Enable timed sequence to change WDT
    WDTCR |= (1 << WDCE) | (1 << WDE);

    // Set to interrupt mode only (WDE=0)
    // Set prescaler WDP3 + WDP0 = ~8s
    WDTCR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
}


// Watchdog interrupt handler
ISR(WDT_vect)
{
	drift_counter++;
	// Called every ~8 seconds
	seconds_elapsed += 8;
	if (drift_counter < DRIFT_MODIFICATION){
		seconds_elapsed += DRIFT_MODIFICATION_DIRECTION;
	}
	if (drift_counter == DRIFT_TICK_COUNT ) {
		drift_counter = 0;
	}
	watchdog_fired = 1;
}

int my_itoa(int num, char* str, int base) {
	int i = 0, isNegative = 0;

	if (num == 0) {
		str[i++] = '0';
		str[i] = '\0';
		return i;
	}

	if (num < 0 && base == 10) {
		isNegative = 1;
		num = -num;
	}

	while (num != 0) {
		int rem = num % base;
		str[i++] = (rem > 9)? (rem - 10) + 'a' : rem + '0';
		num = num / base;
	}

	if (isNegative) str[i++] = '-';

	str[i] = '\0';

	for (int j = 0, k = i - 1; j < k; j++, k--) {
		char temp = str[j];
		str[j] = str[k];
		str[k] = temp;
	}
	return i;
}

void get_elapsed_time_string(uint32_t seconds, char* time_text)
{
	#define SECONDS_ONE_MONTH (uint32_t)2592000
	#define SECONDS_THREE_DAYS (uint32_t)259200
	#define SECONDS_ONE_DAY (uint32_t)86400
	
	int len = 0;
	if (seconds >= SECONDS_ONE_MONTH)
	{
		strcpy_P(time_text, PSTR(">1 hónapja\n"));  //more than a month
	}
	else if (seconds >= SECONDS_THREE_DAYS) //3 days
	{
		uint16_t days = seconds / SECONDS_ONE_DAY; 
		len = my_itoa(days, time_text,10);
		strcpy_P(&time_text[len],PSTR(" napja\n"));
			
	}
	else if (seconds >= SECONDS_ONE_DAY) //1 day
	{
		uint16_t days = seconds / SECONDS_ONE_DAY;
		uint16_t partial_days =(seconds % SECONDS_ONE_DAY)/(SECONDS_ONE_DAY/10);
		len = my_itoa(days, time_text,10);
		if (partial_days >0){
			time_text[len++] = ',';
			len += my_itoa(partial_days, &time_text[len],10);
		}
		strcpy_P(&time_text[len],PSTR(" napja\n"));
			
	}
	else if (seconds >= 3600) //60 min
	{
		uint16_t minutes = seconds / 60;
		uint8_t hours = minutes / 60;
		minutes = minutes % 60;
		len = my_itoa(hours, time_text, 10);
		time_text[len++] = ':';
		if (minutes < 10) {
			time_text[len++] = '0';
		}
		len += my_itoa(minutes, &time_text[len], 10);
		strcpy_P(&time_text[len],PSTR(" órája\n"));
	}
	else if (seconds >= 60) // 60 sec
	{
		uint8_t minutes = seconds / (60);
		len = my_itoa(minutes, time_text, 10);
		strcpy_P(&time_text[len],PSTR(" perce\n"));
	}
	else {
		len = my_itoa(seconds, time_text,10);
		strcpy_P(&time_text[len],PSTR(" másodperce\n"));
	}

}

void update_display()
{
#ifdef OLED_CONNECTED
	char time_string[17];
		
	oledm_clear(&display, 0x00);
	
	text.column = 0;
	text.row = 0;
	text_str(&text, "Utolsó nyitások\n");
	
	if (last_open_times[0] != 0){
		get_elapsed_time_string(seconds_elapsed - last_open_times[0], time_string);
		text_str(&text, time_string);
	}
	if (last_open_times[1] != 0){
		get_elapsed_time_string(seconds_elapsed - last_open_times[1], time_string);
		text_str(&text, time_string);
	}
	if (last_open_times[2] != 0){
		get_elapsed_time_string(seconds_elapsed - last_open_times[2], time_string);
		text_str(&text, time_string);
	} 
	screen_content = 1;
#endif
}