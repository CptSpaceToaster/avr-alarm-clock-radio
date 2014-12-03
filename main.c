/* It's a Final Project
 * main.c
 * By: Guest and CptSpaceToaster */
#define BAUD (9600)
#define MYUBRR (F_CPU/16/BAUD-1) 
#define NOT_USING_PRINTF

#define NUMBER_OF_PRESETS     5
#define ALARM_SLEEP_TIME      10
#define NUMBER_OF_ADC_SOURCES 3
#define ADC_MUX_OFFSET        1
#define PHOTO_AVG_RESOLUTION  20
#define THERM_AVG_RESOLUTION  30
#define MAX_BRIGHTNESS        10000
#define USING_SI4705

#define DEBOUNCE(REG, PORT, PIN) REG=(REG<<1) | ((PORT & _BV(PIN)) >> PIN) | 0xF800
	
#include <stdlib.h>
#ifdef USING_PRINTF
#include <stdio.h>
#endif
#include <stdbool.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <inttypes.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#ifdef USING_PRINTF
#include "serial/uart.h"
#endif
#include "ds1307/ds1307.h"
#include "si4705/si4705.h"
#include "mcp4017/mcp4017.h"
#include "twi/i2cmaster.h"
#include "ipf/adc_setup.h"
#include "ipf/timer_setup.h"
#include "ipf/watchdog.h"
#include "nokia5110/nokia5110.h"
#include "avr-scheduler/avr-scheduler.h"

typedef enum STATE {
	home,
	menu,
	set_clock,
	set_alarm_A,
	set_alarm_B,
	alarm_A,
	alarm_B,
	radio,
	preset,
} STATE_t;

typedef enum EVENT_ID {
	no_event,
	start_ADC_conversion,
	get_time,
	get_rbds,
	get_status,
	//get_radio_channel,
	display_home_screen,
	display_temp_and_alarm_settings,
	display_rbds_information,
	display_radio_screen,
	display_radio_channel,
	beep_on,
	beep_off,
	radio_volume_change,
	button_up,
	button_down,
} EVENT_ID_t;

typedef enum CLOCK_PREFIX {
	months,
	days,
	years,
	hours,
	minutes,
	seconds,
} CLOCK_PREFIX_t;

#ifdef USING_PRINTF
void serial_init();
#endif
void set_data_directions();
void inline state_handler();
void inline home_state();
void inline menu_state();
void inline set_clock_state();
void inline set_alarm_state();
void inline alarm_state();
void inline preset_state();
void inline radio_state();
void check_schedule();
void set_volume(uint8_t volume);
void clear_radio_strings(void);
void set_brightness(uint8_t intensity);
uint8_t set_time_variable(CLOCK_PREFIX_t selection, time_t *input_time);
uint8_t set_preset_variable(uint8_t preset_num);
void watchdog_entertain(void);

// Variables used in interrupts
volatile uint16_t volume = 0;
volatile uint16_t ADC_values[NUMBER_OF_ADC_SOURCES];
volatile uint8_t ADC_index = 2;
volatile uint16_t dbnc[] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
volatile bool bns[] = {false, false, false, false, false};
volatile bool hol[] = {false, false, false, false, false};
volatile bool is_fresh = false;
volatile uint16_t ms_clock = 0;
volatile uint16_t wd_clock = 0;
volatile uint8_t i_index = 0;

// Other variables

uint8_t sb_vals[] = {0,0,0};
uint8_t sb_index = 0;
uint8_t menu_index = 0;
uint8_t text_index = 0;
uint8_t radio_activity = 0;
uint8_t err = 0;
uint8_t photo_avg_index = 0;
uint8_t therm_avg_index = 0;
uint32_t photo_avg = 255*PHOTO_AVG_RESOLUTION;
uint32_t temperature_avg = 700*THERM_AVG_RESOLUTION;
uint16_t sb_avg = 0;


bool init = true;
bool radio_is_on = false;
bool what_the_beep = false;
bool alarm_A_is_beep = false;
bool alarm_B_is_beep = true;
bool alarm_A_went_off_today = false;
bool alarm_B_went_off_today = false;
STATE_t state = home;
STATE_t last_state = home;
STATE_t init_detector = home;
uint16_t channel;
uint8_t rbds_pixel_offset=0;
uint16_t presets[NUMBER_OF_PRESETS];
uint16_t EEMEM saved_channel;
uint16_t EEMEM saved_presets[NUMBER_OF_PRESETS];
uint8_t EEMEM alarm_A_minutes;
uint8_t EEMEM alarm_A_hours;
uint8_t EEMEM alarm_A_tone_setting;
uint8_t EEMEM alarm_B_minutes;
uint8_t EEMEM alarm_B_hours;
uint8_t EEMEM alarm_B_tone_setting;
si4705_tune_status_t radio_tune_status;
si4705_rsq_status_t radio_rsq_status;

time_t time;
time_t time_alarm_A;
time_t time_alarm_B;
char line[13];
char scroll_text[66+DISPLAY_OFFSET]; // used to store the RBDS radio text           (rt)
char station_text[9];                // used to store the RBDS program service text (ps)
uint8_t length=0;
char *days_of_week[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
uint8_t time_limits[] = {12, 31, 100, 24, 60, 60};
uint8_t day_pixel_offsets[] = {14,14,10,3,7,14,7}; // Used to center the day of week text on the home screen
uint8_t days_in_month[] = {31,29,31,30,31,30,31,31,30,31,30,31};

char menu_options[12*5];

schedule_t *system_schedule;
event_t next_event;
bool event_is_ready = false;

int main(void) {
	watchdog_clear_status();
#ifdef USING_PRINTF
	serial_init();
#endif
	cli();
	clear_radio_strings();
	system_schedule = new_schedule();
	
	//Be aware: order may matter, especially with si4703/5_init and nokia5110_init.  (they all share a reset pin)
	watchdog_set(WD_INTERRUPT_AND_SYSTEM_RESET, WD_TIMEOUT_1_S);

	set_data_directions();
	timer0_pwm_prescaler_compare_A(0, 3, true, false);
	timer1_pwm_prescaler_compare(0, MAX_BRIGHTNESS, 1, 1, false, false);
	timer2_ctc(0.001, true);
	nokia5110_spi_init(0x51);
	i2c_init();
	nokia5110_power_on();
	set_volume(0);
	ADC_init(ADC_PC3, ADC_NO_TRIGGER, ADC_REFERENCE_1V1, true, true); //ADC_TIMER0_OVERFLOW
	sei();
	
	ds1307_getdate_s(&time);
		
	//If the clock isn't set, it reads all values as zero.  Set a default time
	if (time.year == 0) {
		ds1307_setdate(14,10,21,12,28,30);
	}
	
	time_alarm_A.hour = eeprom_read_byte(&alarm_A_hours);
	time_alarm_A.minute = eeprom_read_byte(&alarm_A_minutes);
	alarm_A_is_beep = eeprom_read_byte(&alarm_A_tone_setting);
	if (time_alarm_A.hour >= time_limits[hours]) {
		time_alarm_A.hour = 0;
		eeprom_update_byte(&alarm_A_hours, time_alarm_A.hour);
	}
	if (time_alarm_A.minute >= time_limits[minutes]) {
		time_alarm_A.minute = 0;
		eeprom_update_byte(&alarm_A_minutes, time_alarm_A.minute);
	}
	if (alarm_A_is_beep > true) {
		alarm_A_is_beep = true;
	}
	
	time_alarm_B.hour = eeprom_read_byte(&alarm_B_hours);
	time_alarm_B.minute = eeprom_read_byte(&alarm_B_minutes);
	alarm_B_is_beep = eeprom_read_byte(&alarm_B_tone_setting);
	if (time_alarm_B.hour >= time_limits[hours]) {
		time_alarm_B.hour = 0;
		eeprom_update_byte(&alarm_B_hours, time_alarm_B.hour);
	}
	if (time_alarm_B.minute >= time_limits[minutes]) {
		time_alarm_B.minute = 0;
		eeprom_update_byte(&alarm_B_minutes, time_alarm_B.minute);
	}
	if (alarm_B_is_beep > true) {
		alarm_B_is_beep = true;
	}
	
	channel = eeprom_read_word(&saved_channel);
	if (channel < SI4705_FM_LOW || channel > SI4705_FM_HIGH) channel = 889;
	
	for (err = 0; err < NUMBER_OF_PRESETS; err++) {
		presets[err] = eeprom_read_word(&saved_presets[err]);
		if (presets[err] < SI4705_FM_LOW || presets[err] > SI4705_FM_HIGH) presets[err] = 945;
	}
	
	ms_clock = 0;
	while(true) {
		if (is_fresh) {
			is_fresh = false;
			check_schedule();
			switch(state) {
				case home:
					home_state(); break;
				case menu: 
					menu_state(); break;
				case set_clock:
					set_clock_state(); break;
				case set_alarm_A:
				case set_alarm_B:
					set_alarm_state(); break;
				case alarm_A:
				case alarm_B:
					alarm_state(); break;
				case radio:
					radio_state(); break;
				case preset:
					preset_state(); break;
				default: break;
			}
			init = false;
			state_handler();
			watchdog_entertain();
		}
	}
}

void inline state_handler() {
	if (hol[3] && //switch is set to true
		time.second == 0 && 
		time.minute == time_alarm_A.minute &&
		time.hour == time_alarm_A.hour ) {
		state = alarm_A;
	}
	if (hol[4] && //switch is set to true
		time.second == 0 && 
		time.minute == time_alarm_B.minute &&
		time.hour == time_alarm_B.hour ) {
		state = alarm_B;
	}
	if (alarm_A_went_off_today || !hol[3]) {
		time_alarm_A.hour = eeprom_read_byte(&alarm_A_hours);
		time_alarm_A.minute = eeprom_read_byte(&alarm_A_minutes);
		alarm_A_went_off_today = false;
	}
	if (alarm_B_went_off_today || !hol[4]) {
		time_alarm_B.hour = eeprom_read_byte(&alarm_B_hours);
		time_alarm_B.minute = eeprom_read_byte(&alarm_B_minutes);
		alarm_B_went_off_today = false;
	}
	init = (init_detector != state); //detects when a state is about to change
	if (init) {
		if (init_detector != menu) {
			last_state = init_detector;
		}
		schedule_clear(system_schedule);
		wd_clock = 0;		
	}
	init_detector = state;
}

void inline home_state() {
	if (init) {
		schedule_insert(system_schedule, 0, ms_clock, get_time);
		schedule_insert(system_schedule, 0, ms_clock, display_home_screen);
		schedule_insert(system_schedule, 0, ms_clock, display_temp_and_alarm_settings);
		schedule_insert(system_schedule, 0, ms_clock, start_ADC_conversion);
		
		nokia5110_clear();
		if (radio_is_on) {
			si4705_power_off();
			radio_is_on = false;
		}
		what_the_beep = false;
		set_volume(0);
		
		#ifndef USING_PRINTF
		PORTD |= _BV(0); // Turn the radio LED off
		PORTD |= _BV(1); // Turn the stereo status LED off
		#endif
	}
	if(bns[0]) {
		bns[0] = false;
	}
	if(bns[1]) {
		bns[1] = false;
		state = menu;
	}
	if(bns[2]) {
		bns[2] = false;
	}
	if (event_is_ready) {
		switch(next_event.id) {
			case start_ADC_conversion:
				ADC_start_conversion();
				set_brightness(photo_avg/PHOTO_AVG_RESOLUTION);
				schedule_insert(system_schedule, 20, ms_clock, start_ADC_conversion);
				break;
		
			case get_time:
				ds1307_getdate_s(&time);
				err = ds1307_getdayofweek(time.year, time.month, time.day);
				schedule_insert(system_schedule, 1000, ms_clock, get_time);
				//printf("%d:%d:%d\n", time.hour, time.minute, time.second);
				break;
		
			case display_home_screen:
				snprintf(line, 12, "%02d:%02d", time.hour - 12*(time.hour>12), time.minute);
				nokia5110_gotoXY(12,1);
				nokia5110_writeString_megaFont(line);
		
				snprintf(line, 12, " %s  ", days_of_week[err]);
				nokia5110_gotoXY(day_pixel_offsets[err],0);
				nokia5110_writeString(line);
		
				snprintf(line, 12, "%s", time.hour<12?"am":"pm");
				nokia5110_gotoXY(68,3);
				nokia5110_writeString(line);
		
				snprintf(line, 9, "%d/%d/%d   ", time.month, time.day, time.year);
				nokia5110_gotoXY(0,4);
				nokia5110_writeString(line);
		
				schedule_insert(system_schedule, 1000, ms_clock, display_home_screen);
				break;
		
			case display_temp_and_alarm_settings:
				init = false;
				snprintf(line, 8, "%02d.%01d%cF  ", (uint8_t)(temperature_avg/THERM_AVG_RESOLUTION/10), (uint8_t)(temperature_avg/THERM_AVG_RESOLUTION%10), 123);
				nokia5110_gotoXY(0,5);
				nokia5110_writeString(line);
			
				nokia5110_gotoXY(63,4);
				if (hol[3]) {
					snprintf(line, 4, "A:%c", alarm_A_is_beep?124+(ADC_values[0]>80)+(ADC_values[0]>180):127);
				} else {
					snprintf(line, 4, "A: ");
				}
				nokia5110_writeString(line);
				nokia5110_gotoXY(63,5);
				if (hol[4]) {
					snprintf(line, 4, "B:%c", alarm_B_is_beep?124+(ADC_values[0]>80)+(ADC_values[0]>180):127);
				} else {
					snprintf(line, 4, "B: ");
				}
				nokia5110_writeString(line);
				schedule_insert(system_schedule, 100, ms_clock, display_temp_and_alarm_settings);
				break;
			
			default:
				break;
		}
	}
	watchdog_feed();
}

void inline menu_state() {
	if (init) {
		schedule_insert(system_schedule, 0, ms_clock, start_ADC_conversion);
		
		menu_index = 0;
		nokia5110_clear();
		nokia5110_gotoXY(0,0);
		if (last_state == home) {
			nokia5110_writeString(">Radio");
		} else {
			nokia5110_writeString(">Radio Off");
		}
		nokia5110_gotoXY(0,1);
		nokia5110_writeString(" Time-Set");
		nokia5110_gotoXY(0,2);
		nokia5110_writeString(" Set AlarmA");
		nokia5110_gotoXY(0,3);
		nokia5110_writeString(" Set AlarmB");
		nokia5110_gotoXY(0,4);
		if (last_state == home) {
			nokia5110_writeString(" Set Presets");
		} else {
			nokia5110_writeString(" FM Presets");
		}
		
		nokia5110_gotoXY(0,5);
		nokia5110_writeString(" Back");
	}
	if (bns[0]) {
		bns[0] = false;
		nokia5110_gotoXY(0, menu_index);
		nokia5110_writeString(" ");
		menu_index = (menu_index+1) % 6;
		nokia5110_gotoXY(0, menu_index);
		nokia5110_writeString(">");
		wd_clock = 0; //reset activity
		watchdog_feed();
	}
	if(bns[1]) {
		bns[1] = false;
		switch(menu_index) {
			case 0:
				if (last_state == home) {
					state = radio; 
				} else {
					state = home; 
				} 
				break;
			case 1:
				state = set_clock; break;
			case 2:
				state = set_alarm_A; break;
			case 3:
				state = set_alarm_B; break;
			case 4:
				state = preset; break;
			default: 
				state = last_state; break;
		}	
	}
	if (bns[2]) {
		bns[2] = false;
		nokia5110_gotoXY(0, menu_index);
		nokia5110_writeString(" ");
		menu_index = (menu_index+5) % 6;
		nokia5110_gotoXY(0, menu_index);
		nokia5110_writeString(">");
		wd_clock = 0; //reset activity
		watchdog_feed();
	}
	if (event_is_ready) {
		switch(next_event.id) {
			case start_ADC_conversion:
				ADC_start_conversion();
				set_brightness(photo_avg/PHOTO_AVG_RESOLUTION);
				schedule_insert(system_schedule, 20, ms_clock, start_ADC_conversion);
				break;
			
			default:
				break;
		}
	}
	if(wd_clock == 60) {
		wd_clock = 0;
		state = last_state;
	}
}

// This state is blocking, and can't be interrupted by other states
void inline set_clock_state() {
	nokia5110_clear();
	err = 0;
	if (!err) err |= set_time_variable(months, &time);
	if (!err) err |= set_time_variable(days, &time);
	if (!err) err |= set_time_variable(years, &time);
	if (!err) err |= set_time_variable(hours, &time);
	if (!err) err |= set_time_variable(minutes, &time);
	if (!err) err |= set_time_variable(seconds, &time);
	if (!err) {
		ds1307_setdate_s(time); // update time
	}
	state = last_state;
}

// This state is blocking, and can't be interrupted by other states
void inline set_alarm_state() {
	nokia5110_clear();
	nokia5110_gotoXY(3,0);
	nokia5110_writeString("Set Alarm "); 
	nokia5110_writeChar(state == set_alarm_A?'A':'B');
	nokia5110_gotoXY(0,4);
	nokia5110_writeString("[ ]Tone");
	nokia5110_gotoXY(0,5);
	nokia5110_writeString("[ ]Radio");
	
	err = 0;
	
	time_t *ptr;
	if (state == set_alarm_A) {
		time_alarm_A.hour = eeprom_read_byte(&alarm_A_hours);
		time_alarm_A.minute = eeprom_read_byte(&alarm_A_minutes);
		ptr = &time_alarm_A;
	} else {
		time_alarm_B.hour = eeprom_read_byte(&alarm_B_hours);
		time_alarm_B.minute = eeprom_read_byte(&alarm_B_minutes);
		ptr = &time_alarm_B;
	}
	if (!err) err |= set_time_variable(hours, ptr);
	if (!err) err |= set_time_variable(minutes, ptr);
	if (!err) {
		if ((state == set_alarm_A && alarm_A_is_beep) || (state == set_alarm_B && alarm_B_is_beep)) {
			nokia5110_gotoXY(7,4);
			nokia5110_writeChar(126);
		} else {
			nokia5110_gotoXY(7,5);
			nokia5110_writeChar(127);
		}
		wd_clock = 0; // reset activity
		while(!bns[1] && (wd_clock <= 60)) {
			if (bns[0] || bns[2]) {
				if ((state == set_alarm_A && alarm_A_is_beep) || (state == set_alarm_B && alarm_B_is_beep)) {
					nokia5110_gotoXY(7,4);
					nokia5110_writeChar(' ');
				} else {
					nokia5110_gotoXY(7,5);
					nokia5110_writeChar(' ');
				}
				bns[0] = bns[2] = false;
				alarm_A_is_beep ^= (state == set_alarm_A);
				alarm_B_is_beep ^= (state == set_alarm_B);
				if ((state == set_alarm_A && alarm_A_is_beep) || (state == set_alarm_B && alarm_B_is_beep)) {
					nokia5110_gotoXY(7,4);
					nokia5110_writeChar(126);
				} else {
					nokia5110_gotoXY(7,5);
					nokia5110_writeChar(127);
				}
			}
			watchdog_entertain();
		}
		if (bns[1]) {
			bns[1] = false;
		} else {
			err = 1;
		}
	}
	if (!err) {
		if (state == set_alarm_A) {
			//alarm_A_enabled = true;
			eeprom_update_byte(&alarm_A_hours, ptr->hour);
			eeprom_update_byte(&alarm_A_minutes, ptr->minute);
			eeprom_update_byte(&alarm_A_tone_setting, alarm_A_is_beep);
		} else {
			//alarm_B_enabled = true;
			eeprom_update_byte(&alarm_B_hours, ptr->hour);
			eeprom_update_byte(&alarm_B_minutes, ptr->minute);
			eeprom_update_byte(&alarm_B_tone_setting, alarm_B_is_beep);
		}
	}
	
	state = last_state;
}

void inline alarm_state() {
	if (init) {
		nokia5110_clear();
		nokia5110_drawSplash();
		
		schedule_insert(system_schedule, 0, ms_clock, get_time);
		schedule_insert(system_schedule, 0, ms_clock, start_ADC_conversion);
		
		if (state == alarm_A) {
			if (alarm_A_is_beep) {
				radio_is_on = false;
				what_the_beep = true;
				schedule_insert(system_schedule, 250, ms_clock, beep_off);
			} else {
				#ifndef USING_PRINTF
				PORTD &= ~_BV(0); // Turn the radio LED on
				#endif
				
				what_the_beep = false;
				radio_is_on = true;
				si4705_power_on();
				si4705_set_channel(channel);
				schedule_insert(system_schedule, 100, ms_clock, radio_volume_change);
			}
		} else {
			if (alarm_B_is_beep) {
				radio_is_on = false;
				what_the_beep = true;
				schedule_insert(system_schedule, 250, ms_clock, beep_off);
			} else {
				#ifndef USING_PRINTF
				PORTD &= ~_BV(0); // Turn the radio LED on
				#endif
				
				what_the_beep = false;
				radio_is_on = true;
				si4705_power_on();
				si4705_set_channel(channel);
				schedule_insert(system_schedule, 100, ms_clock, radio_volume_change);
			}
		}
		err = 25;
		set_volume(25);
		wd_clock = 0;
	}
	if(bns[0] || bns[2]) { // sleep
		//TODO: alarm again after a length of time
		bns[0] = bns[2] = false;
		if (state==alarm_A) {
			time_alarm_A.hour = time.hour;
			time_alarm_A.minute = time.minute + ALARM_SLEEP_TIME;
			if (time_alarm_A.minute >= 60) {
				time_alarm_A.minute -= 60;
				time_alarm_A.hour += 1;
				if (time_alarm_A.hour >= 24) {
					time_alarm_A.hour -= 24;
				}
			}
		}
		if (state==alarm_B) {
			time_alarm_B.hour = time.hour;
			time_alarm_B.minute = time.minute + ALARM_SLEEP_TIME;
			if (time_alarm_B.minute >= 60) {
				time_alarm_B.minute -= 60;
				time_alarm_B.hour += 1;
				if (time_alarm_B.hour >= 24) {
					time_alarm_B.hour -= 24;
				}
			}
		}
		state = last_state;
	}
	if(bns[1] || ((state == alarm_A) && !hol[3]) || ((state == alarm_B) && !hol[4])) { // silence....... or did we have that backwards
		bns[1] = false;
		if (state==alarm_A) {
			alarm_A_went_off_today = true;
		}
		if (state==alarm_B) {
			alarm_B_went_off_today = true;
		}
		state = last_state;
	}
	if (event_is_ready) {
		switch(next_event.id) {
			case start_ADC_conversion:
				ADC_start_conversion();
				set_brightness(photo_avg/PHOTO_AVG_RESOLUTION);
				schedule_insert(system_schedule, 20, ms_clock, start_ADC_conversion);
				break;
			
			case get_time:
				ds1307_getdate_s(&time);
				schedule_insert(system_schedule, 1000, ms_clock, get_time);
				break;
			
			case radio_volume_change:
				if (err < 175 || err < ADC_values[0]) {
					err ++;
					set_volume(err);
				} else {
					if (ADC_values[0] > 175) {
						set_volume(ADC_values[0]);
					} else {
						set_volume(175);
					}
				}
				schedule_insert(system_schedule, 100, ms_clock, radio_volume_change);
				break;
			
			case beep_off:
				set_volume(0);
				schedule_insert(system_schedule, 500, ms_clock, beep_on);
				break;
				
			case beep_on:
				if (err < 175 || err < ADC_values[0]) {
					err += 5;
					set_volume(err);
				} else {
					if (ADC_values[0] > 175) {
						set_volume(ADC_values[0]);
					} else {
						set_volume(175);
					}
				}
				schedule_insert(system_schedule, 250, ms_clock, beep_off);
				break;
			
			default:
				break;
		}
	}
	//todo, volume control, stop the ramp speed, set the limit
	if (wd_clock == 3599) {
		if (state==alarm_A) {
			alarm_A_went_off_today = true;
		}
		if (state==alarm_B) {
			alarm_B_went_off_today = true;
		}
		state = last_state;
	}
}

void inline preset_state() {
	if (init) {
		schedule_insert(system_schedule, 0, ms_clock, start_ADC_conversion);
		
		menu_index = 0;
		nokia5110_clear();
		
		for (err = 0; err < NUMBER_OF_PRESETS; err++) {
			nokia5110_gotoXY(0, err);
			snprintf(line, 12, " %u.%u", presets[err]/10, presets[err]%10);
			nokia5110_writeString(line);
		}
		nokia5110_gotoXY(0,0);
		nokia5110_writeString(">");
		nokia5110_gotoXY(0,5);
		nokia5110_writeString(" Back");
	}
	if (bns[0]) {
		bns[0] = false;
		nokia5110_gotoXY(0, menu_index);
		nokia5110_writeString(" ");
		menu_index = (menu_index+1) % 6;
		nokia5110_gotoXY(0, menu_index);
		nokia5110_writeString(">");
		wd_clock = 0; // reset activity
		watchdog_feed();
	}
	if (bns[1]) {
		bns[1] = false;
		if(menu_index == 5) {
			state = last_state;
		} else {
			// choose preset
			if (last_state == home) {
				nokia5110_gotoXY(0,menu_index);
				nokia5110_writeString(" ");
				err = set_preset_variable(menu_index);
				if (err) {
					state = last_state;
				}
				nokia5110_gotoXY(0,menu_index);
				snprintf(line, 12, ">%u.%u", presets[menu_index]/10, presets[menu_index]%10);
				nokia5110_writeString(line);
			} else {	
				if (channel != presets[menu_index]) {
					channel = presets[menu_index];
					si4705_set_channel(channel);
					clear_radio_strings();
				}
				state = radio;
			}
		}
	}
	if (bns[2]) {
		bns[2] = false;
		nokia5110_gotoXY(0, menu_index);
		nokia5110_writeString(" ");
		menu_index = (menu_index+5) % 6;
		nokia5110_gotoXY(0, menu_index);
		nokia5110_writeString(">");
		wd_clock = 0; //reset activity
		watchdog_feed();
	}
	if (event_is_ready) {
		switch(next_event.id) {
			case start_ADC_conversion:
				ADC_start_conversion();
				set_brightness(photo_avg/PHOTO_AVG_RESOLUTION);
				schedule_insert(system_schedule, 20, ms_clock, start_ADC_conversion);
				break;
			
			default:
				break;
		}
	}
	if(wd_clock == 60) {
		wd_clock = 0;
		state = last_state;
	}
}

void inline radio_state() {
	if (init) {
		schedule_insert(system_schedule, 0, ms_clock, get_time);
		schedule_insert(system_schedule, 0, ms_clock, get_rbds);
		schedule_insert(system_schedule, 0, ms_clock, get_status);
		schedule_insert(system_schedule, 0, ms_clock, display_radio_screen);
		schedule_insert(system_schedule, 0, ms_clock, display_radio_channel);
		schedule_insert(system_schedule, 0, ms_clock, display_rbds_information);
		schedule_insert(system_schedule, 0, ms_clock, start_ADC_conversion);
		
		if (!radio_is_on) {		
			si4705_power_on();
			si4705_set_channel(channel);
			set_volume(ADC_values[0]);
		}
		radio_is_on = true;
		what_the_beep = false;
		nokia5110_clear();
		wd_clock = 0; //second counter
		#ifndef USING_PRINTF
		PORTD &= ~_BV(0); // Turn the radio LED on
		PORTD |= _BV(1); // Turn the stereo status LED off
		#endif
	}
	if(bns[0]) {
		bns[0] = false;
		if (radio_activity == 1) {
			channel -= 2;
			if (channel%2 == 0) {
				channel += 1;
			}
			if (channel < SI4705_FM_LOW) channel = SI4705_FM_HIGH;
			si4705_set_channel(channel);
			clear_radio_strings();
		}
		radio_activity = 1;
		wd_clock = 0; //reset activity
		schedule_insert(system_schedule, 200, ms_clock, button_down);
	}
	if(bns[1]) {
		bns[1] = false;
		eeprom_update_word(&saved_channel, channel);
		state = menu;
	}
	if(bns[2]) {
		//printf("I was pressed up\n");
		bns[2] = false;
		if (radio_activity == 2) {
			channel += 2;
			if (channel%2 == 0) {
				channel -= 1;
			}
			if (channel < SI4705_FM_LOW) channel = SI4705_FM_HIGH;
			si4705_set_channel(channel);
			clear_radio_strings();
		}
		radio_activity = 2;
		wd_clock = 0; //reset activity
		////printf("event scheduled\n");
		schedule_insert(system_schedule, 200, ms_clock, button_up);
	}
	if (event_is_ready) {
		switch(next_event.id) {
			case start_ADC_conversion:
				ADC_start_conversion();
				set_brightness(photo_avg/PHOTO_AVG_RESOLUTION);
				set_volume(ADC_values[0]);
				schedule_insert(system_schedule, 20, ms_clock, start_ADC_conversion);
				break;
			
			case get_time:
				ds1307_getdate_s(&time);
				schedule_insert(system_schedule, 1000, ms_clock, get_time);
				//printf("%d:%d:%d\n", time.hour, time.minute, time.second);
				break;
			
			case get_rbds:
				si4705_get_rdbs(station_text, scroll_text);
				schedule_insert(system_schedule, 200, ms_clock, get_rbds);
				break;
			
			case get_status:
				si4705_get_tune_status(&radio_tune_status);
				si4705_get_rsq_status(&radio_rsq_status);
				channel = radio_tune_status.tuneFrequency;
				//printf("%u\n", channel);
				schedule_insert(system_schedule, 100, ms_clock, get_status);
				break;
			
			case display_radio_screen:
				nokia5110_gotoXY(14,0);
				snprintf(line, 10, "%02d:%02d %s", time.hour - 12*(time.hour>12), time.minute, time.hour<12?"am":"pm");
				nokia5110_writeString(line);
				
				schedule_insert(system_schedule, 1000, ms_clock, display_radio_screen);
				break;
			
			case display_radio_channel:
				nokia5110_gotoXY(8-8*(channel>=1000),1);
				snprintf(line, 8, " %d.%d ", channel/10, channel%10);
				nokia5110_writeString_megaFont(line);
				
				nokia5110_gotoXY(0, 2);
				snprintf(line, 2, "%c", 124+(ADC_values[0]>80)+(ADC_values[0]>180));
				nokia5110_writeString(line);
				
				nokia5110_gotoXY(0, 3);
				snprintf(line, 2, "%c", 128 + (radio_rsq_status.rssi>22) + (radio_rsq_status.rssi>35));
				nokia5110_writeString(line);
				
				nokia5110_gotoXY(70, 3);
				snprintf(line, 3, "%s", radio_rsq_status.pilot?"st":"  ");
				nokia5110_writeString(line);
				
				#ifndef USING_PRINTF
				if (radio_rsq_status.pilot) {
					PORTD &= ~(_BV(1)); // Turn the stereo status on
				} else {
					PORTD |= _BV(1); // Turn the stereo status off
				}
				#endif
				
				schedule_insert(system_schedule, 100, ms_clock, display_radio_channel);
				break;
			
			case display_rbds_information:
				rbds_pixel_offset = (rbds_pixel_offset+1) % 7;
				if (rbds_pixel_offset == 0) {
					text_index = (text_index + 1) % (strlen(scroll_text));
				}
				if (strlen(scroll_text) > DISPLAY_OFFSET+1) { // we have a radio string
					nokia5110_gotoXY(0,5);
					nokia5110_writeString_L(&scroll_text[text_index], rbds_pixel_offset);
					//nokia5110_writeString_C(&scroll_text[text_index]);
				} else if (strlen(station_text) > 0) { // we don't have a radio string, but we have program service!
					snprintf(line, 12, "%s            ", station_text);
					nokia5110_gotoXY(14,5);
					nokia5110_writeString_C(line);
				} else { // we got nothing
					nokia5110_gotoXY(0,5);
					nokia5110_writeString("            ");
				}
				
				schedule_insert(system_schedule, 100, ms_clock, display_rbds_information);
				break;
			
			case button_down:
				if (hol[0]) {
					si4705_seek(DOWN);
					clear_radio_strings();
				} else {
					channel -= 2;
					if (channel%2 == 0) {
						channel += 1;
					}
					if (channel < SI4705_FM_LOW) channel = SI4705_FM_HIGH;
					si4705_set_channel(channel);
					clear_radio_strings();
				}
				radio_activity = 0;
				break;
			
			case button_up:
				//printf("handling event - ");
				if (hol[2]) {
					//printf("you were holding\n");
					si4705_seek(UP);
					clear_radio_strings();
				} else {
					//printf("you were tapping\n");
					channel += 2;
					if (channel%2 == 0) {
						channel -= 1;
					}
					if (channel > SI4705_FM_HIGH) channel = SI4705_FM_LOW;
					si4705_set_channel(channel);
					clear_radio_strings();
				}
				radio_activity = 0;
				break;
			
			default:
				break;
		}
	}
	
	if (wd_clock == 3599) {
		state = last_state;
	}
}

uint8_t set_time_variable(CLOCK_PREFIX_t selection, time_t *input_time) {
	uint8_t value = *((uint8_t*)input_time + selection); //get the selected value from the memory address
	if (input_time == &time) { //If we're setting the clock's time
		sprintf(line, "%02d/%02d/%02d", input_time->month, input_time->day, input_time->year);
		nokia5110_gotoXY(0,1);
		nokia5110_writeString(line);
		sprintf(line, "%02d:%02d:%02d %s", input_time->hour - 12*(input_time->hour>12), input_time->minute, input_time->second, input_time->hour<12?"am":"pm");
		nokia5110_gotoXY(0,2);
		nokia5110_writeString(line);
	} else { //If we're setting an alarm time
		sprintf(line, "%02d:%02d %s", input_time->hour - 12*(input_time->hour>12), input_time->minute, input_time->hour<12?"am":"pm");
		nokia5110_gotoXY(0,2);
		nokia5110_writeString(line);
	}

	char str[] = "00";
	if (selection == days) {
		time_limits[selection] = days_in_month[input_time->month-1];
	}

	while(!bns[1] && (wd_clock <= 60)) {
		if(bns[0]) {
			bns[0] = false;
			if (selection == months || selection == days) {
				value = ((value+time_limits[selection]-2) % (time_limits[selection]) + 1);
			} else {
				value = ((value+time_limits[selection]-1) % (time_limits[selection]));
			}
			ms_clock = 0; //force a redraw, reset activity
			wd_clock = 0;
			watchdog_feed();
		}
		if(bns[2]) {
			bns[2] = false;
			if (selection == months || selection == days) {
				value = ((value % time_limits[selection]) + 1);
			} else {
				value = ((value+1) % (time_limits[selection]));
			}
			ms_clock = 0; //force a redraw, reset activity
			wd_clock = 0;
			watchdog_feed();
		}
		if ((ms_clock+125)%250 == 0) {
			if (hol[0]) {
				if (selection == months || selection == days) {
					value = ((value+time_limits[selection]-2) % (time_limits[selection]) + 1);
				} else {
					value = ((value+time_limits[selection]-1) % (time_limits[selection]));
				}
				ms_clock = 0; //force a redraw, reset activity
				wd_clock = 0;
				watchdog_feed();
			}
			if (hol[2]) {
				if (selection == months || selection == days) {
					value = ((value % time_limits[selection]) + 1);
				} else {
					value = ((value+1) % (time_limits[selection]));
				}
				ms_clock = 0; //force a redraw, reset activity
				wd_clock = 0;
				watchdog_feed();
			}
		}
		if (ms_clock%1000 == 0) {
			watchdog_entertain();
			nokia5110_gotoXY((selection%3)*21, selection/3+1);
			if (selection == hours) {
				snprintf(str, 3, "%02d", value - 12*(value>12));
				nokia5110_writeString(str);
				nokia5110_gotoXY(42 + 21*(input_time == &time), 2);
				nokia5110_writeString(value<12?"am":"pm");
			} else {
				snprintf(str, 3, "%02d", value);
				nokia5110_writeString(str);
			}
		} else if ((ms_clock + 500)%1000 == 0 && is_fresh) {
			watchdog_entertain();
			nokia5110_gotoXY((selection%3)*21, selection/3+1);
			nokia5110_writeString("  ");
		}
	}
	if (bns[1]) {
		bns[1] = false;
		wd_clock = 0;
		*((uint8_t*)input_time + selection) = value; //store the edited value in the memory address
		if (selection == days) {
			if (time.day > days_in_month[time.month - 1]) {
				input_time->day = (days_in_month[time.month - 1] - (time.month == 2 && !ds1307_isleapyear(time.year))); //don't let users send the wrong number of days
			}
		}
		if (selection == years) {
			if ( time.day > (days_in_month[time.month - 1] - (time.month == 2 && !ds1307_isleapyear(time.year))) ) {
				input_time->day = (days_in_month[time.month - 1] - (time.month == 2 && !ds1307_isleapyear(time.year))); //don't let users send the wrong number of days if it's not a leap year
			}
		}
		
		if (input_time == &time) { //If we're setting the clock's time
			sprintf(line, "%02d/%02d/%02d", input_time->month, input_time->day, input_time->year);
			nokia5110_gotoXY(0,1);
			nokia5110_writeString(line);
			sprintf(line, "%02d:%02d:%02d %s", input_time->hour - 12*(input_time->hour>12), input_time->minute, input_time->second, input_time->hour<12?"am":"pm");
			nokia5110_gotoXY(0,2);
			nokia5110_writeString(line);
		} else { //If we're setting an alarm time
			sprintf(line, "%02d:%02d %s", input_time->hour - 12*(input_time->hour>12), input_time->minute, input_time->hour<12?"am":"pm");
			nokia5110_gotoXY(0,2);
			nokia5110_writeString(line);
		}
		return 0;
	} else {
		wd_clock = 0;
		return 1;
	}
}

uint8_t set_preset_variable(uint8_t num) {
	if (num < 0 || num >= NUMBER_OF_PRESETS) {
		//Out of range
		return 1;
	}
	
	while(!bns[1] && (wd_clock <= 60)) {
		if(bns[0]) {
			bns[0] = false;
			presets[num] -= 2;
			if (presets[num] < SI4705_FM_LOW) presets[num] = SI4705_FM_HIGH;
			ms_clock = 0; //force a redraw, reset activity
			wd_clock = 0;
			watchdog_feed();
		}
		if(bns[2]) {
			bns[2] = false;
			presets[num] += 2;
			if (presets[num] > SI4705_FM_HIGH) presets[num] = SI4705_FM_LOW;
			ms_clock = 0; //force a redraw, reset activity
			wd_clock = 0;
			watchdog_feed();
		}
		if ((ms_clock+125)%250 == 0) {
			if (hol[0]) {
				presets[num] -= 2;
				if (presets[num] < SI4705_FM_LOW) presets[num] = SI4705_FM_HIGH;
				ms_clock = 0; //force a redraw, reset activity
				wd_clock = 0;
				watchdog_feed();
			}
			if (hol[2]) {
				presets[num] += 2;
				if (presets[num] > SI4705_FM_HIGH) presets[num] = SI4705_FM_LOW;
				ms_clock = 0; //force a redraw, reset activity
				wd_clock = 0;
				watchdog_feed();
			}
		}
		if (ms_clock%1000 == 0) {
			watchdog_entertain();
			nokia5110_gotoXY(7, num);
			snprintf(line, 12, "%d.%d  ", presets[num]/10, presets[num]%10);
			nokia5110_writeString(line);
		} else if ((ms_clock + 500)%1000 == 0 && is_fresh) {
			watchdog_entertain();
			nokia5110_gotoXY(7, num);
			nokia5110_writeString("     ");
		}
	}
	if (bns[1]) {
		eeprom_update_word(&saved_presets[num], presets[num]);
		bns[1] = false;
		wd_clock = 0;
		return 0;
	} else {
		wd_clock = 0;
		return 1;
	}
}

void set_brightness(uint8_t intensity) {
	sb_avg -= sb_vals[sb_index];
	sb_vals[sb_index] = intensity;
	sb_avg += sb_vals[sb_index];
	
	sb_index = (sb_index + 1)%3;
	
	OCR1A = (200-sb_avg/3)*25;
	//OCR1A = ICR1; //this disables brightness control, and also an annoying whine
}

void check_schedule() {
	if (!schedule_is_empty(system_schedule)) {
		//printf("%u\t%d\t%u\t%d\t%d\n", ms_clock, system_schedule->size, system_schedule->last_event->ms_time, system_schedule->last_event->ms_duration, system_schedule->last_event->id);
		next_event = schedule_peek(system_schedule);
		if (ms_clock >= next_event.ms_time) {
			if ((ms_clock - next_event.ms_time) >= next_event.ms_duration) {
				schedule_pop(system_schedule);		
				event_is_ready = true;
				return;
			}
		} else {
			if ((ms_clock+SCHEDULE_THRESHOLD) >= ((next_event.ms_time+next_event.ms_duration+SCHEDULE_THRESHOLD)%SCHEDULE_TIME_CEILING)) {
				schedule_pop(system_schedule);
				event_is_ready = true;
				return;
			}	
		}
	}
	event_is_ready = false;
}

/* Takes in a volume from 0 to 255 */
void set_volume(uint8_t volume) {
	if (what_the_beep) {
		si4705_set_volume(0);
		mcp4017_set_resistance(volume/2);
		OCR0A = 60;
	} else if (radio_is_on) {
		mcp4017_set_resistance(0);
		OCR0A = 0;
		si4705_set_volume(volume/4);
	} else {
		mcp4017_set_resistance(0);
		OCR0A = 0;
		si4705_set_volume(0);
	}
}

void clear_radio_strings(void) {
	uint8_t str_index;
	for (str_index = 0; str_index < 8; str_index++) {
		station_text[str_index] = ' ';
	}
	station_text[0] = '\0';
	for (str_index = 0; str_index < 64+DISPLAY_OFFSET; str_index++) {
		scroll_text[str_index] = ' ';
	}
	scroll_text[DISPLAY_OFFSET] = '\0';
	text_index = 0;
}

void watchdog_entertain(void) {
	WDTCSR |= _BV(WDIE);
}

#ifdef USING_PRINTF
void serial_init() {
	USART_Init(MYUBRR);
	stdout = &uart_output;	
	stdin  = &uart_input;
	printf("Serial Terminal Initialized\n");
}
#endif

void set_data_directions() {
	DDRD &= ~(_BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(7)); //set PD3, PD4, PD5, PD6, and PD7 to inputs
	PORTD |= _BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(7);   //turn on the internal pullups for them inputs
	
	#ifndef USING_PRINTF // Status LED's disabled while the serial terminal is being used
	DDRD |= _BV(0) | _BV(1); //LED's are outputs (but we sink current into them)
	PORTD |= _BV(0); // Turn the radio LED off
	PORTD |= _BV(1); // Turn the stereo status LED off
	#endif
}

ISR(TIMER0_COMPA_vect) {}

ISR(TIMER2_COMPA_vect) { //1 msec
	ms_clock =  ((ms_clock+1) % SCHEDULE_TIME_CEILING);
	// If we wait for 10 consecutive 0's or 1's, then the switch would have returned 10 constant values over the course of 10ms
	DEBOUNCE(dbnc[0], PIND, 5);
	DEBOUNCE(dbnc[1], PIND, 4);
	DEBOUNCE(dbnc[2], PIND, 7);	
	DEBOUNCE(dbnc[3], PIND, 2);
	DEBOUNCE(dbnc[4], PIND, 3);
	// States need to clear the button (set it to false) after they read it	
	for (i_index = 0; i_index < 5; i_index++) {
		bns[i_index] |= (dbnc[i_index] == 0xFC00);
		hol[i_index] = (dbnc[i_index] == 0xF800);
	}
	
	is_fresh = true;
}

ISR(ADC_vect) { // ADC has finished reading
	ADC_values[ADC_index] = ADC_get_value();
	
	if ((ADC_index+ADC_MUX_OFFSET) == ADC_PC2) {
		//photo_avg -= photo_readings[p_index];
		//photo_readings[p_index] = ADC_values[ADC_index];
		//photo_avg += photo_readings[p_index];
		//p_index = ((p_index+1) % READINGS_IN_AVG);
		photo_avg = photo_avg - photo_avg/PHOTO_AVG_RESOLUTION + ADC_values[ADC_index];
	}
	if ((ADC_index+ADC_MUX_OFFSET) == ADC_PC3) {
		//therm_avg -= therm_readings[t_index];
		//therm_readings[t_index] = ADC_values[ADC_index];
		//therm_avg += therm_readings[t_index];
		//t_index = ((t_index+1) % READINGS_IN_AVG);
		temperature_avg = temperature_avg - temperature_avg/THERM_AVG_RESOLUTION + (255 - ADC_values[ADC_index]) * 18 / 5 + 370; // into Fahrenheit;
	}
	
	ADC_index = ((ADC_index+ADC_MUX_OFFSET) % NUMBER_OF_ADC_SOURCES);	
	if ((ADC_index+ADC_MUX_OFFSET) == ADC_PC1) {
		ADC_reference_switch(ADC_REFERENCE_VCC);
	}
	ADC_channel_switch(ADC_index+ADC_MUX_OFFSET);
	if ((ADC_index+ADC_MUX_OFFSET) == ADC_PC3) {
		ADC_reference_switch(ADC_REFERENCE_1V1);
	}
}

/* The design specification demands that the watchdog timer must be implemented for a
 * user-timeout delay, while also maintaining system sanity */
ISR(WDT_vect) {
	wd_clock ++; // Hey look, it's a 1 Second timer!
}