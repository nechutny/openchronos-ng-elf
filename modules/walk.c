/*
    modules/walk.c: Walk meter for Openchronos

    Copyright (C) 2016 Stanislav Nechutný <stanislav@nechutny.net>

    This code was written as project for subject Inteligent sensors at Faculty
    of Information Technology at Brno University Technology.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "messagebus.h"
#include "menu.h"

/* drivers */
#include "drivers/rtca.h"
#include "drivers/display.h"
#include "drivers/vti_as.h"
#include "drivers/buzzer.h"
#include "../messagebus.h"


// *************************************************************************************************
// Global Variable section
struct accel
{
	// Sensor raw data
	uint8_t			xyz[3];

	short debug:1;
	short hash:1;
	int disp1;
	int disp2;

};
extern struct accel sAccel;

// Global flag for proper acceleration sensor operation
extern uint8_t as_ok;

// *************************************************************************************************


uint16_t counter = 0;

/* NUM (#) button pressed callback */
// This change the sub menu
static void num_pressed()
{
	sAccel.hash = !sAccel.hash;
}

static void up_btn()
{
	int tmp = sAccel.disp1;
	sAccel.disp1 = sAccel.disp2;
	sAccel.disp2 = tmp;
}

static void down_btn()
{
	int tmp = sAccel.disp1;
	sAccel.disp1 = sAccel.disp2;
	sAccel.disp2 = tmp;
}

/* Star button long press callback. */
// This set/unset the background mode
static void star_long_pressed()
{

	if(sAccel.debug) {
		sAccel.debug = 0;
	} else
	{
		sAccel.debug = 1;
		sAccel.hash = 0;
		_printf(0, sAccel.disp1, "C%03x", as_read_register(ADDR_CTRL));
		_printf(0, sAccel.disp2, "M%03x", as_read_register(ADDR_MDFFTMR));
	}
}

uint16_t convert_acceleration_value_to_mgrav(uint8_t value);

uint16_t steps = 0;
int8_t holdoff = 0;
char skip = 0;
uint16_t result = 0;
uint16_t a_prev = 0;
uint16_t prev[3];

static void wlk_event(enum sys_message msg)
{
	uint16_t change = 0;
	uint16_t tmp;


	// Make 10Hz from 20 Hz

	if (msg & SYS_MSG_TIMER_20HZ)
	{
		skip = !skip;
		if(skip) {
			as_get_data(sAccel.xyz);

			short i;
			// Computing acceleration change value from 3-axis accelerometer sensor signals.
			for(i = 2; i >= 0; --i) {
				tmp = convert_acceleration_value_to_mgrav(sAccel.xyz[i]);
				change += abs(tmp - prev[i]);
				prev[i] += tmp / 8;
			}


			// adaptive threshold level
			result = a_prev / 8;

			if(sAccel.hash) {
				display_clear(0, 1);
				display_clear(0, 2);
				display_chars(0, sAccel.disp1, _sprintf("C%03x", change), SEG_ON);
				display_chars(0, sAccel.disp2, _sprintf("R%03x", result), SEG_ON);
			}

			if(change > a_prev) {
				a_prev = ((change - a_prev) / 2) + a_prev;
			} else {
				a_prev = ((change - a_prev) / 16) + a_prev;
			}


			// Steps detect algorithm
			if(holdoff > 0) {
				holdoff -= 1;
			} else if(holdoff < 0) {
				if(change > result) {
					// nothing
				} else {
					holdoff = 1;
				}
			} else {
				if(change > result) {
					steps += 1;
					holdoff = -1;
				}
			}
		}

	}

	if( msg & SYS_MSG_RTC_SECOND) {
		if(!sAccel.debug && !sAccel.hash)
		{
			display_clear(0, 1);
			display_clear(0, 2);

			display_chars(0, sAccel.disp1, _sprintf("STEPS", result), SEG_ON);
			display_chars(0, sAccel.disp2, _sprintf("%03u", steps), SEG_ON);
		}

	}
}



/* Enter the accelerometer menu */
static void wlk_activated()
{
	//register to the system bus for vti events as well as the RTC second events
	sys_messagebus_register(&wlk_event, SYS_MSG_RTC_SECOND | SYS_MSG_TIMER_20HZ);

	/* create screen */
	lcd_screens_create(1);

	display_chars(0, LCD_SEG_L1_3_0 , "HELLO", SEG_SET);
	display_chars(0, LCD_SEG_L2_4_0 , "WORLD", SEG_SET);

	// 2 g range
	as_config.range=2;
	// 10 Hz sampling rate
	as_config.sampling=SAMPLING_10_HZ;
	// keep mode
	as_config.mode=MEASUREMENT_MODE;
	//time window is 10 msec for free fall and 100 msec for activity
	//2g multiple 71 mg: 0F=4 * 71 mg= 1.065 g
	as_config.MDTHR=2;
	as_config.MDFFTMR=1;
	as_start(ACTIVITY_MODE);

	if (!as_ok)
	{
		display_chars(0, LCD_SEG_L1_3_0, " FAIL", SEG_SET);
	}

	steps = 0;
	holdoff = 0;

	sAccel.disp1 = LCD_SEG_L1_3_0;
	sAccel.disp2 = LCD_SEG_L2_4_0;
	sAccel.debug = 0;
	sAccel.hash = 0;

	/* return to main screen */
	lcd_screen_activate(0);
}




/* Exit */
static void wlk_deactivated()
{
	sys_messagebus_unregister_all(&wlk_event);

	/* destroy virtual screens */
	lcd_screens_destroy();

	/* clean up screen */
	display_clear(0, 1);
	display_clear(0, 2);

	/* Stop acceleration sensor */
	as_stop();

}



void mod_walk_init()
{
	menu_add_entry("STEPS",&up_btn, &down_btn,
			&num_pressed,
			&star_long_pressed,
			NULL,NULL,
			&wlk_activated,
			&wlk_deactivated);
}
