/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PLASMA_CUTTER_CONTROL_MODULE_H
#define PLASMA_CUTTER_CONTROL_MODULE_H


//class Pin;

#include "libs/Module.h"
#include "libs/Pin.h"
#include "GCode.h"
#include <string>
#include <vector>

// This module implements control of the spindle speed by seting a PWM from 0-100% which needs
// to be converted to 0-10V by an external circuit
class PlasmaCutterControl : public Module
{
    public:
        PlasmaCutterControl();
        void on_module_loaded();
        void on_gcode_received(void* argument);
        void on_get_public_data(void* argument);
        void on_halt(void *arg);
        void on_config_reload(void* argument);
        uint32_t pinpoll_tick(uint32_t dummy);
        

    private:
		//configs
        bool use_plasma_arc_ok = false;

		//states
		bool plasma_is_on = false;
		bool arc_status = false;

		//Strings
        std::string    gcode_on_start;
        std::string    gcode_on_stop;

		//pins
		Pin *arc_ok_pin;
		Pin *plasma_trigger_pin;
		
		int arc_ok_timeout = 1500; //milliseconds
		int arc_pierce_delay = 1500; //millisecond

		float arc_pierce_height = 2.0; //mm
		float arc_cut_height = 1.2; //mm
		//float arc_cut_voltage = 63.2; //v
		
		Gcode *gcode_instance;

		void read_inputs(void);
		void start_arc(void);
		void stop_arc(void);
		void refresh_values(void);
		void report_arc_status(void);
		void send_gcode(std::string msg);
		bool wait_for_pin(uint32_t timeout, Pin *wait_pin, bool state_to_wait_for, uint32_t debounce_count_limit, bool halt_on_timeout);
		void just_wait(uint32_t timeout);
		void log(std::string logstr);
		bool in_mid_sequence = false;

};

#endif
