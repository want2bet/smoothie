/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>


namespace mbed {
    class PwmOut;
}
class Pin;
class Block;

class Plasma : public Module{
    public:
        Plasma();
       // virtual ~Plasma() {};
        void on_module_loaded();
        void on_halt(void* argument);
        void on_gcode_received(void *argument);
        void on_console_line_received(void *argument);
        void on_get_public_data(void* argument);

    private:
        mbed::PwmOut  *amp_servo_pin;
        Pin *arc_ok_pin;
        Pin *trigger_pin;				// TTL output to fire laser
        Pin *arc_voltage_pin;

        std::string trigger_pin_string;

        void start();
        void stop();
        void set_plasma_amps(float amps);
        void set_digital_pin(std::string pin_str, bool t_or_f);
        bool wait_for_pin(uint32_t timeout, Pin *wait_pin, bool state_to_wait_for, uint32_t debounce_count_limit, bool halt_on_timeout);
        float interpolation(float in_value, float min_in, float max_in, float min_out, float max_out);
        void thc_adjust_z_axis();
        float get_arc_voltage();

        bool plasma_on=false;      // set if the laser is on
        bool trigger_inverting=false;
        bool verbose=true;

        float min_amp_value=10;
        float max_amp_value=50;
        float min_amp_servo_us=500;
        float max_amp_servo_us=3500;
        float target_amps=40;
        float max_arc_ok_delay_ms=1100;

        float arc_voltage_min=0;
        float arc_voltage_max=0;
        float arc_voltage=0;
        float arc_voltage_target=0;

};
