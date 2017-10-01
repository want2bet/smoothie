/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Plasma.h"
#include "Module.h"
#include "Conveyor.h"
#include "Kernel.h"
#include "nuts_bolts.h"
#include "Config.h"
#include "StreamOutputPool.h"
#include "SerialMessage.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StepTicker.h"
#include "Block.h"
#include "SlowTicker.h"
#include "Robot.h"
#include "utils.h"
#include "Pin.h"
#include "Gcode.h"
#include "mbed.h"
#include "PwmOut.h" // mbed.h lib
#include "PublicDataRequest.h"
#include "Adc.h"


#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>

#define plasma_checksum                          	CHECKSUM("plasma")
#define enable_checksum            					CHECKSUM("enable")

#define trigger_pin_checksum               			CHECKSUM("trigger_pin")

#define arc_ok_pin_checksum               			CHECKSUM("arc_ok_pin")

#define amp_servo_pin_checksum               		CHECKSUM("amp_servo_pin")

#define amp_servo_freq_checksum						CHECKSUM("amp_servo_freq")

#define arc_voltage_pin_checksum					CHECKSUM("arc_voltage_pin")
#define arc_voltage_min_checksum					CHECKSUM("arc_voltage_min")
#define arc_voltage_max_checksum					CHECKSUM("arc_voltage_max")

#define min_amp_value_checksum               		CHECKSUM("min_amp_value")
#define max_amp_value_checksum               		CHECKSUM("max_amp_value")
#define min_amp_servo_us_checksum               	CHECKSUM("min_amp_servo_us")
#define max_amp_servo_us_checksum               	CHECKSUM("max_amp_servo_us")

/*
 * plasma.enable					true
 * plasma.trigger_pin				 2.6
 * plasma.arc_ok_pin				 2.7
 *
 * plasma.arc_voltage_pin            0.26
 * plasma.arc_voltage_min            0
 * plasma.arc_voltage_min            250
 *
 * plasma.amp_servo_pin				 2.4
 * plasma.amp_servo_freq			  50
 * plasma.min_amp_value				  10
 * plasma.max_amp_value				  50
 * plasma.min_amp_servo_us   		 500
 * plasma.max_amp_servo_us			3500
 *
 */


//plasma.enable						true
//plasma.trigger_pin				2.12o

Plasma::Plasma()
{
    plasma_on = false;
}

void Plasma::on_module_loaded()
{
    /*
     if(!THEKERNEL->config->value(plasma_checksum,enable_checksum)->by_default(false)->as_bool()) {
        // as not needed free up resource
        delete this;
        return;
    }
    */

    // Get smoothie-style pin from config

	//trigger pin
	Pin *pin;
	float freq;
	float period_us;

	pin = new Pin();

    pin->from_string(THEKERNEL->config->value(plasma_checksum,trigger_pin_checksum)->by_default("nc")->as_string())->as_output();

    this->trigger_pin = pin;

    if (!this->trigger_pin->connected()) {
        THEKERNEL->streams->printf("Error: Plasma cannot use P%d.%d Plasma module disabled.\n", this->trigger_pin->port_number, this->trigger_pin->pin);
        delete this->trigger_pin;
        return;
    } else {
    	THEKERNEL->streams->printf("Plasma: Plasma trigger is  P%d.%d \n", this->trigger_pin->port_number, this->trigger_pin->pin);
    	this->trigger_inverting = this->trigger_pin->is_inverting();
    	this->trigger_pin->set(false);
    }


    //Amp Servo Freq

    freq = THEKERNEL->config->value(plasma_checksum, min_amp_value_checksum)->by_default((float)50)->as_number();
    period_us = (1000.0f/freq)*1000.0f;

    //Amp hobby servo pin
    pin = new Pin();

    pin->from_string(THEKERNEL->config->value(plasma_checksum,amp_servo_pin_checksum)->by_default("nc")->as_string())->as_output();
    this->amp_servo_pin= pin->hardware_pwm();
    delete pin;
    if(this->amp_servo_pin == nullptr) {
        THEKERNEL->streams->printf("Selected Amp Servo output pin is not PWM capable - disabled");
    } else{
    	this->amp_servo_pin->period_us(period_us);  //20,000 uS period = 50 Hz
    	this->amp_servo_pin->write(0);  //set duty cycle to 0
    }

    this->min_amp_value = THEKERNEL->config->value(plasma_checksum, min_amp_value_checksum)->by_default((float)10.0)->as_number();
    this->max_amp_value = THEKERNEL->config->value(plasma_checksum, max_amp_value_checksum)->by_default((float)50.0)->as_number();
    this->min_amp_servo_us = THEKERNEL->config->value(plasma_checksum, min_amp_servo_us_checksum)->by_default((float)500)->as_number();
    this->max_amp_servo_us = THEKERNEL->config->value(plasma_checksum, max_amp_servo_us_checksum)->by_default((float)3500)->as_number();

    //Arc OK pin
	pin = new Pin();
    pin->from_string(THEKERNEL->config->value(plasma_checksum,arc_ok_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->arc_ok_pin = pin;

    if (!this->arc_ok_pin->connected()) {
        THEKERNEL->streams->printf("Error: Plasma cannot use P%d.%d for Arc OK.\n", this->arc_ok_pin->port_number, this->arc_ok_pin->pin);
        delete this->arc_ok_pin;
        return;
    } else {
    	THEKERNEL->streams->printf("Plasma: Plasma Arc OK is  P%d.%d \n", this->arc_ok_pin->port_number, this->arc_ok_pin->pin);
    }

    //register for events
    this->register_for_event(ON_HALT);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    //this->register_for_event(ON_GET_PUBLIC_DATA);

}

void Plasma::set_plasma_amps(float amps){

	float us_target=0;
	float duty_cycle;

	//assume freq = 50hz and period =  20,000 us
	// 2.5% duty cycle = 500 us
	// 5% duty cycle = 1,000 us
	//10% duty cycle = 2,000 us	`
	//15% duty cycle = 3,000 us

	us_target = this->interpolation(amps, this->min_amp_value, this->max_amp_value, this->min_amp_servo_us, this->max_amp_servo_us);

	duty_cycle = (us_target / 20000) * 100.0;

	if (this->verbose) THEKERNEL->streams->printf("Setting amp servo to duty cycle %5.1f \n", duty_cycle);

	this->amp_servo_pin->write(duty_cycle);
}

float Plasma::interpolation(float in_value, float min_in, float max_in, float min_out, float max_out){

	float in_span = 0;
	float out_span = 0;
	float out_2_in_ratio = 0;
	float out_value = 0;

	if(in_value <= min_in) return(min_out);
	if(in_value >= max_in) return(max_out);

	in_span = max_in - min_in;

	out_span = max_out - min_out;

	out_2_in_ratio = out_span / in_span;

	out_value = out_2_in_ratio * (in_value - min_out);

	return out_value;

	/*
	 * in_val 38
	 * in_min 10
	 * in_max 50
	 * out_min 0
	 * out_max 270
	 *
	 * in_span=40
	 * out_span=270
	 * out_2_in_ratio = 6.75
	 *
	 * 28 * 6.75 = 189
	 *
	 */

}

void Plasma::set_digital_pin(string pin_str, bool t_or_f){
	Pin* some_pin=new Pin();

    some_pin->from_string(pin_str)->as_output();

    if (!some_pin->connected()) return;

    if(this->verbose) THEKERNEL->streams->printf("Setting pin %s to %d\n",pin_str.c_str(),t_or_f);

    some_pin->set(t_or_f);

    delete some_pin;

}

bool Plasma::wait_for_pin(uint32_t timeout, Pin *wait_pin, bool state_to_wait_for, uint32_t debounce_count_limit, bool halt_on_timeout)
{
	bool wait_pin_status = false;    // assume the pin starts at false
	bool pin_at_wait_state = false;  // assume current pin state isn't wait we wait for
	uint32_t debounce_count=0;  // number of times in a row we get the target state
	uint32_t delta_ticks=0;
	float ms_delay=0;

    uint32_t start = us_ticker_read(); // mbed call

    //While we are not at desired state, and we are not out of time....
    while (!pin_at_wait_state && ( delta_ticks < (timeout * 1000))) {
    	// admit our idle state....
        THEKERNEL->call_event(ON_IDLE, this);
    	//THEKERNEL->conveyor->wait_for_idle();
        // check for a halt status and break out....
        if(THEKERNEL->is_halted()) return(pin_at_wait_state);
        //if the pin in defined....
        if(wait_pin){
        	/// get the target pin status....
			wait_pin_status = wait_pin->get();
			// see if it is what we are looking for
			if (wait_pin_status == state_to_wait_for) {
				// if so, inc our count
				debounce_count++;
			} else {
				// if not, reset to 0
				debounce_count=0;
			}
			// if we have reached the stable condition we are looking for, throw the flag
			if (debounce_count >= debounce_count_limit) pin_at_wait_state=true;
        }
        delta_ticks=(us_ticker_read() - start);
        ms_delay=delta_ticks/1000.0f;
    }

    if (halt_on_timeout && !pin_at_wait_state) THEKERNEL->call_event(ON_HALT, nullptr);

    if(this->verbose){
    	if(wait_pin == nullptr) {
    		THEKERNEL->streams->printf("Waited for NULL ");
    	} else {
    	THEKERNEL->streams->printf("Waited for P%d.%d ", wait_pin->port_number, wait_pin->pin);
       }

       THEKERNEL->streams->printf("for %6.2f ms. /n", ms_delay );
       if(!pin_at_wait_state)	THEKERNEL->streams->printf("But timed out. /n");
    }

    return(pin_at_wait_state);
}

void Plasma::on_console_line_received( void *argument )
{
    if(THEKERNEL->is_halted()) return; // if in halted state ignore any commands

    SerialMessage *msgp = static_cast<SerialMessage *>(argument);
    string possible_command = msgp->message;

    // ignore anything that is not lowercase or a letter
    if(possible_command.empty() || !islower(possible_command[0]) || !isalpha(possible_command[0])) {
        return;
    }

    string cmd = shift_parameter(possible_command);

    // Act depending on command
    if (cmd == "setpin") {

        string pin_str = shift_parameter(possible_command);
        string pin_type = shift_parameter(possible_command);
        string pin_val = shift_parameter(possible_command);
        bool t_or_f = false;

        if(pin_str.empty() || pin_type.empty() || pin_val.empty()) {
            msgp->stream->printf("Usage: setpin pin_config_string pin_type pin_value.\n");
            return;
        }

        pin_type.erase(pin_type.find_last_not_of(" \n\r\t")+1);
        std::transform(pin_type.begin(),pin_type.end(),pin_type.begin(),::tolower);

        pin_val.erase(pin_val.find_last_not_of(" \n\r\t")+1);
        std::transform(pin_val.begin(),pin_val.end(),pin_val.begin(),::tolower);

        if(pin_val == "1") t_or_f=true;
        if(pin_val == "true") t_or_f=true;
        if(pin_val == "on") t_or_f=true;

        this->set_digital_pin(pin_str,t_or_f);

    }
}

// returns instance
void Plasma::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);
    if(!pdr->starts_with(plasma_checksum)) return;
    pdr->set_data_ptr(this);
    pdr->set_taken();
}

float Plasma::get_arc_voltage(){
	float vin = 0;
	float arc_volt;

	if (this->arc_voltage_pin == nullptr) return 0.0f;

	vin=THEKERNEL->adc->read(this->arc_voltage_pin);

	arc_volt = this->interpolation(vin, 0.0f ,THEKERNEL->adc->get_max_value(),this->arc_voltage_min,this->arc_voltage_max);

	return arc_volt;
}


void Plasma::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    // M codes execute immediately
    if (gcode->has_m) {
        if (gcode->m == 3) {
        	if(this->verbose) gcode->stream->printf("Starting plasma.\n");
        	this->start();
            } else if(gcode->m == 5){
            	if(this->verbose) gcode->stream->printf("Stopping plasma.\n");
            	this->stop();
            } else if(gcode->m == 770){  // M770 A38
            	if(gcode->has_letter('A')){
            		this->target_amps = gcode->get_value('A');
            		if(this->target_amps > 0) this->set_plasma_amps(this->target_amps);
            	}
            } else if(gcode->m == 771){  // M771 P1100  would be Arc OK Pause for 1100ms max
            	if(gcode->has_letter('P')){
            		this->max_arc_ok_delay_ms = gcode->get_value('P');
            		if(this->max_arc_ok_delay_ms > 0) this->wait_for_pin(this->max_arc_ok_delay_ms,this->arc_ok_pin,true,5,false);
            	}
            } else if(gcode->m == 772){  // M771 P1100  would be Arc OK Pause for 1100ms max

            	if (this->trigger_pin != nullptr) {
            		THEKERNEL->streams->printf("Arc Trigger Pin P%d.%d. \n", this->trigger_pin->port_number, this->trigger_pin->pin);
            	} else {
            		THEKERNEL->streams->printf("Arc Trigger Pin is NULL. \n");
            	}


            	if (this->arc_ok_pin != nullptr) {
            		THEKERNEL->streams->printf("Arc OK Pin P%d.%d. \n", this->arc_ok_pin->port_number, this->arc_ok_pin->pin);

            	} else {
            		THEKERNEL->streams->printf("Arc OK Pin is NULL. \n");
            	}


            	if (this->amp_servo_pin != nullptr) {
            		THEKERNEL->streams->printf("Arc Servo Pin is not NULL. \n");
            		//THEKERNEL->streams->printf("Arc Servo output is %f \n",amp_servo_pin.read());
            	} else {
            		THEKERNEL->streams->printf("Arc Servo Pin is NULL. \n");
            	}

            	//Amp_servo
            	//Arc_trigger_pin
            	//Arc_ok_pin
            	//Arc_voltag_pin

        	}
    }
}


void Plasma::start()
{
	THEKERNEL->conveyor->wait_for_idle(true);
	//this->set_digital_pin(this->trigger_pin_string,true);
	this->trigger_pin->set(true);
	this->plasma_on=true;
}

void Plasma::stop(){
	THEKERNEL->conveyor->wait_for_idle(true);
	//this->set_digital_pin(this->trigger_pin_string,false);
	this->trigger_pin->set(false);
	this->plasma_on=true;
}

void Plasma::on_halt(void *argument)
{
    if(argument == nullptr) {
    	this->stop();
    }
}
