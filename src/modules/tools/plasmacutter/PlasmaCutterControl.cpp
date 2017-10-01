/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "libs/Adc.h"
#include "mbed.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"
#include "PwmOut.h"
#include "PublicDataRequest.h"
#include "SlowTicker.h"

#include "libs/SerialMessage.h"
#include <math.h>
#include "modules/robot/Conveyor.h"
#include "PublicDataRequest.h"
#include "Gcode.h"
#include "StreamOutput.h"
#include "MRI_Hooks.h"
#include <algorithm>
#include <sstream>
#include "PlasmaCutterControl.h"
#include "PlasmaCutterPublicAccess.h"
#include "SerialConsole.h"

#include <iostream>
#include <fstream>
#include <iomanip>

// Config examples

// via gcode
// M3.1  D2100 H2.2 P1400 C1.2  V104
// Arc Delay Timeout   D <milliseconds>
// Arc Pierce Height   H <default units (mm)>
// Arc Pierce Time     P <milliseconds>
// Arc Cut Height      C <default units (mm)>
// Target Cut Voltage  V <volts>


//via settings
/*
plasma_cutter.enable   true
plasma_cutter.arc_ok_enable   true
plasma_cutter.plasma_arc_ok_pin    2.7
plasma_cutter.trigger_pin    2.6^!
plasma_cutter.arc_ok_timeout   2200
plasma_cutter.pierce_height   2.1
plasma_cutter.pierce_delay   1700
plasma_cutter.cut_height  1.1
plasma_cutter.arc_gcode_on_start   "G30_Z-2.8"
plasma_cutter.arc_gcode_on_stop   ""

*/

#define plasma_cutter_checksum                   CHECKSUM("plasma_cutter")
#define arc_ok_pin_checksum                      CHECKSUM("arc_ok_pin")
#define trigger_pin_checksum                     CHECKSUM("trigger_pin")
#define enable_checksum                          CHECKSUM("enable")
#define arc_ok_enable_checksum                   CHECKSUM("arc_ok_enable")
#define arc_ok_timeout_checksum                  CHECKSUM("arc_ok_timeout")
#define pierce_height_checksum                   CHECKSUM("pierce_height")
#define pierce_delay_checksum                    CHECKSUM("pierce_delay")
#define cut_height_checksum                      CHECKSUM("cut_height")
#define arc_gcode_on_start_checksum              CHECKSUM("arc_gcode_on_start")
#define arc_gcode_on_stop_checksum               CHECKSUM("arc_gcode_on_stop")


PlasmaCutterControl::PlasmaCutterControl(){
   this->arc_ok_pin=nullptr;
   this->plasma_trigger_pin=nullptr;
}


void PlasmaCutterControl::on_module_loaded()
{
	//If the plasma cutter isn't enabled, just bail

	if( !THEKERNEL->config->value(plasma_cutter_checksum, enable_checksum )->by_default(false)->as_bool() ){
		delete this;
		return; 
	} 

	// load config
	this->on_config_reload(this);
	
	// register for events
    this->register_for_event(ON_GCODE_RECEIVED);
    //this->register_for_event(ON_MAIN_LOOP);
    //this->register_for_event(ON_GET_PUBLIC_DATA);
    //this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_HALT);

}



void PlasmaCutterControl::on_config_reload(void *argument)
{	


    this->use_plasma_arc_ok = THEKERNEL->config->value(plasma_cutter_checksum,arc_ok_enable_checksum)->by_default(false)->as_bool();
    this->arc_ok_timeout = THEKERNEL->config->value(plasma_cutter_checksum,arc_ok_timeout_checksum)->by_default(1500)->as_int();
    this->arc_pierce_height = THEKERNEL->config->value(plasma_cutter_checksum,pierce_height_checksum)->by_default((float)2.1)->as_number();
    this->arc_cut_height = THEKERNEL->config->value(plasma_cutter_checksum,cut_height_checksum)->by_default((float)1.2)->as_number();
    this->arc_pierce_delay = THEKERNEL->config->value(plasma_cutter_checksum,pierce_delay_checksum)->by_default(1500)->as_int();
    this->gcode_on_start = THEKERNEL->config->value(plasma_cutter_checksum,arc_gcode_on_start_checksum)->by_default("")->as_string();
    this->gcode_on_stop = THEKERNEL->config->value(plasma_cutter_checksum,arc_gcode_on_stop_checksum)->by_default("")->as_string();
    //this->arc_cut_voltage == THEKERNEL->config->value(plasma_cutter_checksum,cut_height_checksum)->by_default((float)63.2)->as_number();

	//arc ok pin (digital in)
	{
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(plasma_cutter_checksum,arc_ok_pin_checksum)->by_default("nc")->as_string());
        this->arc_ok_pin = smoothie_pin->as_input();
        delete smoothie_pin;
    }
	
	//plasma trigger pin (digital out)
	{
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(plasma_cutter_checksum,trigger_pin_checksum)->by_default("nc")->as_string());
        this->plasma_trigger_pin = smoothie_pin->as_output();
        delete smoothie_pin;
    }
	
	
	//Check for config errors....
	//Need at least this pin configured....
	/*
	if (!this->plasma_trigger_pin){
        THEKERNEL->streams->printf("Error: Must configure trigger_pin for valid output pin.\n");
        delete this;
        return;
	}
	*/

	if (this->use_plasma_arc_ok == true){
		if (this->arc_ok_pin == NULL) {
			//THEKERNEL->serial->puts("Error: Cant use arc_ok without valid arc_ok_pin config.\n");
			this->use_plasma_arc_ok = false;
		} else {
		    if(this->arc_ok_pin->connected()) {
		        // set to initial state
		        this->arc_status = this->arc_ok_pin->get();
		        // input pin polling
		    }
		}
	}

	THEKERNEL->slow_ticker->attach( 100, this, &PlasmaCutterControl::pinpoll_tick);

	//if (this->arc_pierce_height < this->arc_cut_height){
	//}

    std::replace(gcode_on_start.begin(), gcode_on_start.end(), '_', ' '); // replace _ with space
    std::replace(gcode_on_stop.begin(), gcode_on_stop.end(), '_', ' '); // replace _ with space

}

void PlasmaCutterControl::send_gcode(std::string msg)
{
	//This almost works, but kills any commands behind it.
     Gcode gc(msg.c_str(),&(StreamOutput::NullStream));
     if(this->gcode_instance) this->gcode_instance->stream->printf(";Send %s\n",msg.c_str());
     THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);

    //struct SerialMessage message;
    //message.message = msg;
    //message.stream = &StreamOutput::NullStream;
    //if(this->gcode_instance) message.stream = this->gcode_instance->stream;
    //THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );

}


void PlasmaCutterControl::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if (this->in_mid_sequence) return;
    this->gcode_instance=gcode;

    // Add the gcode to the queue ourselves if we need it
    //Is it M3,M4, or M5?
   if (gcode->has_m) {
    	if((gcode->m == 3) || (gcode->m == 4)){
    	    if(this->gcode_instance) this->gcode_instance->stream->printf(";Plasma Module Start");
    		//If we detect M3.1 or M4.1 then there should be sub params
    		if(gcode->subcode == 1){
    			//M3.1  D2100 H2.2 P1400 C1.2  V104
				//Arc Delay Timeout   D <milliseconds>
				//Arc Pierce Height   H <default units (mm)>
				//Arc Pierce Time     P <milliseconds>
				//Arc Cut Height      C <default units (mm)>
				//Target Cut Voltage  V <volts>
				if(gcode->has_letter('D')) this->arc_ok_timeout = gcode->get_value('D');
				if(gcode->has_letter('H')) this->arc_pierce_height = gcode->get_value('H');
				if(gcode->has_letter('P')) this->arc_pierce_delay = gcode->get_value('P');
				if(gcode->has_letter('C')) this->arc_cut_height = gcode->get_value('C');
				//if(gcode->has_letter('V')) this->arc_cut_voltage = gcode->get_value('V');
    		}
    		this->start_arc();
    	}
    	else if(gcode->m == 5) {
    		if(this->gcode_instance) this->gcode_instance->stream->printf(";Plasma Module Stop");
    		this->stop_arc();
    	}
    }
   this->gcode_instance=nullptr;
}

bool PlasmaCutterControl::wait_for_pin(uint32_t timeout, Pin *wait_pin, bool state_to_wait_for, uint32_t debounce_count_limit, bool halt_on_timeout)
{
	bool wait_pin_status = false;    // assume the pin starts at false
	bool pin_at_wait_state = false;  // assume current pin state isn't wait we wait for
	uint32_t debounce_count=0;  // number of times in a row we get the target state

    uint32_t start = us_ticker_read(); // mbed call

    //While we are not at desired state, and we are not out of time....
    while (!pin_at_wait_state && ((us_ticker_read() - start) < timeout * 1000)) {

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
    }
    if (halt_on_timeout && !pin_at_wait_state) THEKERNEL->call_event(ON_HALT, nullptr);
    return(pin_at_wait_state);
}

void PlasmaCutterControl::just_wait(uint32_t timeout)
{
	this->wait_for_pin(timeout, nullptr, true, 0, false);
}

void PlasmaCutterControl::start_arc()
{
	std::stringstream new_gcode_strm;

	this->in_mid_sequence=true;

	THEKERNEL->conveyor->wait_for_idle();
	if(this->gcode_instance) this->gcode_instance->stream->printf(";Plasma start gcode\n");
	//gcode_on_start="G30_Z-2.8"
	if (!this->gcode_on_start.empty()) this->send_gcode(this->gcode_on_start);
	//wait for that code to execute
	THEKERNEL->conveyor->wait_for_idle(true);

	if(this->gcode_instance) this->gcode_instance->stream->printf(";Move to pierce Z\n");
	//create the new gcode string to move to pierce height
	new_gcode_strm << "G0 Z" << std::fixed << std::setprecision(2) << this->arc_pierce_height << std::endl;
	//send it
	this->send_gcode(new_gcode_strm.str());
	if(this->gcode_instance) this->gcode_instance->stream->printf(";Wait for idle\n");
	//wait for it to process....
	THEKERNEL->conveyor->wait_for_idle(true);


	//squeeze the plasma trigger pin
	if(this->gcode_instance) this->gcode_instance->stream->printf(";Starting Plasma\n");
	if(this->plasma_trigger_pin != NULL) this->plasma_trigger_pin->set(true);

	//wait for the Arc Ok bit
	if(this->gcode_instance) this->gcode_instance->stream->printf(";Wait for arc\n");

	if(use_plasma_arc_ok){
		this->plasma_is_on = this->wait_for_pin(arc_ok_timeout, this->arc_ok_pin, true, 5, false);
	} else {
		this->plasma_is_on = this->wait_for_pin(arc_ok_timeout, nullptr, true, 5, false);
	}

	//wait for Hawkeye
	if(this->gcode_instance) this->gcode_instance->stream->printf(";Wait to pierce\n");
	this->just_wait(arc_pierce_delay);


	//move to cut height
	if(this->gcode_instance) this->gcode_instance->stream->printf(";Move to cut Z\n");
	new_gcode_strm.str("");
	new_gcode_strm << "G0 Z" << std::fixed << std::setprecision(2) << this->arc_cut_height << std::endl;
	this->send_gcode(new_gcode_strm.str());

	//wait for it to process....
	//THEKERNEL->conveyor->wait_for_idle(true);

	this->in_mid_sequence=false;
}

void PlasmaCutterControl::stop_arc()
{
	//Wait for the robot to quit moving
	THEKERNEL->conveyor->wait_for_idle(true);

	//Turn off the plasma
	if(this->plasma_trigger_pin != NULL)
		this->plasma_trigger_pin->set(false);
	this->plasma_is_on = false;

	//Execute final gcode.
	this->send_gcode(this->gcode_on_stop);
}


void PlasmaCutterControl::refresh_values()
{
	if (this->use_plasma_arc_ok) {
		this->arc_status = this->arc_ok_pin->get();
	}
}

void PlasmaCutterControl::report_arc_status()
{
	if (this->use_plasma_arc_ok) {
		THEKERNEL->streams->printf("Plasma Arc Status: %s\n", this->arc_status ? "true" : "false");
	}
}

// TODO Make this use InterruptIn
// Check the state of the button and act accordingly
uint32_t PlasmaCutterControl::pinpoll_tick(uint32_t dummy)
{
	if(!this->arc_ok_pin) return 0;
    if(!this->arc_ok_pin->connected()) return 0;

    // If pin changed
    bool current_state = this->arc_ok_pin->get();
    if(this->arc_status != current_state) {
        this->arc_status = current_state;
    }
    return 0;
}

void PlasmaCutterControl::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(plasma_cutter_checksum)) return;
    pdr->set_data_ptr(this);
    pdr->set_taken();
}

void PlasmaCutterControl::on_halt(void *argument)
{
    if(argument == nullptr) {
        this->stop_arc();
        this->arc_status=false;
    }
}



