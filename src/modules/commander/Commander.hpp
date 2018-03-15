/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef COMMANDER_HPP_
#define COMMANDER_HPP_

#include "state_machine_helper.h"

#include <controllib/blocks.hpp>
#include <px4_module.h>
#include <mathlib/mathlib.h>

// publications
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

using control::BlockParamFloat;
using control::BlockParamInt;
using math::constrain;
using uORB::Publication;
using uORB::Subscription;

class Commander : public control::SuperBlock, public ModuleBase<Commander>
{
public:
	Commander() :
		SuperBlock(nullptr, "COM"),
		_home_eph_threshold(this, "HOME_H_T"),
		_home_epv_threshold(this, "HOME_V_T"),
		_eph_threshold(this, "POS_FS_EPH"),
		_epv_threshold(this, "POS_FS_EPV"),
		_evh_threshold(this, "VEL_FS_EVH"),
		_failsafe_pos_delay(this, "POS_FS_DELAY"),
		_failsafe_pos_probation(this, "POS_FS_PROB"),
		_failsafe_pos_gain(this, "POS_FS_GAIN"),
		_mission_result_sub(ORB_ID(mission_result), 0, 0, &getSubscriptions()),
		_global_position_sub(ORB_ID(vehicle_global_position), 0, 0, &getSubscriptions()),
		_local_position_sub(ORB_ID(vehicle_local_position), 0, 0, &getSubscriptions())
	{
		updateParams();
	}

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Commander *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	void enable_hil();

private:

	BlockParamFloat	_home_eph_threshold;
	BlockParamFloat	_home_epv_threshold;

	BlockParamFloat	_eph_threshold;
	BlockParamFloat	_epv_threshold;
	BlockParamFloat	_evh_threshold;

	BlockParamInt	_failsafe_pos_delay;
	BlockParamInt	_failsafe_pos_probation;
	BlockParamInt	_failsafe_pos_gain;

	// Subscriptions
	Subscription<mission_result_s> _mission_result_sub;
	Subscription<vehicle_global_position_s> _global_position_sub;
	Subscription<vehicle_local_position_s> _local_position_sub;


	static constexpr int64_t sec_to_usec = (1000 * 1000);
	static constexpr int64_t POSVEL_PROBATION_MIN = 1 * sec_to_usec;	/**< minimum probation duration (usec) */
	static constexpr int64_t POSVEL_PROBATION_MAX = 100 * sec_to_usec;	/**< maximum probation duration (usec) */

	hrt_abstime	_last_gpos_fail_time_us{0};	/**< Last time that the global position validity recovery check failed (usec) */
	hrt_abstime	_last_lpos_fail_time_us{0};	/**< Last time that the local position validity recovery check failed (usec) */
	hrt_abstime	_last_lvel_fail_time_us{0};	/**< Last time that the local velocity validity recovery check failed (usec) */

	// Probation times for position and velocity validity checks to pass if failed
	hrt_abstime	_gpos_probation_time_us = POSVEL_PROBATION_MIN;
	hrt_abstime	_lpos_probation_time_us = POSVEL_PROBATION_MIN;
	hrt_abstime	_lvel_probation_time_us = POSVEL_PROBATION_MIN;

	hrt_abstime	get_posctl_nav_loss_delay() { return constrain(_failsafe_pos_delay.get() * sec_to_usec, POSVEL_PROBATION_MIN, POSVEL_PROBATION_MAX); }
	hrt_abstime	get_posctl_nav_loss_prob() { return constrain(_failsafe_pos_probation.get() * sec_to_usec, POSVEL_PROBATION_MIN, POSVEL_PROBATION_MAX); }


	bool handle_command(vehicle_status_s *status, const safety_s *safety, vehicle_command_s *cmd, actuator_armed_s *armed,
			    home_position_s *home, orb_advert_t *home_pub, orb_advert_t *command_ack_pub, bool *changed);

	bool set_home_position(orb_advert_t &homePub, home_position_s &home, bool set_alt_only_to_lpos_ref);

	// Set the main system state based on RC and override device inputs
	transition_result_t set_main_state(vehicle_status_s *status, bool *changed);
	transition_result_t set_main_state_override_on(vehicle_status_s *status_local, bool *changed);
	transition_result_t set_main_state_rc(vehicle_status_s *status, bool *changed);

	void check_valid(hrt_abstime timestamp, hrt_abstime timeout, bool valid_in, bool *valid_out, bool *changed);
	bool check_posvel_validity(const bool data_valid, const float data_accuracy, const float required_accuracy,
				   const hrt_abstime &data_timestamp_us, hrt_abstime *last_fail_time_us, hrt_abstime *probation_time_us, bool *valid_state,
				   bool *validity_changed);
	void reset_posvel_validity(bool *changed);

	void mission_init();

};

#endif /* COMMANDER_HPP_ */
