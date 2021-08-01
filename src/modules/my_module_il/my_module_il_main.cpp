/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

/*
 * @file attitude_estimator_q_main.cpp
 *
 * Attitude estimator (quaternion based)
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <lib/ecl/geo_lookup/geo_mag_declination.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_magnetometer.h>

#include <uORB/topics/custom_topic.h>  // custom topic for publication

using matrix::Vector3f;


using namespace time_literals;

class ModuleInLoop : public ModuleBase<ModuleInLoop>, public ModuleParams, public px4::WorkItem
{
public:

	ModuleInLoop();
	~ModuleInLoop() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:

	void Run() override;

	void update_parameters(bool force = false);



	const float _eo_max_std_dev = 100.0f;		/**< Maximum permissible standard deviation for estimated orientation */
	const float _dt_min = 0.00001f;
	const float _dt_max = 0.02f;

	uORB::SubscriptionCallbackWorkItem _sensors_sub{this, ORB_ID(sensor_combined)};

	uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s}; //parameter update
	uORB::Subscription		_magnetometer_sub{ORB_ID(vehicle_magnetometer)};

	uORB::Publication<custom_topic_s>	_custom_pub{ORB_ID(custom_topic)};

	Vector3f	_mag;

	hrt_abstime	_last_time{0};

	bool		_inited{false};   //is initialized ??
	bool		_data_good{false};
	bool		_ext_hdg_good{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::ATT_W_MAG>) _param_att_w_mag,
		(ParamFloat<px4::params::ATT_MAG_DECL>) _param_att_mag_decl,
		(ParamInt<px4::params::ATT_MAG_DECL_A>) _param_att_mag_decl_a,
		(ParamInt<px4::params::ATT_EXT_HDG_M>) _param_att_ext_hdg_m,
		(ParamInt<px4::params::SYS_HAS_MAG>) _param_sys_has_mag
	)
};

ModuleInLoop::ModuleInLoop() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers) //set the priority
{
	_mag.zero();

	update_parameters(true);
}

bool
ModuleInLoop::init()
{
	if (!_sensors_sub.registerCallback()) {
		PX4_ERR("sensor combined callback registration failed!");
		return false;
	}

	return true;
}

void
ModuleInLoop::Run()
{
	if (should_exit()) {
		_sensors_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	sensor_combined_s sensors;

	if (_sensors_sub.update(&sensors)) {  //.update check if sensor_combined topic updated..

		update_parameters();

		// Update magnetometer
		if (_magnetometer_sub.updated()) {
			vehicle_magnetometer_s magnetometer;

			if (_magnetometer_sub.copy(&magnetometer)) {
				_mag(0) = magnetometer.magnetometer_ga[0];
				_mag(1) = magnetometer.magnetometer_ga[1];
				_mag(2) = magnetometer.magnetometer_ga[2];

				if (_mag.length() < 0.01f) {
					PX4_ERR("degenerate mag!");
					return;
				}
			}

		}

		_data_good = true;

		// Update vision and motion capture heading
		_ext_hdg_good = false;




		if (true) {
			custom_topic_s cstm = {};

			/* the instance count is not used here */
			cstm.timestamp = hrt_absolute_time();
			cstm.magneto[0] = _mag(0);
			cstm.magneto[1] = _mag(1);
			cstm.magneto[2] = _mag(2);
			cstm.is_ext = true;
			_custom_pub.publish(cstm);

		}
	}
}

void
ModuleInLoop::update_parameters(bool force)  //intact
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		// disable mag fusion if the system does not have a mag
		if (_param_sys_has_mag.get() == 0) {
			_param_att_w_mag.set(0.0f);
		}

		// if the weight is zero (=mag disabled), make sure the estimator initializes
		if (_param_att_w_mag.get() < FLT_EPSILON) {
			_mag(0) = 1.f;
			_mag(1) = 0.f;
			_mag(2) = 0.f;
		}
	}
}




int
ModuleInLoop::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
ModuleInLoop::task_spawn(int argc, char *argv[])   //
{
	ModuleInLoop *instance = new ModuleInLoop();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ModuleInLoop::print_usage(const char *reason) //just for shell
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude estimator q.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ModuleInLoop", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int my_module_il_main(int argc, char *argv[])
{
	return ModuleInLoop::main(argc, argv);
}
