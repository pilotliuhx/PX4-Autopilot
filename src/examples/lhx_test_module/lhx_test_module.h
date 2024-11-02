#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <lib/geo/geo.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint_lhx.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/yaw_estimator_status.h>
#include <uORB/topics/vehicle_global_position.h>

using namespace time_literals;

extern "C" __EXPORT int lhx_test_module_main(int argc, char *argv[]);


class LhxTestModule : public ModuleBase<LhxTestModule>, public ModuleParams
{
public:
	LhxTestModule(int example_param, bool example_flag);

	virtual ~LhxTestModule() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static LhxTestModule *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	static const position_setpoint_s empty_position_setpoint;
private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	void record_a();
	void record_b();
	void go_left();
	void go_right();
	void update_ab(matrix::Vector2f, matrix::Vector2f);
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	MapProjection _geo_projection{};

	#define LEFT 1;
	#define RIGHT 0;
	bool _direction;
	bool _mission_start = false;
	vehicle_global_position_s _global_pos;
	vehicle_global_position_s _global_pos_a;
	vehicle_global_position_s _global_pos_b;
};

