
#include "lhx_test_module.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <drivers/drv_hrt.h>


const position_setpoint_s LhxTestModule::empty_position_setpoint = {0, NAN, NAN, 0, 0, 0, NAN, NAN, 0, 80.0,
	2.0, -1.0, NAN, false, position_setpoint_s::SETPOINT_TYPE_IDLE,
		false, false, false, false, false};

int LhxTestModule::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}
void LhxTestModule::record_a()
{
	memcpy(&_global_pos_a, &_global_pos, sizeof(_global_pos));
	PX4_INFO("Record point A, Lat: %.6f \t Lon: %.6f", _global_pos_a.lat, _global_pos_a.lon);
}
int LhxTestModule::custom_command(int argc, char *argv[])
{

	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "RA")) {
		get_instance()->record_a();
		return 0;
	}


	return print_usage("unknown command");
}


int LhxTestModule::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

LhxTestModule *LhxTestModule::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	LhxTestModule *instance = new LhxTestModule(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

LhxTestModule::LhxTestModule(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void LhxTestModule::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	orb_set_interval(global_pos_sub, 500);

	int yaw_sub = orb_subscribe(ORB_ID(yaw_estimator_status));
	trajectory_setpoint_lhx_s target{0, empty_position_setpoint, empty_position_setpoint,
		empty_position_setpoint};
	// memset(&target, 0, sizeof(target));
	// orb_advert_t trajectory_pub = orb_advertise(ORB_ID(trajectory_setpoint_lhx), &target);

	px4_pollfd_struct_t fds[2];
	fds[0].fd = global_pos_sub;
	fds[0].events = POLLIN;
	fds[1].fd = yaw_sub;
	fds[1].events = POLLIN;

	// initialize parameters
	parameters_update(true);

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &_global_pos);

			yaw_estimator_status_s yaw_status;
			orb_copy(ORB_ID(yaw_estimator_status), yaw_sub, &yaw_status);

			// TODO: do something with the data...
			target.previous.yaw = yaw_status.yaw[0];
			target.previous.lat = _global_pos.lat;
			target.previous.lon = _global_pos.lon;
			target.previous.alt = _global_pos.alt;
			// target.previous.loiter_radius = 80.0;
			// target.previous.acceptance_radius = 2.0;
			// target.previous.cruising_speed = -1.0;
			// target.previous.cruising_throttle = NAN;
			// target.previous.valid = false;
			// target.previous.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
			// target.previous.yaw_valid = false;
			target.previous.yawspeed_valid = false;

			target.previous.timestamp = hrt_absolute_time();

			// memcpy(&(target.current), &(target.previous), sizeof(target.previous));
			target.current.yaw = NAN;
			target.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
			target.current.valid = true;
			target.current.lat = _global_pos.lat;
			target.current.lon = _global_pos.lon + 0.0004;
			target.current.alt = _global_pos.alt;

			target.current.loiter_radius = 80.0;
			target.current.acceptance_radius = 2.0;
			target.current.timestamp = hrt_absolute_time();

			memcpy(&(target.next), &(target.current), sizeof(target.current));
			target.next.valid = false;

			target.timestamp = hrt_absolute_time();
			// orb_publish(ORB_ID(trajectory_setpoint_lhx), trajectory_pub, &target);
			// break;
		}

		parameters_update();
	}

	orb_unsubscribe(global_pos_sub);
}

void LhxTestModule::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int LhxTestModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int lhx_test_module_main(int argc, char *argv[])
{
	return LhxTestModule::main(argc, argv);
}
