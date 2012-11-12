/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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

/**
 * @file ardrone_opt_control_main.c
 *
 * Implementation of multirotor attitude control main loop.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>
#include <errno.h>
#include <termios.h>
#include <systemlib/err.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <getopt.h>
#include <time.h>
#include <math.h>
#include <poll.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <drivers/drv_gyro.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>

#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>

#include "ardrone_motor_control.h"

 __EXPORT int ardrone_opt_control_main(int argc, char *argv[]);

static int mc_task;

static orb_advert_t actuator_pub;

static struct vehicle_status_s state;

static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int ardrone_interface_task;		/**< Handle of deamon task / thread */
static int ardrone_write;			/**< UART to write AR.Drone commands to */

/**
 * Open the UART connected to the motor controllers
 */
static int ardrone_open_uart(char *uart_name, struct termios *uart_config_original);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static int ardrone_open_uart(char *uart_name, struct termios *uart_config_original)
{
	/* baud rate */
	int speed = B115200;
	int uart;

	/* open uart */
	uart = open(uart_name, O_RDWR | O_NOCTTY);

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
		fprintf(stderr, "[ardrone_interface] ERROR getting baudrate / termios config for %s: %d\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(uart, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		fprintf(stderr, "[ardrone_interface] ERROR setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
		close(uart);
		return -1;
	}


	if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
		fprintf(stderr, "[ardrone_interface] ERROR setting baudrate / termios config for %s (tcsetattr)\n", uart_name);
		close(uart);
		return -1;
	}

	return uart;
}

static int
mc_thread_main(int argc, char *argv[])
{
	/* declare and safely initialize all structs */
	memset(&state, 0, sizeof(state));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));
	struct offboard_control_setpoint_s offboard_sp;
	memset(&offboard_sp, 0, sizeof(offboard_sp));
	struct vehicle_rates_setpoint_s rates_sp;
	memset(&rates_sp, 0, sizeof(rates_sp));

	struct actuator_controls_s actuators;

	/* subscribe to attitude, motor setpoints and system state */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int param_sub = orb_subscribe(ORB_ID(parameter_update));
	int att_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int setpoint_sub = orb_subscribe(ORB_ID(offboard_control_setpoint));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));

	/* 
	 * Do not rate-limit the loop to prevent aliasing
	 * if rate-limiting would be desired later, the line below would
	 * enable it.
	 *
	 * rate-limit the attitude subscription to 200Hz to pace our loop
	 * orb_set_interval(att_sub, 5);
	 */
	struct pollfd fds[2] = {
					{ .fd = att_sub, .events = POLLIN },
					{ .fd = param_sub, .events = POLLIN }
				};

	char *device = "/dev/ttyS1";

	/* welcome user */
	printf("[ardrone_interface] Control started, taking over motors\n");

	/* File descriptors */
	int gpios;

	char *commandline_usage = "\tusage: ardrone_interface start|status|stop [-d /dev/ttySx]\n";

	/* read commandline arguments */
	for (int i = 0; i < argc && argv[i]; i++) {
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) { //device set
			if (argc > i + 1) {
				device = argv[i + 1];

			} else {
				thread_running = false;
				errx(1, "missing parameter to -d, e.g. /dev/ttyS1\n %s", commandline_usage);
			}
		}
	}

	struct termios uart_config_original;

	/* Led animation */
	int counter = 0;
	int led_counter = 0;

	/* declare and safely initialize all structs */
	struct actuator_controls_s actuator_controls;
	memset(&actuator_controls, 0, sizeof(actuator_controls));
	struct actuator_armed_s armed;
	armed.armed = false;

	/* subscribe to attitude, motor setpoints and system state */
	int actuator_controls_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	printf("[ardrone_interface] Motors initialized - ready.\n");
	fflush(stdout);

	/* enable UART, writes potentially an empty buffer, but multiplexing is disabled */
	ardrone_write = ardrone_open_uart(device, &uart_config_original);

	/* initialize multiplexing, deactivate all outputs - must happen after UART open to claim GPIOs on PX4FMU */
	gpios = ar_multiplexing_init();

	if (ardrone_write < 0) {
		fprintf(stderr, "[ardrone_interface] Failed opening AR.Drone UART, exiting.\n");
		thread_running = false;
		exit(ERROR);
	}

	/* initialize motors */
	if (OK != ar_init_motors(ardrone_write, gpios)) {
		close(ardrone_write);
		fprintf(stderr, "[ardrone_interface] Failed initializing AR.Drone motors, exiting.\n");
		thread_running = false;
		exit(ERROR);
	}

	ardrone_write_motor_commands(ardrone_write, 0, 0, 0, 0);


	// XXX Re-done initialization to make sure it is accepted by the motors
	// XXX should be removed after more testing, but no harm

	/* close uarts */
	close(ardrone_write);

	/* enable UART, writes potentially an empty buffer, but multiplexing is disabled */
	ardrone_write = ardrone_open_uart(device, &uart_config_original);

	/* initialize multiplexing, deactivate all outputs - must happen after UART open to claim GPIOs on PX4FMU */
	gpios = ar_multiplexing_init();

	if (ardrone_write < 0) {
		fprintf(stderr, "[ardrone_interface] Failed opening AR.Drone UART, exiting.\n");
		thread_running = false;
		exit(ERROR);
	}

	/* initialize motors */
	if (OK != ar_init_motors(ardrone_write, gpios)) {
		close(ardrone_write);
		fprintf(stderr, "[ardrone_interface] Failed initializing AR.Drone motors, exiting.\n");
		thread_running = false;
		exit(ERROR);
	}

	/* publish actuator controls */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
	orb_advert_t rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);
	int rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));

	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "ardrone_opt_control");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "ardrone_opt_control_err");

	/* welcome user */
	printf("[multirotor_att_control] starting\n");

	/* store last control mode to detect mode switches */
	bool flag_control_manual_enabled = false;
	bool flag_control_attitude_enabled = false;
	bool flag_system_armed = false;
	bool man_yaw_zero_once = false;

	uint64_t last_run = 0;

	while (!thread_should_exit) {

		/* wait for a sensor update, check for exit condition every 500 ms */
		int ret = poll(fds, 2, 500);

		if (ret < 0) {
			/* poll error, count it in perf */
			perf_count(mc_err_perf);
		} else if (ret == 0) {
			/* no return value, ignore */
		} else {

			/* only update parameters if they changed */
			if (fds[1].revents & POLLIN) {
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), param_sub, &update);

				/* update parameters */
				// XXX no params here yet
			}

			/* only run controller if attitude changed */
			if (fds[0].revents & POLLIN) {

				perf_begin(mc_loop_perf);

				/* get a local copy of system state */
				bool updated;
				orb_check(state_sub, &updated);
				if (updated) {
					orb_copy(ORB_ID(vehicle_status), state_sub, &state);
				}
				/* get a local copy of manual setpoint */
				orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
				/* get a local copy of attitude */
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
				/* get a local copy of attitude setpoint */
				orb_copy(ORB_ID(vehicle_attitude_setpoint), att_setpoint_sub, &att_sp);
				/* get a local copy of rates setpoint */
				orb_check(setpoint_sub, &updated);
				orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);

				if (updated) {
					orb_copy(ORB_ID(offboard_control_setpoint), setpoint_sub, &offboard_sp);
				}
				/* get a local copy of the current sensor values */
				orb_copy(ORB_ID(sensor_combined), sensor_sub, &raw);



				/** CALCULATE TIME DIFFERENCE TO LAST RUN */
				float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
				last_run = hrt_absolute_time();


				/** STEP 1: Define which input is the dominating control input */
				if (state.flag_control_offboard_enabled) {



					#
					#
					#
					#
					#
					#
					#
					#
					#
					#
					#




					#warning Enter here offboard control outputs

												/* offboard inputs */
					if (offboard_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_RATES) {
						rates_sp.roll = offboard_sp.p1;
						rates_sp.pitch = offboard_sp.p2;
						rates_sp.yaw = offboard_sp.p3;
						rates_sp.thrust = offboard_sp.p4;
//						printf("thrust_rate=%8.4f\n",offboard_sp.p4);
						rates_sp.timestamp = hrt_absolute_time();
						orb_publish(ORB_ID(vehicle_rates_setpoint), rates_sp_pub, &rates_sp);
					} else if (offboard_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_ATTITUDE) {
						att_sp.roll_body = offboard_sp.p1;
						att_sp.pitch_body = offboard_sp.p2;
						att_sp.yaw_body = offboard_sp.p3;
						att_sp.thrust = offboard_sp.p4;
//						printf("thrust_att=%8.4f\n",offboard_sp.p4);
						att_sp.timestamp = hrt_absolute_time();
						/* STEP 2: publish the result to the vehicle actuators */
						orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
					}

					uint16_t outputs[4] = {0, 0, 0, 0};

					outputs[0] = 10;
					outputs[1] = 10;
					outputs[2] = 0;
					outputs[3] = 50 * att.roll;

					/* for now only spin if armed and immediately shut down
					 * if in failsafe
					 */
					if (armed.armed && !armed.lockdown) {
						ardrone_mixing_and_output(ardrone_write, &actuator_controls);

					} else {
						/* Silently lock down motor speeds to zero */
						ardrone_write_motor_commands(ardrone_write, outputs[0], outputs[1], outputs[2], outputs[3]);
					}

					#
					#
					#
					#
					#
					#
					#
					#
					#
					#
					#



				} else if (state.flag_control_manual_enabled) {

					#
					#
					#
					#
					#
					#
					#
					#
					#
					#
					#

					// if (state.flag_control_attitude_enabled) {

					// 	/* initialize to current yaw if switching to manual or att control */
					// 	if (state.flag_control_attitude_enabled != flag_control_attitude_enabled ||
					//  	    state.flag_control_manual_enabled != flag_control_manual_enabled ||
					//  	    state.flag_system_armed != flag_system_armed) {
					// 		att_sp.yaw_body = att.yaw;
					// 	}

					// 	att_sp.roll_body = manual.roll;
					// 	att_sp.pitch_body = manual.pitch;

					// 	/* only move setpoint if manual input is != 0 */
					// 	// XXX turn into param
					// 	if ((manual.yaw < -0.01f || 0.01f < manual.yaw) && manual.throttle > 0.3f) {
					// 		att_sp.yaw_body = att_sp.yaw_body + manual.yaw * 0.0025f;
					// 	} else if (manual.throttle <= 0.3f) {
					// 		att_sp.yaw_body = att.yaw;
					// 	}
					// 	att_sp.thrust = manual.throttle;
					// 	att_sp.timestamp = hrt_absolute_time();
					// }

					#warning Enter here manual control outputs

					uint16_t outputs[4] = {0, 0, 0, 0};

					outputs[0] = 10;
					outputs[1] = 10;
					outputs[2] = 0;
					outputs[3] = 50 * att.roll;

					/* for now only spin if armed and immediately shut down
					 * if in failsafe
					 */
					if (armed.armed && !armed.lockdown) {
						ardrone_mixing_and_output(ardrone_write, &actuator_controls);

					} else {
						/* Silently lock down motor speeds to zero */
						ardrone_write_motor_commands(ardrone_write, outputs[0], outputs[1], outputs[2], outputs[3]);
					}

					#
					#
					#
					#
					#
					#
					#
					#
					#
					#
					#
				}






				/* led fun */
				if (counter % 10 == 0) {
					if (led_counter == 0) ar_set_leds(ardrone_write, 0, 1, 0, 0, 0, 0, 0 , 0);

					if (led_counter == 1) ar_set_leds(ardrone_write, 1, 1, 0, 0, 0, 0, 0 , 0);

					if (led_counter == 2) ar_set_leds(ardrone_write, 1, 0, 0, 0, 0, 0, 0 , 0);

					if (led_counter == 3) ar_set_leds(ardrone_write, 0, 0, 0, 1, 0, 0, 0 , 0);

					if (led_counter == 4) ar_set_leds(ardrone_write, 0, 0, 1, 1, 0, 0, 0 , 0);

					if (led_counter == 5) ar_set_leds(ardrone_write, 0, 0, 1, 0, 0, 0, 0 , 0);

					if (led_counter == 6) ar_set_leds(ardrone_write, 0, 0, 0, 0, 0, 1, 0 , 0);

					if (led_counter == 7) ar_set_leds(ardrone_write, 0, 0, 0, 0, 1, 1, 0 , 0);

					if (led_counter == 8) ar_set_leds(ardrone_write, 0, 0, 0, 0, 1, 0, 0 , 0);

					if (led_counter == 9) ar_set_leds(ardrone_write, 0, 0, 0, 0, 0, 0, 0 , 1);

					if (led_counter == 10) ar_set_leds(ardrone_write, 0, 0, 0, 0, 0, 0, 1 , 1);

					if (led_counter == 11) ar_set_leds(ardrone_write, 0, 0, 0, 0, 0, 0, 1 , 0);

					led_counter++;

					if (led_counter == 12) led_counter = 0;
				}

				perf_end(mc_loop_perf);
			} /* end of poll call for attitude updates */
		} /* end of poll return value check */
	}

	printf("[multirotor att control] stopping, disarming motors.\n");

	/* restore old UART config */
	int termios_state;

	if ((termios_state = tcsetattr(ardrone_write, TCSANOW, &uart_config_original)) < 0) {
		fprintf(stderr, "[ardrone_interface] ERROR setting baudrate / termios config for (tcsetattr)\n");
	}

	printf("[ardrone_interface] Restored original UART config, exiting..\n");

	/* close uarts */
	close(ardrone_write);
	ar_multiplexing_deinit(gpios);

	fflush(stdout);

	thread_running = false;

	close(att_sub);
	close(state_sub);
	close(manual_sub);
	close(actuator_pub);
	close(att_sp_pub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	exit(0);
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: ardrone_opt_control [-d <serial port>] {start|status|stop}\n");
	exit(1);
}

int ardrone_opt_control_main(int argc, char *argv[])
{

	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		thread_should_exit = false;
		mc_task = task_spawn("ardrone_opt_control",
				     SCHED_DEFAULT,
				     SCHED_PRIORITY_MAX - 15,
				     2048,
				     mc_thread_main,
				     NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
