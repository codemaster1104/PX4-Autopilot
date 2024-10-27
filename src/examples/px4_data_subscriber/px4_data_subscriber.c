/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Application example for PX4 autopilot with roll, pitch, yaw calculations.
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

static void quaternion_to_euler(const float q[4], float *roll, float *pitch, float *yaw);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	/* subscribe to vehicle_acceleration topic */
	int accel_sub_fd = orb_subscribe(ORB_ID(vehicle_acceleration));
	orb_set_interval(accel_sub_fd, 200);

	/* subscribe to vehicle_attitude topic for roll, pitch, yaw */
	int attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_set_interval(attitude_sub_fd, 200);

	/* advertise attitude topic */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	px4_pollfd_struct_t fds[] = {
		{ .fd = accel_sub_fd,   .events = POLLIN },
		{ .fd = attitude_sub_fd,   .events = POLLIN },
	};

	int error_counter = 0;

	for (int i = 0; i < 5; i++) {
		int poll_ret = px4_poll(fds, 2, 1000);

		if (poll_ret == 0) {
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			if (error_counter < 10 || error_counter % 50 == 0) {
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}
			error_counter++;

		} else {
			if (fds[0].revents & POLLIN) {
				struct vehicle_acceleration_s accel;
				orb_copy(ORB_ID(vehicle_acceleration), accel_sub_fd, &accel);
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)accel.xyz[0],
					 (double)accel.xyz[1],
					 (double)accel.xyz[2]);

				att.q[0] = accel.xyz[0];
				att.q[1] = accel.xyz[1];
				att.q[2] = accel.xyz[2];
				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
			}

			if (fds[1].revents & POLLIN) {
				struct vehicle_attitude_s att_data;
				orb_copy(ORB_ID(vehicle_attitude), attitude_sub_fd, &att_data);

				/* Calculate roll, pitch, yaw from quaternion */
				float roll, pitch, yaw;
				quaternion_to_euler(att_data.q, &roll, &pitch, &yaw);

				PX4_INFO("Roll: %.2f, Pitch: %.2f, Yaw: %.2f",
					(double)roll, (double)pitch, (double)yaw);
			}
		}
	}

	PX4_INFO("Exiting");

	return 0;
}

/* Helper function to convert quaternion to roll, pitch, and yaw */
static void quaternion_to_euler(const float q[4], float *roll, float *pitch, float *yaw)
{
	*roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
		       1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
	*pitch = asinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
	*yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
		      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}
