/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef TILTROTOR_ANGLE_HPP
#define TILTROTOR_ANGLE_HPP

#include <uORB/topics/vehicle_tiltrotor_angle_setpoint.h>

class MavlinkStreamTiltrotorAngle : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamTiltrotorAngle(mavlink); }

	static constexpr const char *get_name_static() { return "TILTROTOR_ANGLE_SETPOINT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TILTROTOR_ANGLE_SETPOINT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _tiltrotor_angle_sub.advertised() ? (MAVLINK_MSG_ID_TILTROTOR_ANGLE_SETPOINT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamTiltrotorAngle(Mavlink *mavlink) : MavlinkStream(mavlink) {}
	
	uORB::Subscription _tiltrotor_angle_sub{ORB_ID(vehicle_tiltrotor_angle_setpoint)};
	bool send() override
	{
		vehicle_tiltrotor_angle_setpoint_s tilt;

		if (_tiltrotor_angle_sub.update(&tilt)) {
			// Create and populate the MAVLink message structure
			mavlink_tiltrotor_angle_setpoint_t msg{};
			msg.timestamp = tilt.timestamp / 1000ULL;
			msg.tiltrotor_angle = tilt.tiltrotor_angle;

			// Send the message over MAVLink
			mavlink_msg_tiltrotor_angle_setpoint_send_struct(_mavlink->get_channel(), &msg);
			PX4_WARN("uorb => mavlink - message was sent !!!!");
			return true;
		}
		else {
			PX4_WARN("No data received from vehicle_tiltrotor_angle_setpoint.");
		}

		return false;
	}
};

#endif // TILTROTOR_ANGLE_HPP
