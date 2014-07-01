/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file gnss_receiver.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Andrew Chambers <achamber@gmail.com>
 *
 */

#include "gnss_receiver.hpp"
#include <systemlib/err.h>

UavcanGnssReceiver::UavcanGnssReceiver(uavcan::INode &node) :
	_node(node),
	_uavcan_sub_status(node),
	_report_pub(-1)
{
}

int UavcanGnssReceiver::init()
{
	int res = -1;

	// GNSS fix subscription
	res = _uavcan_sub_status.start(FixCbBinder(this, &UavcanGnssReceiver::gnss_fix_sub_cb));
	if (res < 0)
	{
		warnx("GNSS fix sub failed %i", res);
		return res;
	}

	// Clear the uORB GPS report
	memset(&_report, 0, sizeof(_report));

	return res;
}

void UavcanGnssReceiver::gnss_fix_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix> &msg)
{
	_report.timestamp_position = hrt_absolute_time();

	_report.lat = (int32_t)msg.lat_1e7;
	_report.lon = (int32_t)msg.lon_1e7;
	_report.alt = (int32_t)msg.alt_1e2 * 10;			// Convert from decimeter to centimeter

	_report.timestamp_variance = hrt_absolute_time();
	_report.s_variance_m_s = 10.0f;
	_report.p_variance_m = 10.0f;
	_report.c_variance_rad = 0.1f;
	_report.fix_type = msg.status;

	_report.eph_m = 3.0f;
	_report.epv_m = 7.0f;
	_report.timestamp_velocity = hrt_absolute_time();

	_report.vel_n_m_s = (float)msg.ned_velocity[0];
	_report.vel_e_m_s = (float)msg.ned_velocity[1];
	_report.vel_d_m_s = (float)msg.ned_velocity[2];
	_report.vel_m_s = sqrtf(_report.vel_n_m_s * _report.vel_n_m_s + _report.vel_e_m_s * _report.vel_e_m_s + _report.vel_d_m_s * _report.vel_d_m_s);

	_report.cog_rad = 0.0f;
	_report.vel_ned_valid = true;

	_report.satellites_visible = msg.sats_used;

	warnx("GPS message received: %i, %i, %i", _report.lat, _report.lon, _report.alt);

	if (_report_pub > 0) {
		orb_publish(ORB_ID(vehicle_gps_position), _report_pub, &_report);

	} else {
		_report_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_report);
	}

}
