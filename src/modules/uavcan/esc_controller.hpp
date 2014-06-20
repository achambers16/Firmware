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
 * @file esc_controller.hpp
 *
 * UAVCAN <--> ORB bridge for ESC messages:
 *     uavcan.equipment.esc.RawCommand
 *     uavcan.equipment.esc.RPMCommand
 *     uavcan.equipment.esc.Status
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/esc/Status.hpp>
#include <uavcan/equipment/gnss/Fix.hpp>

class UavcanEscController
{
public:
	UavcanEscController(uavcan::INode& node);

	int init();

private:
	/**
	 * ESC status message reception will be reported via this callback.
	 */
	void gnss_fix_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix> &msg);

	/**
	 * ESC status will be published to ORB from this callback (fixed rate).
	 */
	void orb_pub_timer_cb(const uavcan::TimerEvent &event);


	static const unsigned MAX_RATE_HZ = 100;			///< XXX make this configurable
	static const unsigned ESC_STATUS_UPDATE_RATE_HZ = 5;


	typedef uavcan::MethodBinder<UavcanEscController*,
		void (UavcanEscController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix>&)>
		FixCbBinder;

	typedef uavcan::MethodBinder<UavcanEscController*, void (UavcanEscController::*)(const uavcan::TimerEvent&)>
		TimerCbBinder;

	/*
	 * libuavcan related things
	 */
	uavcan::MonotonicTime											_prev_cmd_pub;   ///< rate limiting
	uavcan::INode													&_node;
	uavcan::Publisher<uavcan::equipment::esc::RawCommand>			_uavcan_pub_raw_cmd;
	uavcan::Subscriber<uavcan::equipment::gnss::Fix, FixCbBinder>	_uavcan_sub_status;
	uavcan::TimerEventForwarder<TimerCbBinder>						_orb_timer;

	/*
	 * GNSS msg
	 */
	uavcan::equipment::gnss::Fix	fixMsg;

	/*
	 * uORB
	 */
	struct vehicle_gps_position_s 	_report;					///< uORB topic for gps position
	orb_advert_t			_report_pub;					///< uORB pub for gps position

};
