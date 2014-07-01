/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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

#include <nuttx/config.h>

#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/mixer/mixer.h>
#include <arch/board/board.h>
#include <arch/chip/chip.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>

#include "uavcan_main.hpp"

/**
 * @file uavcan_main.cpp
 *
 * Implements basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

/*
 * UavcanNode
 */
UavcanNode *UavcanNode::_instance;

UavcanNode::UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock) :
	_node(can_driver, system_clock),
	_gnss_receiver(_node)
{
	// memset(_controls, 0, sizeof(_controls));
	// memset(_poll_fds, 0, sizeof(_poll_fds));
}

UavcanNode::~UavcanNode()
{
	if (_task != -1) {
		/* tell the task we want it to go away */
		_task_should_exit = true;

		unsigned i = 10;

		do {
			/* wait 5ms - it should wake every 10ms or so worst-case */
			usleep(5000);

			/* if we have given up, kill it */
			if (--i == 0) {
				task_delete(_task);
				break;
			}

		} while (_task != -1);
	}

	/* clean up the alternate device node */
		// unregister_driver(PWM_OUTPUT_DEVICE_PATH);

	_instance = nullptr;
}

int UavcanNode::start(uavcan::NodeID node_id, uint32_t bitrate)
{
	if (_instance != nullptr) {
		warnx("Already started");
		return -1;
	}

	/*
	 * GPIO config.
	 * Forced pull up on CAN2 is required for Pixhawk v1 where the second interface lacks a transceiver.
	 * If no transceiver is connected, the RX pin will float, occasionally causing CAN controller to
	 * fail during initialization.
	 */
	stm32_configgpio(GPIO_CAN1_RX);
	stm32_configgpio(GPIO_CAN1_TX);
	stm32_configgpio(GPIO_CAN2_RX | GPIO_PULLUP);
	stm32_configgpio(GPIO_CAN2_TX);

	/*
	 * CAN driver init
	 */
	static CanInitHelper can;
	static bool can_initialized = false;

	if (!can_initialized) {
		const int can_init_res = can.init(bitrate);

		if (can_init_res < 0) {
			warnx("CAN driver init failed %i", can_init_res);
			return can_init_res;
		}

		can_initialized = true;
	}

	/*
	 * Node init
	 */
	_instance = new UavcanNode(can.driver, uavcan_stm32::SystemClock::instance());

	if (_instance == nullptr) {
		warnx("Out of memory");
		return -1;
	}

	const int node_init_res = _instance->init(node_id);

	if (node_init_res < 0) {
		delete _instance;
		_instance = nullptr;
		warnx("Node init failed %i", node_init_res);
		return node_init_res;
	}

	/*
	 * Start the task. Normally it should never exit.
	 */
	static auto run_trampoline = [](int, char *[]) {return UavcanNode::_instance->run();};
	_instance->_task = task_spawn_cmd("uavcan", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, StackSize,
			      static_cast<main_t>(run_trampoline), nullptr);

	if (_instance->_task < 0) {
		warnx("start failed: %d", errno);
		return -errno;
	}

	return OK;
}

int UavcanNode::init(uavcan::NodeID node_id)
{
	int ret = -1;

	ret = _gnss_receiver.init();
	if (ret < 0)
		return ret;

	uavcan::protocol::SoftwareVersion swver;
	swver.major = 12;                        // TODO fill version info
	swver.minor = 34;
	_node.setSoftwareVersion(swver);

	uavcan::protocol::HardwareVersion hwver;
	hwver.major = 42;                        // TODO fill version info
	hwver.minor = 42;
	_node.setHardwareVersion(hwver);

	_node.setName("org.pixhawk"); // Huh?

	_node.setNodeID(node_id);

	return _node.start();
}

void UavcanNode::node_spin_once()
{
	const int spin_res = _node.spin(uavcan::MonotonicTime());
	if (spin_res < 0) {
		warnx("node spin error %i", spin_res);
	}
}

int UavcanNode::run()
{
	const unsigned PollTimeoutMs = 50;

	const int busevent_fd = ::open(uavcan_stm32::BusEvent::DevName, 0);
	if (busevent_fd < 0)
	{
		warnx("Failed to open %s", uavcan_stm32::BusEvent::DevName);
		_task_should_exit = true;
	}

	_node.setStatusOk();

	while (!_task_should_exit) {

		if (_poll_fds_num == 0) {
			/*
			 * This event is needed to wake up the thread on CAN bus activity (RX/TX/Error).
			 * Please note that with such multiplexing it is no longer possible to rely only on
			 * the value returned from poll() to detect whether actuator control has timed out or not.
			 * Instead, all ORB events need to be checked individually (see below).
			 */
			_poll_fds[_poll_fds_num] = ::pollfd();
			_poll_fds[_poll_fds_num].fd = busevent_fd;
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num += 1;
		}

		const int poll_ret = ::poll(_poll_fds, _poll_fds_num, PollTimeoutMs);

		/*
		 * If a bus event is detected or the timeout occurred, the CAN bus
		 * node will spin once and re-enter the loop.
		 */
		node_spin_once();  // Non-blocking

		// Check the state of poll return value
		if (poll_ret < 0) {
			// Error occurred
			warnx("poll error %d", errno);
			continue;
		} else if (poll_ret == 0) {
			// Timeout occurred

		} else {

			// CAN bus event
			// Don't think that I need to do anything here
			bool controls_updated = false;
			unsigned poll_id = 0;
			if (_poll_fds[poll_id].revents & POLLIN) {

			}

		}

	}

	teardown();
	warnx("exiting.");

	exit(0);
}

int
UavcanNode::teardown()
{
	return 1;
}

void
UavcanNode::print_info()
{
	if (!_instance) {
		warnx("not running, start first");
	}

	warnx("groups: sub: %u / req: %u / fds: %u", (unsigned)0, (unsigned)0, _poll_fds_num);
}

/*
 * App entry point
 */
static void print_usage()
{
	warnx("usage: uavcan start <node_id> [can_bitrate]");
}

extern "C" __EXPORT int uavcan_main(int argc, char *argv[]);

int uavcan_main(int argc, char *argv[])
{
	constexpr unsigned DEFAULT_CAN_BITRATE = 1000000;

	if (argc < 2) {
		print_usage();
		::exit(1);
	}

	if (!std::strcmp(argv[1], "start")) {
		if (argc < 3) {
			print_usage();
			::exit(1);
		}

		/*
		 * Node ID
		 */
		const int node_id = atoi(argv[2]);

		if (node_id < 0 || node_id > uavcan::NodeID::Max || !uavcan::NodeID(node_id).isUnicast()) {
			warnx("Invalid Node ID %i", node_id);
			::exit(1);
		}

		/*
		 * CAN bitrate
		 */
		unsigned bitrate = 0;

		if (argc > 3) {
			bitrate = atol(argv[3]);
		}

		if (bitrate <= 0) {
			bitrate = DEFAULT_CAN_BITRATE;
		}

		if (UavcanNode::instance()) {
			errx(1, "already started");
		}

		/*
		 * Start
		 */
		warnx("Node ID %u, bitrate %u", node_id, bitrate);
		return UavcanNode::start(node_id, bitrate);

	}

	/* commands below require the app to be started */
	UavcanNode *inst = UavcanNode::instance();

	if (!inst) {
		errx(1, "application not running");
	}

	if (!std::strcmp(argv[1], "status") || !std::strcmp(argv[1], "info")) {
		
			inst->print_info();
			return OK;
	}

	if (!std::strcmp(argv[1], "stop")) {
		
			delete inst;
			inst = nullptr;
			return OK;
	}

	print_usage();
	::exit(1);
}
