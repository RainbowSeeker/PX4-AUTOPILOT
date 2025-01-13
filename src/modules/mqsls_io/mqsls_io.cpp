#include "mqsls_io.hpp"
#include <termios.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/cli.h>

MqslsIo::MqslsIo(const char *port, int baudrate) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": interval")),
	_baudrate(baudrate)
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';
}

MqslsIo::~MqslsIo()
{
	perf_free(_loop_perf);
}

void MqslsIo::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_fd < 0) {
		_fd = open_serial(_port, _baudrate);

		if (_fd < 0) {
			PX4_ERR("open failed");
			return;
		}
	}

	collect_data();

	perf_end(_loop_perf);
}

int MqslsIo::collect_data()
{
	// Check the number of bytes available in the buffer
	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if (!bytes_available) {
		return -EAGAIN;
	}

	char rbuf[128];
	unsigned rlen = sizeof(rbuf) - 1;

	/* read from the uart buffer */
	int nread = ::read(_fd, &rbuf[0], rlen);

	if (nread < 0) {
		PX4_DEBUG("read err: %d", errno);
		return -EAGAIN;

	} else if (nread == 0) {
		return -EAGAIN;
	}

	_byte_count += nread;

	bool valid = false;
	for (int i = 0; i < nread; i++) {
		if (unicore::parse(rbuf[i], _msg) == PX4_OK) {
			switch (_msg.header.msg_id) {
			case unicore::msg_id::MSG_ID_BESTNAVXYZ:
			{
				auto body = reinterpret_cast<unicore::msg::bestnavxyz *>(&_msg.payload);
				_mqsls_share.uav_vel[0] = body->vel[0];
				_mqsls_share.uav_vel[1] = -body->vel[1];
				_mqsls_share.uav_vel[2] = -body->vel[2];
				PX4_INFO("BESTNAVXYZ: %f, %f, %f", _mqsls_share.uav_vel[0], _mqsls_share.uav_vel[1], _mqsls_share.uav_vel[2]);
				break;
			}
			case unicore::msg_id::MSG_ID_BESTNAVXYZH:
			{
				auto body = reinterpret_cast<unicore::msg::bestnavxyz *>(&_msg.payload);
				_mqsls_share.load_vel[0] = body->vel[0];
				_mqsls_share.load_vel[1] = -body->vel[1];
				_mqsls_share.load_vel[2] = -body->vel[2];
				PX4_INFO("BESTNAVXYZH: %f, %f, %f", _mqsls_share.load_vel[0], _mqsls_share.load_vel[1], _mqsls_share.load_vel[2]);
				break;
			}
			case unicore::msg_id::MSG_ID_UNIHEADING:
			{
				auto body = reinterpret_cast<unicore::msg::uniheading *>(&_msg.payload);
				float heading_rad = body->heading * M_PI_F / 180.0f;
				if (heading_rad > M_PI_F) {
					heading_rad -= 2.f * M_PI_F;
				}
				float pitch_rad = body->pitch * M_PI_F / 180.0f;
				_mqsls_share.delta_pos[0] = cosf(heading_rad) * cosf(pitch_rad) * body->baseline;
				_mqsls_share.delta_pos[1] = sinf(heading_rad) * cosf(pitch_rad) * body->baseline;
				_mqsls_share.delta_pos[2] = -sinf(pitch_rad) * body->baseline;
				_mqsls_share.timestamp = hrt_absolute_time();
				_mqsls_share_pub.publish(_mqsls_share);

				// PX4_INFO("UNIHEADING: len: %.3f, heading: %.2f, pitch: %.2f", (double)body->baseline, (double)body->heading, (double)body->pitch);
				// PX4_INFO("UNIHEADING: delta_pos: %.3f, %.3f, %.3f", _mqsls_share.delta_pos[0], _mqsls_share.delta_pos[1], _mqsls_share.delta_pos[2]);
				break;
			}
			default:
				PX4_ERR("Unknown message ID: %d", _msg.header.msg_id);
				break;
			}
			_packet_count++;
			valid = true;
		}
	}

	if (!valid) {
		return -EAGAIN;
	}

	return PX4_OK;
}

bool MqslsIo::init()
{
	// 5ms interval
	ScheduleOnInterval(5_ms);

	return true;
}

int MqslsIo::print_status()
{
	perf_print_counter(_loop_perf);
	PX4_INFO("Bytes: %" PRIu64 ", Packets: %" PRIu64, _byte_count, _packet_count);
	return 0;
}


int MqslsIo::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	// format: command + port
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for MQSLS_IO.

### Usage
Start the driver with a given device:
$ mqsls_io <command> <port>
	)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mqsls_io", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Name of device for serial communication with MQSLS_IO", false);
	PRINT_MODULE_USAGE_PARAM_STRING('b', nullptr, "<int>", "Baudrate for serial communication", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}


int MqslsIo::open_serial(const char *port, int baudrate)
{
	int speed = B115200;
	switch (baudrate)
	{
	case 9600:speed = B9600; break;
	case 19200:speed = B19200; break;
	case 38400:speed = B38400; break;
	case 57600:speed = B57600; break;
	case 115200:speed = B115200; break;
	case 230400:speed = B230400; break;
	case 460800:speed = B460800; break;
	case 921600:speed = B921600; break;
	default:
		PX4_ERR("Unsupported baudrate: %d", baudrate);
		return -1;
	}

	/* open fd */
	int fd = ::open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		PX4_ERR("open failed: %s", strerror(errno));
		return fd;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(fd, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_ERR("CFG: %d ISPD:%s", termios_state, strerror(errno));
		goto fail;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_ERR("CFG: %d OSPD:%s", termios_state, strerror(errno));
		goto fail;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("baud %d ATTR:%s", termios_state, strerror(errno));
		goto fail;
	}

	PX4_INFO("Opened %s, baudrate: %d", port, baudrate);

	return fd;

fail:
	::close(fd);
	return -1;
}

int MqslsIo::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MqslsIo::task_spawn(int argc, char *argv[])
{
	int ch;
	int option_index = 1;
	const char *option_arg;
	const char *device_name = "/dev/ttyS2";
	int baudrate = 115200;

	while ((ch = px4_getopt(argc, argv, "d:b:", &option_index, &option_arg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = option_arg;
			break;

		case 'b':
			baudrate = atoi(option_arg);
			break;

		default:
			PX4_WARN("Unrecognized flag: %c", ch);
			break;
		}
	}

	PX4_INFO("Setup with device: %s, baudrate: %d", device_name, baudrate);

	MqslsIo *instance = new MqslsIo(device_name, baudrate);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}
	else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

extern "C" __EXPORT int mqsls_io_main(int argc, char *argv[])
{
	return MqslsIo::main(argc, argv);
}
