#include <iostream>
#include "libservo_controller.h"

int main()
{
	ServoCtl servo_ctl;
	//	bool status = servo_ctl.initialize("/dev/ttyUSB0", 115200, KtechServo::CONTROL_RS485);
	bool status = servo_ctl.initialize("can0");
	auto res = servo_ctl.readMultiAngle(2);
	std::cout << "status " << status << std::endl;
	std::cout << "res angle " << (int)res.temp << std::endl;

    return 0;
}
