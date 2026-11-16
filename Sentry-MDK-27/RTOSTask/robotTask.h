#ifndef __ROBOTTASK_H
#define __ROBOTTASK_H

#include "cmsis_os2.h"
#include "Serial.h"
#include "math.h"
#include "remote.h"
#include "deamon.h"
#include "BMI088.h"
#include "IMU.h"
#include "bsp_usb.h"
#include "vision.h"

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "cmd.h"

void robot_init(void);

#endif
