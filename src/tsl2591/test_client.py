#!/usr/bin/env python3
# Copyright 2022 Shizhi Xu, All rights reserved.
#
#  This file is part of ROSBRIDGE.
#
#  ROSBRIDGE is free software: you can redistribute it and/or modify it under
#  the terms of the GNU General Public License as published by the Free Software
#  Foundation, either version 3 of the License, or (at your option) any later
#  version.
#
#  ROSBRIDGE is distributed in the hope that it will be useful, but WITHOUT ANY
#  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
#  A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License along with
#  ROSBRIDGE.  If not, see <http://www.gnu.org/licenses/

# Core packages
from __future__ import print_function
import sys

# 3rd party packages
import rospy

# Project packages
from tsl2591.srv import readings


def tsl2591_driver_client(status):
    rospy.wait_for_service('tsl2591/readings_service')
    try:
        light_sensor = rospy.ServiceProxy('tsl2591/readings_service', readings)
        resp1 = light_sensor(status)
        return resp1.reading1, resp1.reading2, resp1.reading3, resp1.reading4
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    return "%s [status]" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 2:
        status = int(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)

    print("Requesting status:%s" % (status))
    r1, r2, r3, r4 = tsl2591_driver_client(status)
    print("Status: %d\nr1: %f\nr2: %f\nr3: %f\nr4: %f\n" % (status, r1, r2, r3, r4))
    print("---------------------------------------------")
