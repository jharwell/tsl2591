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
from tsl2591.srv import ReadingsService


def usage():
    return "Usage: %s " % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) != 1:
        print(usage())
        sys.exit(1)

    rospy.init_node('tsl2591_test_client', anonymous=True)

    rospy.wait_for_service('tsl2591/readings_service')
    readings_proxy = rospy.ServiceProxy('tsl2591/readings_service',
                                        ReadingsService)
    while not rospy.is_shutdown():
        rospy.loginfo(f"{__name__}: Requesting readings")
        try:

            resp = readings_proxy()
        except rospy.ServiceException as e:
            rospy.logerr(f"{__name__}: Service call failed: {e}")

        for r in resp.readings:
            rospy.loginfo(f"{__name__}: Got reading:\n{r}")

        rospy.sleep(1.0)
