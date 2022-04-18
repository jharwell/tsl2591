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
import typing as tp
import math

# 3rd party packages
import rospy
import board
import adafruit_tca9548a
import adafruit_tsl2591
from std_msgs.msg import Empty

# Project packages
from tsl2591.srv import ReadingsService, ReadingsServiceResponse, ReadingsServiceRequest
from tsl2591.msg import SensorReading


class TSL2591ArrayDriver:
    def __init__(self) -> None:
        i2c = board.I2C()
        mp_bus = adafruit_tca9548a.TCA9548A(i2c)
        self.sensors = [
            adafruit_tsl2591.TSL2591(mp_bus[0]),
            adafruit_tsl2591.TSL2591(mp_bus[1]),
            adafruit_tsl2591.TSL2591(mp_bus[2]),
            adafruit_tsl2591.TSL2591(mp_bus[3])
        ]

    def detect_lux(self) -> tp.List[float]:
        return [s.lux for s in self.sensors]

    def handle_service_request(self, req: ReadingsServiceRequest) -> ReadingsServiceResponse:
        luxes = self.detect_lux()

        # Swap last two readings because of wiring on the turtlebot
        readings = [SensorReading(luxes[0], 0),
                    SensorReading(luxes[1], math.pi / 2.0),
                    SensorReading(luxes[3], math.pi * 3.0 / 2.0),
                    SensorReading(luxes[2], math.pi)]

        return ReadingsServiceResponse(readings)
