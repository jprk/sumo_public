#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2008-2021 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Jakob Erdmann
# @date    2017-01-23


from __future__ import print_function
from __future__ import absolute_import
import os
import sys
sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import traci  # noqa
import sumolib  # noqa


sumoBinary = os.environ["SUMO_BINARY"]

cmd = [
    sumoBinary,
    '-n', 'input_net2.net.xml',
    '--no-step-log', ]

vehID = "tripTest"
traci.start(cmd)
traci.route.add("trip", ["SC", "NC"])
traci.vehicle.add(vehID, "trip", departSpeed="desired", departPos="-1")
traci.vehicle.moveTo(vehID, "SC_1", traci.lane.getLength("SC_1") - 1)
print("route initial:", traci.vehicle.getRoute(vehID))
traci.simulationStep()
print("route computed:", traci.vehicle.getRoute(vehID))
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
traci.close()
