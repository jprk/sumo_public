/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2012-2026 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    MSElecHybridExport.cpp
/// @author  Jakub Sevcik (RICE)
/// @author  Jan Prikryl (RICE)
/// @date    2019-11-25
///
// Realises dumping Electric hybrid vehicle data
/****************************************************************************/
#include <config.h>

#include <microsim/MSEdgeControl.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSGlobals.h>
#include <utils/iodevices/OutputDevice.h>
#include <microsim/MSNet.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/devices/MSDevice_ElecHybrid.h>
#include "MSElecHybridExport.h"


// ===========================================================================
// internal method definitions
// ===========================================================================
void writeElecHybridData(OutputDevice& of, const SUMOVehicle *veh, const MSDevice_ElecHybrid* elecHybridToExport) {
    // Write Actual battery capacity
    of.writeAttr(SUMO_ATTR_ACTUALBATTERYCAPACITY, elecHybridToExport->getActualBatteryCapacity());
    // Write consumed energy [Wh] (computed by HelpersEnergy::compute)
    of.writeAttr(SUMO_ATTR_ENERGYCONSUMED, elecHybridToExport->getConsum());
    // Write Energy charged in the Battery [Wh] (drawn energy from overhead wire minus consumed energy)
    of.writeAttr(SUMO_ATTR_ENERGYCHARGED, elecHybridToExport->getEnergyCharged());
    // Write Power demand (requsted from overhed wire) [W]
    of.writeAttr(SUMO_ATTR_CHARGINGPOWER, elecHybridToExport->getPowerWanted());
    // Write OverheadWire Segment ID
    of.writeAttr(SUMO_ATTR_OVERHEADWIREID, elecHybridToExport->getOverheadWireSegmentID());
    // Write Traction Substation ID
    of.writeAttr(SUMO_ATTR_TRACTIONSUBSTATIONID, elecHybridToExport->getTractionSubstationID());
    // Write current from overheadwire
    of.writeAttr(SUMO_ATTR_CURRENTFROMOVERHEADWIRE, elecHybridToExport->getCurrentFromOverheadWire());
    // Write voltage of overheadwire
    of.writeAttr(SUMO_ATTR_VOLTAGEOFOVERHEADWIRE, elecHybridToExport->getVoltageOfOverheadWire());
    // Write circuit alpha best (1 if the traction substation is not overloaded, number from interval [0,1) if the traction substation is overloaded, NAN if it is not applicable)
    of.writeAttr(SUMO_ATTR_ALPHACIRCUITSOLVER, elecHybridToExport->getCircuitAlpha());
	// Write the current charging current limit when the vehicle is stopped
    of.writeAttr(SUMO_ATTR_MAXCURRENT_STOPPED, elecHybridToExport->getMaxLineCurrentStopped());

    // Write Speed
    of.writeAttr(SUMO_ATTR_SPEED, veh->getSpeed());
    // Write Acceleration
    of.writeAttr(SUMO_ATTR_ACCELERATION, veh->getAcceleration());
    // Write Distance
    of.writeAttr(SUMO_ATTR_DISTANCE, veh->getOdometer());
    // Write pos x
    of.writeAttr(SUMO_ATTR_X, veh->getPosition().x());
    // Write pos y
    of.writeAttr(SUMO_ATTR_Y, veh->getPosition().y());
    // Write pos z
    of.writeAttr(SUMO_ATTR_Z, veh->getPosition().z());
    // Write slope
    of.writeAttr(SUMO_ATTR_SLOPE, veh->getSlope());
	// Get the micro vehicle
	// RICE_TODO: Does it make sense to check for nullptr here?
    const MSVehicle* microVeh = dynamic_cast<const MSVehicle*>(veh);
    if (microVeh != nullptr) {
        // Write Lane ID
        of.writeAttr(SUMO_ATTR_LANE, microVeh->getLane()->getID());
    }
    // Write vehicle position in the lane
    of.writeAttr(SUMO_ATTR_POSONLANE, veh->getPositionOnLane());

}

// ===========================================================================
// method definitions
// ===========================================================================
void
MSElecHybridExport::writeAggregated(OutputDevice& of, SUMOTime timestep, int precision) {

	bool timestepTagOpened = false;

    MSVehicleControl& vc = MSNet::getInstance()->getVehicleControl();
    for (MSVehicleControl::constVehIt it = vc.loadedVehBegin(); it != vc.loadedVehEnd(); ++it) {

        const SUMOVehicle* veh = it->second;

        if (!veh->isOnRoad()) {
            continue;
        }

        MSDevice_ElecHybrid* elecHybridToExport = static_cast<MSDevice_ElecHybrid*>(veh->getDevice(typeid(MSDevice_ElecHybrid)));
        if (elecHybridToExport == nullptr || elecHybridToExport->getMaximumBatteryCapacity() <= 0) {
            // If the device is not present or the battery capacity is zero, do not export
			// Do not write any warnings in this case as this is expected for some vehicles
            continue;
		}
        if (elecHybridToExport->getLastNotifyMoveStep() != timestep) {
            continue;
        }

		// The vehicle has an ElecHybrid device and a non-zero maximum battery capacity, so we proceed with the export
		// If the timestep tag is not opened yet, open it
        if (!timestepTagOpened) {
            of.openTag(SUMO_TAG_TIMESTEP).writeAttr(SUMO_ATTR_TIME, time2string(timestep));
            of.setPrecision(precision);
            timestepTagOpened = true;
        }

        //
        // The <vehicle> tag is unique for aggrgated output
        //
        // Open Row
        of.openTag(SUMO_TAG_VEHICLE);
        // Write the vehicle ID
        of.writeAttr(SUMO_ATTR_ID, veh->getID());
		// Write the maximum battery capacity of this vehicle
        of.writeAttr(SUMO_ATTR_MAXIMUMBATTERYCAPACITY, elecHybridToExport->getMaximumBatteryCapacity());

        // This is the part that is common for both aggreagted and non-aggregated output
        writeElecHybridData(of, veh, elecHybridToExport);

        // Close Row
        of.closeTag();
    }

    if (timestepTagOpened) of.closeTag();
}


void
MSElecHybridExport::write(OutputDevice& of, const SUMOVehicle* veh, SUMOTime timestep, int precision) {
	// Check if the vehicle is on road, if not, do not export
	// RICE_TODO: Can it happen that the export is called for a vehicle that is not on road?
    if (!veh->isOnRoad()) {
		WRITE_WARNING("Vehicle " + veh->getID() + " is not on road, skipping ElecHybrid export.");
        return;
    }

    // Get the instance of the ElecHybrid device associated with this vehicle
	// I am using `static_cast` here as we explicitly ask the vehicle to give us the device of type MSDevice_ElecHybrid if present and NULL if not.
    MSDevice_ElecHybrid* elecHybridToExport = static_cast<MSDevice_ElecHybrid*>(veh->getDevice(typeid(MSDevice_ElecHybrid)));
    if (elecHybridToExport == nullptr || elecHybridToExport->getMaximumBatteryCapacity() <= 0) {
        // If the device is not present or the battery capacity is zero, do not export
        if (elecHybridToExport == nullptr) {
            WRITE_WARNING("Vehicle " + veh->getID() + " does not have an ElecHybrid device, but MSElecHybridExport::write() has been called - skipping.");
        } else {
            WRITE_WARNING("Vehicle " + veh->getID() + " has zero maximum battery capacity, skipping ElecHybrid export.");
		}
        return;
	}
    if (elecHybridToExport->getLastNotifyMoveStep() != timestep) {
        return;
    }
    
    // Open the <timestep> tag with the time attribute and set the desired precision
    of.openTag(SUMO_TAG_TIMESTEP).writeAttr(SUMO_ATTR_TIME, time2string(timestep));
    of.setPrecision(precision);

    // Write the common attributes for both aggregated and non-aggregated output
	writeElecHybridData(of, veh, elecHybridToExport);

	// Close the <timestep> tag
    of.closeTag();
}
