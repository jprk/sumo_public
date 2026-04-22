/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2002-2025 German Aerospace Center (DLR) and others.
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
/// @file    MSOverheadWire.cpp
/// @author  Jakub Sevcik (RICE)
/// @author  Jan Prikryl (RICE)
/// @date    2019-12-15
///
// Member method definitions for MSOverheadWire and MSTractionSubstation.
/****************************************************************************/
#include <config.h>

#include <cassert>
#include <tuple>
#include <mutex>
#include <string.h>

#include <utils/vehicle/SUMOVehicle.h>
#include <utils/common/ToString.h>
#include <utils/common/FileHelpers.h>
#include <microsim/MSVehicleType.h>
#include <microsim/MSStoppingPlace.h>
#include <microsim/MSJunction.h>
#include <microsim/MSLane.h>
#include <microsim/MSLink.h>
#include <microsim/MSNet.h>
#include <microsim/devices/MSDevice_ElecHybrid.h>

// due to gOverheadWireSolver
#include <microsim/MSGlobals.h>

// due to solving circuit as endEndOfTimestepEvents
#include <utils/common/StaticCommand.h>
#include <utils/common/WrappingCommand.h>
#include <microsim/MSEventControl.h>

#include <utils/traction_wire/Node.h>
#include "MSOverheadWire.h"

// Prefixes for overhead wire names
inline const std::string OWSID_PREFIX_EXT = "ows_";
inline const std::string OWSID_PREFIX_INT = "ows.in_";

Command* MSTractionSubstation::myCommandForSolvingCircuit = nullptr;
static std::mutex ow_mutex;

// ===========================================================================
//                                                              MSOverheadWire
// ===========================================================================

MSOverheadWire::MSOverheadWire(const std::string& overheadWireSegmentID, const std::string& overheadWireSectionID, 
                               MSLane& lane, double startPos, double endPos, 
                               OverheadWireType& owt, bool voltageSource) :
    MSStoppingPlace(overheadWireSegmentID, SUMO_TAG_OVERHEAD_WIRE_SEGMENT, std::vector<std::string>(), lane, startPos, endPos),
    myOverheadWireSectionID(overheadWireSectionID),
    myVoltage(0),
    myChargingVehicle(false),
    myTotalCharge(0),
    myChargingVehicles({}),
    myWireType(owt),
    // RICE_TODO: think about some better structure storing circuit pointers below
    myTractionSubstation(nullptr),
    myVoltageSource(voltageSource),
    myCircuitElementPos(nullptr),
    myCircuitStartNodePos(nullptr),
    myCircuitEndNodePos(nullptr) {
    if (getBeginLanePosition() > getEndLanePosition()) {
        WRITE_WARNING(toString(SUMO_TAG_OVERHEAD_WIRE_SEGMENT) + " with ID = " + getID() + " doesn't have a valid range (" + toString(getBeginLanePosition()) + " < " + toString(getEndLanePosition()) + ").");
    }
}

MSOverheadWire::~MSOverheadWire() {
    if (myTractionSubstation != nullptr) {
        Circuit* circuit = myTractionSubstation->getCircuit();
        if (circuit != nullptr 
            && myCircuitElementPos != nullptr 
            && myCircuitElementPos->getPosNode() == myCircuitStartNodePos
            && myCircuitElementPos->getNegNode() == myCircuitEndNodePos)
        {
            // Ask Circuit object to remove circuit element and possibly also its nodes
            circuit->eraseElement(myCircuitElementPos);
        }

        if (myTractionSubstation->numberOfOverheadSegments() <= 1) {
            myTractionSubstation->eraseOverheadWireSegmentFromCircuit(this);
            //RICE_TODO We should "delete myTractionSubstation;" here ... or somewhere else?
        } else {
            myTractionSubstation->eraseOverheadWireSegmentFromCircuit(this);
        }
    }
}


std::string
MSOverheadWire::getOWSIDforLane(const MSLane& lane) {
    return OWSID_PREFIX_EXT + lane.getID();
}

void
MSOverheadWire::addVehicle(SUMOVehicle& veh) {
    std::lock_guard<std::mutex> guard(ow_mutex);
    setChargingVehicle(true);
    myChargingVehicles.push_back(&veh);
    sort(myChargingVehicles.begin(), myChargingVehicles.end(), vehicle_position_sorter());
}

void
MSOverheadWire::eraseVehicle(SUMOVehicle& veh) {
    std::lock_guard<std::mutex> guard(ow_mutex);
    myChargingVehicles.erase(std::remove(myChargingVehicles.begin(), myChargingVehicles.end(), &veh), myChargingVehicles.end());
    if (myChargingVehicles.size() == 0) {
        setChargingVehicle(false);
    }
    //sort(myChargingVehicles.begin(), myChargingVehicles.end(), vehicle_position_sorter());
}

void
MSOverheadWire::lock() const {
    ow_mutex.lock();
}

void
MSOverheadWire::unlock() const {
    ow_mutex.unlock();
}

std::string MSOverheadWire::getOverheadWireSegmentName() {
    return toString(getID());
}

Circuit*
MSOverheadWire::getCircuit() const {
    if (getTractionSubstation() != nullptr) {
        return getTractionSubstation()->getCircuit();
    }
    return nullptr;
}

Node*
MSOverheadWire::getCircuitStartNodePos() const
{
    // Check if the start node has already been created
    if (!myCircuitStartNodePos) {
        // No start node yet.
        // Get the circuit, as the circuit is responsible for node memory management
        Circuit* circuit = getCircuit();
        assert(circuit != nullptr);

        // Create the `pNode` (non-owning reference, pointer managed by the circuit)
        myCircuitStartNodePos = circuit->addNode(CIRCUIT_NODE_PLUS_P_PFX + myID);

        // Register it with all neighbours
        for (MSOverheadWire* neighbour : myIncomingSegments) {
            neighbour->setCircuitEndNodePos(myCircuitStartNodePos);
        }
    }
    return myCircuitStartNodePos;
}

Node*
MSOverheadWire::getCircuitEndNodePos() const
{
    // Check if the end node has already been created
    if (!myCircuitEndNodePos) {
        // No end node yet.
        // Get the circuit, as the circuit is responsible for node memory management
        Circuit* circuit = getCircuit();
        assert(circuit != nullptr);

        // Create the `nNode` (non-owning reference, pointer managed by the circuit)
        myCircuitEndNodePos = circuit->addNode(CIRCUIT_NODE_PLUS_N_PFX + myID);

        // Register it with all neighbours
        for (MSOverheadWire* neighbour : myOutgoingSegments) {
            neighbour->setCircuitStartNodePos(myCircuitEndNodePos);
        }
    }
    return myCircuitEndNodePos;
}

std::string
MSOverheadWire::getJoinedIncomingSegmentIDs() const {
    // Returned sequence of segment IDs
    std::string ids = myID;
    // Loop over incoming segments and concatenate their names
    for (MSOverheadWire* neighbour : myIncomingSegments) {
        ids += '/' + neighbour->getID();
    }
    return ids;
}

std::string
MSOverheadWire::getJoinedOutgoingSegmentIDs() const {
    // Returned sequence of segment IDs
    std::string ids = myID;
    // Loop over incoming segments and concatenate their names
    for (MSOverheadWire* neighbour : myOutgoingSegments) {
        if (!ids.empty()) ids += '/';
        ids += neighbour->getID();
    }
    return ids;
}

double
MSOverheadWire::getResistance() {
    double wireResistancePerLength = myWireType.getResistancePerLength();
    double wireLength = myLane.getLength();
    // The wire may be shorter than the lane
    if (myBegPos > 0) {
        wireLength -= myBegPos;
    }
    if (myEndPos < myLane.getLength()) {
        wireLength -= (myLane.getLength() - myEndPos);
    }
    return wireResistancePerLength * wireLength;
}

double
MSOverheadWire::getVoltage() const {
    return myVoltage;
}

void
MSOverheadWire::setVoltage(double voltage) {
    if (voltage < 0) {
        WRITE_WARNING("New " + toString(SUMO_ATTR_VOLTAGE) + " for " + toString(SUMO_TAG_OVERHEAD_WIRE_SEGMENT) + " with ID = " + getID() + " isn't valid (" + toString(voltage) + ").")
    } else {
        myVoltage = voltage;
    }
}

void
MSOverheadWire::setChargingVehicle(bool value) {
    myChargingVehicle = value;
}

bool
MSOverheadWire::vehicleIsInside(const double position) const {
    if ((position >= getBeginLanePosition()) && (position <= getEndLanePosition())) {
        return true;
    } else {
        return false;
    }
}

bool
MSOverheadWire::isCharging() const {
    return myChargingVehicle;
}


void
MSOverheadWire::addChargeValueForOutput(double WCharged, MSDevice_ElecHybrid* elecHybrid, bool ischarging) {
    std::string status = "charging";
    if (!ischarging) {
        status = "not-charging";
    }

    // update total charge
    myTotalCharge += WCharged;
    // create charge row and insert it in myChargeValues
    const std::string vehID = elecHybrid->getHolder().getID();
    if (myChargeValues.count(vehID) == 0) {
        myChargedVehicles.push_back(vehID);
    }
    Charge C(MSNet::getInstance()->getCurrentTimeStep(), elecHybrid->getHolder().getID(), elecHybrid->getHolder().getVehicleType().getID(),
             status, WCharged, elecHybrid->getActualBatteryCapacity(), elecHybrid->getMaximumBatteryCapacity(),
             elecHybrid->getVoltageOfOverheadWire(), myTotalCharge);
    myChargeValues[vehID].push_back(C);
}


void
MSOverheadWire::writeOverheadWireSegmentOutput(OutputDevice& output) {
    int chargingSteps = 0;
    std::vector<SUMOTime> chargingSteps_list;
    for (const auto& item : myChargeValues) {
        for (auto it : item.second) {
            if (std::find(chargingSteps_list.begin(), chargingSteps_list.end(), it.timeStep) == chargingSteps_list.end()) {
                chargingSteps_list.push_back(it.timeStep);
            }
        }
    }
    chargingSteps = (int) chargingSteps_list.size();
    output.openTag(SUMO_TAG_OVERHEAD_WIRE_SEGMENT);
    output.writeAttr(SUMO_ATTR_ID, myID);
    if (getTractionSubstation() != nullptr) {
        output.writeAttr(SUMO_ATTR_TRACTIONSUBSTATIONID, getTractionSubstation()->getID());
    } else {
        output.writeAttr(SUMO_ATTR_TRACTIONSUBSTATIONID, "");
    }
    output.writeAttr(SUMO_ATTR_TOTALENERGYCHARGED, myTotalCharge);

    // RICE_TODO QUESTION myChargeValues.size() vs. chargingSteps
    // myChargeValues.size() is the number of vehicles charging sometimes from this overheadwire segment during simulation
    // chargingSteps is now the sum of chargingSteps of each vehicle, but takes also into account that at the given
    // step more than one vehicle may be charged from this segment
    output.writeAttr(SUMO_ATTR_CHARGINGSTEPS, chargingSteps);
    // output.writeAttr(SUMO_ATTR_EDGE, getLane().getEdge());
    output.writeAttr(SUMO_ATTR_LANE, getLane().getID());

    // Start writing
    if (myChargeValues.size() > 0) {
        for (const std::string& vehID : myChargedVehicles) {
            int iStart = 0;
            const auto& chargeSteps = myChargeValues[vehID];
            while (iStart < (int)chargeSteps.size()) {
                int iEnd = iStart + 1;
                double charged = chargeSteps[iStart].WCharged;
                while (iEnd < (int)chargeSteps.size() && chargeSteps[iEnd].timeStep == chargeSteps[iEnd - 1].timeStep + DELTA_T) {
                    charged += chargeSteps[iEnd].WCharged;
                    iEnd++;
                }
                writeVehicle(output, chargeSteps, iStart, iEnd, charged);
                iStart = iEnd;
            }
        }
    }
    // close charging station tag
    output.closeTag();
}


void
MSOverheadWire::writeVehicle(OutputDevice& out, const std::vector<Charge>& chargeSteps, int iStart, int iEnd, double charged) {
    const Charge& first = chargeSteps[iStart];
    out.openTag(SUMO_TAG_VEHICLE);
    out.writeAttr(SUMO_ATTR_ID, first.vehicleID);
    out.writeAttr(SUMO_ATTR_TYPE, first.vehicleType);
    out.writeAttr(SUMO_ATTR_TOTALENERGYCHARGED_VEHICLE, charged);
    out.writeAttr(SUMO_ATTR_CHARGINGBEGIN, time2string(first.timeStep));
    out.writeAttr(SUMO_ATTR_CHARGINGEND, time2string(chargeSteps[iEnd - 1].timeStep));
    out.writeAttr(SUMO_ATTR_MAXIMUMBATTERYCAPACITY, first.maxBatteryCapacity);
    for (int i = iStart; i < iEnd; i++) {
        const Charge& c = chargeSteps[i];
        out.openTag(SUMO_TAG_STEP);
        out.writeAttr(SUMO_ATTR_TIME, time2string(c.timeStep));
        // charge values
        out.writeAttr(SUMO_ATTR_CHARGING_STATUS, c.status);
        out.writeAttr(SUMO_ATTR_ENERGYCHARGED, c.WCharged);
        out.writeAttr(SUMO_ATTR_PARTIALCHARGE, c.totalEnergyCharged);
        // charging values of charging station in this timestep
        out.writeAttr(SUMO_ATTR_VOLTAGE, c.voltage);
        // battery status of vehicle
        out.writeAttr(SUMO_ATTR_ACTUALBATTERYCAPACITY, c.actualBatteryCapacity);
        // close tag timestep
        out.closeTag();
    }
    out.closeTag();
}


// ===========================================================================
//                                                        MSTractionSubstation
// ===========================================================================
// RICE_TODO Split MSTractionSubstation and MSOverheadWire?
// Probably no as the traction substation cannot stand alone and is always
// used together with the overhead wire. It is a bit disorganised, though.

MSTractionSubstation::MSTractionSubstation(const std::string& substationId, double voltage, double currentLimit) :
    Named(substationId),
    myChargingVehicle(false),
    myElecHybridCount(0),
    mySubstationVoltage(voltage),
    myCircuit(new Circuit(currentLimit)),
    myTotalEnergy(0)
{}

MSTractionSubstation::~MSTractionSubstation() {
}

void
MSTractionSubstation::addVehicle(MSDevice_ElecHybrid* elecHybrid) {
    myElecHybrid.push_back(elecHybrid);
}

void
MSTractionSubstation::setChargingVehicle(bool value) {
    myChargingVehicle = value;
}

void
MSTractionSubstation::eraseVehicle(MSDevice_ElecHybrid* veh) {
    myElecHybrid.erase(std::remove(myElecHybrid.begin(), myElecHybrid.end(), veh), myElecHybrid.end());
}

void
MSTractionSubstation::writeOut() {
    std::cout << "substation " << getID() << " constrols segments: \n";
    for (std::vector<MSOverheadWire*>::iterator it = myOverheadWireSegments.begin(); it != myOverheadWireSegments.end(); ++it) {
        std::cout << "        " << (*it)->getOverheadWireSegmentName() << "\n";
    }
}

void
MSTractionSubstation::addOverheadWireSegmentToCircuit(MSOverheadWire* newOverheadWireSegment) {

    // Sanity check: The `newOverheadWireSegment` should reference this traction substation
    assert(newOverheadWireSegment->getTractionSubstation() == this);

    // RICE_TODO: consider the possibility of having more segments that belong to one lane.
    // The rationale behind this is the possibility to have a wire section split placed
    // on certain position over the lane. Currently we have to split the underlying edge to
    // get an appropriate split placement.

    // Add the segment to the segments powered by this traction substation.
    myOverheadWireSegments.push_back(newOverheadWireSegment);

    if (MSGlobals::gOverheadWireSolver) {
#ifdef HAVE_EIGEN
        // Remember the segment ID
        const std::string segmentID = newOverheadWireSegment->getID();

        // Make sure that the cicuit has a negative node connected to ground.
        // RICE_TODO: Creation of a nonexisting node may be made part of a custom getter for `Circuit`Make sure that the cicuit has a negative node connected to ground.
        Node* gndNode = myCircuit->getNode(CIRCUIT_NODE_NEG_GROUND);
        if (gndNode == nullptr) {
            gndNode = myCircuit->addNode(CIRCUIT_NODE_NEG_GROUND);
        }

        /*
         * Get the nodes of the electric circuit that mark the beginning and end of the element
         * representing this overhead wire segment. If the nodes do not exist, try to create them,
         * insert them into the circuit and propagate information about their existence to all
         * incoming or outgoing segments of this segment.
         * Convention: `pNode` is the node at the beginning of the wire segment, `nNode` is the node at the end
         */
        // This is the beginning of the segment
        Node* pNode = newOverheadWireSegment->getCircuitStartNodePos();
        // Now the same with the end of the segment
        Node* nNode = newOverheadWireSegment->getCircuitEndNodePos();

        // Historically we had fixed wire parameters for the whole network ...
        // Replaced with `getResistance()`: newOverheadWireSegment->getLane().getLength() * WIRE_RESISTIVITY,
        // RICE_TODO: Check that the `getResistance()` takes `startPos` and `endPos` of the overhead wire segment
        // into account.
        newOverheadWireSegment->setCircuitElementPos(
            myCircuit->addElement(
                CIRCUIT_ELEMENT_PLUS_PFX + segmentID,
                newOverheadWireSegment->getResistance(),
                pNode, nNode,
                Element::ElementType::RESISTOR_traction_wire));
#else
        WRITE_WARNING(TL("Overhead circuit solver requested, but solver support (Eigen) not compiled in."));
#endif

        if (newOverheadWireSegment->isThereVoltageSource()) {
            // Add the segment to the list of segments powering the circuit
            if (!myVoltageSources.empty()) myVoltageSources += " ";
            myVoltageSources += newOverheadWireSegment->getID();

#ifdef HAVE_EIGEN
            // if node "voltage_source_node" does not exist (i.e. there is no voltage source in the circuit yet),
            // we create the node and we "connect" it to the traction substation, that is modelled as one voltage source and serial resistor
            Node* vSrcNode = myCircuit->getNode(CIRCUIT_NODE_VOLTAGE_SRC);
            if (vSrcNode == nullptr) {
                // No voltage source in the circuir, add ít
                vSrcNode = myCircuit->addNode(CIRCUIT_NODE_VOLTAGE_SRC);
                // And another node representing a connection to a small resistor element
                Node* vResNode = myCircuit->addNode(CIRCUIT_NODE_VOLTAGE_RES);
                myCircuit->addElement(
                    CIRCUIT_ELEMENT_VOLTAGE_SRC,
                    mySubstationVoltage,
                    vResNode, gndNode,
                    Element::ElementType::VOLTAGE_SOURCE_traction_wire);

                myCircuit->addElement(
                    CIRCUIT_ELEMENT_VOLTAGE_SRC_RES,
                    0.001,  // RICE_TODO: Used to have 0.12 Ohm here for trolleybuses, make configurable!
                    vSrcNode, vResNode,
                    Element::ElementType::RESISTOR_traction_wire);
            }
            // connect the start of overhead wire segment with the voltage source using a small resistor element (for simple computation of the circuit) 
            myCircuit->addElement(
                CIRCUIT_ELEMENT_VOLTAGE_RES_PFX + segmentID,
                0.001, // RICE_TODO: Make configurable!
                pNode, vSrcNode,
                Element::ElementType::RESISTOR_traction_wire);
            /*
             * The circuit contains:
             * 
             *   vResNode --[small resistor]-- vSrcNode --[small resistor]-- pNode --[ow segment]-- nNode/pNode --[ow segment]-- ... -- nNode
             *       |                               |                                                                                    |
             * (voltage source)                      +--[possibly another resistor]-- pNode --[ow segment]-- ...                ... -- gndNode
             *       |
             *    gndNode
             */
#else
            WRITE_WARNING(TL("Overhead circuit solver requested, but solver support (Eigen) not compiled in."));
#endif
        }

#ifdef OVERHEAD_WIRE_DEBUG
        /* ------------------------------------------------------------
         * Save the current circuit in the form of DOT / GraphViz graph
         * ------------------------------------------------------------ */
         // Build output filename
        std::string& fileName = fmt::format("circuit_init.{}.p{:02d}.dot", myID, myOverheadWireSegments.size());
        // Get the SUMO options (command line + config file)
        OptionsCont& oc = OptionsCont::getOptions();
        // Determine config directory (empty if current dir if not set)
        std::string configDir;
        if (oc.isSet("configuration-file")) {
            std::string configPath = oc.getString("configuration-file");
            configDir = FileHelpers::getFilePath(configPath); // includes trailing slash if non-empty
            fileName = configDir + fileName;
        }
        // Prepend output-prefix if set
        if (oc.isSet("output-prefix")) {
            std::string prefix = oc.getString("output-prefix");
            // Preprends the prefix before the last path component of fileName
            fileName = FileHelpers::prependToLastPathComponent(prefix, fileName);
        }
        myCircuit->exportToDOTFile(fileName);
#endif
   }
}

void MSTractionSubstation::addOverheadWireClampToCircuit(const std::string id, MSOverheadWire* startSegment, MSOverheadWire* endSegment) {
    PositionVector pos_start = startSegment->getLane().getShape();
    PositionVector pos_end = endSegment->getLane().getShape();
    double distance = pos_start[0].distanceTo2D(pos_end.back());

    if (distance > 10) {
        WRITE_WARNING("The distance between two overhead wires during adding overhead wire clamp '" + id + "' defined for traction substation '" + startSegment->getTractionSubstation()->getID() + "' is " + toString(distance) + " m.")
    }
    // The clamp wiring is made of the "default" wire. This is definitiely not correct, 
    // but the wire is short and the difference yo using the exact clamp resistance and 
    // length will be negligible.
    getCircuit()->addElement(
        id, 
        distance * WIRE_DEFAULTTYPE.getResistancePerLength(), 
        startSegment->getCircuitStartNodePos(), 
        endSegment->getCircuitEndNodePos(), 
        Element::ElementType::RESISTOR_traction_wire);
}


void
MSTractionSubstation::eraseOverheadWireSegmentFromCircuit(MSOverheadWire* oldSegment) {
    //myOverheadWireSegments.push_back(static_cast<MSOverheadWire*>(MSNet::getInstance()->getStoppingPlace(overheadWireSegmentID, SUMO_TAG_OVERHEAD_WIRE_SEGMENT)));
    myOverheadWireSegments.erase(std::remove(myOverheadWireSegments.begin(), myOverheadWireSegments.end(), oldSegment), myOverheadWireSegments.end());
}


bool
MSTractionSubstation::isCharging() const {
    return myChargingVehicle;
}


void
MSTractionSubstation::increaseElecHybridCount() {
    myElecHybridCount++;
}


void
MSTractionSubstation::decreaseElecHybridCount() {
    myElecHybridCount--;
}


void MSTractionSubstation::addForbiddenLane(MSLane* lane) {
    myForbiddenLanes.push_back(lane);
}


bool MSTractionSubstation::isForbidden(const MSLane* lane) {
    return std::find(myForbiddenLanes.begin(), myForbiddenLanes.end(), lane) != myForbiddenLanes.end();
}


void
MSTractionSubstation::addClamp(const std::string& id, MSOverheadWire* startPos, MSOverheadWire* endPos) {
    OverheadWireClamp clamp(id, startPos, endPos, false);
    myOverheadWireClamps.push_back(clamp);
}


MSTractionSubstation::OverheadWireClamp*
MSTractionSubstation::findClamp(std::string clampId) {
    for (auto it = myOverheadWireClamps.begin(); it != myOverheadWireClamps.end(); it++) {
        if (it->id == clampId) {
            return &(*it);
        }
    }
    return nullptr;
}


bool
MSTractionSubstation::isAnySectionPreviouslyDefined() {
    if (myOverheadWireSegments.size() > 0 || myForbiddenLanes.size() > 0 || getCircuit()->getLastId() > 0) {
        return true;
    }
    return false;
}


void
MSTractionSubstation::addSolvingCircuitToEndOfTimestepEvents() {
    if (!myChargingVehicle) {
        // myCommandForSolvingCircuit = new StaticCommand<MSTractionSubstation>(&MSTractionSubstation::solveCircuit);
        myCommandForSolvingCircuit = new WrappingCommand<MSTractionSubstation>(this, &MSTractionSubstation::solveCircuit);
        MSNet::getInstance()->getEndOfTimestepEvents()->addEvent(myCommandForSolvingCircuit);
        setChargingVehicle(true);
    }
}


SUMOTime
MSTractionSubstation::solveCircuit(SUMOTime /*currentTime*/) {
    /*Circuit evaluation*/
    setChargingVehicle(false);

#ifdef HAVE_EIGEN

    // RICE_TODO: Allow for updating current limits in each time step if changed e.g. via traci or similar
    // getCircuit()->setCurrentLimit(myCurrentLimit);

    // Solve the electrical circuit
    myCircuit->solve();

#ifdef OVERHEAD_WIRE_DEBUG
    if (myID == "hydro.sumavska.3" || (myCircuit->getAlphaReason() > 0 && myCircuit->getAlphaBest() < 0.5)) {
        /* ------------------------------------------------------------
         * Save the current circuit in the form of DOT / GraphViz graph
         * ------------------------------------------------------------ */
        // Build output filename
        SUMOTime ts = MSNet::getInstance()->getCurrentTimeStep();
        std::string& fileName = fmt::format("circuit.{}.{:05d}.dot", myID, int(ts / 1000));
        // Get the SUMO options (command line + config file)
        OptionsCont& oc = OptionsCont::getOptions();
        // Determine config directory (empty if current dir if not set)
        std::string configDir;
        if (oc.isSet("configuration-file")) {
            std::string configPath = oc.getString("configuration-file");
            configDir = FileHelpers::getFilePath(configPath); // includes trailing slash if non-empty
            fileName = configDir + fileName;
        }
        // Prepend output-prefix if set
        if (oc.isSet("output-prefix")) {
            std::string prefix = oc.getString("output-prefix");
            // Preprends the prefix before the last path component of fileName
            fileName = FileHelpers::prependToLastPathComponent(prefix, fileName);
        }
        myCircuit->exportToDOTFile(fileName);

        if (myCircuit->getAlphaReason() > 0 && myCircuit->getAlphaBest() < 0.5) {
            WRITE_WARNINGF(TL("Suspiciously low alpha=% for substation `%` at time %. Circuit graph saved to `{}` for further analysis."),
                toString(myCircuit->getAlphaBest()),
                myID,
                time2string(ts),
                fileName);
        }
    }
#endif

    if (myCircuit->getAlphaBest() != 1.0) {
        WRITE_WARNINGF(TL("The requested total power could not be delivered by the overhead wire at `%`. Only % of originally requested power was provided."), 
            myID,
            toString(myCircuit->getAlphaBest()));
    }
#endif

    // RICE_TODO: verify what happens if eigen is not defined?
    // Note: addSolvingCircuitToEndOfTimestepEvents() and thus solveCircuit() should be called from notifyMove only if eigen is defined.
    addChargeValueForOutput(WATT2WATTHR(myCircuit->getTotalPowerOfCircuitSources()), myCircuit->getTotalCurrentOfCircuitSources(), myCircuit->getAlphaBest(), myCircuit->getAlphaReason());

    for (auto* it : myElecHybrid) {

        Element* vehElem = it->getVehElem();
        double voltage = vehElem->getVoltage();
        double current = -vehElem->getCurrent();  // Vehicle is a power source, hence its current (returned by getCurrent()) flows in opposite direction

        it->setCurrentFromOverheadWire(current);
        it->setVoltageOfOverheadWire(voltage);

        it->getPowerManagement()->distributePower(voltage * current, true, true, it);
    }

    return 0;
}

void
MSTractionSubstation::addChargeValueForOutput(double energy, double current, double alpha, Circuit::alphaFlag alphaReason) {
    std::string status = "";

    myTotalEnergy += energy; //[Wh]

    std::string vehicleIDs = "";
    for (std::vector<MSDevice_ElecHybrid*>::iterator it = myElecHybrid.begin(); it != myElecHybrid.end(); it++) {
        vehicleIDs += (*it)->getID() + " ";
    }
    //vehicleIDs.erase(vehicleIDs.end());
    // TODO vehicleIDs should not be empty, but in some case, it is (due to teleporting of vehicle?)
    if (!vehicleIDs.empty()) {
        vehicleIDs.pop_back();
    }

    std::string currents = "";
    currents = myCircuit->getCurrentsOfCircuitSource(currents);

    // create charge row and insert it in myChargeValues
    chargeTS C(MSNet::getInstance()->getCurrentTimeStep(), getID(), vehicleIDs, energy, current, currents, mySubstationVoltage, status,
               (int)myElecHybrid.size(), (int)getCircuit()->getNumVoltageSources(), alpha, alphaReason);
    myChargeValues.push_back(C);
}

void
MSTractionSubstation::writeTractionSubstationOutput(OutputDevice& output) {
    output.openTag(SUMO_TAG_TRACTION_SUBSTATION);
    output.writeAttr(SUMO_ATTR_ID, myID);
    output.writeAttr(SUMO_ATTR_TOTALENERGYCHARGED, myTotalEnergy); //[Wh]
    double length = 0;
    for (auto it = myOverheadWireSegments.begin(); it != myOverheadWireSegments.end(); it++) {
        length += (*it)->getEndLanePosition() - (*it)->getBeginLanePosition();
    }
    output.writeAttr(SUMO_ATTR_LENGTH, length);
    output.writeAttr("numVoltageSources", myCircuit->getNumVoltageSources());
    output.writeAttr("voltageSourcesElementNames", myVoltageSources);
    output.writeAttr("numClamps", myOverheadWireClamps.size());
    output.writeAttr(SUMO_ATTR_CHARGINGSTEPS, myChargeValues.size());

    // start writting
    if (myChargeValues.size() > 0) {
        // iterate over charging values
        for (std::vector<MSTractionSubstation::chargeTS>::const_iterator i = myChargeValues.begin(); i != myChargeValues.end(); i++) {
            // open tag for timestep and write all parameters
            output.openTag(SUMO_TAG_STEP);
            output.writeAttr(SUMO_ATTR_TIME, time2string(i->timeStep));
            // charge values
            output.writeAttr("vehicleIDs", i->vehicleIDs);
            output.writeAttr("numVehicles", i->numVehicles);
            // same number of numVoltageSources for all time, parameter is written in the superordinate tag
            //output.writeAttr("numVoltageSources", i->numVoltageSources);
            // charging status is always ""
            //output.writeAttr(SUMO_ATTR_CHARGING_STATUS, i->status);
            output.writeAttr(SUMO_ATTR_ENERGYCHARGED, i->energy);
            output.writeAttr(SUMO_ATTR_CURRENTFROMOVERHEADWIRE, i->current);
            output.writeAttr("currents", i->currentsString);
            // charging values of charging station in this timestep
            output.writeAttr(SUMO_ATTR_VOLTAGE, i->voltage);
            output.writeAttr(SUMO_ATTR_ALPHACIRCUITSOLVER, i->alpha);
            output.writeAttr("alphaFlag", i->alphaReason);
            // close tag timestep
            output.closeTag();
            // update timestep of charge
        }
    }
    // close charging station tag
    output.closeTag();
}

/****************************************************************************/
