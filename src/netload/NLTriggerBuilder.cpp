/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2025 German Aerospace Center (DLR) and others.
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
/// @file    NLTriggerBuilder.cpp
/// @author  Daniel Krajzewicz
/// @author  Tino Morenz
/// @author  Jakob Erdmann
/// @author  Eric Nicolay
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @author  Johannes Rummel
/// @date    Thu, 17 Oct 2002
///
// Builds trigger objects for microsim
/****************************************************************************/
#include <config.h>

#include <string>
#include <mesosim/MELoop.h>
#include <mesosim/METriggeredCalibrator.h>
#include <microsim/MSEventControl.h>
#include <microsim/MSJunctionControl.h>
#include <microsim/MSLane.h>
#include <microsim/MSLink.h>
#include <microsim/MSEdge.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSParkingArea.h>
#include <microsim/MSStoppingPlace.h>
#include <microsim/output/MSDetectorControl.h>
#include <microsim/output/MSRouteProbe.h>
#include <microsim/trigger/MSLaneSpeedTrigger.h>
#include <microsim/trigger/MSTriggeredRerouter.h>
#include <microsim/trigger/MSCalibrator.h>
#include <microsim/trigger/MSChargingStation.h>
#include <microsim/trigger/MSOverheadWire.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/FileHelpers.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/WrappingCommand.h>
#include <utils/common/RGBColor.h>
#include <utils/options/OptionsCont.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/xml/XMLSubSys.h>
#include "NLHandler.h"
#include "NLTriggerBuilder.h"

// Print out debug information about overhead wire parsing and processing
#define OVERHEAD_WIRE_DEBUG

// ===========================================================================
// method definitions
// ===========================================================================
NLTriggerBuilder::NLTriggerBuilder()
    : myHandler(nullptr), myParkingArea(nullptr), myCurrentStop(nullptr) {}


NLTriggerBuilder::~NLTriggerBuilder() {}

void
NLTriggerBuilder::setHandler(NLHandler* handler) {
    myHandler = handler;
}


void
NLTriggerBuilder::buildVaporizer(const SUMOSAXAttributes& attrs) {
    WRITE_WARNING(TL("Vaporizers are deprecated. Use rerouters instead."));
    bool ok = true;
    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (!ok) {
        return;
    }
    MSEdge* e = MSEdge::dictionary(id);
    if (e == nullptr) {
        WRITE_ERRORF(TL("Unknown edge ('%') referenced in a vaporizer."), id);
        return;
    }
    SUMOTime begin = attrs.getSUMOTimeReporting(SUMO_ATTR_BEGIN, nullptr, ok);
    SUMOTime end = attrs.getSUMOTimeReporting(SUMO_ATTR_END, nullptr, ok);
    if (!ok) {
        return;
    }
    if (begin < 0) {
        WRITE_ERRORF(TL("A vaporization begin time is negative (edge id='%')."), id);
        return;
    }
    if (begin >= end) {
        WRITE_ERRORF(TL("A vaporization ends before it starts (edge id='%')."), id);
        return;
    }
    if (end >= string2time(OptionsCont::getOptions().getString("begin"))) {
        Command* cb = new WrappingCommand< MSEdge >(e, &MSEdge::incVaporization);
        MSNet::getInstance()->getBeginOfTimestepEvents()->addEvent(cb, begin);
        Command* ce = new WrappingCommand< MSEdge >(e, &MSEdge::decVaporization);
        MSNet::getInstance()->getBeginOfTimestepEvents()->addEvent(ce, end);
    }
}


void
NLTriggerBuilder::parseAndBuildLaneSpeedTrigger(MSNet& net, const SUMOSAXAttributes& attrs,
        const std::string& base) {
    // get the id, throw if not given or empty...
    bool ok = true;
    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (!ok) {
        return;
    }
    // get the file name to read further definitions from
    std::string file = getFileName(attrs, base, true);
    std::string objectid = attrs.get<std::string>(SUMO_ATTR_LANES, id.c_str(), ok);
    std::vector<MSLane*> lanes;
    for (const std::string& laneID : attrs.get<std::vector<std::string> >(SUMO_ATTR_LANES, id.c_str(), ok)) {
        MSLane* lane = MSLane::dictionary(laneID);
        if (lane == nullptr) {
            throw InvalidArgument("The lane '" + laneID + "' to use within MSLaneSpeedTrigger '" + id + "' is not known.");
        }
        lanes.push_back(lane);
    }
    if (!ok) {
        throw InvalidArgument("The lanes to use within MSLaneSpeedTrigger '" + id + "' are not known.");
    }
    if (lanes.size() == 0) {
        throw InvalidArgument("No lane defined for MSLaneSpeedTrigger '" + id + "'.");
    }
    try {
        MSLaneSpeedTrigger* trigger = buildLaneSpeedTrigger(net, id, lanes, file);
        if (file == "") {
            trigger->registerParent(SUMO_TAG_VSS, myHandler);
        }
    } catch (ProcessError& e) {
        throw InvalidArgument(e.what());
    }
}


void
NLTriggerBuilder::parseAndBuildChargingStation(MSNet& net, const SUMOSAXAttributes& attrs) {
    bool ok = true;

    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (!ok) {
        throw ProcessError();
    }

    MSLane* const lane = getLane(attrs, "chargingStation", id);
    double frompos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, id.c_str(), ok, 0);
    double topos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), ok, lane->getLength());
    const double chargingPower = attrs.getOpt<double>(SUMO_ATTR_CHARGINGPOWER, id.c_str(), ok, 22000);
    const double efficiency = attrs.getOpt<double>(SUMO_ATTR_EFFICIENCY, id.c_str(), ok, 0.95);
    const bool chargeInTransit = attrs.getOpt<bool>(SUMO_ATTR_CHARGEINTRANSIT, id.c_str(), ok, 0);
    const SUMOTime chargeDelay = attrs.getOptSUMOTimeReporting(SUMO_ATTR_CHARGEDELAY, id.c_str(), ok, 0);
    const std::string chargeType = attrs.getOpt<std::string>(SUMO_ATTR_CHARGETYPE, id.c_str(), ok, "normal");
    const SUMOTime waitingTime = attrs.getOptSUMOTimeReporting(SUMO_ATTR_WAITINGTIME, id.c_str(), ok, 900);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), ok, false);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), ok, "");
    MSParkingArea* parkingArea = getParkingArea(attrs, "parkingArea", id);

    // check charge type
    if ((chargeType != "normal") && (chargeType != "battery-exchange") && (chargeType != "fuel")) {
        throw InvalidArgument("The chargeType to use within MSLaneSpeedTrigger '" + id + "' is invalid.");
    }

    if (!ok || (myHandler->checkStopPos(frompos, topos, lane->getLength(), POSITION_EPS, friendlyPos) != SUMORouteHandler::StopPos::STOPPOS_VALID)) {
        throw InvalidArgument("Invalid position for charging station '" + id + "'.");
    }

    buildChargingStation(net, id, lane, frompos, topos, name, chargingPower, efficiency, chargeInTransit, chargeDelay, chargeType, waitingTime, parkingArea);
}


void
NLTriggerBuilder::parseAndBuildOverheadWireSegment(MSNet& net, const SUMOSAXAttributes& attrs) {
    bool ok = true;

    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        throw ProcessError();
    }

    /* The following call may either throw InvalidArgument exeption or return NULL:
        NULL is returned in case when the overhead wire segment should be built over an already
        ignored internal lane of an intersection, the exeption is thrown in case that
        the overhead wire segment references a non-existent lane. */
    MSLane* const lane = getLane(attrs, "overheadWireSegment", id);
    if (lane == nullptr) {
        WRITE_MESSAGEF(TL("The overheadWireSegment '%' was not created as it is attached to internal lane. It will be build automatically."), id);
        return;
    }

    if (lane->isInternal()) {
        WRITE_MESSAGEF(TL("The overheadWireSegment '%' not built as it is attached to internal lane. It will be build automatically."), id);
        return;
    }

    double frompos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, id.c_str(), ok, 0);
    double topos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), ok, lane->getLength());
    const bool voltageSource = attrs.getOpt<bool>(SUMO_ATTR_VOLTAGESOURCE, id.c_str(), ok, false);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), ok, false);

    if (!ok || myHandler->checkStopPos(frompos, topos, lane->getLength(), POSITION_EPS, friendlyPos) != SUMORouteHandler::StopPos::STOPPOS_VALID) {
        frompos = 0;
        topos = lane->getLength();
        WRITE_MESSAGEF(TL("The overheadWireSegment '%' has wrong position. Automatically set from 0 to the length of the lane."), id);
        //throw InvalidArgument("Invalid position for overheadWireSegment'" + id + "'.");
    }

    // This code is deprecated but until we remove it, we need a working overhead wire type here.
    // We will use the default type here.

    buildOverheadWireSegment(net, id, "unknown_yet", lane, frompos, topos, (OverheadWireType&)WIRE_DEFAULTTYPE, voltageSource);
#ifndef HAVE_EIGEN
    if (MSGlobals::gOverheadWireSolver && !myHaveWarnedAboutEigen) {
        myHaveWarnedAboutEigen = true;
        WRITE_WARNING(TL("Overhead wire solver (Eigen) not compiled in, expect errors in overhead wire simulation"))
    }
#endif // !HAVE_EIGEN
}

void
NLTriggerBuilder::parseAndBuildOverheadWireSection(MSNet& net, const SUMOSAXAttributes& attrs) {
    bool ok = true;
    std::string sectionID = attrs.getOpt<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        throw ProcessError();
    }
    if (!attrs.hasAttribute(SUMO_ATTR_ID)) {
        throw InvalidArgument("Found an overhead wire element without 'id' attribute. If this is the old style definition, please update it.");
    }

    // Get the ID of a substation that this section is connected to
    std::string substationID = attrs.get<std::string>(SUMO_ATTR_SUBSTATIONID, 0, ok);
    if (!ok) {
        throw ProcessError();
    }

    // The substation has to exist
    MSTractionSubstation* substation = MSNet::getInstance()->findTractionSubstation(substationID);
    if (substation == nullptr) {
        throw InvalidArgument("Traction substation '" + substationID + "' referenced by an <overheadWire> element '" + sectionID + "' is not defined.");
    }
    else if (substation->isAnySectionPreviouslyDefined()) {
        /// @todo if substation->isAnySectionPreviouslyDefined() && the old syntax of input xml is used, then error
        WRITE_MESSAGEF("Traction substation '%' referenced by an <overheadWire> element '%' is probably referenced multiple times. This is allowed to enable constructing an overheadwire segment with not strictly consecutive lanes.", substationID, sectionID);
        /// throw InvalidArgument("Traction substation '" + substationId + "' referenced by an <overheadWire> element '" + id + "' is probably referenced twice (a known limitation of the actual version of overhead wire simulation).");
    }

    // The lanes where the substation is connected to the overhead wire are listed using voltageSource="..." attribute
    const std::vector<std::string>& constVoltageSources = attrs.getOpt<std::vector<std::string>>(SUMO_ATTR_VOLTAGESOURCE, sectionID.c_str(), ok);
    // RICE_TODO: We need an editable version, is this an appropriate approach?
    std::vector<std::string> voltageSources(constVoltageSources);

    // Process forbidden internal lanes
    const std::vector<std::string>& forbiddenInnerLanesIDs = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_OVERHEAD_WIRE_FORBIDDEN, substationID.c_str(), ok);
    /// @todo for cycle abbreviation?
    for (const std::string& laneID : forbiddenInnerLanesIDs) {
        MSLane* lane = MSLane::dictionary(laneID);
        if (lane != nullptr) {
            substation->addForbiddenLane(lane);
            /// @todo: if isAnySectionPreviouslyDefined(), there could be a overheadwire segment over a newly added forbiddenInnerLane. Such the segment should be deleted.
        }
        else {
            throw InvalidArgument("Unknown forbidden lane '" + laneID + "' for <overheadWire> element '" + sectionID + "' (traction substation '" + substationID + "')lk.");
        }
    }

    /*
     * Every section may reference a pre-defined overhead wire type. This is used in cases when two different overhead wire cross-sections would be used
     * or if we are modelling e.g. trolleybuses and tramways.
     * In case that no overhead wire type is referenced, a default type will be used.
     */
    OverheadWireType& owt = (OverheadWireType&)WIRE_DEFAULTTYPE;  // Overhead wire type with the default parameters
    if (attrs.hasAttribute(SUMO_ATTR_OVERHEAD_WIRE_TYPEID)) {
        // We have a reference to some overhead wire type
        std::string typeID = attrs.get<std::string>(SUMO_ATTR_OVERHEAD_WIRE_TYPEID, 0, ok);
        if (!ok) {
            throw InvalidArgument("Malformed <overheadWire wireid=...> attribute.");
        }
        // Find the referenced OverheadWireType in the map
        auto it = myOverheadWireTypeMap.find(typeID);
        // And handle the case when it has not been found
        if (it == myOverheadWireTypeMap.end()) {
            throw InvalidArgument("Overhead wire type '" + typeID + "' referenced by an <overheadWire> element '" + sectionID + "' is not defined.");
        }
        // Replace the default `owt` with the overhead wire type stored in the map
        owt = it->second;
    }

    // Ignore the old-style definition completely, the `lanes` attribute has to be present.
    // RICE_TODO: This migth be the job of a XSD schema and we do not need to check it here.
    if (!attrs.hasAttribute(SUMO_ATTR_LANES)) {
        throw InvalidArgument(fmt::format("Overhead wire element '{}' does not have 'lanes' attribute. If this is the old style definition, please update it.", sectionID));
    }
    // Get the list of lanes over which the overhead wire segments shall be built
    const std::vector<std::string>& laneIDs = attrs.get<std::vector<std::string>>(SUMO_ATTR_LANES, sectionID.c_str(), ok);

    // Check that the first and the last segments of the overhead wire section really do not have
    // any incoming or outgoing lanes that would be a part of this 
    const MSLane* firstLane = MSLane::dictionary(laneIDs.front());
    const MSLane* lastLane = MSLane::dictionary(laneIDs.back());

    // Check the firstLane for incoming lanes
    for (auto const& incomingLaneInfo : firstLane->getIncomingLanes()) {
        // The incoming lane for this lane
        const MSLane* incomingLane = incomingLaneInfo.lane;
        // Make sure the lane is not part of `laneIDs`
        if (std::find(laneIDs.begin(), laneIDs.end(), incomingLane->getID()) != laneIDs.end())
        {
            throw InvalidArgument(fmt::format("First lane '{}' of <overheadWire> element '{}' has predecessor '{}' referenced by the same element. This is not allowed.", firstLane->getID(), sectionID, incomingLane->getID()));
        }
    }
    // Check the lastLane for outgoing lanes
    for (auto const& outgoingLaneAndEdge : lastLane->getOutgoingViaLanes()) {
        // The outgoing lane for this lane
        const MSLane* outgoingLane = const_cast<MSLane*>(outgoingLaneAndEdge.first);
        // Make sure the lane is not part of `laneIDs`
        if (std::find(laneIDs.begin(), laneIDs.end(), outgoingLane->getID()) != laneIDs.end())
        {
            throw InvalidArgument(fmt::format("Last lane '{}' of <overheadWire> element '{}' has successor '{}' referenced by the same element. This is not allowed.", lastLane->getID(), sectionID, outgoingLane->getID()));
        }
    }

    /*
     * Create a lane connection map for the lanes referenced by this overhead wire section.
     * The map records for every lane object (represented as a MSLane*, as building the overhead
     * wire later requires passing mutable MSLane*) the set of lanes where the overhead wire
     * section can continue, i.e. lanes that are connected to its beginning or to its end and
     * that already have overhead wire segment over them or such segment will be constructed
     * beacuse the overhead wire segments are just being created (and hence listed in `laneIDs`).
     * The lane objects are again represented as MSLane* pointers.
     */
    std::unordered_map<const MSLane*, std::pair<std::set<const MSLane*>, std::set<const MSLane*>>> laneConnectionMap;
    // Initialise the `laneConnectionMap` with the lanes referenced by this overhead wire section
    for (auto laneID : laneIDs) {
        MSLane* lane = MSLane::dictionary(laneID);
        if (lane == nullptr) {
            throw InvalidArgument(fmt::format("The lane '{}' referenced by <overheadWire> element '{}' does not exist or is broken.", laneID, sectionID));
        }
        // Add the lane to the map and initialise the connections as empty sets
        laneConnectionMap[lane] = { {}, {} };
    }
    /*
     * Having initialised the map, we need to establish lane-to-lane connections. The order of lanes
     * written in the `lanes` attribute is arbitrary, there may be splits of the wire into several
     * directions within an intersection or sections that are composed of more than a single set of
     * wire segments. Also, the internal lanes of intersections are involved and have to be added to
     * the map.
     */
    // ows_326065806#0_0 -> ows_326065806#3_0
    for (const auto& [lane, connections] : laneConnectionMap) {
        /*
         * ------------------------------------------------------
         * Step #1: Check all outgoing lanes of the current lane.
         * ------------------------------------------------------
         */
         // The `getOutgoingViaLanes()` method returns a vector of pairs of MSLane* and corresponding MSEdge*.
        for (auto const& outgoingLaneAndEdge : lane->getOutgoingViaLanes()) {
            // The outgoing lane for this lane
            const MSLane* outgoingLane = const_cast<MSLane*>(outgoingLaneAndEdge.first);
            // Comparing lanes 326065806#0_0 -> 12844782#0_0 ... no wire
            // Comparing lanes 326065806#0_0 -> 326065806#3_0 ... no wire yet
            if (lane->getID() == "326065806#0_0") {
                std::cout << "Comparing lanes " << lane->getID() << " -> " << outgoingLane->getID() << std::endl;
            }
            // The connection between the two "regular" lanes may not be direct - there may be an internal lane in between.
            // If there is an internal lane connecting the two lanes, we need to find it.
            const MSLane* connection = lane->getInternalFollowingLane(outgoingLane);
            if (connection == nullptr) {
                // No internal lane found, the outgoing lane is directly connected to `lane`.
                // Is the `outgoingLane` part of this overhead wire section?
                auto it = laneConnectionMap.find(outgoingLane);
                if (it != laneConnectionMap.end()) {
                    // Yes, add the connection to the map
                    laneConnectionMap[lane].second.insert(outgoingLane);
                    laneConnectionMap[outgoingLane].first.insert(lane);
                }
            }
            else {
                /*
                 * There is an internal lane connecting `lane` and `outgoingLane`. Due to the way
                 * how SUMO internal lanes are constructed, we may have more than a single internal lane
                 * connecting `lane` and `outgoingLane`. We need to check whether this is the case.
                 */
                const MSLane* connFN = lane->getInternalFollowingLane(connection);
                // RICE_TODO: Is it possible that `connFN` is alaways nullptr?
                if (connFN != nullptr) {
                    WRITE_WARNING(fmt::format("Lane `{}` getInternalFollowingLane(`{}`) returns `{}`, but connection seems to be `{}`-`{}`-`{}`...`{}`. Should this be possible?",
                        lane->getID(), outgoingLane->getID(), connection->getID(),
                        lane->getID(), connFN->getID(), connection->getID(), outgoingLane->getID()));
                }
                const MSLane* connFC = connection->getInternalFollowingLane(outgoingLane);
                // Check whether the internal lane is forbidden for this substation
                if (substation->isForbidden(connection) || substation->isForbidden(connFN) || substation->isForbidden(connFC)) {
                    // The internal lane is forbidden, we cannot use it to connect the two lanes
                    continue;
                }

                /*
                 * The connection from `lane` to `outgoingLane` is not forbidden. But we want to connect
                 * both lanes only if there is an overhead wire section over the `outgoingLane` as well.
                 * The outgoing lane does not have to be a part of this overhead wire section, but may be
                 * part of another one. Our standard assumption is that the internal lanes directly following
                 * a lane with overhead wire are also equipped with overhead wire _if_ there is an overhead
                 * wire on the other side of the intersection and the wire constructed here ends at the end
                 * of the last internal lane. The connection to `outgoingLane` is made only in case that the
                 * `outgoingLane` is part of this overhead wire section.
                 */

                 // Check whether `outgoingLane` is part of this overhead wire section and will be connected
                 // to internal overhad wires at the intersection
                bool connectToOutgoingLane = (laneConnectionMap.find(outgoingLane) != laneConnectionMap.end());
                // If the connection is to be made, we need to create internal overhead wires
                bool createInternalWires = connectToOutgoingLane;
                // But we may need to create overhead wires over the internal lanes also if there is an overhad
                // wire over the `outgoingLane` connected to another substation. In this case, however, no
                // connection fo `outgoingLane` will be made
                if (!createInternalWires) {
                    // Is there an overhead wire segment over the outgoing lane?
                    std::string outgoingOWSID = MSNet::getInstance()->getStoppingPlaceID(outgoingLane, NUMERICAL_EPS, SUMO_TAG_OVERHEAD_WIRE_SEGMENT);
                    if (!outgoingOWSID.empty()) {
                        // Yes, there is a segment that is connected to another substation endpoint
                        createInternalWires = true;
                    }
                }

                // Early exit in case that no internal wires shall be created
                if (!createInternalWires) continue;

                // Internal wires, segment #1: from `lane` to `connection`
                if (connFN != nullptr) {
                    // The internal lane sequence is `lane` -> `connFN` -> `connection`
                    // Part 1: from `lane` to `connFN`
                    laneConnectionMap[lane].second.insert(connFN);
                    laneConnectionMap[connFN].first.insert(lane);
                    // Part 2: from `connFN` to `connection`
                    laneConnectionMap[connFN].second.insert(connection);
                    laneConnectionMap[connection].first.insert(connFN);
                }
                else {
                    // The internal lane sequence is `lane` -> `connection`
                    laneConnectionMap[lane].second.insert(connection);
                    laneConnectionMap[connection].first.insert(lane);
                }

                // Internal wires, segment #2: from `connection` to `outgoingLane`
                if (connFC != nullptr) {
                    // The internal lane sequence is `connection` -> `connFC` -> `outgoingLane`
                    // Part 1: from `connection` to `connFC`
                    laneConnectionMap[connection].second.insert(connFC);
                    laneConnectionMap[connFC].first.insert(connection);
                    // Part 2: from `connFC` to `outgoingLane`
                    if (connectToOutgoingLane) {
                        laneConnectionMap[connFC].second.insert(outgoingLane);
                        laneConnectionMap[outgoingLane].first.insert(connFC);
                    }
                }
                else if (connectToOutgoingLane) {
                    // The internal lane sequence is `connection` -> `outgoingLane`
                    // Check that we have no connections yet
                    // ***** checkEmptyConnections(laneConnectionMap, connection, outgoingLane);
                    // And update both lane pointers
                    laneConnectionMap[connection].second.insert(outgoingLane);
                    laneConnectionMap[outgoingLane].first.insert(connection);
                }
            }
        }

        /*
         * ------------------------------------------------------
         * Step #2: Check all incoming lanes of the current lane.
         * ------------------------------------------------------
         * The second step is needed to finalize overhead wire connections over internal lanes
         * of intersections: Due to sequential nature of overhead wire setup, the overhead wires
         * over internal lanes may not be created in step #1 due to the overhead wire not yet
         * been constructed for the `outgongLane` of an intersection. This step takes care of
         * this situation by finalizing connection in the "bacward" direction and instantiating
         * wires between `incomingLane` and `lane` in case that the `incomingLane` is part of
         * another overhead wire segment.
         */

         // The `getIncomingLanes()` method returns a vector of MSLane::IncomingLaneInfo elements.
        for (auto const& incomingLaneInfo : lane->getIncomingLanes()) {
            // The directr predecessor lane for this lane. May be internal
            const MSLane* incomingLane = incomingLaneInfo.lane;
            while (incomingLane->isInternal()) {
                // Fetch the intersection inoput -> output link for this lane
                const MSLink* link = incomingLane->getEntryLink();
                incomingLane = link->getLaneBefore();
            }
            // Comparing lanes -12844782#1_0 -> 326065806#3_0 ... no wire
            // Comparing lanes 326065806#0_0 -> 326065806#3_0 ... should have wire
            if (lane->getID() == "326065806#3_0") {
                std::cout << "Comparing lanes " << incomingLane->getID() << " -> " << lane->getID() << std::endl;
            }
            // The connection between the two "regular" lanes may not be direct - there may be an 
            // internal lane in between.
            // If there is an internal lane connecting the two lanes, we need to find it.
            const MSLane* connection = incomingLane->getInternalFollowingLane(lane);
            // Early exit if there is no internal connection. In such case no additional wires
            // will be built.
            if (connection == nullptr) continue;
            /*
             * There is an internal lane connecting `incomingLane` and `lane`. Due to the way
             * how SUMO internal lanes are constructed, we may have more than a single internal lane
             * connecting `internalLane` and `lane` and we need to check whether this is the case.
             */
            const MSLane* connFN = incomingLane->getInternalFollowingLane(connection);
            // RICE_TODO: Is it possible that `connFN` is alaways nullptr?
            if (connFN != nullptr) {
                WRITE_WARNING(fmt::format("Lane `{}` getInternalFollowingLane(`{}`) returns `{}`, but connection seems to be `{}`-`{}`-`{}`...`{}`. Should this be possible?",
                    incomingLane->getID(), lane->getID(), connection->getID(),
                    incomingLane->getID(), connFN->getID(), connection->getID(), lane->getID()));
            }
            const MSLane* connFC = connection->getInternalFollowingLane(lane);
            // Check whether the internal lane is forbidden for this substation
            if (substation->isForbidden(connection) || substation->isForbidden(connFN) || substation->isForbidden(connFC)) {
                // The internal lane is forbidden, we cannot use it to connect the two lanes
                continue;
            }

            /*
             * The connection from `incomingLane` to `lane` is not forbidden. But we want to connect
             * both lanes only if there is an overhead wire section over the `incomingLane` as well.
             * The incoming lane will not be a part of this overhead wire section (that case has been
             * already handled in step #1).
            */

            // Is there an overhead wire segment over the outgoing lane?
            std::string incomingOWSID = MSNet::getInstance()->getStoppingPlaceID(incomingLane, NUMERICAL_EPS, SUMO_TAG_OVERHEAD_WIRE_SEGMENT);

            // Early exit in case that no internal wires shall be created
            if (incomingOWSID.empty()) continue;

            // Get the instance of the overhead wire segment over the `incomingLane`
            const MSOverheadWire* incomingWire = static_cast<MSOverheadWire*>(MSNet::getInstance()->getStoppingPlace(incomingOWSID, SUMO_TAG_OVERHEAD_WIRE_SEGMENT));
            // Get the substation where the incoming wire is connected to
            const MSTractionSubstation* incomingSubstation = incomingWire->getTractionSubstation();
            // We want to connect both wires only if they are connected to the same substation
            bool connectToIncomingLane = (incomingSubstation == substation);

            // Internal wires, segment #1: from `incomingLane` to `connection`
            // The internal wires will be connected to the substation of this overhead wire section.
            // The connection of the overhead segment over `incomingLane` will occur only in case that
            // the lane is connected to the same substation.
            if (connFN != nullptr) {
                // The internal lane sequence is `incomingLane` -> `connFN` -> `connection`
                // Part 1: Connection between `incominglane` and `connFN` only if they both belong to the
                // same substation
                if (connectToIncomingLane) {
                    laneConnectionMap[incomingLane].second.insert(connFN);
                    laneConnectionMap[connFN].first.insert(incomingLane);
                }
                // Part 2: from `connFN` to `connection`
                laneConnectionMap[connFN].second.insert(connection);
                laneConnectionMap[connection].first.insert(connFN);
            } else {
                // If connFN == nullptr, the internal lane sequence is `incomingLane` -> `connection` and 
                // the connection between them is setup only in case that they both belong to the same 
                // substation.
                if (connectToIncomingLane) {
                    laneConnectionMap[incomingLane].second.insert(connection);
                    laneConnectionMap[connection].first.insert(incomingLane);
                }
            }
            // Internal wires, segment #2: from `connection` to `lane`
            if (connFC != nullptr) {
                // The internal lane sequence is `connection` -> `connFC` -> `lane`
                // Part 1: from `connection` to `connFC`
                laneConnectionMap[connection].second.insert(connFC);
                laneConnectionMap[connFC].first.insert(connection);
                // Part 2: from `connFC` to `lane`
                laneConnectionMap[connFC].second.insert(lane);
                laneConnectionMap[lane].first.insert(connFC);
            }
            else {
                // The internal lane sequence is `connection` -> `lane`
                laneConnectionMap[connection].second.insert(lane);
                laneConnectionMap[lane].first.insert(connection);
            }
        }
    }

#ifdef OVERHEAD_WIRE_DEBUG
    // Debug output of the lane connection map
    std::cout << "--------------------" << std::endl << "OWS ID " + sectionID << std::endl << "--------------------" << std::endl;
    for (const auto& [lane, connections] : laneConnectionMap) {
        const auto& [incomingLanes, outgoingLanes] = connections;

        std::cout << lane->getID() << ":" << std::endl << "- incoming ";

        if (incomingLanes.empty()) {
            std::cout << "(none) ";
        }
        else {
            for (const auto& incomingLane : incomingLanes) {
                std::cout << incomingLane->getID() << " ";
            }
        }

        std::cout << std::endl << "- outgoing ";

        if (outgoingLanes.empty()) {
            std::cout << "(none)";
        }
        else {
            for (const auto& outgoingLane : outgoingLanes) {
                std::cout << outgoingLane->getID() << " ";
            }
        }

        std::cout << std::endl;
    }
#endif // OVERHEAD_WIRE_DEBUG

    // Shall we automatically update the beginning / end of the overhead wire in case that the
    // positions written in the section definitions are outside the fist or last segment?
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, sectionID.c_str(), ok, false);

    // Need to create segments over the given lanes and assign segment ids derived from lane ids.
    std::unordered_map<const MSLane*, MSOverheadWire*> segments;
    for (const auto& [lane, connections] : laneConnectionMap) {
        // Derive the overhead wire segment ID from the lane ID
        std::string segmentID = MSOverheadWire::getOWSIDforLane(*lane);
        // Handle `startPos` and `endPos` attributes, which are used for the first and last lane of the section.
        double frompos = (lane == firstLane) ? attrs.getOpt<double>(SUMO_ATTR_STARTPOS, sectionID.c_str(), ok, 0.0) : 0.0;
        double topos = (lane == lastLane) ? attrs.getOpt<double>(SUMO_ATTR_ENDPOS, sectionID.c_str(), ok, lane->getLength()) : lane->getLength();
        // RICE_TODO: This is not necessary for intermediate lanes
        // Handle friendlyPos ...
        if (myHandler->checkStopPos(frompos, topos, lane->getLength(), POSITION_EPS, friendlyPos) != SUMORouteHandler::StopPos::STOPPOS_VALID) {
            frompos = 0.0;
            topos = lane->getLength();
            WRITE_MESSAGE(fmt::format("The overhead wire segment '{}' has wrong stop position. Automatically set from 0 to the length of the lane '{}'.", segmentID, lane->getID()));
        }
        // Check if the beginning of this overhead wire segment is connected to the substation
        bool isVoltageSource = false;
        auto it = std::find(voltageSources.begin(), voltageSources.end(), lane->getID());
        if (it != voltageSources.end())
        {
            // Yes, the current lane is referenced as a voltage source
            isVoltageSource = true;
            // Remove the lane from the list of lanes with substation connection, the voltage sources list has to be empty at the end
            voltageSources.erase(it);
        }
        // Build a bare overhead wire segment over the lane
        MSOverheadWire* ovrhdSegment = buildOverheadWireSegment(net, segmentID, sectionID, lane, frompos, topos, owt, isVoltageSource);
        // Add traction substation to this segment
        ovrhdSegment->setTractionSubstation(substation);
        // Add the overhead wire segment to the map of segment instances
        segments[lane] = ovrhdSegment;
    }
    // Check that the overhead wire section is connected at all referenced connection points
    if (!voltageSources.empty())
    {
        // @todo Is there a convenience function to convert vector of strings to string?
        std::string s;
        s.reserve(voltageSources.size() * 32);  // arbitrarily chosen element size
        auto lastIt = voltageSources.end();
        --lastIt;
        for (auto it = voltageSources.begin(); it != voltageSources.end(); ++it) {
            s += "'" + *it + "'";
            if (it != lastIt) s += ", ";
        }
        throw InvalidArgument(fmt::format("The <overheadWire> element '{}' does not contain lane(s) {} where the substation shall be connected.", sectionID, s));
    }

    /*
     * Now that the bare segments have been constructed, we need to interconnect them into a linked list of
     * overhead wire segments. The connections are needed later for overhead wire circuit construction to
     * save repeated parsing of the network when determinging the connections between segments.
     */
    for (const auto& [lane, connections] : laneConnectionMap) {
        // Get the overhead wire segment for this lane
        MSOverheadWire* ovrhdSegment = segments[lane];
        // Connect the segment for the incoming lane
        for (const auto& incomingLane : connections.first) {
            MSOverheadWire* ovrhdSegmentFrom = segments[incomingLane];
            ovrhdSegmentFrom->addOutgoingSegment(ovrhdSegment);
            ovrhdSegment->addIncomingSegment(ovrhdSegmentFrom);
        }
        // Connect the segment for the outgoing lane
        for (const auto& outgoingLane : connections.second) {
            MSOverheadWire* ovrhdSegmentTo = segments[outgoingLane];
            ovrhdSegmentTo->addIncomingSegment(ovrhdSegment);
            ovrhdSegment->addOutgoingSegment(ovrhdSegmentTo);
        }
    }

    /*
     * Add all constructed segments to the overhead wire circuit of the current substation.
     * The key in the `segments` map is the lane as `MSLane*`, the value is `MSOverheadWire*`
     */
    for (const auto& [lane, ovrhdSegment] : segments) {
        substation->addOverheadWireSegmentToCircuit(ovrhdSegment);
    }

    /*
     * Add overhead wire clamps, if referenced.
     */
    std::string clampsString = attrs.getOpt<std::string>(SUMO_ATTR_OVERHEAD_WIRE_CLAMPS, nullptr, ok, "");
    if (clampsString != "" && MSGlobals::gOverheadWireSolver) {
#ifdef HAVE_EIGEN
        const std::vector<std::string>& clampIDs = attrs.get<std::vector<std::string> >(SUMO_ATTR_OVERHEAD_WIRE_CLAMPS, nullptr, ok);
        for (const std::string& clampID : clampIDs) {
            MSTractionSubstation::OverheadWireClamp* clamp = substation->findClamp(clampID);
            if (clamp != nullptr) {
                if (clamp->start->getTractionSubstation() == substation && clamp->end->getTractionSubstation() == substation) {
                    substation->addOverheadWireClampToCircuit(clamp->id, clamp->start, clamp->end);
                    buildOverheadWireClamp(net, clamp->id, const_cast<MSLane*>(&clamp->start->getLane()), const_cast<MSLane*>(&clamp->end->getLane()));
                    clamp->usage = true;
                }
                else {
                    if (clamp->start->getTractionSubstation() != substation) {
                        WRITE_WARNINGF(TL("A connecting overhead wire start segment '%' defined for overhead wire clamp '%' is not assigned to the traction substation '%'."), clamp->start->getID(), clampID, substationID);
                    }
                    else {
                        WRITE_WARNINGF(TL("A connecting overhead wire end segment '%' defined for overhead wire clamp '%' is not assigned to the traction substation '%'."), clamp->end->getID(), clampID, substationID);
                    }
                }
            }
            else {
                WRITE_WARNINGF(TL("The overhead wire clamp '%' defined in an overhead wire section was not assigned to the substation '%'. Please define proper <overheadWireClamp .../> in additional files before defining overhead wire section."), clampID, substationID);
            }
        }
#else
        WRITE_WARNING(TL("Adding overhead wire clamps requires solver support (Eigen), which has bot been compiled in."));
#endif
    }

    if (segments.size() == 0) {
        throw InvalidArgument(fmt::format("No segments found for overHeadWireSection '{}'.", substationID));
    }
    else if (MSGlobals::gOverheadWireSolver) {
#ifdef HAVE_EIGEN
        // check that the electric circuit makes sense
        // @todo Circuit should be checked after loading of all additional files. Not at this place.
        // @todo Since we allow not to define a voltageSource in <overheadWire> definition, the circuit checking should also verify that at least one voltage source is in the circuit.
        // @todo The checking now takes place in NLBuilder.cpp
        // segments[0]->getCircuit()->checkCircuit(substationId);
#else
        WRITE_WARNING(TL("Cannot check circuit, overhead circuit solver support (Eigen) not compiled in."));
#endif
    }
}

void
NLTriggerBuilder::parseAndBuildOverheadWireType(MSNet& net, const SUMOSAXAttributes& attrs) {
    bool ok = true;

    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        throw ProcessError();
    }

    // Fetch resistivity and crosssection area of this wire type
    const double resistivity = attrs.getOpt<double>(SUMO_ATTR_OVERHEAD_WIRE_RESISTIVITY, id.c_str(), ok, WIRE_RESISTIVITY);
    const double crossSection = attrs.getOpt<double>(SUMO_ATTR_OVERHEAD_WIRE_CROSSSECTION, id.c_str(), ok, WIRE_CROSSSECTION);
    // Add wire type information to map of known wire types
    OverheadWireType owt(id, resistivity, crossSection);
    // Note: Using [...] would require a default constructor creating a dummy object first
    myOverheadWireTypeMap.insert({ id, owt });
}

void
NLTriggerBuilder::parseAndBuildTractionSubstation(MSNet& net, const SUMOSAXAttributes& attrs) {
    bool ok = true;

    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        throw ProcessError();
    }

    // RICE_TODO Limits are fixed, change them to some predefined constants ...
    const double voltage = attrs.getOpt<double>(SUMO_ATTR_VOLTAGE, id.c_str(), ok, 600);
    const double currentLimit = attrs.getOpt<double>(SUMO_ATTR_CURRENTLIMIT, id.c_str(), ok, 400);
    buildTractionSubstation(net, id, voltage, currentLimit);
}

void
NLTriggerBuilder::parseAndBuildOverheadWireClamp(MSNet& /*net*/, const SUMOSAXAttributes& attrs) {
    if (MSGlobals::gOverheadWireSolver) {
#ifdef HAVE_EIGEN
        bool ok = true;
        std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
        if (!ok) {
            throw ProcessError();
        }

        std::string substationId = attrs.get<std::string>(SUMO_ATTR_SUBSTATIONID, 0, ok);
        if (!ok) {
            throw ProcessError();
        }
        MSTractionSubstation* substation = MSNet::getInstance()->findTractionSubstation(substationId);
        if (substation == nullptr) {
            throw InvalidArgument("Traction substation '" + substationId + "' using within an overheadWireClamp '" + id + "' is not known.");
        }

        std::string overhead_fromItsStart = attrs.get<std::string>(SUMO_ATTR_OVERHEAD_WIRE_CLAMP_START, 0, ok);
        if (!ok) {
            throw ProcessError();
        }
        MSOverheadWire* ovrhdSegment_fromItsStart = dynamic_cast<MSOverheadWire*>(MSNet::getInstance()->getStoppingPlace(overhead_fromItsStart, SUMO_TAG_OVERHEAD_WIRE_SEGMENT));
        if (ovrhdSegment_fromItsStart == nullptr) {
            throw InvalidArgument("The overheadWireSegment '" + overhead_fromItsStart + "' to use within overheadWireClamp '" + id + "' is not known.");
        }
        /*if (ovrhdSegment_fromItsStart->getTractionSubstation() != substation) {
            throw InvalidArgument("The overheadWireSegment '" + overhead_fromItsStart + "' to use within overheadWireClamp is assign to a different overhead wire section or substation.");
        }
        */
        std::string overhead_fromItsEnd = attrs.get<std::string>(SUMO_ATTR_OVERHEAD_WIRE_CLAMP_END, 0, ok);
        if (!ok) {
            throw ProcessError();
        }
        MSOverheadWire* ovrhdSegment_fromItsEnd = dynamic_cast<MSOverheadWire*>(MSNet::getInstance()->getStoppingPlace(overhead_fromItsEnd, SUMO_TAG_OVERHEAD_WIRE_SEGMENT));
        if (ovrhdSegment_fromItsEnd == nullptr) {
            throw InvalidArgument("The overheadWireSegment '" + overhead_fromItsEnd + "' to use within overheadWireClamp '" + id + "' is not known.");
        }
        /*
        if (ovrhdSegment_fromItsEnd->getTractionSubstation() != substation) {
            throw InvalidArgument("The overheadWireSegment '" + overhead_fromItsEnd + "' to use within overheadWireClamp is assign to a different overhead wire section or substation.");
        }
        */
        if (substation->findClamp(id) == nullptr) {
            substation->addClamp(id, ovrhdSegment_fromItsStart, ovrhdSegment_fromItsEnd);
        } else {
            WRITE_ERROR("The overhead wire clamp '" + id + "' is probably declared twice.")
        }
#else
        UNUSED_PARAMETER(attrs);
        WRITE_WARNING(TL("Not building overhead wire clamps, overhead wire solver support (Eigen) not compiled in."));
#endif
    } else {
        WRITE_WARNING(TL("Ignoring overhead wire clamps, they make no sense when overhead wire circuit solver is off."));
    }
}


void
NLTriggerBuilder::parseAndBuildStoppingPlace(MSNet& net, const SUMOSAXAttributes& attrs, const SumoXMLTag element) {
    bool ok = true;
    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (!ok) {
        throw ProcessError();
    }

    //get the name, leave blank if not given
    const std::string ptStopName = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), ok, "");

    //get the color, use default if not given
    // default color, copy from GUIVisualizationStoppingPlaceSettings::busStopColor / containerStopColor
    RGBColor color = attrs.getOpt<RGBColor>(SUMO_ATTR_COLOR, id.c_str(), ok, RGBColor::INVISIBLE);

    MSLane* lane = getLane(attrs, toString(element), id);
    // get the positions
    double frompos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, id.c_str(), ok, 0.);
    double topos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), ok, lane->getLength());
    double angle = attrs.getOpt<double>(SUMO_ATTR_ANGLE, id.c_str(), ok, 90);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), ok, false);
    if (!ok || (myHandler->checkStopPos(frompos, topos, lane->getLength(), POSITION_EPS, friendlyPos) != SUMORouteHandler::StopPos::STOPPOS_VALID)) {
        throw InvalidArgument("Invalid position for " + toString(element) + " '" + id + "'.");
    }
    const std::vector<std::string>& lines = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_LINES, id.c_str(), ok);
    int defaultCapacity;
    SumoXMLAttr capacityAttr;
    if (element != SUMO_TAG_CONTAINER_STOP) {
        defaultCapacity = MAX2(MSStoppingPlace::getDefaultTransportablesAbreast(topos - frompos, element) * 3, 6);
        capacityAttr = SUMO_ATTR_PERSON_CAPACITY;
    } else {
        defaultCapacity = MSStoppingPlace::getDefaultTransportablesAbreast(topos - frompos, element);
        capacityAttr = SUMO_ATTR_CONTAINER_CAPACITY;
    }
    const int transportableCapacity = attrs.getOpt<int>(capacityAttr, id.c_str(), ok, defaultCapacity);
    const double parkingLength = attrs.getOpt<double>(SUMO_ATTR_PARKING_LENGTH, id.c_str(), ok, 0);
    // build the bus stop
    buildStoppingPlace(net, id, lines, lane, frompos, topos, element, ptStopName, transportableCapacity, parkingLength, color, angle);
}


void
NLTriggerBuilder::addAccess(MSNet& /* net */, const SUMOSAXAttributes& attrs) {
    if (myCurrentStop == nullptr) {
        throw InvalidArgument("Could not add access outside a stopping place.");
    }
    // get the lane
    MSLane* lane = getLane(attrs, "access", myCurrentStop->getID());
    if (!lane->allowsVehicleClass(SVC_PEDESTRIAN)) {
        WRITE_WARNINGF(TL("Ignoring invalid access from non-pedestrian lane '%' in busStop '%'."), lane->getID(), myCurrentStop->getID());
        return;
    }
    // get the positions
    bool ok = true;
    const std::string accessPos = attrs.getOpt<std::string>(SUMO_ATTR_POSITION, "access", ok);
    const bool random = accessPos == "random";
    MSStoppingPlace::AccessExit exit = MSStoppingPlace::AccessExit::PLATFORM;
    if (accessPos == "doors") {
        exit = MSStoppingPlace::AccessExit::DOORS;
    } else if (accessPos == "carriage") {
        exit = MSStoppingPlace::AccessExit::CARRIAGE;
    }
    double startPos = random || exit != MSStoppingPlace::AccessExit::PLATFORM ? 0. : attrs.getOpt<double>(SUMO_ATTR_POSITION, "access", ok, 0);
    double endPos = random || exit != MSStoppingPlace::AccessExit::PLATFORM ? lane->getLength() : startPos;
    const double length = attrs.getOpt<double>(SUMO_ATTR_LENGTH, "access", ok, -1);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, "access", ok, false);
    if (!ok || (myHandler->checkStopPos(startPos, endPos, lane->getLength(), 0, friendlyPos) != SUMORouteHandler::StopPos::STOPPOS_VALID)) {
        throw InvalidArgument("Invalid position " + attrs.getString(SUMO_ATTR_POSITION) + " for access on lane '" + lane->getID() + "' in stop '" + myCurrentStop->getID() + "'.");
    }
    // add bus stop access
    if (!myCurrentStop->addAccess(lane, startPos, endPos, length, exit)) {
        throw InvalidArgument("Duplicate access on lane '" + lane->getID() + "' for stop '" + myCurrentStop->getID() + "'");
    }
}


void
NLTriggerBuilder::parseAndBeginParkingArea(MSNet& net, const SUMOSAXAttributes& attrs) {
    bool ok = true;
    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (!ok) {
        throw ProcessError();
    }
    // get the lane
    MSLane* lane = getLane(attrs, "parkingArea", id);
    // get the positions
    double frompos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, id.c_str(), ok, 0);
    double topos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), ok, lane->getLength());
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), ok, false);
    unsigned int capacity = attrs.getOpt<int>(SUMO_ATTR_ROADSIDE_CAPACITY, id.c_str(), ok, 0);
    myParkingAreaCapacitySet = attrs.hasAttribute(SUMO_ATTR_ROADSIDE_CAPACITY);
    bool onRoad = attrs.getOpt<bool>(SUMO_ATTR_ONROAD, id.c_str(), ok, false);
    double width = attrs.getOpt<double>(SUMO_ATTR_WIDTH, id.c_str(), ok, 0);
    double length = attrs.getOpt<double>(SUMO_ATTR_LENGTH, id.c_str(), ok, 0);
    double angle = attrs.getOpt<double>(SUMO_ATTR_ANGLE, id.c_str(), ok, 0);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), ok);
    const std::string departPos = attrs.getOpt<std::string>(SUMO_ATTR_DEPARTPOS, id.c_str(), ok);
    bool lefthand = attrs.getOpt<bool>(SUMO_ATTR_LEFTHAND, id.c_str(), ok, false);
    const std::vector<std::string>& acceptedBadges = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_ACCEPTED_BADGES, id.c_str(), ok);

    if (!ok || (myHandler->checkStopPos(frompos, topos, lane->getLength(), POSITION_EPS, friendlyPos) != SUMORouteHandler::StopPos::STOPPOS_VALID)) {
        throw InvalidArgument("Invalid position for parking area '" + id + "'.");
    }
    const std::vector<std::string>& lines = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_LINES, id.c_str(), ok);
    // build the parking area
    beginParkingArea(net, id, lines, acceptedBadges, lane, frompos, topos, capacity, width, length, angle, name, onRoad, departPos, lefthand);
}



void
NLTriggerBuilder::parseAndAddLotEntry(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    // Check for open parking area
    if (myParkingArea == nullptr) {
        throw ProcessError();
    }
    // get the positions
    double x = attrs.get<double>(SUMO_ATTR_X, "", ok);
    if (!ok) {
        throw InvalidArgument("Invalid x position for lot entry.");
    }
    double y = attrs.get<double>(SUMO_ATTR_Y, "", ok);
    if (!ok) {
        throw InvalidArgument("Invalid y position for lot entry.");
    }
    double z = attrs.getOpt<double>(SUMO_ATTR_Z, "", ok, 0.);
    double width = attrs.getOpt<double>(SUMO_ATTR_WIDTH, "", ok, myParkingArea->getWidth());
    double length = attrs.getOpt<double>(SUMO_ATTR_LENGTH, "", ok, myParkingArea->getLength());
    double angle = attrs.getOpt<double>(SUMO_ATTR_ANGLE, "", ok, myParkingArea->getAngle());
    double slope = attrs.getOpt<double>(SUMO_ATTR_SLOPE, "", ok, 0.);
    // add the lot entry
    addLotEntry(x, y, z, width, length, angle, slope);
}


void
NLTriggerBuilder::parseAndBuildCalibrator(MSNet& net, const SUMOSAXAttributes& attrs,
        const std::string& base) {
    bool ok = true;
    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (!ok) {
        throw ProcessError();
    }
    MSLane* lane = nullptr;
    MSEdge* edge = nullptr;
    MSJunction* node = nullptr;
    if (attrs.hasAttribute(SUMO_ATTR_NODE)) {
        if (attrs.hasAttribute(SUMO_ATTR_LANE) || attrs.hasAttribute(SUMO_ATTR_EDGE)) {
            throw InvalidArgument("The node calibrator '" + id + "' cannot define an edge or lane as well.");
        }
        const std::string nodeID = attrs.get<std::string>(SUMO_ATTR_NODE, id.c_str(), ok);
        node = net.getJunctionControl().get(nodeID);
        if (node == nullptr) {
            throw InvalidArgument("The node " + nodeID + " to use within the calibrator '" + id + "' is not known.");
        }
    } else {
        if (attrs.hasAttribute(SUMO_ATTR_EDGE)) {
            const std::string edgeID = attrs.get<std::string>(SUMO_ATTR_EDGE, id.c_str(), ok);
            edge = MSEdge::dictionary(edgeID);
            if (edge == nullptr) {
                throw InvalidArgument("The edge " + edgeID + " to use within the calibrator '" + id + "' is not known.");
            }
            if (attrs.hasAttribute(SUMO_ATTR_LANE)) {
                lane = getLane(attrs, "calibrator", id);
                if (&lane->getEdge() != edge) {
                    throw InvalidArgument("The edge " + edgeID + " to use within the calibrator '" + id
                                          + "' does not match the calibrator lane '" + lane->getID() + ".");
                }
            }
        } else {
            lane = getLane(attrs, "calibrator", id);
            edge = &lane->getEdge();
        }
    }
    const double pos = node != nullptr ? 0 : getPosition(attrs, lane, "calibrator", id, edge);
    const SUMOTime period = attrs.getOptPeriod(id.c_str(), ok, DELTA_T); // !!! no error handling
    const std::string vTypes = attrs.getOpt<std::string>(SUMO_ATTR_VTYPES, id.c_str(), ok, "");
    const std::string file = getFileName(attrs, base, true);
    const std::string outfile = attrs.getOpt<std::string>(SUMO_ATTR_OUTPUT, id.c_str(), ok, "");
    const std::string routeProbe = attrs.getOpt<std::string>(SUMO_ATTR_ROUTEPROBE, id.c_str(), ok, "");
    // differing defaults for backward compatibility, values are dimensionless
    const double invalidJamThreshold = attrs.getOpt<double>(SUMO_ATTR_JAM_DIST_THRESHOLD, id.c_str(), ok, MSGlobals::gUseMesoSim ? 0.8 : 0.5);
    const bool local = attrs.getOpt<bool>(SUMO_ATTR_LOCAL, id.c_str(), ok, false);
    MSRouteProbe* probe = nullptr;
    if (routeProbe != "") {
        probe = dynamic_cast<MSRouteProbe*>(net.getDetectorControl().getTypedDetectors(SUMO_TAG_ROUTEPROBE).get(routeProbe));
        if (probe == nullptr) {
            throw InvalidArgument("The routeProbe '" + routeProbe + "' to use within the calibrator '" + id + "' is not known.");
        }
    }
    if (MSGlobals::gUseMesoSim) {
        if (lane != nullptr && edge->getLanes().size() > 1) {
            WRITE_WARNING("Meso calibrator '" + id
                          + "' defined for lane '" + lane->getID()
                          + "' will collect data for all lanes of edge '" + edge->getID() + "'.");
        }
        METriggeredCalibrator* trigger = buildMECalibrator(id, edge, pos, file, outfile, period, probe, invalidJamThreshold, vTypes);
        if (file == "") {
            trigger->registerParent(SUMO_TAG_CALIBRATOR, myHandler);
        }
    } else {
        MSCalibrator* trigger = buildCalibrator(id, edge, lane, node, pos, file, outfile, period, probe, invalidJamThreshold, vTypes, local);
        if (file == "") {
            trigger->registerParent(SUMO_TAG_CALIBRATOR, myHandler);
        }
    }
}


void
NLTriggerBuilder::parseAndBuildRerouter(MSNet& net, const SUMOSAXAttributes& attrs) {
    bool ok = true;
    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (!ok) {
        throw ProcessError();
    }
    if (MSTriggeredRerouter::getInstances().count(id) > 0) {
        throw InvalidArgument("Could not build rerouter '" + id + "'; probably declared twice.");
    }
    MSEdgeVector edges;
    for (const std::string& edgeID : attrs.get<std::vector<std::string> >(SUMO_ATTR_EDGES, id.c_str(), ok)) {
        MSEdge* edge = MSEdge::dictionary(edgeID);
        if (edge == nullptr) {
            throw InvalidArgument("The edge '" + edgeID + "' to use within rerouter '" + id + "' is not known.");
        }
        edges.push_back(edge);
    }
    if (!ok) {
        throw InvalidArgument("The edge to use within rerouter '" + id + "' is not known.");
    }
    if (edges.size() == 0) {
        throw InvalidArgument("No edges found for rerouter '" + id + "'.");
    }
    const double prob = attrs.getOpt<double>(SUMO_ATTR_PROB, id.c_str(), ok, 1);
    const bool off = attrs.getOpt<bool>(SUMO_ATTR_OFF, id.c_str(), ok, false);
    const bool optional = attrs.getOpt<bool>(SUMO_ATTR_OPTIONAL, id.c_str(), ok, false);
    const SUMOTime timeThreshold = TIME2STEPS(attrs.getOpt<double>(SUMO_ATTR_HALTING_TIME_THRESHOLD, id.c_str(), ok, 0));
    const std::string vTypes = attrs.getOpt<std::string>(SUMO_ATTR_VTYPES, id.c_str(), ok, "");
    const std::string pos = attrs.getOpt<std::string>(SUMO_ATTR_POSITION, id.c_str(), ok, "");
    const double radius = attrs.getOpt<double>(SUMO_ATTR_RADIUS, id.c_str(), ok, std::numeric_limits<double>::max());
    if (attrs.hasAttribute(SUMO_ATTR_RADIUS) && !attrs.hasAttribute(SUMO_ATTR_POSITION)) {
        WRITE_WARNINGF(TL("It is strongly advisable to give an explicit position when using radius in the definition of rerouter '%'."), id)
    }
    Position p = Position::INVALID;
    if (pos != "") {
        const std::vector<std::string> posSplit = StringTokenizer(pos, ",").getVector();
        if (posSplit.size() == 1) {
            double lanePos = StringUtils::toDouble(pos);
            if (lanePos < 0) {
                lanePos += edges.front()->getLanes()[0]->getLength();
            }
            p = edges.front()->getLanes()[0]->geometryPositionAtOffset(lanePos);
        } else if (posSplit.size() == 2) {
            p = Position(StringUtils::toDouble(posSplit[0]), StringUtils::toDouble(posSplit[1]));
        } else if (posSplit.size() == 3) {
            p = Position(StringUtils::toDouble(posSplit[0]), StringUtils::toDouble(posSplit[1]), StringUtils::toDouble(posSplit[2]));
        } else {
            throw InvalidArgument("Invalid position for rerouter '" + id + "'.");
        }
    }
    if (!ok) {
        throw InvalidArgument("Could not parse rerouter '" + id + "'.");
    }
    MSTriggeredRerouter* trigger = buildRerouter(net, id, edges, prob, off, optional, timeThreshold, vTypes, p, radius);
    // read in the trigger description
    trigger->registerParent(SUMO_TAG_REROUTER, myHandler);
}


// -------------------------


MSLaneSpeedTrigger*
NLTriggerBuilder::buildLaneSpeedTrigger(MSNet& /*net*/, const std::string& id,
                                        const std::vector<MSLane*>& destLanes,
                                        const std::string& file) {
    return new MSLaneSpeedTrigger(id, destLanes, file);
}


METriggeredCalibrator*
NLTriggerBuilder::buildMECalibrator(const std::string& id,
                                    MSEdge* edge,
                                    double pos,
                                    const std::string& file,
                                    const std::string& outfile,
                                    const SUMOTime freq,
                                    MSRouteProbe* probe,
                                    const double invalidJamThreshold,
                                    const std::string& vTypes) {
    return new METriggeredCalibrator(id, edge, pos, file, outfile, freq,
                                     edge == nullptr ? 0. : MSGlobals::gMesoNet->getSegmentForEdge(*edge, pos)->getLength(),
                                     probe, invalidJamThreshold, vTypes);
}


MSCalibrator*
NLTriggerBuilder::buildCalibrator(const std::string& id,
                                  MSEdge* edge,
                                  MSLane* lane,
                                  MSJunction* node,
                                  double pos,
                                  const std::string& file,
                                  const std::string& outfile,
                                  const SUMOTime freq,
                                  const MSRouteProbe* probe,
                                  const double invalidJamThreshold,
                                  const std::string& vTypes,
                                  const bool local) {
    return new MSCalibrator(id, edge, lane, node, pos, file, outfile, freq,
                            edge == nullptr ? 0. : edge->getLength(),
                            probe, invalidJamThreshold, vTypes, local, true);
}


MSTriggeredRerouter*
NLTriggerBuilder::buildRerouter(MSNet&, const std::string& id,
                                MSEdgeVector& edges, double prob, bool off, bool optional,
                                SUMOTime timeThreshold, const std::string& vTypes, const Position& pos, const double radius) {
    return new MSTriggeredRerouter(id, edges, prob, off, optional, timeThreshold, vTypes, pos, radius);
}


void
NLTriggerBuilder::buildStoppingPlace(MSNet& net, std::string id, std::vector<std::string> lines, MSLane* lane,
                                     double frompos, double topos, const SumoXMLTag element, std::string ptStopName,
                                     int personCapacity, double parkingLength, RGBColor& color, double angle) {
    myCurrentStop = new MSStoppingPlace(id, element, lines, *lane, frompos, topos, ptStopName, personCapacity, parkingLength, color, angle);
    if (!net.addStoppingPlace(element, myCurrentStop)) {
        delete myCurrentStop;
        myCurrentStop = nullptr;
        throw InvalidArgument("Could not build " + toString(element) + " '" + id + "'; probably declared twice.");
    }
}


void
NLTriggerBuilder::beginParkingArea(MSNet& net, const std::string& id,
                                   const std::vector<std::string>& lines,
                                   const std::vector<std::string>& badges,
                                   MSLane* lane, double frompos, double topos,
                                   unsigned int capacity,
                                   double width, double length, double angle, const std::string& name,
                                   bool onRoad,
                                   const std::string& departPos,
                                   bool lefthand) {
    // Close previous parking area if there are no lots inside
    MSParkingArea* stop = new MSParkingArea(id, lines, badges, *lane, frompos, topos, capacity, width, length, angle, name, onRoad, departPos, lefthand);
    if (!net.addStoppingPlace(SUMO_TAG_PARKING_AREA, stop)) {
        delete stop;
        throw InvalidArgument("Could not build parking area '" + id + "'; probably declared twice.");
    } else {
        myParkingArea = stop;
    }
}


void
NLTriggerBuilder::addLotEntry(double x, double y, double z,
                              double width, double length,
                              double angle, double slope) {
    if (myParkingArea != nullptr) {
        if (!myParkingArea->parkOnRoad()) {
            myParkingArea->addLotEntry(x, y, z, width, length, angle, slope);
            myParkingAreaCapacitySet = true;
        } else {
            throw InvalidArgument("Cannot not add lot entry to on-road parking area.");
        }
    } else {
        throw InvalidArgument("Could not add lot entry outside a parking area.");
    }
}


void
NLTriggerBuilder::endParkingArea() {
    if (myParkingArea != nullptr) {
        myParkingArea = nullptr;
        myParkingAreaCapacitySet = false;
    } else {
        throw InvalidArgument("Could not end a parking area that is not opened.");
    }
}


void
NLTriggerBuilder::endStoppingPlace() {
    if (myCurrentStop != nullptr) {
        myCurrentStop->finishedLoading();
        myCurrentStop = nullptr;
    } else {
        throw InvalidArgument("Could not end a stopping place that is not opened.");
    }
}


void
NLTriggerBuilder::buildChargingStation(MSNet& net, const std::string& id, MSLane* lane, double frompos, double topos,
                                       const std::string& name, double chargingPower, double efficiency, bool chargeInTransit,
                                       SUMOTime chargeDelay, std::string chargeType, SUMOTime waitingTime, MSParkingArea* parkingArea) {
    MSChargingStation* chargingStation = (parkingArea == nullptr) ? new MSChargingStation(id, *lane, frompos, topos, name, chargingPower, efficiency,
                                         chargeInTransit, chargeDelay, chargeType, waitingTime) : new MSChargingStation(id, parkingArea, name, chargingPower, efficiency,
                                                 chargeInTransit, chargeDelay, chargeType, waitingTime);
    if (!net.addStoppingPlace(SUMO_TAG_CHARGING_STATION, chargingStation)) {
        delete chargingStation;
        throw InvalidArgument("Could not build charging station '" + id + "'; probably declared twice.");
    }
    myCurrentStop = chargingStation;
}


MSOverheadWire*
NLTriggerBuilder::buildOverheadWireSegment(
    MSNet& net, 
    const std::string& id,
    const std::string& sectionID,
    const MSLane* lane,
    double frompos, 
    double topos,
    OverheadWireType& owt, 
    bool voltageSource) 
{
    // RICE_TODO: `MSOverheadWire` requires `MSLane&` as a paremter, i.e. a reference to a mutable 
    // lane object. We are working with const, unumtable lane obects. Hence the dangerous const_cast<MSLane*>
    // that is used here.
    MSOverheadWire* overheadWireSegment = new MSOverheadWire(id, sectionID, *const_cast<MSLane*>(lane), frompos, topos, owt, voltageSource);
    if (!net.addStoppingPlace(SUMO_TAG_OVERHEAD_WIRE_SEGMENT, overheadWireSegment)) {
        delete overheadWireSegment;
        throw InvalidArgument(fmt::format("Could not build overheadWireSegment '{}'; probably declared twice.", id));
    }
    return overheadWireSegment;
}

void
NLTriggerBuilder::buildTractionSubstation(MSNet& net, std::string id, double voltage, double currentLimit) {
    MSTractionSubstation* myTractionSubstation = new MSTractionSubstation(id, voltage, currentLimit);
    if (!net.addTractionSubstation(myTractionSubstation)) {
        delete myTractionSubstation;
        throw InvalidArgument("Could not build traction substation '" + id + "'; probably declared twice.");
    }
}

void
NLTriggerBuilder::buildOverheadWireClamp(MSNet& /*net*/, const std::string& /*id*/, MSLane* /*lane_start*/, MSLane* /*lane_end*/) {
}

std::string
NLTriggerBuilder::getFileName(const SUMOSAXAttributes& attrs,
                              const std::string& base,
                              const bool allowEmpty) {
    // get the file name to read further definitions from
    bool ok = true;
    std::string file = attrs.getOpt<std::string>(SUMO_ATTR_FILE, nullptr, ok, "");
    if (file == "") {
        if (allowEmpty) {
            return file;
        }
        throw InvalidArgument("No filename given.");
    }
    // check whether absolute or relative filenames are given
    if (!FileHelpers::isAbsolute(file)) {
        return FileHelpers::getConfigurationRelative(base, file);
    }
    return file;
}


MSLane*
NLTriggerBuilder::getLane(const SUMOSAXAttributes& attrs,
                          const std::string& tt,
                          const std::string& tid) {
    bool ok = true;
    std::string objectid = attrs.get<std::string>(SUMO_ATTR_LANE, tid.c_str(), ok);
    MSLane* lane = MSLane::dictionary(objectid);
    if (lane == nullptr) {
        // Either a lane that is non-existent/broken, or a lane that is internal and has been ignored.
        // We assume that internal lane names start with ':'.
        if (objectid[0] == ':' && !MSGlobals::gUsingInternalLanes) {
            return nullptr;
        }
        // Throw the exception only in case that the lane really does not exist in the network file
        // or it is broken.
        throw InvalidArgument("The lane " + objectid + " to use within the " + tt + " '" + tid + "' is not known.");
    }
    return lane;
}


MSParkingArea*
NLTriggerBuilder::getParkingArea(const SUMOSAXAttributes& attrs, const std::string& tt, const std::string& tid) {
    bool ok = true;
    std::string objectID = attrs.getOpt<std::string>(SUMO_ATTR_PARKING_AREA, tid.c_str(), ok);
    if (!ok || objectID.size() == 0) {
        return nullptr;
    }
    MSParkingArea* pa = static_cast<MSParkingArea*>(MSNet::getInstance()->getStoppingPlace(objectID, SUMO_TAG_PARKING_AREA));
    if (pa == nullptr) {
        // Throw the exception only in case that the lane really does not exist in the network file
        // or it is broken.
        throw InvalidArgument("The parkingArea " + objectID + " to use within the " + tt + " '" + tid + "' is not known.");
    }
    return pa;
}


double
NLTriggerBuilder::getPosition(const SUMOSAXAttributes& attrs,
                              MSLane* lane,
                              const std::string& tt, const std::string& tid,
                              MSEdge* edge) {
    assert(lane != nullptr || edge != nullptr);
    const double length = lane != nullptr ? lane->getLength() : edge->getLength();
    bool ok = true;
    double pos = attrs.get<double>(SUMO_ATTR_POSITION, nullptr, ok);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, nullptr, ok, false);
    if (!ok) {
        throw InvalidArgument("Error on parsing a position information.");
    }
    if (pos < 0) {
        pos = length + pos;
    }
    if (pos > length) {
        if (friendlyPos) {
            pos = length - (double) 0.1;
        } else {
            if (lane != nullptr) {
                throw InvalidArgument("The position of " + tt + " '" + tid + "' lies beyond the lane's '" + lane->getID() + "' length.");
            } else {
                throw InvalidArgument("The position of " + tt + " '" + tid + "' lies beyond the edge's '" + edge->getID() + "' length.");
            }
        }
    }
    return pos;
}


void
NLTriggerBuilder::updateParkingAreaDefaultCapacity() {
    if (myParkingArea != nullptr && !myParkingAreaCapacitySet) {
        myParkingArea->setRoadsideCapacity(1);
    }
}


MSStoppingPlace*
NLTriggerBuilder::getCurrentStop() {
    return myParkingArea == nullptr ? myCurrentStop : myParkingArea;
}


/****************************************************************************/
