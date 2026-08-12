/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2002-2026 German Aerospace Center (DLR) and others.
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
/// @file    MSDevice_ElecHybrid.h
/// @author  Jakub Sevcik (RICE)
/// @author  Jan Prikryl (RICE)
/// @date    2019-12-15
///
// A device which stands as an implementation ElecHybrid and which outputs movereminder calls
/****************************************************************************/
#pragma once
#include <config.h>

#include <microsim/devices/MSVehicleDevice.h>
#include <microsim/MSVehicle.h>
#include <microsim/trigger/MSOverheadWire.h>
#include <utils/common/SUMOTime.h>
#include <utils/emissions/EnergyParams.h>

// ===========================================================================
// class declarations
// ===========================================================================
class SUMOVehicle;
class MSDevice_Emissions;
class MSPowerManagement;


// ===========================================================================
// class definitions
// ===========================================================================
/**
* @class MSDevice_ElecHybrid
* @brief A device which collects info on the vehicle trip (mainly on departure and arrival)
*
* Each device collects departure time, lane and speed and the same for arrival.
*
* @see MSDevice
*/
class MSDevice_ElecHybrid : public MSVehicleDevice {
public:
    /** @brief Inserts MSDevice_ElecHybrid-options
    * @param[filled] oc The options container to add the options to
    */
    static void insertOptions(OptionsCont& oc);


    /** @brief Build devices for the given vehicle, if needed
    *
    * The options are read and evaluated whether a ElecHybrid-device shall be built
    *  for the given vehicle.
    *
    * The built device is stored in the given vector.
    *
    * @param[in] v The vehicle for which a device may be built
    * @param[filled] into The vector to store the built device in
    */
    static void buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into);

    /// @brief Destructor.
    ~MSDevice_ElecHybrid();



    /// @name Methods called on vehicle movement / state change, overwriting MSDevice
    /// @{

    /** @brief Checks for waiting steps when the vehicle moves
    *
    * @param[in] veh Vehicle that asks this reminder.
    * @param[in] oldPos Position before move.
    * @param[in] newPos Position after move with newSpeed.
    * @param[in] newSpeed Moving speed.
    *
    * @return True (always).
    */
    bool notifyMove(SUMOTrafficObject& tObject, double oldPos, double newPos, double newSpeed);

    /** @brief Saves departure info on insertion
    *
    * @param[in] veh The entering vehicle.
    * @param[in] reason how the vehicle enters the lane
    * @return Always true
    * @see MSMoveReminder::notifyEnter
    * @see MSMoveReminder::Notification
    */
    bool notifyEnter(SUMOTrafficObject& tObject, MSMoveReminder::Notification reason, const MSLane* enteredLane = 0);


    /** @brief Saves arrival info
    *
    * @param[in] veh The leaving vehicle.
    * @param[in] lastPos Position on the lane when leaving.
    * @param[in] isArrival whether the vehicle arrived at its destination
    * @param[in] isLaneChange whether the vehicle changed from the lane
    * @return True if it did not leave the net.
    */
    bool notifyLeave(SUMOTrafficObject& tObject, double lastPos, MSMoveReminder::Notification reason, const MSLane* enteredLane = 0);

    /** @brief Internal notification about the vehicle moves
     *  @see MSMoveReminder::notifyMoveInternal()
     */
    virtual void notifyMoveInternal(
        const SUMOTrafficObject& tObject,
        const double frontOnLane,
        const double timeOnLane,
        const double meanSpeedFrontOnLane,
        const double meanSpeedVehicleOnLane,
        const double travelledDistanceFrontOnLane,
        const double travelledDistanceVehicleOnLane,
        const double meanLengthOnLane);
    /// @}

    /// @brief return the name for this type of device
    const std::string deviceName() const {
        return "elecHybrid";
    }

    /// @brief try to retrieve the given parameter from this device. Throw exception for unsupported key
    std::string getParameter(const std::string& key) const;

    double getParameterDouble(const std::string& key) const;

    /// @brief try to set the given parameter for this device. Throw exception for unsupported key
    void setParameter(const std::string& key, const std::string& value);

    /// @brief try to set the given double (floating point) parameter for this device. Throw exception for unsupported key
    void setParameterDouble(const std::string& key, const double val);

    /** @brief Called on writing tripinfo output
     *
     * @param[in] tripinfoOut The output device to write the information into
     * @exception IOError not yet implemented
     * @see MSDevice::tripInfoOutput
     */
    void generateOutput(OutputDevice* tripinfoOut) const;

    /// @brief Get the actual vehicle's Battery Capacity in kWh
    double getActualBatteryCapacity() const;

    /// @brief Get the total vehicle's Battery Capacity in kWh
    double getMaximumBatteryCapacity() const;

    /// @brief Get actual overhead wire segment ID
    std::string getOverheadWireSegmentID() const;

    /// @brief Get actual traction substation ID
    std::string getTractionSubstationID() const;

    // Return pointer to MSPowerManagement
    MSPowerManagement* getPowerManagement() const { return myPowerManagement; };

    /// @brief Get charged energy
    double getEnergyCharged() const;

    void setEnergyCharged(double energyCharged);

    double getCircuitAlpha() const;

    double getPowerWanted() const;

    /// @brief Get actual current in the overhead wire segment
    double getCurrentFromOverheadWire() const;

    /// @brief Get actual value of electric current limit from the overhead line for vehicle that has stopped
    double getMaxLineCurrentStopped() const;

    void setCurrentFromOverheadWire(double current);

    /// @brief Get actual voltage on the overhead wire segment
    double getVoltageOfOverheadWire() const;

    void setVoltageOfOverheadWire(double voltage);

    /// @brief Return the last simulation step when notifyMove refreshed the device state.
    SUMOTime getLastNotifyMoveStep() const {
        return myLastNotifyMoveStep;
    }

    /// @brief Get consum
    double getConsum() const;

    double getDistance() const {
        return myDistance;
    }

    /// @brief Get consum
    bool isBatteryDischarged() const;

    /// @brief Set actual vehicle's Battery Capacity in kWh
    void setActualBatteryCapacity(const double actualBatteryCapacity);

    /// @brief Attempt to store energy into battery pack and return the energy that could not be accommodated due to SOC limits
    double storeEnergyToBattery(const double energy);

    /// @brief Add energyWasted to the total sum myTotalEnergyWasted
    void updateTotalEnergyWasted(const double energyWasted);

    /// @brief Update tripInfo's statistics myMaxBatteryCharge and myMinBatteryCharge according to actual state of the charge: myActualBatteryCapacity
    void updateMinMaxBatteryCharge();

    void setConsum(const double consumption);

    double acceleration(SUMOVehicle& veh, double power, double oldSpeed);

    /// @brief return energy consumption in Wh (power multiplied by TS)
    double consumption(SUMOVehicle& veh, double a, double newSpeed);

    /// @brief compute charged energy properly considering recuperation and propulsion efficiency during charging battery from overhead wire or discharging battery to recuperate into overhead wire
    double computeChargedEnergy(double energyIn);

    MSOverheadWire* getActOverheadWireSegment() {
        return myActOverheadWireSegment;
    };

    Element* getVehElem() {
        return veh_elem;
    }

    MSPowerManagement* getPowerManagement() {
        return myPowerManagement;
    };
private:
    /** @brief Constructor
    *
    * @param[in] holder The vehicle that holds this device
    * @param[in] id The ID of the device
    */
    MSDevice_ElecHybrid(SUMOVehicle& holder, const std::string& id,
                        const double actualBatteryCapacity, const double maximumBatteryCapacity);

protected:
    /// @brief Parameter, The actual vehicles's Battery Capacity in Wh, [myActualBatteryCapacity <= myMaximumBatteryCapacity]
    double myActualBatteryCapacity;

    /// @brief Parameter, The total vehicles's Battery Capacity in Wh, [myMaximumBatteryCapacity >= 0]
    double myMaximumBatteryCapacity;

    /// @brief Parameter holding emission device
    MSDevice_Emissions* myEmissionDevice;

    /// @brief Parameter, Vehicle's last angle
    double myLastAngle;

    /// @brief Parameter, Vehicle consumption during a time step (by default is 0.)
    double myConsum;

    /// @brief Parameter, Flag: Battery of Vehicle is fully discharged (by default is false)
    bool myBatteryDischargedLogic;

    /// @brief Parameter, Flag: Vehicle is charging (by default is false)
    bool myCharging;

    /// @brief Energy flowing into (+) or from (-) the battery pack in the given timestep
    double myEnergyCharged;

    /// @brief Parameter, Current wanted at overhead wire in next timestep
    double myCircuitCurrent;

    double myCircuitVoltage;

    /// @name Tripinfo statistics
    /// @{
    double myMaxBatteryCharge;
    double myMinBatteryCharge;
    double myTotalEnergyConsumed;
    double myTotalEnergyRegenerated;

    /// @brief Energy that could not be stored back to the battery or traction station
    /// and was wasted on resistors. This is approximate, we ignore the use of classical
    /// brakes in lower speeds.
    double myTotalEnergyWasted;
    /// @}

    MSPowerManagement* myPowerManagement;

    /// @brief Parameter, Pointer to the actual overhead wire segment in which vehicle is placed (by default is nullptr)
    MSOverheadWire* myActOverheadWireSegment;

    /// @brief Parameter, Pointer to the act overhead wire segment in previous step  (by default is nullptr), i.e. auxiliar pointer for disabling charging vehicle from previous (not current) overherad wire segment
    MSOverheadWire* myPreviousOverheadWireSegment;

    double myDistance;

    /// @brief Last simulation step when notifyMove updated the electrical state.
    SUMOTime myLastNotifyMoveStep;

    //circuit element of elecHybrid device
    //  ----|veh_pos_tail_elem|---------|pos_veh_node|--------
    //										  |
    //										  |
    //									  |veh_elem|
    //										  |
    //										  |
    //  ----------------------------------|ground|------------
    Element* veh_elem;
    Element* veh_pos_tail_elem;
    Node* pos_veh_node;

    void deleteVehicleFromCircuit(SUMOVehicle& veh);


private:
    /// @brief Invalidated copy constructor.
    MSDevice_ElecHybrid(const MSDevice_ElecHybrid&);

    /// @brief Invalidated assignment operator.
    MSDevice_ElecHybrid& operator=(const MSDevice_ElecHybrid&);


};

// ===========================================================================
// class definitions
// ===========================================================================
/**
* @class MSPowerManagemnt
* @brief A class that define power management for elechybrid device
*
* TODO: 
*    -descripion
*    -should the powermanagement class inherit some base class as Named for exmaple? 
*    -what is a proper name of the class, why MS?
*/
class MSPowerManagement {
private:
    /// @brief Maximal SOC of the battery pack, battery will not be charged above this level.
    double reducedSOC_ub;
    /// @brief Minimal SOC of the battery pack, below this value the battery is assumed discharged
    double reducedSOC_lb;
    double maxLineCurrent_driving; // 400 A
    /// @brief Maximum current that can be drawn from the overhead line when stopped
    double maxLineCurrent_stopped; // 80 A
    double recupBatteryPLimit; // 150 KW
    double maxBatteryChargingPower_driving; // 55 kW
    double maxBatteryChargingPower_stopped; // 45 kW
    bool   eco_mode;
    double eco_maxBatteryChargingPower_driving; // 25 kW
    double eco_maxBatteryChargingPower_stopped; // 25 kW
    double eco_socLimitCharging; // 0.9
    double eco_socThresholdForPeakShaving; // 40 %
    double eco_socHysteresisForPeakShaving; // 50 %
    mutable bool eco_peakShavingEnabled;
    double eco_minCurrentForPeakShaving; // 250 A
    double SUMO_ATTR_INPUTCHOKEEFFICIENCY;
    double SUMO_ATTR_CHARGINEFFICIENCY;

    // OLD
    double myMaximumBatteryCapacity;

public:
    MSPowerManagement(SUMOVehicle& v);  // Constructor

    /*
    TODO: These are not defined anywhere, why?
    
    double calculateBatteryRequest(double requiredPower);
    double calculateEngineRequest(double requiredPower);
    */
    
    std::pair<double, double> computePowerDemand(double consum, double soc, double speed, double voltage, bool hasOvrHdWire, bool hasBattery) const;
    void distributePower(double powerFromOverheadWire, bool hasOvrHdWire, bool charging, MSDevice_ElecHybrid* elecHybrid);

	//@brief Set the maximum current drawn from the overhead line when the vehicle is stopped
    void setMaxLineCurrentStopped(double current) {
        maxLineCurrent_stopped = current;
	};

    //@brief Get the maximum current drawn from the overhead line when the vehicle is stopped
    double getMaxLineCurrentStopped() const {
        return maxLineCurrent_stopped;
    };

    //@brief Set if the power managemnt eco mode is activated or not
    void setEcoMode(bool activated) {
        eco_mode = activated;
    };

    //@brief Get info if the power managemnt eco mode is activated or not
    bool getEcoMode() const {
        return eco_mode;
    };

    //@brief Get the input choke efficiency for drawing current from overhead wire to trolleybus or vice versa
    double getInputChokeEff() const {
        return SUMO_ATTR_INPUTCHOKEEFFICIENCY;
    };

    //@brief Get the battery charging efficiency for charging power from vehicle's inner circuit to battery pack or vice versa
    double getBatCharEff() const {
        return SUMO_ATTR_CHARGINEFFICIENCY;
    };

    //@brief Get the limit for the maximal SOC of the battery pack
    double getMaxSOCLim() const {
        return reducedSOC_ub;
    };

    //@brief Get the limit for the minimal SOC of the battery pack
    double getMinSOCLim() const {
        return reducedSOC_lb;
    };
};
