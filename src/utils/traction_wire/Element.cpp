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
/// @file    Element.cpp
/// @author  Jakub Sevcik (RICE)
/// @author  Jan Prikryl (RICE)
/// @date    2019-12-15
///
/// @note    based on work 2017 Ahmad Khaled, Ahmad Essam, Omnia Zakaria, Mary Nader
///
// Representation of electric circuit elements: resistors, voltage and current sources
/****************************************************************************/
#include <cfloat>
#include <cmath>
#include <utils/common/MsgHandler.h>
#include "Element.h"
#include "Node.h"


// ===========================================================================
// method definitions
// ===========================================================================
Element::Element(
    std::string name, 
    ElementType type, 
    double value
) :
    id(-2),  // RICE_TODO: Why -2?
    name(name),
    type(type),
    isenabled(true),
    powerWanted(NAN),
    pNode(nullptr),
    nNode(nullptr)
{
    resistance = 0.0;
    current = 0.0;
    voltage = 0.0;
    switch (type) {
        case CURRENT_SOURCE_traction_wire:
            current = value;
            break;
        case VOLTAGE_SOURCE_traction_wire:
            voltage = value;
            break;
        case RESISTOR_traction_wire:
            resistance = value;
            break;
        default:
            WRITE_ERRORF(TL("Undefined element type for '%'."), name);
            break;
    }
}

void Element::setVoltage(double voltageIn) {
    voltage = voltageIn;
}
void Element::setCurrent(double currentIn) {
    current = currentIn;
}
void Element::setResistance(double resistanceIn) {
    if (resistanceIn <= 1e-6) {
        resistance = 1e-6;
    } else {
        resistance = resistanceIn;
    }
}
void Element::setPowerWanted(double powerWantedIn) {
    powerWanted = powerWantedIn;
}
double Element::getVoltage() const {
    if (!isenabled) {
        return DBL_MAX;
    }
    if (getType() == Element::ElementType::VOLTAGE_SOURCE_traction_wire) {
        return voltage;
    }
    return pNode->getVoltage() - nNode->getVoltage();
}
double Element::getCurrent() const {
    if (!isenabled) {
        return DBL_MAX;
    }
    switch (type) {
        case Element::ElementType::RESISTOR_traction_wire:
            return -1 * getVoltage() / resistance;
        case Element::ElementType::CURRENT_SOURCE_traction_wire:
        case Element::ElementType::VOLTAGE_SOURCE_traction_wire:
            return current;
        default:
            return 0;
    }
}
double Element::getResistance() const {
    return resistance;
}
double Element::getPowerWanted() const {
    return 	powerWanted;
}
double Element::getPower() const {
    return 	-1 * getCurrent() * getVoltage();
}
int Element::getId() const {
    return id;
}
Node* Element::getPosNode() {
    return pNode;
}
Node* Element::getNegNode() {
    return nNode;
}

Element::ElementType Element::getType() const {
    return type;
}
std::string Element::getName() const {
    return name;
}

void Element::setPosNode(Node* node) {
    pNode = node;

}
void Element::setNegNode(Node* node) {
    nNode = node;
}
void Element::setId(int newId) {
    id = newId;
}

// if node == pNode, return nNode, else if node == nNode return pNode, else return nullptr
Node* Element::getTheOtherNode(Node* node) {
    if (node == pNode) {
        return nNode;
    } else if (node == nNode) {
        return pNode;
    } else {
        return nullptr;
    }
}

bool Element::isEnabled() {
    return isenabled;
}

void Element::setEnabled(bool newIsEnabled) {
    isenabled = newIsEnabled;
}

void Element::setType(ElementType ET) {
    type = ET;
}
