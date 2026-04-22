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
/// @file    Node.cpp
/// @author  Jakub Sevcik (RICE)
/// @author  Jan Prikryl (RICE)
/// @date    2019-12-15
///
/// @note    based on work 2017 Ahmad Khaled, Ahmad Essam, Omnia Zakaria, Mary Nader
///
// Representation of electric circuit nodes, i.e. wire junctions and connection points.
/****************************************************************************/
#include <config.h>

#include <string>
#include <algorithm>
#include <utils/common/MsgHandler.h>
#include "Node.h"
#include "Element.h"


// A constructor, same functionality as "init" functions
Node::Node(std::string name, int id) : 
    isground(false), 
    isremovable(false),
    name(name), // unique property, each object in circuit should have distinctive and unique name
    id(id), // a sequential ID number, might be useful when making the equation
    num_matrixRow(-1), 
    num_matrixCol(-1),
    voltage(0),
    elements({})
{
    // Just initialization of class variables done in initializer list
}

// connects an element to the node
void Node::addElement(Element* element) {
    elements.push_back(element);
}

void Node::eraseElement(Element* element) {
    elements.erase(std::remove(elements.begin(), elements.end(), element), elements.end());
}

// getters and setters
double Node::getVoltage() {
    return voltage;
}

void Node::setVoltage(double volt) {
    voltage = volt;
}

int Node::getNumOfElements() {
    return (int) elements.size();
}

std::string& Node::getName() {
    return name;
}

bool Node::isGround() {
    return isground;
}

void Node::setGround(bool newIsGround) {
    isground = newIsGround;
}

int Node::getId() {
    return id;
}

void Node::setId(int newId) {
    id = newId;
}

void Node::setNumMatrixRow(int num) {
    num_matrixRow = num;
}

int Node::getNumMatrixRow() {
    return num_matrixRow;
}

void Node::setNumMatrixCol(int num) {
    num_matrixCol = num;
}

int Node::getNumMatrixCol() {
    return num_matrixCol;
}

std::vector<Element*> Node::getElements() {
    return elements;
}

void Node::setRemovability(bool newIsRemovable) {
    isremovable = newIsRemovable;
}

Element* 
Node::getAnOtherElement(Element* element)
{
    // RICE_TODO: This returns the first element that is not `element`, is this intended?
    // What if there are multiple elements connected to the node?
    if (elements.size() >= 3) {
        WRITE_WARNINGF(TL("Node '%' has more than two connected elements, getAnOtherElement() may not return expected result."), name);
    }

    for (auto el : elements) {
        if (el != element) {
            return el;
        }
    }
    return nullptr;
}
