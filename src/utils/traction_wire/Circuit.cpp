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
/// @file    Circuit.cpp
/// @author  Jakub Sevcik (RICE)
/// @author  Jan Prikryl (RICE)
/// @date    2019-12-15
///
/// @note    based on console-based C++ DC circuits simulator,
///          https://github.com/rka97/Circuits-Solver by
///          Ahmad Khaled, Ahmad Essam, Omnia Zakaria, Mary Nader
///          and available under MIT license, see https://github.com/rka97/Circuits-Solver/blob/master/LICENSE
///
// Representation of electric circuit of overhead wires
/****************************************************************************/
#include <config.h>

#include <cfloat>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <mutex>
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include <microsim/MSGlobals.h>
#include "Element.h"
#include "Node.h"
#include "Circuit.h"

static const double CIRCUIT_MINIMAL_RESISTANCE = 1e-6;

static std::mutex circuit_lock;

Node* Circuit::addNode(std::string name)
{
    // Do not insert duplicate nodes
    if (nodeNameMap.find(name) != nodeNameMap.end()) {
        WRITE_ERRORF(TL("The node: '%' already exists."), name);
        return nullptr;
    }

    // Make sure to lock the circuit while modifying it
    circuit_lock.lock();

    // Create the node as unique_ptr to manage memory automatically
    // The unique_ptr assures that the node pointer is exlusively owned by the circuit
    // and will be automatically deleted when the circuit is destroyed or the node is erased.
    auto node = std::make_unique<Node>(name, lastId);
    Node* nodePtr = node.get();

    // If this is the first node, set it as ground
    if (lastId == -1) {
        nodePtr->setGround(true);
    }

    // Add the node to the list of circuit nodes and to the name map
    // Move the unique_ptr into the vector to transfer ownership
    nodes.push_back(std::move(node));
    nodeNameMap[name] = nodePtr;
    nodeIdMap[lastId] = nodePtr;

    // Increment the last used ID
    lastId++;

    // Unlock the circuit for other operations
    circuit_lock.unlock();  

    return nodePtr;
}

void Circuit::eraseNode(Node* node)
{
    // Do not erase node if it is referenced by any element
    if (!node->getElements().empty()) {
        // Create list of element names for elements that reference the node
        std::string elementNames;
        for (auto& el : node->getElements()) {
            if (!elementNames.empty()) {
                elementNames += ", ";
            }
            elementNames += el->getName();
        }
        // Report error and return
        WRITE_ERRORF(TL("The node: '%' cannot be erased because it is still referenced by the following elements: %."), node->getName(), elementNames);
        return;
    }   

    // Lock the access to circuit nodes and elements
    circuit_lock.lock();
    // Remove node from name map
    nodeNameMap.erase(node->getName());
    // Remove node from id map
    nodeIdMap.erase(node->getId());
    // Remove node from storage vector (note will be automatically deleted via unique_ptr)
    nodes.erase(
        std::remove_if(nodes.begin(), nodes.end(), [node](const auto& n) { return n.get() == node; }),
        nodes.end()
    );
    // Unlock the circuit again
    circuit_lock.unlock();
}

double Circuit::getCurrent(std::string name) {
    Element* tElement = getElement(name);
    if (tElement == nullptr) {
        return DBL_MAX;
    }
    return tElement->getCurrent();
}

double Circuit::getVoltage(std::string name) {
    Element* tElement = getElement(name);
    if (tElement == nullptr) {
        Node* node = getNode(name);
        if (node != nullptr) {
            return node->getVoltage();
        } else {
            return DBL_MAX;
        }
    } else {
        return tElement->getVoltage();
    }
}

double Circuit::getResistance(std::string name) {
    Element* tElement = getElement(name);
    if (tElement == nullptr) {
        return -1;
    }
    return tElement->getResistance();
}

Node* Circuit::getNode(std::string name) {
    // Use the `nodeNameMap` for faster lookup
    auto it = nodeNameMap.find(name);
    // Return the found node or `nullptr` if not found
    return (it != nodeNameMap.end()) ? it->second : nullptr;
}

Node* Circuit::getNode(int id) {
    // Use the `nodeIdMap` for faster lookup
    auto it = nodeIdMap.find(id);
    // Return the found node or `nullptr` if not found
    return (it != nodeIdMap.end()) ? it->second : nullptr;
}

Element* Circuit::getElement(std::string name) {
    // Use the `elementNameMap` for faster lookup
    // The `elementNameMap` contains both regular elements and voltage sources
    auto it = elementNameMap.find(name);
    // Return the found element or `nullptr` if not found
    return (it != elementNameMap.end()) ? it->second : nullptr;
}

Element* Circuit::getElement(int id) {
    // Use the `elementIdMap` for faster lookup
    // The `elementIdMap` contains both regular elements and voltage sources
    auto it = elementIdMap.find(id);
    // Return the found element or `nullptr` if not found
    return (it != elementIdMap.end()) ? it->second : nullptr;
}

Element* Circuit::getVoltageSource(int id) {
    for (const auto& voltageSource : voltageSources) {
        if (voltageSource->getId() == id) {
            return voltageSource.get();
        }
    }
    return nullptr;
}

double Circuit::getTotalPowerOfCircuitSources() {
    double power = 0;
    for (const auto& voltageSource : voltageSources) {
        power += voltageSource->getPower();
    }
    return power;
}

double Circuit::getTotalCurrentOfCircuitSources() {
    double current = 0;
    for (const auto& voltageSource : voltageSources) {
        current += voltageSource->getCurrent();
    }
    return current;
}

// RICE_CHECK: Locking removed?
std::string& Circuit::getCurrentsOfCircuitSource(std::string& currents) {
    //
    // circuit_lock.lock();
    //
    currents.clear();
    for (const auto& voltageSource : voltageSources) {
        // Separate existing elements by space
        if (!currents.empty()) {
            currents += " ";
        }
        // Append current of the voltage source with 4 decimal places
        currents += toString(voltageSource->getCurrent(), 4);
    }
    //
    // circuit_lock.unlock();
    //
    return currents;
}

std::vector<Element*> 
Circuit::getCurrentSources() const {
    std::vector<Element*> vsources;
    for (const auto& element : elements) {
        if (element->getType() == Element::ElementType::CURRENT_SOURCE_traction_wire) {
            // RICE_TODO: Check if this is needed
            // if ((*it)->getType() == Element::ElementType::CURRENT_SOURCE_traction_wire && !isnan((*it)->getPowerWanted())) {
            //
            vsources.push_back(element.get());
        }
    }
    return vsources;
}

void Circuit::lock() {
    circuit_lock.lock();
}

void Circuit::unlock() {
    circuit_lock.unlock();
}

#ifdef HAVE_EIGEN
void Circuit::removeColumn(Eigen::MatrixXd& matrix, int colToRemove) {
    const int numRows = (int)matrix.rows();
    const int numCols = (int)matrix.cols() - 1;

    if (colToRemove < numCols) {
        matrix.block(0, colToRemove, numRows, numCols - colToRemove) = matrix.rightCols(numCols - colToRemove);
    }

    matrix.conservativeResize(numRows, numCols);
}

bool 
Circuit::solveEquationsNRmethod(
    double* eqn, 
    double* vals, 
    std::vector<int>* removable_ids
) {
    // Vector `removable_ids` includes nodes with voltage source already
    int numofcolumn = (int)voltageSources.size() + (int)nodes.size() - 1;
    int numofeqs = numofcolumn - (int)removable_ids->size();

    // map equations into matrix A
    Eigen::MatrixXd A = Eigen::Map < Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(eqn, numofeqs, numofcolumn);

    int id;
    // remove removable columns of matrix A, i.e. remove equations corresponding to nodes with two resistors connected in series
    // RICE_TODO auto for ?
    for (std::vector<int>::reverse_iterator it = removable_ids->rbegin(); it != removable_ids->rend(); ++it) {
        id = (*it >= 0 ? *it : -(*it));
        removeColumn(A, id);
    }

    // detect number of column for each node
    // in other words: detect elements of x to certain node
    // in other words: assign number of column to the proper non removable node
    int j = 0;
    Element* tElem = nullptr;
    Node* tNode = nullptr;
    for (int i = 0; i < numofcolumn; i++) {
        tNode = getNode(i);
        if (tNode != nullptr) {
            if (tNode->isRemovable()) {
                tNode->setNumMatrixCol(-1);
                continue;
            } else {
                if (j > numofeqs) {
                    WRITE_ERROR(TL("Index of renumbered node exceeded the reduced number of equations."));
                    break;
                }
                tNode->setNumMatrixCol(j);
                j++;
                continue;
            }
        } else {
            tElem = getElement(i);
            if (tElem != nullptr) {
                if (j > numofeqs) {
                    WRITE_ERROR(TL("Index of renumbered element exceeded the reduced number of equations."));
                    break;
                }
                continue;
            }
        }
        // tNode == nullptr && tElem == nullptr
        WRITE_ERROR(TL("Structural error in reduced circuit matrix."));
    }

    // map 'vals' into vector b and initialize solution x
    Eigen::Map<Eigen::VectorXd> b(vals, numofeqs);
    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

    // initialize Jacobian matrix J and vector dx
    Eigen::MatrixXd J = A;
    Eigen::VectorXd dx;
    // initialize progressively increasing maximal number of Newton-Rhapson iterations
    int max_iter_of_NR = 10;
    // value of scaling parameter alpha
    double alpha = 1;
    // the best (maximum) value of alpha that guarantees the existence of solution
    alphaBest = 0;
    // reason why is alpha not 1
    alphaReason = ALPHA_NOT_APPLIED;
    // vector of alphas for that no solution has been found
    std::vector<double> alpha_notSolution;
    // initialize progressively decreasing tolerance for alpha
    double alpha_res = 1e-2;

    double currentSumActual = 0.0;
    // solution x corresponding to the alphaBest
    Eigen::VectorXd x_best = x;
    bool x_best_exist = true;

    if (x.maxCoeff() > 10e6 || x.minCoeff() < -10e6) {
        WRITE_ERROR(TL("Initial solution x used during solving DC circuit is out of bounds.\n"));
    }

    // Search for the suitable scaling value alpha
    while (true) {

        int iterNR = 0;
        // run Newton-Raphson methods
        while (true) {

            // update right-hand side vector vals and Jacobian matrix J
            // node's right-hand side set to zero
            for (int i = 0; i < numofeqs - (int) voltageSources.size(); i++) {
                vals[i] = 0;
            }
            J = A;

            int i = 0;
            for (auto& node : nodes) {
                if (node->isGround() || node->isRemovable() || node->getNumMatrixRow() == -2) {
                    continue;
                }
                if (node->getNumMatrixRow() != i) {
                    WRITE_ERROR(TL("wrongly assigned row of matrix A during solving the circuit"));
                }
                // TODO: Range-based loop
                // loop over all node's elements
                for (auto& element : node->getElements()) {
                    if (element->getType() == Element::ElementType::CURRENT_SOURCE_traction_wire) {
                        // if the element is current source
                        if (element->isEnabled()) {
                            double diff_voltage;
                            int PosNode_NumACol = element->getPosNode()->getNumMatrixCol();
                            int NegNode_NumACol = element->getNegNode()->getNumMatrixCol();
                            // compute voltage on current source
                            if (PosNode_NumACol == -1) {
                                // if the positive node is the ground => U = 0 - phi(NegNode)
                                diff_voltage = -x[NegNode_NumACol];
                            } else if (NegNode_NumACol == -1) {
                                // if the negative node is the ground => U = phi(PosNode) - 0
                                diff_voltage = x[PosNode_NumACol];
                            } else {
                                // U = phi(PosNode) - phi(NegNode)
                                diff_voltage = (x[PosNode_NumACol] - x[NegNode_NumACol]);
                            }

                            if (element->getPosNode() == node.get()) {
                                // the positive current (the element is consuming energy if powerWanted > 0) is flowing from the positive node (sign minus)
                                vals[i] -= alpha * element->getPowerWanted() / diff_voltage;
                                element->setCurrent(-alpha * element->getPowerWanted() / diff_voltage);
                                if (PosNode_NumACol != -1) {
                                    // -1* d_b/d_phiPos = -1* d(-alpha*P/(phiPos-phiNeg) )/d_phiPos = -1* (--alpha*P/(phiPos-phiNeg)^2 )
                                    J(i, PosNode_NumACol) -= alpha * element->getPowerWanted() / diff_voltage / diff_voltage;
                                }
                                if (NegNode_NumACol != -1) {
                                    // -1* d_b/d_phiNeg = -1* d(-alpha*P/(phiPos-phiNeg) )/d_phiNeg = -1* (---alpha*P/(phiPos-phiNeg)^2 )
                                    J(i, NegNode_NumACol) += alpha * element->getPowerWanted() / diff_voltage / diff_voltage;
                                }
                            } else {
                                // the positive current (the element is consuming energy if powerWanted > 0) is flowing to the negative node (sign plus)
                                vals[i] += alpha * element->getPowerWanted() / diff_voltage;
                                //Question: sign before alpha - or + during setting current?
                                //Answer: sign before alpha is minus since we assume positive powerWanted if the current element behaves as load
                                // (*it_element)->setCurrent(-alpha * (*it_element)->getPowerWanted() / diff_voltage);
                                // Note: we should never reach this part of code since the authors assumes the negative node of current source as the ground node
                                WRITE_WARNING(TL("The negative node of current source is not the ground."))
                                if (PosNode_NumACol != -1) {
                                    // -1* d_b/d_phiPos = -1* d(alpha*P/(phiPos-phiNeg) )/d_phiPos = -1* (-alpha*P/(phiPos-phiNeg)^2 )
                                    J(i, PosNode_NumACol) += alpha * element->getPowerWanted() / diff_voltage / diff_voltage;
                                }
                                if (NegNode_NumACol != -1) {
                                    // -1* d_b/d_phiNeg = -1* d(alpha*P/(phiPos-phiNeg) )/d_phiNeg = -1* (--alpha*P/(phiPos-phiNeg)^2 )
                                    J(i, NegNode_NumACol) -= alpha * element->getPowerWanted() / diff_voltage / diff_voltage;
                                }
                            }
                        }
                    }
                }
                i++;
            }


            // RICE_CHECK @20210409 This had to be merged into the master/main manually.
            // Sum of currents going through the all voltage sources
            // the sum is over all nodes, but the nonzero nodes are only those neighboring with current sources,
            // so the sum is negative sum of currents through/from current sources representing trolleybuses
            currentSumActual = 0;
            for (i = 0; i < numofeqs - (int)voltageSources.size(); i++) {
                currentSumActual -= vals[i];
            }
            // RICE_TODO @20210409 This epsilon should be specified somewhere as a constant. Or should be a parameter.
            if ((A * x - b).norm() < 1e-6) {
                //current limits
                if (currentSumActual > getCurrentLimit() && MSGlobals::gOverheadWireCurrentLimits) {
                    alphaReason = ALPHA_CURRENT_LIMITS;
                    alpha_notSolution.push_back(alpha);
                    if (x_best_exist) {
                        x = x_best;
                    }
                    break;
                }
                //voltage limits 70% - 120% of nominal voltage
                // RICE_TODO @20210409 Again, these limits should be parametrized.
                if (x.maxCoeff() > voltageSources.front()->getVoltage() * 1.2 || x.minCoeff() < voltageSources.front()->getVoltage() * 0.7) {
                    alphaReason = ALPHA_VOLTAGE_LIMITS;
                    alpha_notSolution.push_back(alpha);
                    if (x_best_exist) {
                        x = x_best;
                    }
                    break;
                }

                alphaBest = alpha;
                x_best = x;
                x_best_exist = true;
                break;
            } else if (iterNR == max_iter_of_NR) {
                alphaReason = ALPHA_NOT_CONVERGING;
                alpha_notSolution.push_back(alpha);
                if (x_best_exist) {
                    x = x_best;
                }
                break;
            }

            // Newton=Rhapson iteration
            dx = -J.colPivHouseholderQr().solve(A * x - b);
            x = x + dx;
            ++iterNR;
        }

        if (alpha_notSolution.empty()) {
            // no alpha without solution is in the alpha_notSolution, so the solving procedure is terminating
            break;
        }

        if ((alpha_notSolution.back() - alphaBest) < alpha_res) {
            max_iter_of_NR = 2 * max_iter_of_NR;
            // RICE_TODO @20210409 Why division by 10?
            // it follows Sevcik, Jakub, et al. "Solvability of the Power Flow Problem in DC Overhead Wire Circuit Modeling." Applications of Mathematics (2021): 1-19.
            // see Alg 2 (progressive decrease of optimality tolerance)
            alpha_res = alpha_res / 10;
            // RICE_TODO @20210409 This epsilon should be specified somewhere as a constant. Or should be a parameter.
            if (alpha_res < 5e-5) {
                break;
            }
            alpha = alpha_notSolution.back();
            alpha_notSolution.pop_back();
            continue;
        }

        alpha = alphaBest + 0.5 * (alpha_notSolution.back() - alphaBest);
    }

    // vals is pointer to memory and we use it now for saving solution x_best instead of right-hand side b
    for (int i = 0; i < numofeqs; i++) {
        vals[i] = x_best[i];
    }

    // RICE_TODO: Describe what is happening here.
    // we take x_best and alphaBest and update current values in current sources in order to be in agreement with the solution
    int i = 0;
    for (auto& node : nodes) {
        if (node->isGround() || node->isRemovable() || node->getNumMatrixRow() == -2) {
            continue;
        }
        if (node->getNumMatrixRow() != i) {
            WRITE_ERROR(TL("wrongly assigned row of matrix A during solving the circuit"));
        }
        for (auto& it_element : node->getElements()) {
            if (it_element->getType() == Element::ElementType::CURRENT_SOURCE_traction_wire) {
                if (it_element->isEnabled()) {
                    double diff_voltage;
                    int PosNode_NumACol = it_element->getPosNode()->getNumMatrixCol();
                    int NegNode_NumACol = it_element->getNegNode()->getNumMatrixCol();
                    if (PosNode_NumACol == -1) {
                        diff_voltage = -x_best[NegNode_NumACol];
                    } else if (NegNode_NumACol == -1) {
                        diff_voltage = x_best[PosNode_NumACol];
                    } else {
                        diff_voltage = (x_best[PosNode_NumACol] - x_best[NegNode_NumACol]);
                    }

                    if (it_element->getPosNode() == node.get()) {
                        it_element->setCurrent(-alphaBest * it_element->getPowerWanted() / diff_voltage);
                    } else {
                        //Question: sign before alpha - or + during setting current?
                        //Answer: sign before alpha is minus since we assume positive powerWanted if the current element behaves as load
                        // it_element->setCurrent(-alphaBest * it_element->getPowerWanted() / diff_voltage);
                        // Note: we should never reach this part of code since the authors assumes the negative node of current source as the ground node
                        WRITE_WARNING(TL("The negative node of current source is not the ground."))
                    }
                }
            }
        }
        i++;
    }

    return true;
}
#endif

void Circuit::deployResults(double* vals, std::vector<int>* removable_ids) {
    // vals are the solution x

    int numofcolumn = (int)voltageSources.size() + (int)nodes.size() - 1;
    int numofeqs = numofcolumn - (int)removable_ids->size();

    //loop over non-removable nodes: we assign the computed voltage to the non-removables nodes
    int j = 0;
    Element* tElem = nullptr;
    Node* tNode = nullptr;
    for (int i = 0; i < numofcolumn; i++) {
        tNode = getNode(i);
        if (tNode != nullptr)
            if (tNode->isRemovable()) {
                continue;
            } else {
                if (j > numofeqs) {
                    WRITE_ERROR(TL("Results deployment during circuit evaluation was unsuccessful."));
                    break;
                }
                tNode->setVoltage(vals[j]);
                j++;
                continue;
            } else {
            tElem = getElement(i);
            if (tElem != nullptr) {
                if (j > numofeqs) {
                    WRITE_ERROR(TL("Results deployment during circuit evaluation was unsuccessful."));
                    break;
                }
                // tElem should be voltage source - the current through voltage source is computed in a loop below
                // if tElem is current source (JS thinks that no current source's id <= numofeqs), the current is already assign at the end of solveEquationsNRmethod method
                continue;
            }
        }
        WRITE_ERROR(TL("Results deployment during circuit evaluation was unsuccessful."));
    }

    Element* el1 = nullptr;
    Element* el2 = nullptr;
    Node* nextNONremovableNode1 = nullptr;
    Node* nextNONremovableNode2 = nullptr;
    // interpolate result of voltage to removable nodes
    for (auto& node : nodes) {
        if (!node->isRemovable()) {
            continue;
        }
        if (node->getNumOfElements() != 2) {
            continue;
        }

        el1 = node->getElements().front();
        el2 = node->getElements().back();
        // Get the non-owning pointer which is needed as argument
        Node* nodePtr = node.get();
        nextNONremovableNode1 = el1->getTheOtherNode(nodePtr);
        nextNONremovableNode2 = el2->getTheOtherNode(nodePtr);
        double x = el1->getResistance();
        double y = el2->getResistance();

        while (nextNONremovableNode1->isRemovable()) {
            el1 = nextNONremovableNode1->getAnOtherElement(el1);
            x += el1->getResistance();
            nextNONremovableNode1 = el1->getTheOtherNode(nextNONremovableNode1);
        }

        while (nextNONremovableNode2->isRemovable()) {
            el2 = nextNONremovableNode2->getAnOtherElement(el2);
            y += el2->getResistance();
            nextNONremovableNode2 = el2->getTheOtherNode(nextNONremovableNode2);
        }

        x = x / (x + y);
        y = ((1 - x) * nextNONremovableNode1->getVoltage()) + (x * nextNONremovableNode2->getVoltage());
        node->setVoltage(((1 - x)*nextNONremovableNode1->getVoltage()) + (x * nextNONremovableNode2->getVoltage()));
        node->setRemovability(false);
    }

    // Update the electric currents for voltage sources (based on Kirchhof's law: current out = current in)
    for (const auto& voltageSource : voltageSources) {
        double currentSum = 0;
        // We will compare Element* pointers, and `voltageSource` is a std::unique_ptr<Element>
        Element* vsElementPtr = voltageSource.get();
        // Extract the positive node pointer for easier access
        Node* posNodePtr = voltageSource->getPosNode();
        // Loop over all elements connected to the positive node
        for (const auto& el : posNodePtr->getElements()) {
            // Exclude the actual voltage source
            if (el != vsElementPtr) {
                // Current is calculated from voltage difference as (V_pos - V_neg) / R
                currentSum += (posNodePtr->getVoltage() - el->getTheOtherNode(posNodePtr)->getVoltage()) / el->getResistance();
                if (el->getType() == Element::ElementType::VOLTAGE_SOURCE_traction_wire) {
                    WRITE_WARNING(TL("Cannot assign unambigous electric current value to two voltage sources connected in parallel at the same node."));
                }
            }
        }
        voltageSource->setCurrent(currentSum);
    }
}

Circuit::Circuit(
    double currentLimit
) :
    nodes(),
    elements(),
    voltageSources(),
    lastId(-1),
    iscleaned(true),
    circuitCurrentLimit(currentLimit)
{
    // Just initilization of members using initializer list
}

#ifdef HAVE_EIGEN
bool Circuit::_solveNRmethod() {
    double* eqn = nullptr;
    double* vals = nullptr;
    std::vector<int> removable_ids;

    detectRemovableNodes(&removable_ids);
    createEquationsNRmethod(eqn, vals, &removable_ids);
    if (!solveEquationsNRmethod(eqn, vals, &removable_ids)) {
        return false;
    }
    // vals are now the solution x of the circuit
    deployResults(vals, &removable_ids);

    delete[] eqn;
    delete[] vals;
    return true;
}

bool Circuit::solve() {
    if (!iscleaned) {
        cleanUpSP();
    }
    return this->_solveNRmethod();
}

bool Circuit::createEquationsNRmethod(double*& eqs, double*& vals, std::vector<int>* removable_ids) {
    // removable_ids does not include nodes with voltage source yet

    // number of voltage sources + nodes without the ground node
    int n = (int)(voltageSources.size() + nodes.size() - 1);
    // number of equations
    // assumption: each voltage source has different positive node and common ground node,
    //             i.e. any node excluding the ground node is connected to 0 or 1 voltage source
    int m = n - (int)(removable_ids->size() + voltageSources.size());

    // allocate and initialize zero matrix eqs and vector vals
    eqs = new double[m * n];
    vals = new double[m];

    for (int i = 0; i < m; i++) {
        vals[i] = 0;
        for (int j = 0; j < n; j++) {
            eqs[i * n + j] = 0;
        }
    }

    // loop over all nodes
    int i = 0;
    for (const auto& node : nodes) {
        if (node->isGround() || node->isRemovable()) {
            // if the node is grounded or is removable set the corresponding number of row in matrix to -1 (no equation in eqs)
            node->setNumMatrixRow(-1);
            continue;
        }
        assert(i < m);
        // constitute the equation corresponding to `node`, add all passed voltage source elements into removable_ids
        bool noVoltageSource = createEquationNRmethod(node.get(), (eqs + n * i), vals[i], removable_ids);
        // if the node it has element of type "voltage source" we do not use the equation, because some value of current throw the voltage source can be always find
        if (noVoltageSource) {
            node->setNumMatrixRow(i);
            i++;
        } else {
            node->setNumMatrixRow(-2);
            vals[i] = 0;
            for (int j = 0; j < n; j++) {
                eqs[n * i + j] = 0;
            }
        }
    }

    // removable_ids includes nodes with voltage source already
    std::sort(removable_ids->begin(), removable_ids->end(), std::less<int>());


    for (const auto& voltageSource : voltageSources) {
        assert(i < m);
        createEquation(voltageSource.get(), (eqs + n * i), vals[i]);
        i++;
    }

    return true;
}

bool Circuit::createEquation(Element* vsource, double* eqn, double& val) {
    if (!vsource->getPosNode()->isGround()) {
        eqn[vsource->getPosNode()->getId()] = 1;
    }
    if (!vsource->getNegNode()->isGround()) {
        eqn[vsource->getNegNode()->getId()] = -1;
    }
    if (vsource->isEnabled()) {
        val = vsource->getVoltage();
    } else {
        val = 0;
    }
    return true;
}

bool Circuit::createEquationNRmethod(Node* node, double* eqn, double& val, std::vector<int>* removable_ids) {
    // loop over all elements connected to the node
    for (const auto& el : node->getElements()) {
        double x;
        switch (el->getType()) {
            case Element::ElementType::RESISTOR_traction_wire:
                if (el->isEnabled()) {
                    x = el->getResistance();
                    // go through all neighboring removable nodes and sum resistance of resistors in the serial branch
                    Node* nextNONremovableNode = el->getTheOtherNode(node);
                    Element* nextSerialResistor = el;
                    while (nextNONremovableNode->isRemovable()) {
                        nextSerialResistor = nextNONremovableNode->getAnOtherElement(nextSerialResistor);
                        x += nextSerialResistor->getResistance();
                        nextNONremovableNode = nextSerialResistor->getTheOtherNode(nextNONremovableNode);
                    }
                    // compute inverse value and place/add this value at proper places in eqn
                    x = 1 / x;
                    eqn[node->getId()] += x;

                    if (!nextNONremovableNode->isGround()) {
                        eqn[nextNONremovableNode->getId()] -= x;
                    }
                }
                break;
            case Element::ElementType::CURRENT_SOURCE_traction_wire:
                if (el->isEnabled()) {
                    // initialize current in current source
                    if (el->getPosNode() == node) {
                        x = -el->getPowerWanted() / voltageSources.front()->getVoltage();
                    } else {
                        x = el->getPowerWanted() / voltageSources.front()->getVoltage();
                    }
                } else {
                    x = 0;
                }
                val += x;
                break;
            case Element::ElementType::VOLTAGE_SOURCE_traction_wire:
                if (el->getPosNode() == node) {
                    x = -1;
                } else {
                    x = 1;
                }
                eqn[el->getId()] += x;
                // equations with voltage source can be ignored, because some value of current throw the voltage source can be always find
                removable_ids->push_back(el->getId());
                return false;
                break;
            case Element::ElementType::ERROR_traction_wire:
                return false;
                break;
        }
    }
    return true;
}
#endif

/**
 * Select removable nodes, i.e. nodes that are NOT the ground of the circuit
 * and that have exactly two resistor elements connected. Ids of those
 * removable nodes are added into the internal vector `removable_ids`.
 */
void Circuit::detectRemovableNodes(std::vector<int>* removable_ids) {
    // loop over all nodes in the circuit
    for (const auto& node : nodes) 
    {
        // The candidate node is the one that is connected to exactly two elements and it is not a ground node
        if (node->getNumOfElements() == 2 && !node->isGround()) {
            // Set such node by default as removable, but further checks are needed.
            node->setRemovability(true);
            // Check if the two connected elements are both resistors
            for (const auto& element : node->getElements()) {
                if (element->getType() != Element::ElementType::RESISTOR_traction_wire) {
                    node->setRemovability(false);
                    break;
                }
            }
            // If the node is still marked as removable, add its ID to the list of removable nodes
            if (node->isRemovable()) {
                //if the node is removable add pointer into the vector of removable nodes
                removable_ids->push_back(node->getId());
            }
        } else {
            node->setRemovability(false);
        }
    }
    // sort the vector of removable ids
    std::sort(removable_ids->begin(), removable_ids->end(), std::less<int>());
    return;
}

Element* Circuit::addElement(std::string name, double value, Node* pNode, Node* nNode, Element::ElementType et)
{
    // RICE_CHECK: This seems to be a bit of work in progress, is it final?
    //             if ((et == Element::ElementType::RESISTOR_traction_wire && value <= 0) || et == Element::ElementType::ERROR_traction_wire) {
    //             if (et == Element::ElementType::RESISTOR_traction_wire && value <= 1e-6) {
    // Too low resistance makes the equation system ill conditioned and the computation unstable
    if (et == Element::ElementType::RESISTOR_traction_wire && value <= CIRCUIT_MINIMAL_RESISTANCE) {
        if (value > -CIRCUIT_MINIMAL_RESISTANCE) {
            WRITE_WARNING(fmt::format(TL("Adding resistor element `{}` into circuit with resistance {:g} instead of requested {:g}. ", name, CIRCUIT_MINIMAL_RESISTANCE, value)));
            value = CIRCUIT_MINIMAL_RESISTANCE;
        } else {
            WRITE_ERROR(TL("Trying to add resistor element into the overhead wire circuit with resistance < 0. "))
            return nullptr;
        }
    }

    // Do not insert duplicate elements
    if (nodeNameMap.find(name) != nodeNameMap.end()) {
        WRITE_ERRORF(TL("The element '%' already exists."), name);
        return nullptr;
    }

    // Make sure to lock the circuit while modifying it
    circuit_lock.lock();

    // Create the element as unique_ptr to manage memory automatically
    // The unique_ptr assures that the element pointer is exlusively owned by the circuit
    // and will be automatically deleted when the circuit is destroyed or the element is erased.
    auto element = std::make_unique<Element>(name, et, value);
    Element* elementPtr = element.get();

    // Insert the (non owning) element pointer into the appropriate vector
    switch (et) {
        case Element::ElementType::VOLTAGE_SOURCE_traction_wire:
            // Assign unique ID to the voltage source element
            elementPtr->setId(lastId);
            // Add the element to the list of circuit elements and to the name map
            // Move the unique_ptr into the vector to transfer ownership
            voltageSources.push_back(std::move(element));
            // Store the non-owning raw pointer in the name map
            elementNameMap[name] = elementPtr;
            // Store the non-owning raw pointer in the id map
            elementIdMap[lastId] = elementPtr;
            lastId++;
            break;
        case Element::ElementType::RESISTOR_traction_wire:
        case Element::ElementType::CURRENT_SOURCE_traction_wire:
            // RICE_TODO: No unique ID of the resistor and current source elements is needed?
            // Add the element to the list of circuit elements and to the name map
            // Move the unique_ptr into the vector to transfer ownership
            elements.push_back(std::move(element));
            // Store the non-owning raw pointer in the name map
            elementNameMap[name] = elementPtr;
            break;
        default:
            WRITE_ERRORF(TL("Element type % of element '%' is not supported in overhead wire circuit."), et, name);
            element.reset(); // Release the unique_ptr to avoid memory leak
            elementPtr = nullptr;
    }

    // Unlock the circuit after modification
    circuit_lock.unlock();

    if (elementPtr) {
        // Connect the element to its nodes
        elementPtr->setPosNode(pNode);
        elementPtr->setNegNode(nNode);
        // Connect the nodes to the element
        pNode->addElement(elementPtr);
        nNode->addElement(elementPtr);
    }

    return elementPtr;
}

void
Circuit::eraseElement(Element* elementPtr) 
{
    // Do not erase voltage source elements via this method
    if (elementPtr->getType() == Element::ElementType::VOLTAGE_SOURCE_traction_wire) {
        WRITE_ERRORF(TL("Cannot erase voltage source elements: '%' cannot be erased."), elementPtr->getName());
        return;
    }

    // Erase the element references from its nodes
    elementPtr->getPosNode()->eraseElement(elementPtr);
    elementPtr->getNegNode()->eraseElement(elementPtr);

    // Lock the access to circuit nodes and elements
    circuit_lock.lock();
    // Remove element from name map
    elementNameMap.erase(elementPtr->getName());
    // Remove element from the ID map
    elementIdMap.erase(elementPtr->getId());
    // Remove element from storage vector (element will be automatically deleted via unique_ptr)
    elements.erase(
        std::remove_if(elements.begin(), elements.end(), [elementPtr](const auto& element) { return element.get() == elementPtr; }),
        elements.end()
    );
    // Unlock the circuit again
    circuit_lock.unlock();
}

void
Circuit::replaceAndDeleteNode(Node* unusedNode, Node* newNode)
{
    // Replace element endpoint in unusedNode with newNode
    for (auto& voltageSource : voltageSources) {
        if (voltageSource->getNegNode() == unusedNode) {
            // Replace the unusedNode with the newNode as negative node of the voltage source
            voltageSource->setNegNode(newNode);
            // Erase the reference to the voltage source from the unusedNode
            // RICE_TODO: Is this necessary? The unusedNode will be deleted anyway.
            // RICE_TODO: Originally we were erasing from `newNode`, which seems wrong.
            unusedNode->eraseElement(voltageSource.get());
            newNode->addElement(voltageSource.get());
        }
        if (voltageSource->getPosNode() == unusedNode) {
            voltageSource->setPosNode(newNode);
            unusedNode->eraseElement(voltageSource.get());
            newNode->addElement(voltageSource.get());
        }
    }
    for (auto& element : elements) {
        if (element->getNegNode() == unusedNode) {
            element->setNegNode(newNode);
            unusedNode->eraseElement(element.get());
            newNode->addElement(element.get());
        }
        if (element->getPosNode() == unusedNode) {
            element->setPosNode(newNode);
            unusedNode->eraseElement(element.get());
            newNode->addElement(element.get());
        }
    }

    // Erase unusedNode from nodes vector of this circuit
    eraseNode(unusedNode);

    // modify id of other elements and nodes
    int modLastId = this->getLastId() - 1;
    if (unusedNode->getId() != modLastId) {
        Node* node_last = this->getNode(modLastId);
        if (node_last != nullptr) {
            node_last->setId(unusedNode->getId());
        } else {
            Element* elem_last = this->getVoltageSource(modLastId);
            if (elem_last != nullptr) {
                elem_last->setId(unusedNode->getId());
            } else {
                WRITE_ERROR(TL("The element or node with the last Id was not found in the circuit!"));
            }
        }
    }

    this->decreaseLastId();
    delete unusedNode;
}

void Circuit::cleanUpSP() {
    for (const auto& el : elements) {
        if (el->getType() != Element::ElementType::RESISTOR_traction_wire) {
            el->setEnabled(true);
        }
    }

    for (const auto& vs : voltageSources) {
        vs->setEnabled(true);
    }
    this->iscleaned = true;
}

bool Circuit::checkCircuit(std::string substationId) {
    // check empty nodes
    for (const auto& node : nodes) {
        if (node->getNumOfElements() < 2) {
            //cout << "WARNING: Node [" << node->getName() << "] is connected to less than two elements, please enter other elements.\n";
            if (node->getNumOfElements() < 1 && (!node->isGround())) {
                WRITE_ERRORF(TL("Circuit node '%' for substation '%s' has zero elements."), node->getName(), substationId);
                return false;
            }
        }
    }
    // check voltage sources
    for (const auto& voltageSource : voltageSources) {
        if (voltageSource->getPosNode() == nullptr || voltageSource->getNegNode() == nullptr) {
            //cout << "ERROR: Voltage Source [" << voltageSource->getName() << "] is connected to less than two nodes, please enter the other end.\n";
            WRITE_ERRORF(TL("Circuit Voltage Source '%' is connected to less than two nodes, please adjust the definition of the section (with substation '%')."), voltageSource->getName(), substationId);
            return false;
        }
    }

    // RICE_TODO: We should never have this case
    if (voltageSources.size() == 0)
    {
        WRITE_ERRORF(TL("Circuit of substation '%' has no voltage sources."), substationId);
        return false;
    }
    
    // check other elements
    for (const auto& el : elements) {
        if (el->getPosNode() == nullptr || el->getNegNode() == nullptr) {
            //cout << "ERROR: Element [" << el->getName() << "] is connected to less than two nodes, please enter the other end.\n";
            WRITE_ERRORF(TL("Circuit Element '%' is connected to less than two nodes, please adjust the definition of the section (with substation '%')."), el->getName(), substationId);
            return false;
        }
    }

    // check connectivity
    int num = (int)nodes.size() + getNumVoltageSources() - 1;
    bool* nodesVisited = new bool[num];
    for (int i = 0; i < num; i++) {
        nodesVisited[i] = false;
    }
    // TODO: Probably unused
    // int id = -1;
    if (!getNode(-1)->isGround()) {
        //cout << "ERROR: Node id -1 is not the ground \n";
        WRITE_ERRORF(TL("Circuit Node with id '-1' is not the ground, please adjust the definition of the section (with substation '%')."), substationId);
    }
    std::vector<Node*>* queue = new std::vector<Node*>(0);
    Node* node = nullptr;
    Node* neigboringNode = nullptr;
    //start with (voltageSources->front()->getPosNode())
    nodesVisited[voltageSources.front()->getId()] = 1;
    node = voltageSources.front()->getPosNode();
    queue->push_back(node);

    while (!queue->empty()) {
        node = queue->back();
        queue->pop_back();
        if (!nodesVisited[node->getId()]) {
            nodesVisited[node->getId()] = true;
            for (const auto& el : node->getElements()) {
                neigboringNode = el->getTheOtherNode(node);
                if (!neigboringNode->isGround()) {
                    queue->push_back(neigboringNode);
                } else if (el->getType() == Element::ElementType::VOLTAGE_SOURCE_traction_wire) {
                    /// there used to be == 1 which was probably a typo ... check!
                    nodesVisited[el->getId()] = 1;
                } else if (el->getType() == Element::ElementType::RESISTOR_traction_wire) {
                    //cout << "ERROR: The resistor type connects the ground \n";
                    WRITE_ERRORF(TL("A Circuit Resistor Element connects the ground, please adjust the definition of the section (with substation '%')."), substationId);
                }
            }
        }
    }

    for (int i = 0; i < num; i++) {
        if (nodesVisited[i] == 0) {
            //cout << "ERROR: Node or voltage source with id " << (i) << " has been not visited during checking of the circuit => Disconnectivity of the circuit. \n";
            WRITE_WARNINGF(TL("Circuit Node or Voltage Source with internal id '%' has been not visited during checking of the circuit. The circuit is disconnected, please adjust the definition of the section (with substation '%')."), toString(i), substationId);
        }
    }

    return true;
}

int Circuit::getNumVoltageSources() {
    return (int) voltageSources.size();
}

std::string Circuit::getVoltageSourcesNames() {
    std::string oss;
    for (auto& voltageSource : voltageSources) {
        if (!oss.empty()) {
            oss += " ";
        }
        oss += toString(voltageSource->getId());
    }

    return oss;
}

/**
 * @brief Export circuit graph to DOT format string
 *
 * Creates a DOT language representation of the circuit that can be
 * visualized with Graphviz or other compatible tools.
 *
 * @param circuit Pointer to the Circuit object to export
 * @param includeValues If true, include resistance/voltage/current values in labels
 * @return std::string containing the DOT format graph description
 */
std::string 
Circuit::exportToDOT(bool includeValues)
{
    std::ostringstream dot;

    // Start the graph definition
    dot << "graph Circuit {\n";
    dot << "    rankdir=LR;\n";  // Left-to-right layout
    dot << "    node [shape=circle];\n\n";

    // Export nodes with attributes
    dot << "    // Nodes\n";
    for (const auto& node : nodes) {
        dot << "    \"" << node->getName() << "\" [";

        if (node->isGround()) {
            dot << "shape=rectangle, color=green, penwidth=4, label=\"" << node->getName() << "\\nGND\"";
        }
        else if (node->isRemovable()) {
            dot << "color=gray, penwidth=2, label=\"" << node->getName() << "\"";
        }
        else {
            dot << "label=\"" << node->getName() << "\"";
        }

        if (node->getName().rfind("pos_", 0) == 0) {
            // Vehicle nodes start with `pos_`, this is just a hack.
            dot << ", style=filled, fillcolor=azure1";
        }

        if (includeValues && !node->isGround()) {
            dot << ", xlabel=\"" << node->getVoltage() << "V\"";
        }

        dot << "];\n";
    }

    dot << "\n    // Elements\n";

    // Helper lambda to create element label
    auto createElementLabel = [&](const Element& el) -> std::string {
        std::ostringstream label;
        label << el.getName();

        if (includeValues) {
            switch (el.getType()) {
            case Element::ElementType::RESISTOR_traction_wire:
                label << "\\nR=" << el.getResistance() << "Ohm";
                if (el.getCurrent() != DBL_MAX) {
                    label << "\\nI=" << el.getCurrent() << "A";
                }
                break;
            case Element::ElementType::VOLTAGE_SOURCE_traction_wire:
                label << "\\nV=" << el.getVoltage() << "V";
                if (el.getCurrent() != DBL_MAX) {
                    label << "\\nI=" << el.getCurrent() << "A";
                }
                break;
            case Element::ElementType::CURRENT_SOURCE_traction_wire:
                if (el.getCurrent() != DBL_MAX) {
                    label << "\\nI=" << el.getCurrent() << "A";
                }
                if (!std::isnan(el.getPowerWanted())) {
                    label << "\\nP=" << el.getPowerWanted() << "W";
                }
                break;
            default:
                break;
            }
        }

        return label.str();
        };

    // Export regular elements
    for (const auto& el : elements) {
        if (el->getPosNode() == nullptr || el->getNegNode() == nullptr) {
            continue;  // Skip incomplete elements
        }

        std::string style;
        std::string color;

        switch (el->getType()) {
        case Element::ElementType::RESISTOR_traction_wire:
            style = "solid";
            color = "black";
            break;
        case Element::ElementType::CURRENT_SOURCE_traction_wire:
            style = "dashed";
            color = "blue";
            break;
        default:
            style = "solid";
            color = "gray";
            break;
        }

        // std::string label = createElementLabel(*el);
        
        dot << "    \"" << el->getPosNode()->getName() << "\" -- \""
            << el->getNegNode()->getName() << "\" [";
        //dot << "label=\"" << createElementLabel(*el) << "\", ";
        dot << "style=" << style << ", color=" << color;

        if (!el->isEnabled()) {
            dot << ", constraint=false, color=lightgray";
        }

        dot << "];\n";
    }

    // Export voltage sources
    for (const auto& vs : voltageSources) {
        if (vs->getPosNode() == nullptr || vs->getNegNode() == nullptr) {
            continue;
        }

        dot << "    \"" << vs->getPosNode()->getName() << "\" -- \""
            << vs->getNegNode()->getName() << "\" [";
        //dot << "label=\"" << createElementLabel(vs) << "\", ";
        dot << "style=bold, color=red, penwidth=2";

        if (!vs->isEnabled()) {
            dot << ", constraint=false, color=lightgray";
        }

        dot << "];\n";
    }

    // Add legend
    dot << "\n    // Legend\n";
    dot << "    subgraph cluster_legend {\n";
    dot << "        label=\"Legend\";\n";
    dot << "        style=dashed;\n";
    dot << "        \"L1\" [shape=point, style=invis];\n";
    dot << "        \"L2\" [shape=point, style=invis];\n";
    dot << "        \"L3\" [shape=point, style=invis];\n";
    dot << "        \"L4\" [shape=point, style=invis];\n";
    dot << "        \"L5\" [shape=point, style=invis];\n";
    dot << "        \"L6\" [shape=point, style=invis];\n";
    dot << "        \"L1\" -- \"L2\" [label=\"Resistor\", color=black];\n";
    dot << "        \"L3\" -- \"L4\" [label=\"Voltage Source\", color=red, style=bold];\n";
    dot << "        \"L5\" -- \"L6\" [label=\"Vehicle to ground\", color=blue, style=dashed];\n";
    dot << "    }\n";

    dot << "}\n";

    return dot.str();
}

/**
 * @brief Export circuit graph to DOT file
 *
 * @param circuit Pointer to the Circuit object to export
 * @param filename Path to the output file
 * @param includeValues If true, include resistance/voltage/current values in labels
 * @return true if export was successful, false otherwise
 */
bool 
Circuit::exportToDOTFile(
    const std::string& filename, 
    bool includeValues
) {
    std::ofstream outFile(filename);

    if (!outFile.is_open()) {
        return false;
    }

    outFile << exportToDOT(includeValues);
    outFile.close();

    return true;
}

/**
 * @brief Export circuit to simple adjacency list format
 *
 * Creates a simple text representation of the circuit graph
 *
 * @param circuit Pointer to the Circuit object to export
 * @return std::string containing the adjacency list
 */
std::string 
Circuit::exportToAdjacencyList()
{
    std::ostringstream output;

    output << "Circuit Graph - Adjacency List\n";
    output << "================================\n\n";

    for (const auto& node : nodes) {
        output << node->getName();
        if (node->isGround()) {
            output << " [GROUND]";
        }
        output << " (" << node->getVoltage() << "V):\n";

        for (const auto& el : node->getElements()) {
            Node* otherNode = el->getTheOtherNode(node.get());
            if (otherNode) {
                output << "  -> " << otherNode->getName();
                output << " via " << el->getName();

                switch (el->getType()) {
                case Element::ElementType::RESISTOR_traction_wire:
                    output << " [R=" << el->getResistance() << "?]";
                    break;
                case Element::ElementType::VOLTAGE_SOURCE_traction_wire:
                    output << " [V=" << el->getVoltage() << "V]";
                    break;
                case Element::ElementType::CURRENT_SOURCE_traction_wire:
                    output << " [I=" << el->getCurrent() << "A]";
                    break;
                default:
                    break;
                }

                output << "\n";
            }
        }
        output << "\n";
    }

    return output.str();
}
