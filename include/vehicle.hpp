#pragma once

#include <string>

#include "graph.hpp"
#include "raylib.h"

// A vehicle that re-plans its route with A* at every node,
// so it reacts to congestion that builds up while it drives.
class Vehicle
{
public:
    int id;
    std::string type;  // Rickshaw, Car, Bike, Truck, Bus, ...
    float length;      // Average length for the type, in metres
    Node* currentNode; // Last node the vehicle stood on
    Node* goalNode;
    Edge* currentEdge; // Road the vehicle is on
    float x, y;        // Current position
    float speed = 3;
    Node* currentNodeToReach = nullptr;
    bool hasReachedDestination = false;
    Color color = BLUE;

    Vehicle(int id, std::string type, Node* startNode, Node* goalNode);

    // Advance one frame. Returns true once the destination is reached.
    bool moveVehicle();

private:
    void setLengthByType();
    void pickNextEdge();
    void changeCoordinates();
};
