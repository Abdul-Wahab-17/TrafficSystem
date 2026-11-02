#include "vehicle.hpp"

#include <cmath>
#include <iostream>
#include <unordered_map>

using namespace std;

Vehicle::Vehicle(int id, string type, Node* startNode, Node* goalNode)
    : id(id), type(std::move(type)), currentNode(startNode), goalNode(goalNode),
      currentEdge(nullptr), x(startNode->x), y(startNode->y)
{
    pickNextEdge();
    setLengthByType();
}

void Vehicle::setLengthByType()
{
    static const unordered_map<string, float> averageLengths = {
        {"Car", 4.5f},
        {"Truck", 12.0f},
        {"Bus", 10.5f},
        {"Bike", 2.0f},
        {"Rickshaw", 1.8f},
    };

    auto it = averageLengths.find(type);
    length = it != averageLengths.end() ? it->second : 4.5f;
}

// Ask A* for the best road out of the current node, given current congestion
void Vehicle::pickNextEdge()
{
    currentEdge = aStar(currentNode, goalNode);
    if (currentEdge == nullptr)
    {
        cerr << "Vehicle " << id << ": no route from " << currentNode->name
             << " to " << goalNode->name << endl;
        hasReachedDestination = true;
        currentNodeToReach = currentNode;
        return;
    }
    if (currentEdge->node1->x == currentNode->x && currentEdge->node1->y == currentNode->y)
    {
        currentNodeToReach = currentEdge->node2;
    }
    else
    {
        currentNodeToReach = currentEdge->node1;
    }
}

bool Vehicle::moveVehicle()
{
    if (x == goalNode->x && y == goalNode->y)
    {
        // Arrived: leave the road
        findEdge(currentEdge->node1, currentEdge->node2)->no_of_agents--;
        findEdge(currentEdge->node2, currentEdge->node1)->no_of_agents--;
        return true;
    }

    if (x == currentNode->x && y == currentNode->y)
    {
        // At a node: re-plan, then join the chosen road
        pickNextEdge();
        changeCoordinates();
        findEdge(currentEdge->node1, currentEdge->node2)->no_of_agents++;
        findEdge(currentEdge->node2, currentEdge->node1)->no_of_agents++;
    }
    else if (x == currentNodeToReach->x && y == currentNodeToReach->y)
    {
        // Reached the end of the road: leave it
        currentNode = currentNodeToReach;
        findEdge(currentEdge->node1, currentEdge->node2)->no_of_agents--;
        findEdge(currentEdge->node2, currentEdge->node1)->no_of_agents--;
    }
    else
    {
        changeCoordinates();
    }
    return false;
}

void Vehicle::changeCoordinates()
{
    float dx = currentNodeToReach->x - x;
    float dy = currentNodeToReach->y - y;
    float distanceToNextNode = sqrt(dx * dx + dy * dy);

    // Snap to the node once within 5 units
    if (distanceToNextNode <= 5.0f)
    {
        x = currentNodeToReach->x;
        y = currentNodeToReach->y;
    }
    else
    {
        x += dx / distanceToNextNode * speed;
        y += dy / distanceToNextNode * speed;
    }
}
