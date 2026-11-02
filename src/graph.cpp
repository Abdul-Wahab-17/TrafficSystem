#include "graph.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <iostream>
#include <queue>
#include <unordered_map>

using namespace std;

void Node::addNeighbor(Node* neighbor)
{
    neighbors.push_back(neighbor);
    edges.push_back(new Edge(this, neighbor, 3.2f));
    neighbor->neighbors.push_back(this);
    neighbor->edges.push_back(new Edge(neighbor, this, 3.2f));
}

TrafficIntersection::TrafficIntersection(int nodeId, float xCoord, float yCoord, string nodeName, int intersectionType)
    : Node(nodeId, xCoord, yCoord, std::move(nodeName)), type(intersectionType)
{
    initializeSignals(intersectionType);
}

void TrafficIntersection::initializeSignals(int intersectionType)
{
    if (intersectionType == 4 || intersectionType == 3)
    {
        signals = vector<bool>(intersectionType, false);
        signals[0] = true;
    }
    else
    {
        cerr << "Invalid intersection type. Defaulting to 4-way." << endl;
        type = 4;
        signals = vector<bool>(4, false);
        signals[0] = true;
    }
}

void TrafficIntersection::changeLights()
{
    if (type == 4)
    {
        signals[lightRecord4] = false;
        lightRecord4 = (lightRecord4 + 1) % type;
        signals[lightRecord4] = true;
    }
    if (type == 3)
    {
        signals[lightRecord3] = false;
        lightRecord3 = (lightRecord3 + 1) % type;
        signals[lightRecord3] = true;
    }
}

Edge::Edge(Node* n1, Node* n2, float road_width)
    : node1(n1), node2(n2), no_of_agents(0), width(road_width)
{
    length = sqrt(pow(n1->x - n2->x, 2) + pow(n1->y - n2->y, 2));

    // Capacity grows with road area
    const float density_factor = 0.123f;
    max_traffic = static_cast<int>(width * length * density_factor);
}

int cost(Node* currentNode, Node* nextNode)
{
    for (Edge* edge : currentNode->edges)
    {
        if (edge->node1 == nextNode || edge->node2 == nextNode)
        {
            // Each vehicle on the road adds 10 to its cost
            return edge->length + 10 * edge->no_of_agents;
        }
    }
    return INT_MAX;
}

int heuristic(Node* node, Node* goal)
{
    float euclideanDistance = sqrt(pow(node->x - goal->x, 2) + pow(node->y - goal->y, 2));
    return static_cast<int>(euclideanDistance);
}

Edge* findEdge(Node* node1, Node* node2)
{
    for (Edge* edge : node1->edges)
    {
        if (edge->node2 == node2)
        {
            return edge;
        }
    }
    return nullptr;
}

bool areEdgesEqual(Edge* edge1, Edge* edge2)
{
    return (edge1->node1 == edge2->node1 && edge1->node2 == edge2->node2) ||
           (edge1->node1 == edge2->node2 && edge1->node2 == edge2->node1);
}

Edge* aStar(Node* start, Node* goal)
{
    using QueueElement = pair<float, Node*>; // {f_score, node}
    priority_queue<QueueElement, vector<QueueElement>, greater<>> openSet;

    unordered_map<Node*, float> gScore;
    unordered_map<Node*, Edge*> cameFromEdge;

    gScore[start] = 0;
    openSet.push({ static_cast<float>(heuristic(start, goal)), start });

    while (!openSet.empty())
    {
        Node* current = openSet.top().second;
        openSet.pop();

        if (current == goal)
        {
            // Walk back to the start and return the first edge of the path
            vector<Edge*> path;
            while (cameFromEdge.find(current) != cameFromEdge.end())
            {
                Edge* edge = cameFromEdge[current];
                path.push_back(edge);
                current = (edge->node1 == current) ? edge->node2 : edge->node1;
            }
            if (path.empty())
            {
                return nullptr; // start == goal
            }
            reverse(path.begin(), path.end());
            return path[0];
        }

        for (size_t i = 0; i < current->neighbors.size(); ++i)
        {
            Node* neighbor = current->neighbors[i];
            Edge* edge = current->edges[i];

            float tentative_gScore = gScore[current] + cost(current, neighbor);

            if (!gScore.count(neighbor) || tentative_gScore < gScore[neighbor])
            {
                gScore[neighbor] = tentative_gScore;
                float fScore = tentative_gScore + heuristic(neighbor, goal);
                openSet.push({ fScore, neighbor });
                cameFromEdge[neighbor] = edge;
            }
        }
    }

    return nullptr;
}
