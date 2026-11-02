#pragma once

#include <string>
#include <vector>

class Edge;

// A location on the map. Roads are stored as directed Edge pairs.
class Node
{
public:
    int id;                  // Unique identifier for the node
    float x, y;              // Screen coordinates of the node
    std::string name;        // Name of the node
    std::vector<Node*> neighbors; // Adjacent nodes
    std::vector<Edge*> edges;     // edges[i] leads to neighbors[i]

    Node(int nodeId, float xCoord, float yCoord, std::string nodeName)
        : id(nodeId), x(xCoord), y(yCoord), name(std::move(nodeName)) {}
    // A node owns the outgoing edges it created in addNeighbor()
    virtual ~Node();
    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    // Connect this node and `neighbor` with a two-way road
    void addNeighbor(Node* neighbor);

    // 0 for a plain node, 3 or 4 for a 3-way / 4-way intersection
    virtual int getType() { return 0; }
};

// An intersection whose signals turn green one direction at a time
class TrafficIntersection : public Node
{
public:
    int type;                  // 3-way or 4-way
    std::vector<bool> signals; // signals[i] is green for edges[i]
    int lightRecord4 = 0;
    int lightRecord3 = 0;

    TrafficIntersection(int nodeId, float xCoord, float yCoord, std::string nodeName, int intersectionType);

    void initializeSignals(int intersectionType);

    // Advance the green light to the next direction
    void changeLights();

    int getType() override { return type; }
};

// A one-way lane between two nodes
class Edge
{
public:
    Node* node1;
    Node* node2;
    float length;     // Euclidean length
    int no_of_agents; // Vehicles currently on the road
    float width;      // Road width
    int max_traffic;  // Capacity, from width × length

    Edge(Node* n1, Node* n2, float road_width);
};

// Travel cost of the road between two nodes: length plus a congestion penalty
int cost(Node* currentNode, Node* nextNode);

// Straight-line distance, used as the A* heuristic
int heuristic(Node* node, Node* goal);

// The directed edge node1 -> node2, or nullptr
Edge* findEdge(Node* node1, Node* node2);

// True if both edges join the same two nodes, in either direction
bool areEdgesEqual(Edge* edge1, Edge* edge2);

// A* search; returns the first edge of the cheapest path, or nullptr
Edge* aStar(Node* start, Node* goal);
