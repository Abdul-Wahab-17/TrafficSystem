#include <iostream>
#include <vector>
#include <set>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <climits>   // For INT_MAX
#include <algorithm> // For reverse()
#include "raylib.h"
#include< unordered_set>
#include<string>

using namespace std;

class Edge;

class Node
{
public:
    int id;                   // Unique identifier for the node
    float x, y;               // Coordinates of the node
    string name;              // Name of the node
    vector<Node*> neighbors; // List of adjacent nodes
    vector<Edge*> edges;     // List of edges for the node (using pointers)

    // Constructor
    Node(int nodeId, float xCoord, float yCoord, string nodeName)
        : id(nodeId), x(xCoord), y(yCoord), name(nodeName) {}

    // Method to add a neighbor and create the edge
    void addNeighbor(Node* neighbor);

    // Display node details
    void display()
    {
        cout << "Node " << id << " (" << name << ", x: " << x << ", y: " << y << ") is connected to: ";
        for (Node* neighbor : neighbors)
        {
            cout << neighbor->id << " (" << neighbor->name << ") ";
        }
        cout << endl;
    }

    virtual string toString()
    {
        return "node";
    }

    virtual int getType()
    {
        return 0;
    }
};

class TrafficIntersection : public Node
{
public:
    int type;             // Intersection type (4-way, 3-way)
    vector<bool> signals; // Array of traffic signals for each direction
    int lightRecord4 = 0;
    int lightRecord3 = 0;

    // Constructor
    TrafficIntersection(int nodeId, float xCoord, float yCoord, string nodeName, int intersectionType)
        : Node(nodeId, xCoord, yCoord, nodeName), type(intersectionType)
    {
        initializeSignals(intersectionType);
    }

    // Method to display intersection details
    void displayIntersection()
    {
        cout << "Traffic Intersection " << id << " (" << name << ")\n";
        cout << "Type: " << (type == 4 ? "4-way" : "3-way") << endl;
        cout << "Signals: ";
        for (bool signal : signals)
        {
            cout << (signal ? "Green" : "Red") << " ";
        }
        cout << endl;
    }

    void initializeSignals(int intersectionType)
    {
        if (intersectionType == 4)
        {
            signals = vector<bool>(4, false); // 4 signals for a 4-way intersection
            signals[0] = true;
        }
        else if (intersectionType == 3)
        {
            signals = vector<bool>(3, false); // 3 signals for a 3-way intersection
            signals[0] = true;
        }
        else
        {
            cerr << "Invalid intersection type. Defaulting to 4-way." << endl;
            signals = vector<bool>(4, false); // Default to 4 signals
        }
    }

    void changeLights()
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

    int getType() override
    {
        return type;
    }
};

class Edge
{
public:
    Node* node1;      // First node of the edge
    Node* node2;      // Second node of the edge
    float length;     // Length of the edge
    int no_of_agents; // Number of agents on the edge
    float width;      // Width of the road
    int max_traffic;  // Maximum number of allowed traffic

    // Constructor with width parameter
    Edge(Node* n1, Node* n2, float road_width) : node1(n1), node2(n2), width(road_width), no_of_agents(0)
    {
        // Calculate the length using the Euclidean distance formula
        length = sqrt(pow(n1->x - n2->x, 2) + pow(n1->y - n2->y, 2));

        // Calculate the maximum allowed traffic based on width and length
        float density_factor = 0.123f; // Example density: 10 agents per unit area
        max_traffic = static_cast<int>(width * length * density_factor);
    }

    // Constructor with only two nodes (everything else defaults to 0 or flag values)
    Edge(Node* n1, Node* n2) : node1(n1), node2(n2), length(0), no_of_agents(0), width(0), max_traffic(0) {}
};

// Method implementation for adding a neighbor and creating an edge
void Node::addNeighbor(Node* neighbor)
{
    neighbors.push_back(neighbor);
    Edge* edge = new Edge(this, neighbor, 3.2); // Create a new edge dynamically
    edges.push_back(edge);                     // Add the edge pointer to the edges list
    neighbor->neighbors.push_back(this);
    Edge* edgeReverse = new Edge(neighbor, this, 3.2);
    neighbor->edges.push_back(edgeReverse);
}

// Cost function to find the edge and return the length
int cost(Node* currentNode, Node* nextNode)
{
    // Loop through the edges of currentNode to find the one with nextNode
    for (Edge* edge : currentNode->edges)
    {
        if ((edge->node1 == nextNode) || (edge->node2 == nextNode))
        {
            return edge->length + 10*edge->no_of_agents; // Include number of agents in cost
        }
    }
    return INT_MAX; // Indicates no edge found
}

// Heuristic function (Euclidean distance)
int heuristic(Node* node, Node* goal)
{
    float euclideanDistance = sqrt(pow(node->x - goal->x, 2) + pow(node->y - goal->y, 2));
    return static_cast<int>(euclideanDistance);
}

Edge* findEdge(Node* node1, Node* node2);

// A* Algorithm to find the path
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath>

using namespace std;

// Global cost and heuristic functions
extern int cost(Node* currentNode, Node* nextNode);
extern int heuristic(Node* node, Node* goal);

Edge* aStar(Node* start, Node* goal)
{
    // Priority queue to store nodes with their f_score (g_score + heuristic)
    using QueueElement = pair<float, Node*>; // {f_score, Node*}
    priority_queue<QueueElement, vector<QueueElement>, greater<>> openSet;

    // Maps for g_score and cameFrom relationships
    unordered_map<Node*, float> gScore;
    unordered_map<Node*, Edge*> cameFromEdge;

    // Initialize all g_scores to infinity
    gScore[start] = 0;

    // Push the starting node with its f_score
    openSet.push({ heuristic(start, goal), start });

    while (!openSet.empty())
    {
        Node* current = openSet.top().second;
        openSet.pop();

        if (current == goal)
        {
            // Goal found, reconstruct path
            vector<Edge*> path;
            while (cameFromEdge.find(current) != cameFromEdge.end())
            {
                Edge* edge = cameFromEdge[current];
                path.push_back(edge);
                current = (edge->node1 == current) ? edge->node2 : edge->node1;
            }
            reverse(path.begin(), path.end());
            return path[0];
        }

        // Explore neighbors
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

    // If we reach here, no path was found
    return nullptr;
}


// Find an edge between two nodes
Edge* findEdge(Node* node1, Node* node2)
{
    for (Edge* edge : node1->edges)
    {
        if ((edge->node2 == node2))
        {
            return edge; // Return the pointer to the edge
        }
    }
    return nullptr; // Return nullptr if no edge found
}

class Vehicle
{
public:
    int id;              // Unique ID for the vehicle
    string type;         // Vehicle type (car, truck, bus, motorcycle, etc.)
    float length;        // Length of the vehicle in meters
    Node* currentNode;   // Current node the vehicle is at
    Node* goalNode;      // Goal node for the vehicle
    Edge* currentEdge;   // Current edge the vehicle is on
    vector<Node*> path; // Planned path for the vehicle (sequence of nodes)
    float x;             // Current x-coordinate of the vehicle
    float y;             // Current y-coordinate of the vehicle
    float speed = 3;
    Node* currentNodeToReach;
    bool hasReachedDestination = false;
    Color color = BLUE;

    // Constructor
    Vehicle(int id, string type, Node* startNode, Node* goalNode)
        : id(id), type(type), currentNode(startNode), goalNode(goalNode), currentEdge(nullptr), x(startNode->x), y(startNode->y)
    {
        this->currentEdge = aStar(currentNode, this->goalNode);
        if (currentEdge->node1->x == currentNode->x && currentEdge->node1->y == currentNode->y)
        {
            this->currentNodeToReach = currentEdge->node2;
        }
        else
        {
            this->currentNodeToReach = currentEdge->node1;
        }
        setLengthByType();
    }

    // Assign realistic average lengths based on vehicle type
    void setLengthByType()
    {
        static unordered_map<string, float> averageLengths = {
            {"Car", 4.5},        // Average length for cars
            {"Truck", 12.0},     // Average length for trucks
            {"Bus", 10.5},       // Average length for buses
            {"Bike", 2.0}, // Average length for motorcycles
            {"Rickshaw", 1.8}     // Average length for bicycles
        };

        if (averageLengths.find(type) != averageLengths.end())
        {
            length = averageLengths[type];
        }
        else
        {
            cerr << "Unknown vehicle type: " << type << ". Assigning default length of 4.5 meters." << endl;
            length = 4.5; // Default length for unknown vehicle types
        }
    }

    // Move the vehicle to the next node in the path
    void moveToNextNode()
    {
        if (path.empty())
        {
            cerr << "No path defined for the vehicle." << endl;
            return;
        }

        // Update the current node and edge
        currentNode = path.front();
        path.erase(path.begin());
        if (!path.empty())
        {
            currentEdge = findEdge(currentNode, path.front());
        }
        else
        {
            currentEdge = nullptr;
        }

        // Update the x and y coordinates (assuming simplistic linear interpolation for now)
        x = currentNode->x;
        y = currentNode->y;

        // Check if the vehicle has reached its goal
        if (currentNode == goalNode)
        {
            cout << "Vehicle ID " << id << " has reached its destination." << endl;
        }
    }

    bool moveVehicle()
    {
        if (this->x == goalNode->x && this->y == goalNode->y)
        {
            Edge* edgeToChange = findEdge(currentEdge->node1, currentEdge->node2);
            edgeToChange->no_of_agents--;
            edgeToChange = findEdge(currentEdge->node2, currentEdge->node1);
            edgeToChange->no_of_agents--;
            return true;
        }

        if (this->x == currentNode->x && this->y == currentNode->y)
        {
            this->currentEdge = aStar(currentNode, this->goalNode);
            if (currentEdge->node1->x == currentNode->x && currentEdge->node1->y == currentNode->y)
            {
                this->currentNodeToReach = currentEdge->node2;
            }
            else
            {
                this->currentNodeToReach = currentEdge->node1;
            }
            changeCoordinates();
            Edge* edgeToChange = findEdge(currentEdge->node1, currentEdge->node2);
            edgeToChange->no_of_agents++;
            edgeToChange = findEdge(currentEdge->node2, currentEdge->node1);
            edgeToChange->no_of_agents++;
            
        }
        else if (this->x == currentNodeToReach->x && this->y == currentNodeToReach->y)
        {
            currentNode = currentNodeToReach;
            Edge* edgeToChange = findEdge(currentEdge->node1, currentEdge->node2);
            edgeToChange->no_of_agents--;
            edgeToChange = findEdge(currentEdge->node2, currentEdge->node1);
            edgeToChange->no_of_agents--;
        }
        else
        {
            changeCoordinates();
        }
        return false;
    }

    void changeCoordinates()
    {
        float dx = currentNodeToReach->x - this->x; // Difference in x-coordinates
        float dy = currentNodeToReach->y - this->y; // Difference in y-coordinates

        // Distance between current position and next node
        float distanceToNextNode = sqrt(dx * dx + dy * dy);

        // If the vehicle is within the radius of 5 units, snap it to the node
        if (distanceToNextNode <= 5.0f)
        {
            this->x = currentNodeToReach->x;
            this->y = currentNodeToReach->y;
        }
        else
        {
            float directionX = dx / distanceToNextNode;
            float directionY = dy / distanceToNextNode;

            // Move the vehicle based on its speed and deltaTime
            float moveDistance = this->speed * 1;
            this->x += directionX * moveDistance;
            this->y += directionY * moveDistance;
        }
    }
};

void DrawIntersectionCircles(int thickness, Color color, int X, int Y) {
    const int radius = 5;
}

bool areEdgesEqual(Edge* edge1, Edge* edge2) {
    // Check if the two edges connect the same two nodes (in any order)
    return (edge1->node1 == edge2->node1 && edge1->node2 == edge2->node2) ||
        (edge1->node1 == edge2->node2 && edge1->node2 == edge2->node1);
}

int main()
{

    srand(static_cast<unsigned int>(time(0)));


    // Create nodes with names
    TrafficIntersection* node1Intersection = new TrafficIntersection(11, 1000, 800, "Clifton", 4);
    TrafficIntersection* node2Intersection = new TrafficIntersection(12, 600, 400, "Saddar", 4);
    TrafficIntersection* node3Intersection = new TrafficIntersection(13, 800, 100, "Defence", 4);
    TrafficIntersection* node4Intersection = new TrafficIntersection(14, 1300, 950, "Gulshan", 4);
    TrafficIntersection* node5Intersection = new TrafficIntersection(15, 1800, 390, "Korangi", 3);
    TrafficIntersection* node6Intersection = new TrafficIntersection(16, 1860, 490, "Malir", 3);
    TrafficIntersection* node7Intersection = new TrafficIntersection(17, 1700, 490, "ShahFaisal", 3);
    TrafficIntersection* node8Intersection = new TrafficIntersection(18, 1600, 450, "Nazimabad", 3);
    TrafficIntersection* node9Intersection = new TrafficIntersection(19, 1200, 530, "Liaquatabad", 3);
    TrafficIntersection* node10Intersection = new TrafficIntersection(20, 1000, 100, "Orangi", 3);
    TrafficIntersection* node11Intersection = new TrafficIntersection(21, 210, 450, "NorthNazimabad", 3);
    TrafficIntersection* node12Intersection = new TrafficIntersection(22, 500, 900, "Kemari", 3);
    TrafficIntersection* node13Intersection = new TrafficIntersection(23, 800, 800, "Garden", 3);
    TrafficIntersection* node14Intersection = new TrafficIntersection(24, 1600, 800, "UniversityRoad", 3);



    TrafficIntersection* intersections[14] = {
     node1Intersection,
     node2Intersection,
     node3Intersection,
     node4Intersection,
     node5Intersection,
     node6Intersection,
     node7Intersection,
     node8Intersection,
     node9Intersection,
     node10Intersection,
     node11Intersection,
     node12Intersection,
     node13Intersection,
     node14Intersection
    };

    Node* node1 = new Node(1, 200, 200, "JamshedTown");
    Node* node2 = new Node(2, 400, 600, "GardenEast");
    Node* node3 = new Node(3, 1200, 250, "MemonSociety");
    Node* node4 = new Node(4, 830, 500, "FederalBArea");
    Node* node5 = new Node(5, 1000, 1000, "LalKothi");
    Node* node6 = new Node(6, 1200, 1070, "Gulshan-e-Iqbal");
    Node* node7 = new Node(7, 1850, 860, "Kharadar");
    Node* node8 = new Node(8, 1200, 800, "MohammadAliSociety");
    Node* node9 = new Node(9, 1400, 400, "Dhoraji");
    Node* node10 = new Node(10, 1000, 600, "FaisalBase");


    Node* node11 = dynamic_cast<Node*>(node1Intersection);
    Node* node12 = dynamic_cast<Node*>(node2Intersection);
    Node* node13 = dynamic_cast<Node*>(node3Intersection);
    Node* node14 = dynamic_cast<Node*>(node4Intersection);
    Node* node15 = dynamic_cast<Node*>(node5Intersection);
    Node* node16 = dynamic_cast<Node*>(node6Intersection);
    Node* node17 = dynamic_cast<Node*>(node7Intersection);
    Node* node18 = dynamic_cast<Node*>(node8Intersection);
    Node* node19 = dynamic_cast<Node*>(node9Intersection);
    Node* node20 = dynamic_cast<Node*>(node10Intersection);
    Node* node21 = dynamic_cast<Node*>(node11Intersection);
    Node* node22 = dynamic_cast<Node*>(node12Intersection);
    Node* node23 = dynamic_cast<Node*>(node13Intersection);
    Node* node24 = dynamic_cast<Node*>(node14Intersection);
    

    node1->addNeighbor(node21);
    node1->addNeighbor(node13);
    node12->addNeighbor(node21);
    node12->addNeighbor(node2);
    node12->addNeighbor(node13);
    node12->addNeighbor(node4);
    node21->addNeighbor(node22);
    node4->addNeighbor(node19);
    node13->addNeighbor(node3);
    node13->addNeighbor(node20);
    node20->addNeighbor(node3);
    node20->addNeighbor(node15);
    node22->addNeighbor(node6);
    node23->addNeighbor(node5);
    node22->addNeighbor(node23);
    node23->addNeighbor(node11);
    node11->addNeighbor(node5);
    node11->addNeighbor(node8);
    node11->addNeighbor(node10);
    node10->addNeighbor(node19);
    node14->addNeighbor(node6);
    node14->addNeighbor(node7);
    node14->addNeighbor(node24);
    node14->addNeighbor(node8);
    node19->addNeighbor(node9);
    node24->addNeighbor(node17);
    node24->addNeighbor(node16);
    node9->addNeighbor(node18);
    node18->addNeighbor(node17);
    node18->addNeighbor(node15);
    node17->addNeighbor(node16);
    node16->addNeighbor(node15);


    vector<Edge*> uniqueEdges;
    unordered_set<string> edgeSet; // To ensure edges are added only once

    Node* arrayOfNodes[] = {
    node1, node2, node3, node4, node5, node6,
    node7, node8, node9, node10, node11, node12,
    node13, node14, node15, node16, node17, node18,
    node19, node20, node21, node22, node23, node24
    };


    Vehicle v6(1, "Rickshaw", node11, node1);
    Vehicle v2(2, "Car", node13, node24);
    Vehicle v3(3, "Bus", node15, node2);
    Vehicle v4(4, "Bike", node4, node18);
    Vehicle v5(5, "Truck", node21, node19);

    v6.speed = 4;
    v3.speed = 2;
    v5.speed = 1;

    vector<Vehicle> vehicles;
    vehicles.push_back(v6);
    vehicles.push_back(v2);
    vehicles.push_back(v3);
    vehicles.push_back(v4);
    vehicles.push_back(v5);

    /*Image image = LoadImage("car.png");

    Texture2D texture = LoadTextureFromImage(image);*/


    Vector2 buttonPosition = { 120, 950 };
    Vector2 buttonSize = { 150, 50 };
    bool buttonPressed = false;


    for (Node* node : arrayOfNodes)
    {
        for (Edge* edge : node->edges)
        {
            // Create a unique key for the edge (e.g., "1-2" or "2-1")
            int id1 = edge->node1->id;
            int id2 = edge->node2->id;
            string edgeKey = id1 < id2 ? to_string(id1) + "-" + to_string(id2)
                : to_string(id2) + "-" + to_string(id1);

            // Add to uniqueEdges if not already present
            if (edgeSet.find(edgeKey) == edgeSet.end())
            {
                uniqueEdges.push_back(edge);
                edgeSet.insert(edgeKey);
            }
        }
    }

    for (Edge* edge : uniqueEdges) {
        edge->no_of_agents = (rand()%edge->max_traffic);
    }

    

    int choice;

    std::cout << "Welcome to the Traffic Congestion Control System\n";

    while (true) {
        std::cout << "Please select the vehicle you are using:\n";
        std::cout << "1. Rickshaw\n";
        std::cout << "2. Car\n";
        std::cout << "3. Bike\n";
        std::cout << "4. Truck\n";
        std::cout << "5. Bike\n";
        std::cout << "Enter -1 to exit the program.\n";

        std::cout << "Enter the number corresponding to your vehicle: ";
        std::cin >> choice;

        if (choice == -1) {
            std::cout << "Exiting the program...\n";
            return 0;
            
        }

        if (choice >= 1 && choice <= 5) {
            switch (choice) {
            case 1:
                std::cout << "You selected Rickshaw.\n";
                break;
            case 2:
                std::cout << "You selected Car.\n";
                break;
            case 3:
                std::cout << "You selected Bike.\n";
                break;
            case 4:
                std::cout << "You selected Truck.\n";
                break;
            case 5:
                std::cout << "You selected Other vehicle.\n";
                break;
            }
            break;
        }
        else {
            std::cout << "Invalid choice, please select a valid option.\n";
        }
    }

    const char* locations[] = {
        "JamshedTown", "GardenEast", "MemonSociety", "FederalBArea", "LalKothi",
        "Gulshan-e-Iqbal", "Kharadar", "MohammadAliSociety", "Dhoraji", "FaisalBase",
        "Clifton", "Saddar", "Defence", "Gulshan", "Korangi",
        "Malir", "ShahFaisal", "Nazimabad", "Liaquatabad", "Orangi",
        "NorthNazimabad", "Kemari", "Garden", "UniversityRoad"
    };

    const char* vehicleTypes[] = {
    "Rickshaw", "Car", "Bike", "Truck", "Other"
    };

    int start, end;

    while (true) {
        std::cout << "Please select a starting point (1-24):\n";
        for (int i = 0; i < 24; i++) {
            std::cout << (i + 1) << ". " << locations[i] << "\n";
        }

        std::cout << "Enter the number of your starting point: ";
        std::cin >> start;

        std::cout << "Enter the number of your destination point (1-24): ";
        std::cin >> end;

        if (start < 1 || start > 24 || end < 1 || end > 24) {
            std::cout << "Invalid input, please select numbers between 1 and 24.\n";
            continue;
        }

        if (start == end) {
            std::cout << "Starting point and destination cannot be the same. Please try again.\n";
            continue;
        }

        std::cout << "You selected the starting point: " << locations[start - 1] << "\n";
        std::cout << "You selected the destination point: " << locations[end - 1] << "\n";

        break;
    }

    Vehicle mainVehicle(0, vehicleTypes[choice - 1], arrayOfNodes[start - 1], arrayOfNodes[end - 1]);
    mainVehicle.color = ORANGE;
    mainVehicle.speed = 4.5;
    vehicles.push_back(mainVehicle);



    InitWindow(1920, 1080, "Traffic Congestion System");

    Image trafficLight = LoadImage("C:\\Users\\verti\\Downloads\\traffic-light.png");  // Corrected path
    Texture2D textureLight = LoadTextureFromImage(trafficLight);

    SetTargetFPS(10);
    int offsetX = -37;
    int offsetY = -50;
    int i = 0;

    while (!WindowShouldClose()) {


 /*   uniqueEdges.push_back(findEdge(node13, node20));*/

        i++;
        if (i == 100) {
            
            for (TrafficIntersection* intersection : intersections) {
                intersection->changeLights();
            }
            i = 0;
        }
        BeginDrawing();
        ClearBackground(RAYWHITE);

        for (TrafficIntersection* intersection : intersections) {
            DrawTextureEx(textureLight, Vector2{ intersection->x + offsetX -20, intersection->y + offsetY + 5 }, 0.0f, 0.09, WHITE);
        }

        Vector2 mousePosition = GetMousePosition();
        bool isMouseOverButton = CheckCollisionPointRec(mousePosition, Rectangle { buttonPosition.x, buttonPosition.y, buttonSize.x, buttonSize.y });

        if (isMouseOverButton && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            buttonPressed = true;
        }

        if (isMouseOverButton) {
            DrawRectangle(buttonPosition.x, buttonPosition.y, buttonSize.x, buttonSize.y, LIGHTGRAY);
        }
        else {
            DrawRectangle(buttonPosition.x, buttonPosition.y, buttonSize.x, buttonSize.y, DARKGRAY);
        }

        // Draw button text
        DrawText("Add car", buttonPosition.x + 37, buttonPosition.y + 15, 20, BLACK);

        // Display the result of button press
        if (buttonPressed) {
            int k = rand() % 24;
            int j = k;
            while (j == k) {
                j = rand() % 24;
            }
            Vehicle vehicle(1, "Car", arrayOfNodes[k], arrayOfNodes[j]);
            vehicles.push_back(vehicle);
            buttonPressed = false;
        }

        
        for (Node* node: arrayOfNodes) {
            DrawCircle(node->x+offsetX, node->y+ offsetY , 5, RED);
            const char* name = node->name.c_str();
            DrawText(name, node->x + offsetX-30, node->y + offsetY-20, 20, BLACK);
        }
        for (Edge* edge : uniqueEdges) {
            DrawLine(edge->node1->x + offsetX, edge->node1->y + offsetY, edge->node2->x+ offsetX, edge->node2->y + offsetY, RED);
            float midpointX = (edge->node1->x + edge->node2->x) / 2.0f;
            float midpointY = (edge->node1->y + edge->node2->y) / 2.0f;
            string congestionString =  std::to_string(edge->no_of_agents);
            const char* congestion = congestionString.c_str();
            DrawText(congestion, midpointX + offsetX, midpointY + offsetY-20, 25, BLUE);
            
        }
        
        for (Vehicle& v1 : vehicles) {
            if(v1.hasReachedDestination==false){
                DrawRectangle(v1.x + offsetX, v1.y + offsetY, 25, 30, v1.color);
                /DrawTextureEx(texture, Vector2 { v1.x + offsetX, v1.y + offsetY }, 0.0, 0.1, WHITE);/
                
                if (v1.currentNodeToReach->getType() > 2) {
                    if (v1.x == v1.currentNodeToReach->x && v1.y == v1.currentNodeToReach->y) {
                        int i = 0;
                        for (TrafficIntersection* intersection : intersections) {
                            if (v1.currentNodeToReach->x == intersection->x && v1.currentNodeToReach->y == intersection->y) {
                                break;
                            }
                            else {
                                i++;
                            }
                        }
                        int j = 0;
                        for (bool check : intersections[i]->signals) {
                            if (check != false) {
                                break;
                            }
                            else {
                                j++;
                            }
                        }
                        /*cout << intersections[i]->edges[j] << "\n" << v1.currentEdge;
                        cout << "\n\n\n\n";*/
                        if (areEdgesEqual(intersections[i]->edges[j], v1.currentEdge)) {
                            if (v1.moveVehicle()) {
                                v1.hasReachedDestination = true;
                            }
                        }
                    }
                    else {
                        if (v1.moveVehicle()) {
                            v1.hasReachedDestination = true;
                        }
                    }
                }
                else {
                    if (v1.moveVehicle()) {
                        v1.hasReachedDestination = true;
                    }
                }
            }
        }
        EndDrawing();
    }
    CloseWindow();

    

    /*Vehicle v1(1, "Rickshaw", node1, node5);
    Vehicle v2(2, "motor", node2, node4);
    cout << v1.x << "  " << v1.y << endl;
    while (true)
    {
        if (v1.moveVehicle())
        {
            break;
        }
        cout << v1.x << "  " << v1.y << endl;
    }
    cout << "\n\n\n\n";*/

    return 0;
}
