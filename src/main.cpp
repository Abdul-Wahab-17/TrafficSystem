// Traffic Congestion Control System
//
// A map of Karachi neighbourhoods joined by two-way roads. Vehicles route with
// A*, where each road's cost is its length plus a penalty for every vehicle on
// it, and they re-plan at every node. Intersections cycle their signals and
// vehicles wait at red lights.
//
// Usage: traffic_system [vehicle start destination]
//   With no arguments an interactive menu asks for these.

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

#include "graph.hpp"
#include "raylib.h"
#include "vehicle.hpp"

using namespace std;

namespace {

constexpr int kWorldWidth = 1920;
constexpr int kWorldHeight = 1080;
constexpr int kOffsetX = -37;
constexpr int kOffsetY = -50;
constexpr int kFramesPerLightChange = 100;

const char* kVehicleTypes[] = { "Rickshaw", "Car", "Bike", "Truck", "Other" };

int readInt()
{
    int value;
    while (!(cin >> value))
    {
        if (cin.eof())
        {
            exit(0);
        }
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Please enter a number: ";
    }
    return value;
}

int askVehicle()
{
    while (true)
    {
        cout << "Please select the vehicle you are using:\n";
        for (int i = 0; i < 5; i++)
        {
            cout << (i + 1) << ". " << kVehicleTypes[i] << "\n";
        }
        cout << "Enter -1 to exit the program.\n";
        cout << "Enter the number corresponding to your vehicle: ";

        int choice = readInt();
        if (choice == -1)
        {
            cout << "Exiting the program...\n";
            exit(0);
        }
        if (choice >= 1 && choice <= 5)
        {
            cout << "You selected " << kVehicleTypes[choice - 1] << ".\n";
            return choice;
        }
        cout << "Invalid choice, please select a valid option.\n";
    }
}

void askRoute(const vector<Node*>& nodes, int& start, int& end)
{
    const int count = static_cast<int>(nodes.size());
    while (true)
    {
        cout << "Please select a starting point (1-" << count << "):\n";
        for (int i = 0; i < count; i++)
        {
            cout << (i + 1) << ". " << nodes[i]->name << "\n";
        }
        cout << "Enter the number of your starting point: ";
        start = readInt();
        cout << "Enter the number of your destination point (1-" << count << "): ";
        end = readInt();

        if (start < 1 || start > count || end < 1 || end > count)
        {
            cout << "Invalid input, please select numbers between 1 and " << count << ".\n";
            continue;
        }
        if (start == end)
        {
            cout << "Starting point and destination cannot be the same. Please try again.\n";
            continue;
        }
        return;
    }
}

Vector2 screenPos(const Node* node)
{
    return { node->x + kOffsetX, node->y + kOffsetY };
}

// Green when a road is empty, red when it is at capacity
Color congestionColor(const Edge* edge)
{
    float load = edge->max_traffic > 0 ? static_cast<float>(edge->no_of_agents) / edge->max_traffic : 0.0f;
    load = load < 0 ? 0 : (load > 1 ? 1 : load);
    return Color{ static_cast<unsigned char>(60 + 195 * load), static_cast<unsigned char>(180 - 140 * load), 60, 255 };
}

// A small light on each approach to the intersection, green for the direction that may go
void drawSignals(const TrafficIntersection* intersection)
{
    Vector2 center = screenPos(intersection);
    for (size_t i = 0; i < intersection->edges.size() && i < intersection->signals.size(); i++)
    {
        Vector2 other = screenPos(intersection->neighbors[i]);
        Vector2 dir = { other.x - center.x, other.y - center.y };
        float len = sqrtf(dir.x * dir.x + dir.y * dir.y);
        Vector2 light = { center.x + dir.x / len * 22, center.y + dir.y / len * 22 };
        DrawCircleV(light, 6, BLACK);
        DrawCircleV(light, 5, intersection->signals[i] ? GREEN : RED);
    }
}

} // namespace

int main(int argc, char** argv)
{
    srand(static_cast<unsigned int>(time(nullptr)));

    // Intersections (ids 11-24)
    vector<TrafficIntersection*> intersections = {
        new TrafficIntersection(11, 1000, 800, "Clifton", 4),
        new TrafficIntersection(12, 600, 400, "Saddar", 4),
        new TrafficIntersection(13, 800, 100, "Defence", 4),
        new TrafficIntersection(14, 1300, 950, "Gulshan", 4),
        new TrafficIntersection(15, 1800, 390, "Korangi", 3),
        new TrafficIntersection(16, 1860, 490, "Malir", 3),
        new TrafficIntersection(17, 1700, 490, "ShahFaisal", 3),
        new TrafficIntersection(18, 1600, 450, "Nazimabad", 3),
        new TrafficIntersection(19, 1200, 530, "Liaquatabad", 3),
        new TrafficIntersection(20, 1000, 100, "Orangi", 3),
        new TrafficIntersection(21, 210, 450, "NorthNazimabad", 3),
        new TrafficIntersection(22, 500, 900, "Kemari", 3),
        new TrafficIntersection(23, 800, 800, "Garden", 3),
        new TrafficIntersection(24, 1600, 800, "UniversityRoad", 3),
    };

    // Plain locations (ids 1-10), followed by the intersections
    vector<Node*> nodes = {
        new Node(1, 200, 200, "JamshedTown"),
        new Node(2, 400, 600, "GardenEast"),
        new Node(3, 1200, 250, "MemonSociety"),
        new Node(4, 830, 500, "FederalBArea"),
        new Node(5, 1000, 1000, "LalKothi"),
        new Node(6, 1200, 1070, "Gulshan-e-Iqbal"),
        new Node(7, 1850, 860, "Kharadar"),
        new Node(8, 1200, 800, "MohammadAliSociety"),
        new Node(9, 1400, 400, "Dhoraji"),
        new Node(10, 1000, 600, "FaisalBase"),
    };
    nodes.insert(nodes.end(), intersections.begin(), intersections.end());

    // node(id) -> the node with that id
    auto node = [&](int id) { return nodes[id - 1]; };

    // Roads. Each intersection gets exactly as many roads as it has signals.
    const int roads[][2] = {
        {1, 21}, {1, 13}, {12, 21}, {12, 2}, {12, 13}, {12, 4}, {21, 22}, {4, 19},
        {13, 3}, {13, 20}, {20, 3}, {20, 15}, {22, 6}, {23, 5}, {22, 23}, {23, 11},
        {11, 5}, {11, 8}, {11, 10}, {10, 19}, {14, 6}, {14, 7}, {14, 24}, {14, 8},
        {19, 9}, {24, 17}, {24, 16}, {9, 18}, {18, 17}, {18, 15}, {17, 16}, {16, 15},
    };
    for (const auto& road : roads)
    {
        node(road[0])->addNeighbor(node(road[1]));
    }

    // One Edge per road for drawing (each road is stored in both directions)
    vector<Edge*> uniqueEdges;
    unordered_set<string> edgeSet;
    for (Node* n : nodes)
    {
        for (Edge* edge : n->edges)
        {
            int id1 = edge->node1->id;
            int id2 = edge->node2->id;
            string key = id1 < id2 ? to_string(id1) + "-" + to_string(id2) : to_string(id2) + "-" + to_string(id1);
            if (edgeSet.insert(key).second)
            {
                uniqueEdges.push_back(edge);
            }
        }
    }

    // Background traffic: a random load on every road...
    for (Edge* edge : uniqueEdges)
    {
        if (edge->max_traffic > 0)
        {
            edge->no_of_agents = rand() % edge->max_traffic;
        }
    }

    // ...and five other vehicles driving across the city
    vector<Vehicle> vehicles = {
        Vehicle(1, "Rickshaw", node(11), node(1)),
        Vehicle(2, "Car", node(13), node(24)),
        Vehicle(3, "Bus", node(15), node(2)),
        Vehicle(4, "Bike", node(4), node(18)),
        Vehicle(5, "Truck", node(21), node(19)),
    };
    vehicles[0].speed = 4;
    vehicles[2].speed = 2;
    vehicles[4].speed = 1;

    // Your vehicle: from the command line, or ask
    int choice, start, end;
    if (argc == 4)
    {
        choice = atoi(argv[1]);
        start = atoi(argv[2]);
        end = atoi(argv[3]);
        const int count = static_cast<int>(nodes.size());
        if (choice < 1 || choice > 5 || start < 1 || start > count || end < 1 || end > count || start == end)
        {
            cerr << "Usage: " << argv[0] << " [vehicle(1-5) start(1-" << count << ") destination(1-" << count << ")]\n";
            return 1;
        }
    }
    else
    {
        cout << "Welcome to the Traffic Congestion Control System\n";
        choice = askVehicle();
        askRoute(nodes, start, end);
    }
    cout << "Route: " << nodes[start - 1]->name << " -> " << nodes[end - 1]->name << "\n";

    Vehicle mainVehicle(0, kVehicleTypes[choice - 1], nodes[start - 1], nodes[end - 1]);
    mainVehicle.color = ORANGE;
    mainVehicle.speed = 4.5f;
    vehicles.push_back(mainVehicle);
    const size_t mainIndex = vehicles.size() - 1;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(1600, 900, "Traffic Congestion System");
    SetTargetFPS(10);

    // Scale the 1920x1080 map to fit the window
    Camera2D camera = {};

    const Rectangle addButton = { 20, 20, 150, 44 };
    int frame = 0;

    while (!WindowShouldClose())
    {
        if (++frame == kFramesPerLightChange)
        {
            for (TrafficIntersection* intersection : intersections)
            {
                intersection->changeLights();
            }
            frame = 0;
        }

        // "Add car" spawns a car between two random locations
        bool hover = CheckCollisionPointRec(GetMousePosition(), addButton);
        if (hover && IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
        {
            int k = rand() % nodes.size();
            int j = k;
            while (j == k)
            {
                j = rand() % nodes.size();
            }
            vehicles.emplace_back(static_cast<int>(vehicles.size()), "Car", nodes[k], nodes[j]);
        }

        // Move vehicles. At an intersection a vehicle waits until its road has a green light.
        for (Vehicle& v : vehicles)
        {
            if (v.hasReachedDestination)
            {
                continue;
            }
            bool atIntersection = v.currentNodeToReach->getType() > 2 &&
                                  v.x == v.currentNodeToReach->x && v.y == v.currentNodeToReach->y;
            if (atIntersection)
            {
                auto* intersection = static_cast<TrafficIntersection*>(v.currentNodeToReach);
                size_t green = 0;
                while (green < intersection->signals.size() && !intersection->signals[green])
                {
                    green++;
                }
                if (!areEdgesEqual(intersection->edges[green], v.currentEdge))
                {
                    continue; // red light
                }
            }
            if (v.moveVehicle())
            {
                v.hasReachedDestination = true;
            }
        }

        float scaleX = static_cast<float>(GetScreenWidth()) / kWorldWidth;
        float scaleY = static_cast<float>(GetScreenHeight()) / kWorldHeight;
        camera.zoom = scaleX < scaleY ? scaleX : scaleY;

        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode2D(camera);

        for (Edge* edge : uniqueEdges)
        {
            Vector2 a = screenPos(edge->node1);
            Vector2 b = screenPos(edge->node2);
            DrawLineEx(a, b, 4, congestionColor(edge));
            DrawText(to_string(edge->no_of_agents).c_str(), (a.x + b.x) / 2, (a.y + b.y) / 2 - 20, 25, DARKBLUE);
        }
        for (TrafficIntersection* intersection : intersections)
        {
            drawSignals(intersection);
        }
        for (Node* n : nodes)
        {
            Vector2 p = screenPos(n);
            DrawCircleV(p, n->getType() > 2 ? 8 : 5, n->getType() > 2 ? DARKGRAY : RED);
            DrawText(n->name.c_str(), p.x - 30, p.y - 30, 20, BLACK);
        }
        for (const Vehicle& v : vehicles)
        {
            if (!v.hasReachedDestination)
            {
                DrawRectangle(v.x + kOffsetX - 10, v.y + kOffsetY - 12, 20, 24, v.color);
            }
        }

        EndMode2D();

        // Screen-space UI
        DrawRectangleRec(addButton, hover ? LIGHTGRAY : DARKGRAY);
        DrawText("Add car", addButton.x + 35, addButton.y + 12, 20, hover ? BLACK : RAYWHITE);
        DrawText(TextFormat("Your %s: %s -> %s%s", mainVehicle.type.c_str(), nodes[start - 1]->name.c_str(),
                            nodes[end - 1]->name.c_str(), vehicles[mainIndex].hasReachedDestination ? "  (arrived)" : ""),
                 190, 32, 20, DARKGRAY);
        DrawText("Orange: you   Blue: other traffic   Numbers: vehicles on each road", 20, GetScreenHeight() - 30, 18, GRAY);

        EndDrawing();
    }
    CloseWindow();
    return 0;
}
