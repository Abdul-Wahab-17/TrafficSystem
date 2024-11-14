#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <string>
#include <algorithm>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>
 // Requires the JSON for Modern C++ library or a similar JSON parser

using namespace std;

struct Coordinate {
    double x, y;

    bool operator==(const Coordinate& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Coordinate& other) const {
        return !(*this == other);
    }

    bool operator<(const Coordinate& other) const {
        return x < other.x || (x == other.x && y < other.y);
    }
};

// Custom hash function for Coordinate struct
namespace std {
    template<>
    struct hash<Coordinate> {
        size_t operator()(const Coordinate& coord) const {
            return hash<double>()(coord.x) ^ hash<double>()(coord.y);
        }
    };
}

// Function to handle response data from cURL
size_t WriteCallback(void* contents, size_t size, size_t nmemb, string* output) {
    size_t totalSize = size * nmemb;
    output->append((char*)contents, totalSize);
    return totalSize;
}

// Euclidean distance (heuristic function)
double euclideanDistance(const Coordinate& a, const Coordinate& b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// Reverse Geocoding: Convert coordinates to place name
string reverseGeocode(const string& apiKey, double latitude, double longitude) {
    CURL* curl;
    CURLcode res;
    string response;

    // Build the URL for reverse geocoding
    string url = "https://api.tomtom.com/search/2/reverseGeocode/" +
                 to_string(latitude) + "%2C" + to_string(longitude) +
                 ".json?key=" + apiKey;

    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

        res = curl_easy_perform(curl);

        if (res != CURLE_OK) {
            cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << endl;
        }

        curl_easy_cleanup(curl);
    }

    curl_global_cleanup();

    // Parse JSON response to get the place name
    Json::Reader reader;
    Json::Value jsonData;
    string placeName;

    if (reader.parse(response, jsonData) && jsonData.isMember("addresses")) {
        placeName = jsonData["addresses"][0]["address"]["freeformAddress"].asString();
    }

    return placeName;
}

// A* search algorithm to find the optimal path
vector<Coordinate> aStar(const Coordinate& start, const Coordinate& goal) {
    // Directions for moving in 8 possible ways (up, down, left, right, diagonals)
    vector<Coordinate> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    // Priority queue for open set (min-heap based on f-cost)
    priority_queue<pair<double, Coordinate>, vector<pair<double, Coordinate>>, greater<>> openSet;

    // Maps for g-scores and cameFrom path reconstruction
    unordered_map<Coordinate, double> gScore;
    unordered_map<Coordinate, Coordinate> cameFrom;

    // Initialize start node
    openSet.emplace(0.0, start);
    gScore[start] = 0.0;

    while (!openSet.empty()) {
        // Get node with lowest f-cost
        auto [_, current] = openSet.top();
        openSet.pop();

        // If we reached the goal, reconstruct and return the path
        if (current == goal) {
            vector<Coordinate> path;
            while (cameFrom.find(current) != cameFrom.end()) {
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);  // Add start to path
            reverse(path.begin(), path.end());  // Reverse to get path from start to goal
            return path;
        }

        // Explore neighbors
        for (const auto& dir : directions) {
            Coordinate neighbor{current.x + dir.x, current.y + dir.y};

            // Calculate tentative gScore for neighbor
            double tentativeGScore = gScore[current] + euclideanDistance(current, neighbor);

            // If a shorter path to neighbor is found
            if (tentativeGScore < gScore[neighbor] || gScore.find(neighbor) == gScore.end()) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeGScore;
                double fScore = tentativeGScore + euclideanDistance(neighbor, goal);  // f = g + h
                openSet.emplace(fScore, neighbor);
            }
        }
    }

    // If no path found, return an empty vector
    return {};
}

int main() {
    string apiKey = "YOUR_TOMTOM_API_KEY";
    double startLat, startLon, goalLat, goalLon;

    // Input source and destination coordinates
    cout << "Enter start coordinates (latitude longitude): ";
    cin >> startLat >> startLon;

    cout << "Enter destination coordinates (latitude longitude): ";
    cin >> goalLat >> goalLon;

    // Define start and goal coordinates
    Coordinate start{startLat, startLon};
    Coordinate goal{goalLat, goalLon};

    // Perform A* search
    vector<Coordinate> path = aStar(start, goal);

    // Output the path and its corresponding place names
    if (!path.empty()) {
        cout << "Path from start to goal:" << endl;
        for (const auto& coord : path) {
            string placeName = reverseGeocode(apiKey, coord.x, coord.y);
            cout << "(" << coord.x << ", " << coord.y << ") -> " << placeName << endl;
        }
    } else {
        cout << "No path found." << endl;
    }

    return 0;
}
