#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <functional>

// Coord structure
struct Coord {
    double latitude;
    double longitude;

    // Define equality operator for unordered_map compatibility
    bool operator==(const Coord& other) const {
        return latitude == other.latitude && longitude == other.longitude;
    }
};

// Custom hash function for Coord
namespace std {
    template<>
    struct hash<Coord> {
        std::size_t operator()(const Coord& coord) const noexcept {
            std::size_t h1 = std::hash<double>{}(coord.latitude);
            std::size_t h2 = std::hash<double>{}(coord.longitude);
            return h1 ^ (h2 << 1); // Combine the two hash values
        }
    };
}

// RoadSegment structure
struct RoadSegment {
    Coord start;
    Coord end;
    double travelTime;  // Travel time based on real-time data
};

// Example usage
int main() {
    // Example coordinates
    Coord start = {52.379189, 4.899431};
    Coord destination = {52.3676, 4.9041};

    // Create unordered_map with Coord as key
    std::unordered_map<Coord, std::vector<RoadSegment>> graph;

    // Populate with example data
    graph[start] = {{start, destination, 5.0}};

    // Access example data
    for (const auto& segment : graph[start]) {
        std::cout << "Segment from (" << segment.start.latitude << ", " << segment.start.longitude << ") "
                  << "to (" << segment.end.latitude << ", " << segment.end.longitude << ") "
                  << "with travel time: " << segment.travelTime << std::endl;
    }

    return 0;
}
