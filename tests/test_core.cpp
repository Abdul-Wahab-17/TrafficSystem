// Unit tests for the road graph, congestion-aware A* and signal logic (no window needed).
#include <cstdio>
#include <cstdlib>

#include "graph.hpp"
#include "vehicle.hpp"

static int failures = 0;
#define CHECK(cond)                                                                    \
    do {                                                                               \
        if (!(cond)) {                                                                 \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);                \
            failures++;                                                                \
        }                                                                              \
    } while (0)

// A --- B --- D     (A-B-D is short; A-C-D is a longer detour)
//  \         /
//   ---- C --
struct Diamond {
    Node a{1, 0, 0, "A"}, b{2, 100, 0, "B"}, c{3, 50, 120, "C"}, d{4, 200, 0, "D"};
    Diamond() {
        a.addNeighbor(&b);
        b.addNeighbor(&d);
        a.addNeighbor(&c);
        c.addNeighbor(&d);
    }
};

static void shortest_path_is_chosen() {
    Diamond g;
    Edge* first = aStar(&g.a, &g.d);
    CHECK(first != nullptr);
    CHECK(first->node2 == &g.b);  // direct route through B
}

static void congestion_changes_the_route() {
    Diamond g;
    findEdge(&g.a, &g.b)->no_of_agents = 50;  // jam the short road
    Edge* first = aStar(&g.a, &g.d);
    CHECK(first != nullptr);
    CHECK(first->node2 == &g.c);  // detour through C
}

static void no_route_returns_null() {
    Node a(1, 0, 0, "A"), b(2, 10, 0, "B");  // not connected
    CHECK(aStar(&a, &b) == nullptr);
    CHECK(aStar(&a, &a) == nullptr);
}

static void edge_capacity_scales_with_size() {
    Node a(1, 0, 0, "A"), b(2, 300, 400, "B");
    Edge e(&a, &b, 3.2f);
    CHECK(e.length > 499.9f && e.length < 500.1f);
    CHECK(e.max_traffic == static_cast<int>(3.2f * 500 * 0.123f));
}

static void signals_cycle_through_every_approach() {
    TrafficIntersection x(1, 0, 0, "X", 4);
    for (int expected = 1; expected <= 4; expected++) {
        int green = 0, index = -1;
        for (int i = 0; i < 4; i++) {
            if (x.signals[i]) { green++; index = i; }
        }
        CHECK(green == 1);               // exactly one approach is green at a time
        CHECK(index == (expected - 1) % 4);
        x.changeLights();
    }
    TrafficIntersection t(2, 0, 0, "T", 3);
    t.changeLights(); t.changeLights(); t.changeLights();
    CHECK(t.signals[0] && !t.signals[1] && !t.signals[2]);  // back to the start after 3
}

static void vehicle_reaches_its_destination() {
    Diamond g;
    Vehicle v(1, "Car", &g.a, &g.d);
    v.speed = 8;
    bool arrived = false;
    for (int i = 0; i < 500 && !arrived; i++) arrived = v.moveVehicle();
    CHECK(arrived);
    CHECK(v.x == g.d.x && v.y == g.d.y);
}

int main() {
    shortest_path_is_chosen();
    congestion_changes_the_route();
    no_route_returns_null();
    edge_capacity_scales_with_size();
    signals_cycle_through_every_approach();
    vehicle_reaches_its_destination();
    if (failures) { std::printf("%d check(s) failed\n", failures); return 1; }
    std::printf("all tests passed\n");
    return 0;
}
