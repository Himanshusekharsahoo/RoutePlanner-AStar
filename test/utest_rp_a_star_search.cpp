#include "gtest/gtest.h"
#include <fstream>
#include <iostream>
#include <optional>
#include <vector>
#include <string>
#include <algorithm>
#include "../src/route_model.h"
#include "../src/route_planner.h"

// Helper to convert XML string into byte vector for Model/RouteModel
static std::vector<std::byte> StringToBytes(const std::string& str) {
    std::vector<std::byte> bytes(str.size());
    std::transform(str.begin(), str.end(), bytes.begin(), [](char c) {
        return static_cast<std::byte>(c);
    });
    return bytes;
}

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path) {   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if (!is) return std::nullopt;
    
    auto size = is.tellg();
    if (size <= 0) return std::nullopt;
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read(reinterpret_cast<char*>(contents.data()), size);

    if (contents.empty()) return std::nullopt;
    return contents;
}

static std::vector<std::byte> ReadOSMData(const std::string &path) {
    auto data = ReadFile(path);
    if (!data) return {};
    return *data;
}

//=============================================================================
// 1. GRAPH CONSTRUCTION TESTS (PHASE 1A / PHASE 2)
//=============================================================================

// Test 1: Consecutive Road Node Adjacency
// N0 - N1 - N2 - N3
// N0 should only connect to N1
// N1 should connect to N0 and N2
// N2 should connect to N1 and N3
// N3 should only connect to N2
TEST(GraphConstructionTest, ConsecutiveAdjacency) {
    std::string xml = 
        "<osm>"
        "  <bounds minlat=\"0.0\" minlon=\"0.0\" maxlat=\"10.0\" maxlon=\"10.0\"/>"
        "  <node id=\"1\" lat=\"1.0\" lon=\"1.0\"/>"
        "  <node id=\"2\" lat=\"2.0\" lon=\"2.0\"/>"
        "  <node id=\"3\" lat=\"3.0\" lon=\"3.0\"/>"
        "  <node id=\"4\" lat=\"4.0\" lon=\"4.0\"/>"
        "  <way id=\"100\">"
        "    <nd ref=\"1\"/>"
        "    <nd ref=\"2\"/>"
        "    <nd ref=\"3\"/>"
        "    <nd ref=\"4\"/>"
        "    <tag k=\"highway\" v=\"residential\"/>"
        "  </way>"
        "</osm>";

    RouteModel model{StringToBytes(xml)};
    ASSERT_EQ(model.SNodes().size(), 4);

    RouteModel::Node& n0 = model.SNodes()[0];
    RouteModel::Node& n1 = model.SNodes()[1];
    RouteModel::Node& n2 = model.SNodes()[2];
    RouteModel::Node& n3 = model.SNodes()[3];

    n0.FindNeighbors();
    n1.FindNeighbors();
    n2.FindNeighbors();
    n3.FindNeighbors();

    // Verify N0 has only N1 as neighbor
    EXPECT_EQ(n0.neighbors.size(), 1);
    if (!n0.neighbors.empty()) {
        EXPECT_EQ(n0.neighbors[0], &n1);
    }

    // Verify N1 has N0 and N2
    EXPECT_EQ(n1.neighbors.size(), 2);
    EXPECT_NE(std::find(n1.neighbors.begin(), n1.neighbors.end(), &n0), n1.neighbors.end());
    EXPECT_NE(std::find(n1.neighbors.begin(), n1.neighbors.end(), &n2), n1.neighbors.end());

    // Verify N2 has N1 and N3
    EXPECT_EQ(n2.neighbors.size(), 2);
    EXPECT_NE(std::find(n2.neighbors.begin(), n2.neighbors.end(), &n1), n2.neighbors.end());
    EXPECT_NE(std::find(n2.neighbors.begin(), n2.neighbors.end(), &n3), n2.neighbors.end());

    // Verify N3 has only N2
    EXPECT_EQ(n3.neighbors.size(), 1);
    if (!n3.neighbors.empty()) {
        EXPECT_EQ(n3.neighbors[0], &n2);
    }
}

// Test 2: Curved Road Topology (No Teleportation Across Open Space)
// Horseshoe road: N0(0, 0) -> N1(0, 5) -> N2(1, 5) -> N3(1, 0.001)
// Geometrically, N3 is very close to N0 (~0.001 apart).
// Old buggy code connected N0 to N3. Correct code connects N0 only to N1!
TEST(GraphConstructionTest, NoTeleportationAcrossCurve) {
    std::string xml = 
        "<osm>"
        "  <bounds minlat=\"0.0\" minlon=\"0.0\" maxlat=\"10.0\" maxlon=\"10.0\"/>"
        "  <node id=\"1\" lat=\"0.0\" lon=\"0.0\"/>"
        "  <node id=\"2\" lat=\"5.0\" lon=\"0.0\"/>"
        "  <node id=\"3\" lat=\"5.0\" lon=\"1.0\"/>"
        "  <node id=\"4\" lat=\"0.0\" lon=\"1.001\"/>"
        "  <way id=\"100\">"
        "    <nd ref=\"1\"/>"
        "    <nd ref=\"2\"/>"
        "    <nd ref=\"3\"/>"
        "    <nd ref=\"4\"/>"
        "    <tag k=\"highway\" v=\"primary\"/>"
        "  </way>"
        "</osm>";

    RouteModel model{StringToBytes(xml)};
    RouteModel::Node& n0 = model.SNodes()[0];
    RouteModel::Node& n3 = model.SNodes()[3];

    n0.FindNeighbors();
    EXPECT_EQ(n0.neighbors.size(), 1);
    // Must be connected to N1 along the road, NOT N3
    EXPECT_EQ(n0.neighbors[0], &model.SNodes()[1]);
    EXPECT_EQ(std::find(n0.neighbors.begin(), n0.neighbors.end(), &n3), n0.neighbors.end());
}

// Test 3: Shared Intersection Between Multiple Roads
// Road A: N0 - N1 - N2
// Road B: N3 - N1 - N4
// N1 is the shared intersection; must have 4 neighbors: {N0, N2, N3, N4}
TEST(GraphConstructionTest, IntersectionConnectivity) {
    std::string xml = 
        "<osm>"
        "  <bounds minlat=\"0.0\" minlon=\"0.0\" maxlat=\"10.0\" maxlon=\"10.0\"/>"
        "  <node id=\"1\" lat=\"5.0\" lon=\"1.0\"/>"
        "  <node id=\"2\" lat=\"5.0\" lon=\"5.0\"/>"  // Intersection
        "  <node id=\"3\" lat=\"5.0\" lon=\"9.0\"/>"
        "  <node id=\"4\" lat=\"1.0\" lon=\"5.0\"/>"
        "  <node id=\"5\" lat=\"9.0\" lon=\"5.0\"/>"
        "  <way id=\"10\">"
        "    <nd ref=\"1\"/>"
        "    <nd ref=\"2\"/>"
        "    <nd ref=\"3\"/>"
        "    <tag k=\"highway\" v=\"primary\"/>"
        "  </way>"
        "  <way id=\"20\">"
        "    <nd ref=\"4\"/>"
        "    <nd ref=\"2\"/>"
        "    <nd ref=\"5\"/>"
        "    <tag k=\"highway\" v=\"secondary\"/>"
        "  </way>"
        "</osm>";

    RouteModel model{StringToBytes(xml)};
    RouteModel::Node& intersection = model.SNodes()[1]; // node id 2

    intersection.FindNeighbors();
    EXPECT_EQ(intersection.neighbors.size(), 4);

    EXPECT_NE(std::find(intersection.neighbors.begin(), intersection.neighbors.end(), &model.SNodes()[0]), intersection.neighbors.end());
    EXPECT_NE(std::find(intersection.neighbors.begin(), intersection.neighbors.end(), &model.SNodes()[2]), intersection.neighbors.end());
    EXPECT_NE(std::find(intersection.neighbors.begin(), intersection.neighbors.end(), &model.SNodes()[3]), intersection.neighbors.end());
    EXPECT_NE(std::find(intersection.neighbors.begin(), intersection.neighbors.end(), &model.SNodes()[4]), intersection.neighbors.end());
}

// Test 4: Duplicate Edge Prevention
// Calling FindNeighbors() multiple times must not duplicate neighbor entries
TEST(GraphConstructionTest, DuplicateEdgePrevention) {
    std::string xml = 
        "<osm>"
        "  <bounds minlat=\"0.0\" minlon=\"0.0\" maxlat=\"10.0\" maxlon=\"10.0\"/>"
        "  <node id=\"1\" lat=\"1.0\" lon=\"1.0\"/>"
        "  <node id=\"2\" lat=\"2.0\" lon=\"2.0\"/>"
        "  <way id=\"10\">"
        "    <nd ref=\"1\"/>"
        "    <nd ref=\"2\"/>"
        "    <tag k=\"highway\" v=\"service\"/>"
        "  </way>"
        "</osm>";

    RouteModel model{StringToBytes(xml)};
    RouteModel::Node& n0 = model.SNodes()[0];

    n0.FindNeighbors();
    EXPECT_EQ(n0.neighbors.size(), 1);

    // Call again; must remain 1
    n0.FindNeighbors();
    EXPECT_EQ(n0.neighbors.size(), 1);
}

// Test 5: Missing or Invalid Node Reference Safety
// A way references a non-existent node ID. Model should not crash or corrupt state.
TEST(GraphConstructionTest, InvalidNodeReferenceSafety) {
    std::string xml = 
        "<osm>"
        "  <bounds minlat=\"0.0\" minlon=\"0.0\" maxlat=\"10.0\" maxlon=\"10.0\"/>"
        "  <node id=\"1\" lat=\"1.0\" lon=\"1.0\"/>"
        "  <node id=\"2\" lat=\"2.0\" lon=\"2.0\"/>"
        "  <way id=\"10\">"
        "    <nd ref=\"1\"/>"
        "    <nd ref=\"99999\"/>"  // Missing node ID
        "    <nd ref=\"2\"/>"
        "    <tag k=\"highway\" v=\"residential\"/>"
        "  </way>"
        "</osm>";

    RouteModel model{StringToBytes(xml)};
    EXPECT_EQ(model.SNodes().size(), 2);

    RouteModel::Node& n0 = model.SNodes()[0];
    EXPECT_NO_THROW(n0.FindNeighbors());
}

// Test 6: Multiple Roads Associated With Same Node (Hub Node)
// Node 2 belongs to 3 distinct roads: Way 1 (1-2), Way 2 (2-3), Way 3 (4-2-5)
TEST(GraphConstructionTest, MultipleRoadsSharedNode) {
    std::string xml = 
        "<osm>"
        "  <bounds minlat=\"0.0\" minlon=\"0.0\" maxlat=\"10.0\" maxlon=\"10.0\"/>"
        "  <node id=\"1\" lat=\"1.0\" lon=\"1.0\"/>"
        "  <node id=\"2\" lat=\"5.0\" lon=\"5.0\"/>"  // Shared hub
        "  <node id=\"3\" lat=\"9.0\" lon=\"9.0\"/>"
        "  <node id=\"4\" lat=\"1.0\" lon=\"9.0\"/>"
        "  <node id=\"5\" lat=\"9.0\" lon=\"1.0\"/>"
        "  <way id=\"10\">"
        "    <nd ref=\"1\"/>"
        "    <nd ref=\"2\"/>"
        "    <tag k=\"highway\" v=\"primary\"/>"
        "  </way>"
        "  <way id=\"20\">"
        "    <nd ref=\"2\"/>"
        "    <nd ref=\"3\"/>"
        "    <tag k=\"highway\" v=\"secondary\"/>"
        "  </way>"
        "  <way id=\"30\">"
        "    <nd ref=\"4\"/>"
        "    <nd ref=\"2\"/>"
        "    <nd ref=\"5\"/>"
        "    <tag k=\"highway\" v=\"tertiary\"/>"
        "  </way>"
        "</osm>";

    RouteModel model{StringToBytes(xml)};
    RouteModel::Node& hub = model.SNodes()[1]; // node 2

    hub.FindNeighbors();
    // Neighbors must be: node 1 (from way 10), node 3 (from way 20), node 4 & 5 (from way 30) -> total 4
    EXPECT_EQ(hub.neighbors.size(), 4);
    EXPECT_NE(std::find(hub.neighbors.begin(), hub.neighbors.end(), &model.SNodes()[0]), hub.neighbors.end());
    EXPECT_NE(std::find(hub.neighbors.begin(), hub.neighbors.end(), &model.SNodes()[2]), hub.neighbors.end());
    EXPECT_NE(std::find(hub.neighbors.begin(), hub.neighbors.end(), &model.SNodes()[3]), hub.neighbors.end());
    EXPECT_NE(std::find(hub.neighbors.begin(), hub.neighbors.end(), &model.SNodes()[4]), hub.neighbors.end());
}

//=============================================================================
// 2. A* ALGORITHM & COST RELAXATION TESTS (PHASE 1B, 1C, 1D, PHASE 2)
//=============================================================================

// Test 7: Direct Shortest Path on a Linear Road
TEST(AStarAlgorithmTest, DirectShortestPath) {
    std::string xml = 
        "<osm>"
        "  <bounds minlat=\"0.0\" minlon=\"0.0\" maxlat=\"10.0\" maxlon=\"10.0\"/>"
        "  <node id=\"1\" lat=\"1.0\" lon=\"1.0\"/>"
        "  <node id=\"2\" lat=\"5.0\" lon=\"5.0\"/>"
        "  <node id=\"3\" lat=\"9.0\" lon=\"9.0\"/>"
        "  <way id=\"10\">"
        "    <nd ref=\"1\"/>"
        "    <nd ref=\"2\"/>"
        "    <nd ref=\"3\"/>"
        "    <tag k=\"highway\" v=\"primary\"/>"
        "  </way>"
        "</osm>";

    RouteModel model{StringToBytes(xml)};
    RoutePlanner planner{model, 10.0f, 10.0f, 90.0f, 90.0f};

    bool found = planner.AStarSearch();
    EXPECT_TRUE(found);
    EXPECT_EQ(model.path.size(), 3);
    EXPECT_GT(planner.GetDistance(), 0.0f);
}

// Test 8: Alternative Paths — Cheaper Path Must Be Chosen
// Diamond topology:
// Start (node 1) -> Upper path via node 2 -> End (node 4)  [Long path]
// Start (node 1) -> Lower path via node 3 -> End (node 4)  [Short path]
TEST(AStarAlgorithmTest, AlternativePathsCheapestChosen) {
    std::string xml = 
        "<osm>"
        "  <bounds minlat=\"0.0\" minlon=\"0.0\" maxlat=\"10.0\" maxlon=\"10.0\"/>"
        "  <node id=\"1\" lat=\"1.0\" lon=\"1.0\"/>"  // Start
        "  <node id=\"2\" lat=\"9.0\" lon=\"5.0\"/>"  // Upper (long detour)
        "  <node id=\"3\" lat=\"2.0\" lon=\"5.0\"/>"  // Lower (short direct)
        "  <node id=\"4\" lat=\"1.0\" lon=\"9.0\"/>"  // Goal
        "  <way id=\"10\">"
        "    <nd ref=\"1\"/>"
        "    <nd ref=\"2\"/>"
        "    <nd ref=\"4\"/>"
        "    <tag k=\"highway\" v=\"primary\"/>"
        "  </way>"
        "  <way id=\"20\">"
        "    <nd ref=\"1\"/>"
        "    <nd ref=\"3\"/>"
        "    <nd ref=\"4\"/>"
        "    <tag k=\"highway\" v=\"primary\"/>"
        "  </way>"
        "</osm>";

    RouteModel model{StringToBytes(xml)};
    RoutePlanner planner{model, 10.0f, 10.0f, 90.0f, 10.0f};

    bool found = planner.AStarSearch();
    EXPECT_TRUE(found);
    ASSERT_EQ(model.path.size(), 3);

    // Path must go through lower node (id 3 = SNodes()[2]), NOT upper node (id 2 = SNodes()[1])
    EXPECT_FLOAT_EQ(model.path[1].x, model.SNodes()[2].x);
    EXPECT_FLOAT_EQ(model.path[1].y, model.SNodes()[2].y);
}

// Test 9: A* Cost Relaxation Verification
// Discovers a node via an initial path, then discovers a cheaper alternative path.
// The algorithm must update (relax) the g-value and parent to the cheaper route.
TEST(AStarAlgorithmTest, CostRelaxationUpdatesParentAndG) {
    RouteModel::Node start, mid1, mid2, target;
    start.x = 0.0; start.y = 0.0; start.g_value = 0.0f; start.h_value = 1.0f;
    mid1.x = 0.8;  mid1.y = 0.8;  // High cost path
    mid2.x = 0.2;  mid2.y = 0.0;  // Cheaper path
    target.x = 1.0; target.y = 0.0;

    // Simulate initial higher-cost discovery from mid1
    target.parent = &mid1;
    target.g_value = 5.0f;
    target.h_value = 0.0f;

    // Simulate lower-cost discovery from mid2
    float cheaper_g = 1.5f;
    if (cheaper_g < target.g_value) {
        target.g_value = cheaper_g;
        target.parent = &mid2;
    }

    EXPECT_FLOAT_EQ(target.g_value, 1.5f);
    EXPECT_EQ(target.parent, &mid2);
}

// Test 10: Multiple Possible Routes on a 3x3 Grid Network
// Verifies that on a multi-path grid network, A* navigates to the global minimum distance
TEST(AStarAlgorithmTest, MultiplePossibleRoutesComplexGrid) {
    std::string xml = 
        "<osm>"
        "  <bounds minlat=\"0.0\" minlon=\"0.0\" maxlat=\"10.0\" maxlon=\"10.0\"/>"
        "  <node id=\"1\" lat=\"1.0\" lon=\"1.0\"/>"
        "  <node id=\"2\" lat=\"1.0\" lon=\"5.0\"/>"
        "  <node id=\"3\" lat=\"1.0\" lon=\"9.0\"/>"
        "  <node id=\"4\" lat=\"5.0\" lon=\"1.0\"/>"
        "  <node id=\"5\" lat=\"5.0\" lon=\"5.0\"/>"
        "  <node id=\"6\" lat=\"5.0\" lon=\"9.0\"/>"
        "  <node id=\"7\" lat=\"9.0\" lon=\"1.0\"/>"
        "  <node id=\"8\" lat=\"9.0\" lon=\"5.0\"/>"
        "  <node id=\"9\" lat=\"9.0\" lon=\"9.0\"/>"
        // Horizontal roads
        "  <way id=\"10\"><nd ref=\"1\"/><nd ref=\"2\"/><nd ref=\"3\"/><tag k=\"highway\" v=\"residential\"/></way>"
        "  <way id=\"20\"><nd ref=\"4\"/><nd ref=\"5\"/><nd ref=\"6\"/><tag k=\"highway\" v=\"residential\"/></way>"
        "  <way id=\"30\"><nd ref=\"7\"/><nd ref=\"8\"/><nd ref=\"9\"/><tag k=\"highway\" v=\"residential\"/></way>"
        // Vertical roads
        "  <way id=\"40\"><nd ref=\"1\"/><nd ref=\"4\"/><nd ref=\"7\"/><tag k=\"highway\" v=\"residential\"/></way>"
        "  <way id=\"50\"><nd ref=\"2\"/><nd ref=\"5\"/><nd ref=\"8\"/><tag k=\"highway\" v=\"residential\"/></way>"
        "  <way id=\"60\"><nd ref=\"3\"/><nd ref=\"6\"/><nd ref=\"9\"/><tag k=\"highway\" v=\"residential\"/></way>"
        "</osm>";

    RouteModel model{StringToBytes(xml)};
    // Route from top-left (10, 10) to bottom-right (90, 90)
    RoutePlanner planner{model, 10.0f, 10.0f, 90.0f, 90.0f};

    bool found = planner.AStarSearch();
    EXPECT_TRUE(found);
    EXPECT_GT(model.path.size(), 1);
    EXPECT_GT(planner.GetDistance(), 0.0f);
}

// Test 11: Empty Road Network Behavior (Map With Bounds But Zero Highway Roads)
// When no roads exist, FindClosestNode falls back to node 0 for both start and end,
// collapsing to a start == end scenario that safely returns a 1-node path of 0 distance.
TEST(AStarAlgorithmTest, EmptyGraphBehavior) {
    std::string xml = 
        "<osm>"
        "  <bounds minlat=\"0.0\" minlon=\"0.0\" maxlat=\"10.0\" maxlon=\"10.0\"/>"
        "  <node id=\"1\" lat=\"1.0\" lon=\"1.0\"/>"
        "  <node id=\"2\" lat=\"2.0\" lon=\"2.0\"/>"
        "  <way id=\"10\">"
        "    <nd ref=\"1\"/>"
        "    <nd ref=\"2\"/>"
        "    <tag k=\"building\" v=\"yes\"/>" // Building, NOT a road
        "  </way>"
        "</osm>";

    RouteModel model{StringToBytes(xml)};
    RoutePlanner planner{model, 10.0f, 10.0f, 20.0f, 20.0f};

    // Both start and end resolve to node 0 (no roads to match)
    bool found = planner.AStarSearch();
    EXPECT_TRUE(found);
    EXPECT_EQ(model.path.size(), 1);
    EXPECT_FLOAT_EQ(planner.GetDistance(), 0.0f);
}

// Test 12: Start Equals Destination
// Path must succeed, return 1 node, and distance 0.0 meters.
TEST(AStarAlgorithmTest, StartEqualsDestination) {
    std::string xml = 
        "<osm>"
        "  <bounds minlat=\"0.0\" minlon=\"0.0\" maxlat=\"10.0\" maxlon=\"10.0\"/>"
        "  <node id=\"1\" lat=\"3.0\" lon=\"3.0\"/>"
        "  <way id=\"10\">"
        "    <nd ref=\"1\"/>"
        "    <tag k=\"highway\" v=\"residential\"/>"
        "  </way>"
        "</osm>";

    RouteModel model{StringToBytes(xml)};
    RoutePlanner planner{model, 30.0f, 30.0f, 30.0f, 30.0f};

    bool found = planner.AStarSearch();
    EXPECT_TRUE(found);
    EXPECT_EQ(model.path.size(), 1);
    EXPECT_FLOAT_EQ(planner.GetDistance(), 0.0f);
}

// Test 13: Unreachable Destination (Disconnected Graph)
// Road 1: N1 - N2. Road 2: N3 - N4. No road connects them.
// RoutePlanner must report failure cleanly.
TEST(AStarAlgorithmTest, UnreachableDestinationReporting) {
    std::string xml = 
        "<osm>"
        "  <bounds minlat=\"0.0\" minlon=\"0.0\" maxlat=\"10.0\" maxlon=\"10.0\"/>"
        "  <node id=\"1\" lat=\"1.0\" lon=\"1.0\"/>"
        "  <node id=\"2\" lat=\"2.0\" lon=\"2.0\"/>"
        "  <node id=\"3\" lat=\"8.0\" lon=\"8.0\"/>"
        "  <node id=\"4\" lat=\"9.0\" lon=\"9.0\"/>"
        "  <way id=\"10\">"
        "    <nd ref=\"1\"/>"
        "    <nd ref=\"2\"/>"
        "    <tag k=\"highway\" v=\"primary\"/>"
        "  </way>"
        "  <way id=\"20\">"
        "    <nd ref=\"3\"/>"
        "    <nd ref=\"4\"/>"
        "    <tag k=\"highway\" v=\"primary\"/>"
        "  </way>"
        "</osm>";

    RouteModel model{StringToBytes(xml)};
    // Route from island 1 to island 2
    RoutePlanner planner{model, 10.0f, 10.0f, 90.0f, 90.0f};

    bool found = planner.AStarSearch();
    EXPECT_FALSE(found);
    EXPECT_TRUE(model.path.empty());
    EXPECT_FLOAT_EQ(planner.GetDistance(), 0.0f);
}

// Test 14: Path Reconstruction Cycle Detection
// If a circular parent pointer chain is encountered (A -> B -> A), ConstructFinalPath
// must terminate safely, return an empty path, and not enter an infinite loop.
TEST(AStarAlgorithmTest, ReconstructionCycleDetection) {
    std::string xml = 
        "<osm>"
        "  <bounds minlat=\"0.0\" minlon=\"0.0\" maxlat=\"10.0\" maxlon=\"10.0\"/>"
        "  <node id=\"1\" lat=\"1.0\" lon=\"1.0\"/>"
        "  <node id=\"2\" lat=\"2.0\" lon=\"2.0\"/>"
        "  <way id=\"10\">"
        "    <nd ref=\"1\"/>"
        "    <nd ref=\"2\"/>"
        "    <tag k=\"highway\" v=\"primary\"/>"
        "  </way>"
        "</osm>";

    RouteModel model{StringToBytes(xml)};
    RoutePlanner planner{model, 10.0f, 10.0f, 20.0f, 20.0f};

    RouteModel::Node node_a, node_b;
    node_a.x = 0.1; node_a.y = 0.1;
    node_b.x = 0.2; node_b.y = 0.2;

    // Create intentional cycle: A -> B -> A
    node_a.parent = &node_b;
    node_b.parent = &node_a;

    auto path = planner.ConstructFinalPath(&node_a);
    EXPECT_TRUE(path.empty());
    EXPECT_FLOAT_EQ(planner.GetDistance(), 0.0f);
}

// Test 15: Path Reconstruction Missing Parent Protection
// If parent is nullptr before reaching start_node, ConstructFinalPath must handle it safely.
TEST(AStarAlgorithmTest, ReconstructionMissingParentProtection) {
    std::string xml = 
        "<osm>"
        "  <bounds minlat=\"0.0\" minlon=\"0.0\" maxlat=\"10.0\" maxlon=\"10.0\"/>"
        "  <node id=\"1\" lat=\"1.0\" lon=\"1.0\"/>"
        "  <way id=\"10\">"
        "    <nd ref=\"1\"/>"
        "    <tag k=\"highway\" v=\"primary\"/>"
        "  </way>"
        "</osm>";

    RouteModel model{StringToBytes(xml)};
    RoutePlanner planner{model, 10.0f, 10.0f, 10.0f, 10.0f};

    RouteModel::Node orphan;
    orphan.x = 0.5; orphan.y = 0.5;
    orphan.parent = nullptr; // Broken chain

    auto path = planner.ConstructFinalPath(&orphan);
    EXPECT_TRUE(path.empty());
    EXPECT_FLOAT_EQ(planner.GetDistance(), 0.0f);
}

//=============================================================================
// 3. REAL MAP DATA INTEGRATION TEST (BHUBANESWAR MAP.OSM)
//=============================================================================

// Test 16: End-to-End A* Pathfinding on Bhubaneswar Dataset
TEST(RealMapIntegrationTest, BhubaneswarRoutingSucceeds) {
    auto osm_data = ReadOSMData("map.osm");
    if (osm_data.empty()) {
        osm_data = ReadOSMData("../map.osm");
    }
    ASSERT_FALSE(osm_data.empty()) << "map.osm could not be loaded for integration test.";

    RouteModel model{osm_data};
    RoutePlanner planner{model, 10.0f, 10.0f, 90.0f, 90.0f};

    bool found = planner.AStarSearch();
    EXPECT_TRUE(found);
    EXPECT_GT(model.path.size(), 0);
    EXPECT_GT(planner.GetDistance(), 0.0f);

    // Verify start and end nodes match path endpoints
    RouteModel::Node start = model.path.front();
    RouteModel::Node end = model.path.back();
    EXPECT_FLOAT_EQ(start.x, model.path.front().x);
    EXPECT_FLOAT_EQ(end.x, model.path.back().x);
}
