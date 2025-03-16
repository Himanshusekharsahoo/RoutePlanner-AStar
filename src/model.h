#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <cstddef>

/**
 * Represents the Model class, which holds map data (nodes, ways, roads, etc.)
 * parsed from OpenStreetMap (OSM) XML data.
 */
class Model {
public:
    // Represents a node (point) on the map with x and y coordinates.
    struct Node {
        double x = 0.0;  // X-coordinate (longitude)
        double y = 0.0;  // Y-coordinate (latitude)
    };

    // Represents a way, which is an ordered list of node IDs.
    struct Way {
        std::vector<int> nodes;  // List of node IDs forming the way
    };

    // Represents a road, which is associated with a way and has a type.
    struct Road {
        enum Type {
            Invalid,      // Invalid road type
            Unclassified, // Unclassified road
            Service,      // Service road
            Residential,  // Residential road
            Tertiary,     // Tertiary road
            Secondary,    // Secondary road
            Primary,     // Primary road
            Trunk,       // Trunk road
            Motorway,    // Motorway
            Footway       // Footway (pedestrian path)
        };
        int way;  // ID of the associated way
        Type type;  // Type of the road
    };

    // Represents a railway, which is associated with a way.
    struct Railway {
        int way;  // ID of the associated way
    };

    // Represents a multipolygon, which consists of outer and inner rings.
    struct Multipolygon {
        std::vector<int> outer;  // List of node IDs forming the outer ring
        std::vector<int> inner;  // List of node IDs forming the inner ring
    };

    // Represents a building, which is a type of multipolygon.
    struct Building : Multipolygon {};

    // Represents a leisure area, which is a type of multipolygon.
    struct Leisure : Multipolygon {};

    // Represents a water body, which is a type of multipolygon.
    struct Water : Multipolygon {};

    // Represents a land use area, which is a type of multipolygon with a specific type.
    struct Landuse : Multipolygon {
        enum Type {
            Invalid,       // Invalid land use type
            Commercial,    // Commercial area
            Construction,  // Construction site
            Grass,         // Grassland
            Forest,        // Forest
            Industrial,    // Industrial area
            Railway,       // Railway area
            Residential    // Residential area
        };
        Type type;  // Type of the land use area
    };

    // Constructor: Initializes the Model with OSM XML data.
    Model(const std::vector<std::byte>& xml);

    // Returns the metric scale factor for the map.
    auto MetricScale() const noexcept { return m_MetricScale; }

    // Getters for various map features
    auto& Nodes() const noexcept { return m_Nodes; }
    auto& Ways() const noexcept { return m_Ways; }
    auto& Roads() const noexcept { return m_Roads; }
    auto& Buildings() const noexcept { return m_Buildings; }
    auto& Leisures() const noexcept { return m_Leisures; }
    auto& Waters() const noexcept { return m_Waters; }
    auto& Landuses() const noexcept { return m_Landuses; }
    auto& Railways() const noexcept { return m_Railways; }

private:
    // Adjusts the coordinates of nodes to fit within the map bounds.
    void AdjustCoordinates();

    // Builds rings (outer and inner) for multipolygons.
    void BuildRings(Multipolygon& mp);

    // Loads and parses OSM XML data into the model.
    void LoadData(const std::vector<std::byte>& xml);

    // Data structures to store map features
    std::vector<Node> m_Nodes;       // List of nodes
    std::vector<Way> m_Ways;         // List of ways
    std::vector<Road> m_Roads;       // List of roads
    std::vector<Railway> m_Railways; // List of railways
    std::vector<Building> m_Buildings; // List of buildings
    std::vector<Leisure> m_Leisures; // List of leisure areas
    std::vector<Water> m_Waters;     // List of water bodies
    std::vector<Landuse> m_Landuses; // List of land use areas

    // Map bounds and scale
    double m_MinLat = 0.0;      // Minimum latitude
    double m_MaxLat = 0.0;      // Maximum latitude
    double m_MinLon = 0.0;      // Minimum longitude
    double m_MaxLon = 0.0;      // Maximum longitude
    double m_MetricScale = 1.0; // Scale factor for metric conversion
};