#include "route_model.h"
#include <iostream>
#include <algorithm>

RouteModel::RouteModel(const std::vector<std::byte>& xml) : Model(xml) {
    // Create RouteModel nodes from the base Model nodes
    int counter = 0;  // Counter for assigning node indices
    for (const Model::Node& node : this->Nodes()) {
        m_Nodes.emplace_back(Node(counter, this, node));  // Create and add Node
        counter++;  // Increment counter
    }
    CreateNodeToRoadHashmap();  // Create the node-to-road hashmap
}

/**
 * Creates a hashmap that maps node indices to the roads they belong to.
 */
void RouteModel::CreateNodeToRoadHashmap() {
    for (const Model::Road& road : Roads()) {
        // Skip footways (pedestrian paths) as they are not relevant for route planning
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx : Ways()[road.way].nodes) {
                // Initialize the node-to-road mapping if it doesn't exist
                if (node_to_road.find(node_idx) == node_to_road.end()) {
                    node_to_road[node_idx] = std::vector<const Model::Road*>();
                }
                // Add the road to the node's list of roads
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}

/**
 * Finds an adjacent neighbor node from a list of node indices for a road way.
 * Connects immediate predecessor or successor to preserve topological adjacency.
 * @param node_indices The list of node indices representing an OSM way.
 * @return A pointer to an adjacent neighboring node, or nullptr if none is found.
 */
RouteModel::Node* RouteModel::Node::FindNeighbor(const std::vector<int>& node_indices) {
    if (!parent_model) return nullptr;
    const int len = static_cast<int>(node_indices.size());
    for (int i = 0; i < len; ++i) {
        if (node_indices[i] == this->index) {
            // Check consecutive predecessor
            if (i > 0) {
                int prev_idx = node_indices[i - 1];
                if (prev_idx >= 0 && prev_idx < static_cast<int>(parent_model->SNodes().size()) && prev_idx != this->index) {
                    return &parent_model->SNodes()[prev_idx];
                }
            }
            // Check consecutive successor
            if (i + 1 < len) {
                int next_idx = node_indices[i + 1];
                if (next_idx >= 0 && next_idx < static_cast<int>(parent_model->SNodes().size()) && next_idx != this->index) {
                    return &parent_model->SNodes()[next_idx];
                }
            }
        }
    }
    return nullptr;
}

/**
 * Finds and populates the list of neighboring nodes.
 * Connects immediate predecessor and successor nodes along each connected road way,
 * establishing proper topological adjacency rather than arbitrary Euclidean proximity.
 */
void RouteModel::Node::FindNeighbors() {
    if (!this->neighbors.empty()) {
        return;  // Already populated; prevent duplicate edges on subsequent calls
    }

    if (!parent_model) {
        return;
    }

    const auto it = parent_model->node_to_road.find(this->index);
    if (it == parent_model->node_to_road.end()) {
        return;
    }

    for (const auto* road : it->second) {
        if (!road) continue;
        const auto& way_nodes = parent_model->Ways()[road->way].nodes;
        const int way_len = static_cast<int>(way_nodes.size());

        for (int i = 0; i < way_len; ++i) {
            if (way_nodes[i] == this->index) {
                // Connect immediate predecessor along the road way
                if (i > 0) {
                    int prev_idx = way_nodes[i - 1];
                    if (prev_idx >= 0 && prev_idx < static_cast<int>(parent_model->SNodes().size()) && prev_idx != this->index) {
                        Node* prev_node = &parent_model->SNodes()[prev_idx];
                        if (std::find(this->neighbors.begin(), this->neighbors.end(), prev_node) == this->neighbors.end()) {
                            this->neighbors.push_back(prev_node);
                        }
                    }
                }
                // Connect immediate successor along the road way
                if (i + 1 < way_len) {
                    int next_idx = way_nodes[i + 1];
                    if (next_idx >= 0 && next_idx < static_cast<int>(parent_model->SNodes().size()) && next_idx != this->index) {
                        Node* next_node = &parent_model->SNodes()[next_idx];
                        if (std::find(this->neighbors.begin(), this->neighbors.end(), next_node) == this->neighbors.end()) {
                            this->neighbors.push_back(next_node);
                        }
                    }
                }
            }
        }
    }
}

/**
 * Finds the closest node to the given coordinates.
 * @param x The x-coordinate.
 * @param y The y-coordinate.
 * @return A reference to the closest node.
 */
RouteModel::Node& RouteModel::FindClosestNode(float x, float y) {
    Node input;  // Temporary node for the input coordinates
    input.x = x;
    input.y = y;

    float min_dist = std::numeric_limits<float>::max();  // Initialize minimum distance to a large value
    float dist;  // Distance between the input node and the current node
    int closest_idx = 0;  // Index of the closest node

    // Iterate over all roads (excluding footways)
    for (const Model::Road& road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            // Iterate over all nodes in the current road
            for (int node_idx : Ways()[road.way].nodes) {
                if (node_idx >= 0 && node_idx < static_cast<int>(SNodes().size())) {
                    dist = input.distance(SNodes()[node_idx]);  // Calculate the distance
                    // Update the closest node if this node is closer
                    if (dist < min_dist) {
                        closest_idx = node_idx;
                        min_dist = dist;
                    }
                }
            }
        }
    }

    // Return the closest node
    return SNodes()[closest_idx];
}
