#include "route_model.h"
#include <iostream>

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
 * Finds a neighboring node from a list of node indices.
 * @param node_indices The list of node indices to search.
 * @return A pointer to the closest neighboring node, or nullptr if none is found.
 */
RouteModel::Node* RouteModel::Node::FindNeighbor(const std::vector<int>& node_indices) {
    Node* closest_node = nullptr;  // Pointer to the closest node
    Node node;  // Temporary node for distance calculations

    for (int node_index : node_indices) {
        node = parent_model->SNodes()[node_index];  // Get the node from the parent model
        // Check if the node is not the current node and has not been visited
        if (this->distance(node) != 0 && !node.visited) {
            // Update the closest node if this node is closer
            if (closest_node == nullptr || this->distance(node) < this->distance(*closest_node)) {
                closest_node = &parent_model->SNodes()[node_index];
            }
        }
    }
    return closest_node;
}

/**
 * Finds and populates the list of neighboring nodes.
 */
void RouteModel::Node::FindNeighbors() {
    // Iterate over all roads connected to this node
    for (const auto& road : parent_model->node_to_road[this->index]) {
        // Find the closest neighbor from the nodes of the current road
        RouteModel::Node* new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        if (new_neighbor) {
            // Add the neighbor to the list of neighbors
            this->neighbors.emplace_back(new_neighbor);
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
                dist = input.distance(SNodes()[node_idx]);  // Calculate the distance
                // Update the closest node if this node is closer
                if (dist < min_dist) {
                    closest_idx = node_idx;
                    min_dist = dist;
                }
            }
        }
    }

    // Return the closest node
    return SNodes()[closest_idx];
}