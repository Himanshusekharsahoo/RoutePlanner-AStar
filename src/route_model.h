#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>
#include <cstddef>  // For std::byte

#include "model.h"

/**
 * The RouteModel class extends the Model class to support route planning.
 */
class RouteModel : public Model {
public:
    /**
     * The Node class represents a node in the route planning graph.
     */
    class Node : public Model::Node {
    public:
        Node* parent = nullptr;  // Pointer to the parent node in the search tree
        float h_value = std::numeric_limits<float>::max();  // Heuristic value (estimated cost to goal)
        float g_value = 0.0;  // Cost from the start node to this node
        bool visited = false;  // Whether the node has been visited during the search
        std::vector<Node*> neighbors;  // List of neighboring nodes

        /**
         * Finds and populates the list of neighboring nodes.
         */
        void FindNeighbors();

        /**
         * Calculates the Euclidean distance to another node.
         * @param other The other node.
         * @return The distance between this node and the other node.
         */
        float distance(const Node& other) const {
            return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
        }

        // Default constructor
        Node() = default;

        /**
         * Constructor: Initializes a Node with an index, parent model, and base Model::Node.
         * @param idx The index of the node.
         * @param search_model The parent RouteModel.
         * @param node The base Model::Node.
         */
        Node(int idx, RouteModel* search_model, const Model::Node& node)
            : Model::Node(node), parent_model(search_model), index(idx) {}

    private:
        int index;  // Index of the node in the parent model's node list
        RouteModel* parent_model = nullptr;  // Pointer to the parent RouteModel

        /**
         * Finds a neighboring node from a list of node indices.
         * @param node_indices The list of node indices to search.
         * @return A pointer to the neighboring node, or nullptr if not found.
         */
        Node* FindNeighbor(const std::vector<int>& node_indices);
    };

    /**
     * Constructor: Initializes the RouteModel with OSM XML data.
     * @param xml The OSM XML data as a vector of bytes.
     */
    RouteModel(const std::vector<std::byte>& xml);

    /**
     * Finds the closest node to the given coordinates.
     * @param x The x-coordinate.
     * @param y The y-coordinate.
     * @return A reference to the closest node.
     */
    Node& FindClosestNode(float x, float y);

    /**
     * Returns the list of nodes in the model.
     * @return A reference to the list of nodes.
     */
    std::vector<Node>& SNodes() { return m_Nodes; }

    std::vector<Node> path;  // The calculated path from start to goal

private:
    /**
     * Creates a hashmap that maps node indices to the roads they belong to.
     */
    void CreateNodeToRoadHashmap();

    std::unordered_map<int, std::vector<const Model::Road*>> node_to_road;  // Hashmap for node-to-road mapping
    std::vector<Node> m_Nodes;  // List of nodes in the model
};

#endif