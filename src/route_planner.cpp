#include "route_planner.h"
#include <algorithm>

/**
 * Constructor: Initializes the RoutePlanner with start and end coordinates.
 * @param model The RouteModel containing map data.
 * @param start_x The x-coordinate of the start point.
 * @param start_y The y-coordinate of the start point.
 * @param end_x The x-coordinate of the end point.
 * @param end_y The y-coordinate of the end point.
 */
RoutePlanner::RoutePlanner(RouteModel& model, float start_x, float start_y, float end_x, float end_y)
    : m_Model(model) {
    // Convert inputs to percentage (assuming coordinates are in the range 0-100)
    start_x *= 0.01f;
    start_y *= 0.01f;
    end_x *= 0.01f;
    end_y *= 0.01f;

    // Find the closest nodes to the start and end coordinates
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

/**
 * Calculates the heuristic value (estimated cost to the goal) for a node.
 * @param node The node for which to calculate the heuristic.
 * @return The heuristic value.
 */
float RoutePlanner::CalculateHValue(const RouteModel::Node* node) {
    return node->distance(*this->end_node);  // Euclidean distance to the end node
}

/**
 * Adds neighboring nodes to the open list for exploration.
 * @param current_node The current node being explored.
 */
void RoutePlanner::AddNeighbors(RouteModel::Node* current_node) {
    current_node->FindNeighbors();  // Populate the neighbors vector
    for (auto neighbor : current_node->neighbors) {
        neighbor->parent = current_node;  // Set the parent of the neighbor
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);  // Update g-value
        neighbor->h_value = CalculateHValue(neighbor);  // Calculate h-value
        neighbor->visited = true;  // Mark the neighbor as visited
        this->open_list.emplace_back(neighbor);  // Add the neighbor to the open list
    }
}

/**
 * Selects the next node to explore from the open list.
 * @return A pointer to the next node.
 */
RouteModel::Node* RoutePlanner::NextNode() {
    // Sort the open list in descending order of f-value (g-value + h-value)
    std::sort(this->open_list.begin(), this->open_list.end(),
              [](const RouteModel::Node* v1, const RouteModel::Node* v2) {
                  return (v1->h_value + v1->g_value) > (v2->h_value + v2->g_value);
              });

    auto next_node = this->open_list.back();  // Get the node with the lowest f-value
    this->open_list.pop_back();  // Remove the node from the open list
    return next_node;
}

/**
 * Constructs the final path from the start node to the end node.
 * @param current_node The final node in the path.
 * @return A vector of nodes representing the path.
 */
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node* current_node) {
    distance = 0.0f;  // Initialize the total distance
    std::vector<RouteModel::Node> path_found;

    // Traverse the path from the end node to the start node
    while (current_node != this->start_node) {
        path_found.push_back(*current_node);  // Add the current node to the path
        distance += current_node->distance(*current_node->parent);  // Add the distance to the parent node
        current_node = current_node->parent;  // Move to the parent node
    }
    path_found.push_back(*this->start_node);  // Add the start node to the path

    std::reverse(path_found.begin(), path_found.end());  // Reverse the path to start-to-end order

    distance *= m_Model.MetricScale();  // Convert the distance to meters
    return path_found;
}

/**
 * Performs the A* search algorithm to find the shortest path.
 */
void RoutePlanner::AStarSearch() {
    RouteModel::Node* current_node = nullptr;

    // Initialize the start node
    this->start_node->visited = true;
    this->open_list.push_back(this->start_node);

    // Explore nodes until the open list is empty or the end node is found
    while (!this->open_list.empty()) {
        current_node = NextNode();  // Get the next node to explore

        // Check if the current node is the end node
        if (current_node->x == this->end_node->x && current_node->y == this->end_node->y) {
            m_Model.path = ConstructFinalPath(current_node);  // Construct and store the final path
            break;
        }

        AddNeighbors(current_node);  // Add neighbors of the current node to the open list
    }
}