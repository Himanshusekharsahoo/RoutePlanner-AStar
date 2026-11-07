#include "route_planner.h"
#include <algorithm>
#include <unordered_set>
#include <iostream>

/**
 * Constructor: Initializes the RoutePlanner with start and end coordinates.
 * @param model The RouteModel containing map data.
 * @param start_x The x-coordinate of the start point (in range 0-100).
 * @param start_y The y-coordinate of the start point (in range 0-100).
 * @param end_x The x-coordinate of the end point (in range 0-100).
 * @param end_y The y-coordinate of the end point (in range 0-100).
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

    // Initialize start node g_value to 0
    if (this->start_node) {
        this->start_node->g_value = 0.0f;
    }
}

/**
 * Calculates the heuristic value (estimated cost to the goal) for a node.
 * Uses Euclidean straight-line distance, which is admissible and consistent
 * for the 2D projected coordinate space.
 * @param node The node for which to calculate the heuristic.
 * @return The heuristic value.
 */
float RoutePlanner::CalculateHValue(const RouteModel::Node* node) {
    if (!node || !this->end_node) return 0.0f;
    return node->distance(*this->end_node);
}

/**
 * Adds neighboring nodes to the open list for exploration.
 * Applies cost relaxation: if reaching a neighbor through current_node provides
 * a strictly lower g-value than previously recorded, updates its g-value, parent,
 * recalculates h-value, and ensures it is active in the open list.
 * @param current_node The current node being explored.
 */
void RoutePlanner::AddNeighbors(RouteModel::Node* current_node) {
    if (!current_node) return;

    current_node->FindNeighbors();

    for (auto* neighbor : current_node->neighbors) {
        if (!neighbor) continue;

        float tentative_g = current_node->g_value + current_node->distance(*neighbor);

        // Cost relaxation: update if neighbor is undiscovered or a cheaper path is found
        if (tentative_g < neighbor->g_value) {
            neighbor->parent = current_node;
            neighbor->g_value = tentative_g;
            neighbor->h_value = CalculateHValue(neighbor);

            // Reopen node if it was previously closed
            neighbor->visited = false;

            // Add to open list if not already present
            if (std::find(this->open_list.begin(), this->open_list.end(), neighbor) == this->open_list.end()) {
                this->open_list.push_back(neighbor);
            }
        }
    }
}

/**
 * Selects the next node to explore from the open list.
 * Sorts the open list so the lowest f-value (g + h) is extracted.
 * Marks the selected node as expanded (closed).
 * @return A pointer to the next node.
 */
RouteModel::Node* RoutePlanner::NextNode() {
    if (this->open_list.empty()) return nullptr;

    // Sort in descending order of f-value so lowest f-value is at the back
    std::sort(this->open_list.begin(), this->open_list.end(),
              [](const RouteModel::Node* v1, const RouteModel::Node* v2) {
                  return (v1->h_value + v1->g_value) > (v2->h_value + v2->g_value);
              });

    auto next_node = this->open_list.back();
    this->open_list.pop_back();
    next_node->visited = true;  // Mark as expanded (closed)
    return next_node;
}

/**
 * Constructs the final path from the start node to the end node.
 * Includes defensive checks:
 * - Start node equals destination (returns 1-node path).
 * - Detects cycles in the parent chain.
 * - Detects missing parent pointers before reaching start_node.
 * - Safe null-pointer checks throughout traversal.
 * @param current_node The final node reached in the path.
 * @return A vector of nodes representing the path from start to goal.
 */
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node* current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    if (!current_node || !this->start_node) {
        return path_found;
    }

    // Handle case where start node is the destination
    if (current_node == this->start_node) {
        path_found.push_back(*this->start_node);
        return path_found;
    }

    std::unordered_set<const RouteModel::Node*> seen;
    bool reached_start = false;

    while (current_node != nullptr) {
        // Prevent infinite loops caused by cyclic parent chains
        if (!seen.insert(current_node).second) {
            std::cerr << "Warning: Cycle detected in parent chain during path reconstruction." << std::endl;
            path_found.clear();
            distance = 0.0f;
            return path_found;
        }

        path_found.push_back(*current_node);

        if (current_node == this->start_node) {
            reached_start = true;
            break;
        }

        // Prevent null-pointer dereference if parent chain is broken
        if (current_node->parent == nullptr) {
            std::cerr << "Warning: Missing parent pointer during path reconstruction." << std::endl;
            path_found.clear();
            distance = 0.0f;
            return path_found;
        }

        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }

    if (!reached_start) {
        path_found.clear();
        distance = 0.0f;
        return path_found;
    }

    // Reverse to establish start-to-end order
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale();  // Convert distance to meters
    return path_found;
}

/**
 * Performs the A* search algorithm to find the shortest path.
 * Explores nodes by lowest f-value, applying cost relaxation on neighbors.
 * @return True if a valid path was found, false otherwise.
 */
bool RoutePlanner::AStarSearch() {
    if (!this->start_node || !this->end_node) {
        m_Model.path.clear();
        distance = 0.0f;
        return false;
    }

    // Clear open list and initialize start node state
    this->open_list.clear();
    this->start_node->g_value = 0.0f;
    this->start_node->h_value = CalculateHValue(this->start_node);
    this->start_node->visited = false;
    this->open_list.push_back(this->start_node);

    RouteModel::Node* current_node = nullptr;

    while (!this->open_list.empty()) {
        current_node = NextNode();
        if (!current_node) break;

        // Check if goal node has been reached (by pointer or identical coordinates)
        if (current_node == this->end_node ||
            (current_node->x == this->end_node->x && current_node->y == this->end_node->y)) {
            m_Model.path = ConstructFinalPath(current_node);
            return !m_Model.path.empty();
        }

        AddNeighbors(current_node);
    }

    // Destination unreachable (disconnected component or no valid road path)
    m_Model.path.clear();
    distance = 0.0f;
    return false;
}
