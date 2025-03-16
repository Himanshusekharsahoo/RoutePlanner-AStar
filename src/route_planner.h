#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"

/**
 * The RoutePlanner class is responsible for finding the shortest path
 * between two points using the A* search algorithm.
 */
class RoutePlanner {
public:
    /**
     * Constructor: Initializes the RoutePlanner with start and end coordinates.
     * @param model The RouteModel containing map data.
     * @param start_x The x-coordinate of the start point.
     * @param start_y The y-coordinate of the start point.
     * @param end_x The x-coordinate of the end point.
     * @param end_y The y-coordinate of the end point.
     */
    RoutePlanner(RouteModel& model, float start_x, float start_y, float end_x, float end_y);

    /**
     * Returns the total distance of the calculated path.
     * @return The distance of the path.
     */
    float GetDistance() const { return distance; }

    /**
     * Performs the A* search algorithm to find the shortest path.
     */
    void AStarSearch();

    // The following methods have been made public for testing purposes.

    /**
     * Adds neighboring nodes to the open list for exploration.
     * @param current_node The current node being explored.
     */
    void AddNeighbors(RouteModel::Node* current_node);

    /**
     * Calculates the heuristic value (estimated cost to the goal) for a node.
     * @param node The node for which to calculate the heuristic.
     * @return The heuristic value.
     */
    float CalculateHValue(const RouteModel::Node* node);

    /**
     * Constructs the final path from the start node to the end node.
     * @param final_node The final node in the path.
     * @return A vector of nodes representing the path.
     */
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node* final_node);

    /**
     * Selects the next node to explore from the open list.
     * @return A pointer to the next node.
     */
    RouteModel::Node* NextNode();

private:
    std::vector<RouteModel::Node*> open_list;  // List of nodes to be explored
    RouteModel::Node* start_node;  // The starting node
    RouteModel::Node* end_node;  // The goal node

    float distance = 0.0f;  // Total distance of the calculated path
    RouteModel& m_Model;  // Reference to the RouteModel containing map data
};

#endif