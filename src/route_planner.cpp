#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(start_x, start_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    //float distance(Node other)
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors(); // Populate current_node.neighbors vector with all the neighbors.
    for (auto node : current_node->neighbors) {
        if (node->visited){
            continue;
        }
    node->parent = current_node;
    node->h_value = CalculateHValue(node);
    node->g_value = current_node->g_value + current_node->distance(*node);
    node->visited = true;
    open_list.push_back(node);
    }
}

bool CompareFValue(RouteModel::Node *node1, RouteModel::Node *node2){
    float F1 = node1->g_value + node1->h_value;
    float F2 = node2->g_value + node2->h_value;
    return F1 > F2;
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), CompareFValue);
    RouteModel::Node *next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    while(current_node != start_node){
    path_found.push_back(*current_node);
    distance += current_node->distance(*(current_node->parent));
    current_node = current_node->parent;
    }

    path_found.push_back(*start_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
    current_node->visited = true;
    current_node->h_value = CalculateHValue(current_node);
    current_node->g_value = 0;
    open_list.push_back(current_node);
    while(open_list.size() > 0){
        current_node = NextNode();
        AddNeighbors(current_node);
        if (current_node->distance(*end_node) == 0){
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
    }
}
