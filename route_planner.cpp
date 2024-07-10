#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto& neihbor_node : current_node->neighbors) {
        neihbor_node->parent = current_node;
        neihbor_node->h_value = CalculateHValue(neihbor_node);
        neihbor_node->g_value = current_node->g_value + current_node->distance(*neihbor_node);
        open_list.emplace_back(neihbor_node);
        neihbor_node->visited = true;
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(std::begin(open_list), std::end(open_list), 
              [](const RouteModel::Node * a, const RouteModel::Node * b){
                  return (a->g_value + a->h_value) > (b->g_value + b->h_value);
              });
    auto next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    path_found.emplace_back(*current_node);
    while (current_node != start_node) {
        auto parent = current_node->parent;
        distance += current_node->distance(*parent);
        path_found.emplace_back(*parent);
        current_node = parent;
    }
    std::reverse(std::begin(path_found), std::end(path_found));
    
    distance *= m_Model.MetricScale(); 
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    AddNeighbors(start_node);
    start_node->visited = true;
    while (open_list.size() > 0) {
        current_node = NextNode();
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
}