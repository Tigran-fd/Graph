#ifndef GRAPH_H
#define GRAPH_H

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <iostream>
#include <stack>
#include <algorithm>
#include <functional>
#include <set>
#include <string>

#include "Node.h"
#include "Edge.h"
#include "DisjointSet.h"

template <class T, class L>
class Graph
{
    std::unordered_map<T, Node<T>*> all_nodes;
    std::unordered_map<T, std::unordered_set<Edge<T, L>, EdgeHash<T, L>>> in_edges;
    std::unordered_map<T, std::unordered_set<Edge<T, L>, EdgeHash<T, L>>> out_edges;

    Node<T>* get_node(const T& data) {
        return all_nodes.at(data);
    }

public:
    Graph() = default;

    ~Graph() {
        for (auto& [val, node] : all_nodes) {
            delete node;
        }
    }

    bool addNode(const T& v) {
        if (all_nodes.contains(v)) {
            return false;
        }
        all_nodes[v] = new Node<T>(v);
        return true;
    }

    bool addEdge(const T& source, const T& dest, const L& label) {
        if (!all_nodes.contains(source) || !all_nodes.contains(dest)) {
            return false;
        }
        auto source_node = get_node(source);
        auto dest_node = get_node(dest);
        Edge<T, L> new_edge(source_node, dest_node, label);
        if (in_edges[dest].find(new_edge) != in_edges[dest].end()) {
            return false;
        }
        in_edges[dest].insert(new_edge);
        out_edges[source].insert(new_edge);
        return true;
    }

    std::unordered_set<T> getAllNodes() const {
        std::unordered_set<T> nodes;
        for (const auto& pair : all_nodes) {
            nodes.insert(pair.first);
        }
        return nodes;
    }

    std::vector<T> getNextNodes(const T& node_value) const {
        std::vector<T> next_nodes;
        auto it = out_edges.find(node_value);
        if (it != out_edges.end()) {
            for (const auto& edge : it->second) {
                next_nodes.push_back(edge.destination->value);
            }
        }
        return next_nodes;
    }

    std::vector<T> bfs(const T& start) {
        std::vector<T> results;
        std::unordered_map<T, bool> visited;
        std::queue<T> q;
        visited[start] = true;
        q.push(start);
        while (!q.empty()) {
            T current = q.front();
            q.pop();
            results.push_back(current);
            for (const auto& node : getNextNodes(current)) {
                if (!visited[node]) {
                    visited[node] = true;
                    q.push(node);
                }
            }
        }
        return results;
    }
    std::vector<T> dfs(const T& start) {
        std::vector<T> results;
        std::unordered_map<T, bool> visited;
        std::stack<T> s;
        s.push(start);
        while (!s.empty()) {
            T current = s.top();
            s.pop();
            if (visited[current]) {
                continue;
            }
            visited[current] = true;
            results.push_back(current);
            auto next_nodes = getNextNodes(current);
            for (auto it = next_nodes.rbegin(); it != next_nodes.rend(); ++it) {
                if (!visited[*it]) {
                    s.push(*it);
                }
            }
        }
        return results;
    }

    bool hasCycle() {
        std::unordered_map<T, int> visited;
        std::stack<std::pair<T, bool>> s;
        for (const auto& [node, _] : all_nodes) {
            if (visited[node]) {
                continue;
            }
            s.push({node, false});
            while (!s.empty()) {
                auto [current, on_stack] = s.top();
                s.pop();
                if (!visited[current]) {
                    visited[current] = 1;
                    s.push({current, true});
                    for (const auto& next : getNextNodes(current)) {
                        if (!visited[next]) {
                            s.push({next, false});
                        } else if (visited[next] == 1) {
                            return true;
                        }
                    }
                } else if (on_stack) {
                    visited[current] = 2;
                }
            }
        }
        return false;
    }

    void bfsPrint(const T& start) {
        for (const auto& val : bfs(start)) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }

    void dfsPrint(const T& start)
    {
        for (const auto& val : dfs(start)){
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }

    std::vector<T> findPath(const T& start, const T& end) {
        std::unordered_map<T, T> parent;
        std::unordered_map<T, bool> visited;
        std::stack<T> s;
        s.push(start);
        visited[start] = true;

        while (!s.empty()) {
            T current = s.top();
            s.pop();

            if (current == end) {
                std::vector<T> path;
                for (T at = end; at != start; at = parent[at]) {
                    path.push_back(at);
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (const auto& neighbor : getNextNodes(current)) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    parent[neighbor] = current;
                    s.push(neighbor);
                }
            }
        }
        return {};
    }

  std::vector<std::vector<T>> Dijkstra(const T& start) {

        std::priority_queue<std::pair<int, Node<T>*>, std::vector<std::pair<int, Node<T>*>>, std::greater<>> q;

        for (auto& [val, node] : all_nodes) {node->distance = INT_MAX; node->parent = nullptr;}

        auto start_node = get_node(start);
        start_node->distance = 0;
        q.emplace(0, start_node);

        while (!q.empty()) {
            auto [current_dist, current_node] = q.top();
            q.pop();

            for (const auto& edge : out_edges[current_node->value]) {
                Node<T>* neighbor = edge.destination;
                int weight = edge.label;
                int new_dist = current_node->distance + weight;

                if (new_dist < neighbor->distance) {
                    neighbor->distance = new_dist;
                    neighbor->parent = current_node;
                    q.emplace(new_dist, neighbor);
                }
            }
        }

        std::vector<std::vector<T>> all_paths;
        for (const auto& [val, node] : all_nodes) {
            if (node->distance == INT_MAX) continue;

            std::vector<T> path;
            for (Node<T>* at = node; at != nullptr; at = at->parent) {
                path.push_back(at->value);
            }

            std::reverse(path.begin(), path.end());

            if (path.size() > 1 || path.front() == start) {
                all_paths.push_back(path);
            }
        }

        for (auto& [val, node] : all_nodes) {
            node->distance = INT_MAX;
            node->parent = nullptr;
        }

        return all_paths;
    }
    std::unordered_map<T, std::unordered_map<T, L>> FloydWarshall() {

        std::unordered_map<T, std::unordered_map<T, L>> dist;
        for (const auto& [u, node_u] : all_nodes) {
            for (const auto& [v, node_v] : all_nodes) {
                if (u == v) {
                    dist[u][v] = 0;
                } else {
                    dist[u][v] = INT_MAX;
                }
            }
        }

        for (const auto& [u, edges] : out_edges) {
            for (const auto& edge : edges) {
                dist[edge.source->value][edge.destination->value] = edge.label;
            }
        }

        for (const auto& [k, _] : all_nodes) {
            for (const auto& [i, _] : all_nodes) {
                for (const auto& [j, _] : all_nodes) {
                    if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX) {
                        if (dist[i][j] > dist[i][k] + dist[k][j]) {
                            dist[i][j] = dist[i][k] + dist[k][j];
                        }
                    }
                }
            }
        }
        return dist;
    }

    std::vector<Edge<T, L>> getUndirectedEdges() {
        std::vector<Edge<T, L>> edges;
        std::set<std::pair<T, T>> visited;
        for (const auto& [val, node] : all_nodes) {
            for (const auto& [val, node] : all_nodes) {
                if (out_edges.contains(val)) {
                    for (const auto& edge : out_edges[val]) {
                        auto u = edge.source;
                        auto v = edge.destination;
                        if (u > v) std::swap(u, v);
                        if (!visited.contains({u->value, v->value})) {
                            visited.emplace(u->value, v->value);
                            edges.push_back(edge);
                        }
                    }
                }
            }
        }
        return edges;
    }
    std::vector<Edge<T, L>> mstKruskal() {
        auto edges = getUndirectedEdges();

        for (const auto& [val, node] : all_nodes) {
            DisjointSet<T>::makeSet(node);
        }

        std::sort(edges.begin(), edges.end());

        std::vector<Edge<T, L>> results;
        for (const auto& edge : edges) {
            Node<T>* source = DisjointSet<T>::findSet(edge.source);
            Node<T>* dest = DisjointSet<T>::findSet(edge.destination);

            if (source != dest) {
                results.push_back(edge);
                DisjointSet<T>::unionSet(dest, source);
            }
        }
        return results;
    }
    bool dfsTopological(const T& node_val, std::unordered_map<T, int>& visited,int& time)
    {
        visited[node_val] = 1;
        Node<T>* node = get_node(node_val);
        node->start = ++time;

        for (const T& neighbor : getNextNodes(node_val)) {
            if (visited[neighbor] == 1) {
                return false;
            }
            if (visited[neighbor] == 0) {
                if (!dfsTopological(neighbor, visited, time))
                    return false;
            }
        }
        node->finish = ++time;
        visited[node_val] = 2;
        return true;
    }
    std::vector<T> topologicalSort() {
        std::unordered_map<T, int> visited;
        int time = 0;

        for (auto& [val, node] : all_nodes) {
            node->start = 0;
            node->finish = 0;
            visited[val] = 0;
        }

        for (const auto& [val, _] : all_nodes) {
            if (visited[val] == 0) {
                if (!dfsTopological(val, visited, time)) {
                    std::cout << "Cycle detected - no topological order\n";
                    return {};
                }
            }
        }

        std::vector<Node<T>*> nodes_vec;
        for (auto& [val, node] : all_nodes) {
            nodes_vec.push_back(node);
        }

        std::sort(nodes_vec.begin(), nodes_vec.end(), [](Node<T>* a, Node<T>* b) {
            return a->finish > b->finish;
        });

        std::vector<T> sorted;
        for (const auto& node : nodes_vec) {
            sorted.push_back(node->value);
        }
        return sorted;
    }


};

#endif // GRAPH_H
