#ifndef GRAPH_H
#define GRAPH_H

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <iostream>
#include <stack>
#include <algorithm>

#include "Node.h"
#include "Edge.h"

template <class T, class L>
class Graph {
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


};

#endif // GRAPH_H
