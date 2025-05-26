#include "GraphClass/Graph.h"
#include <iostream>
#include <string>

int main() {
    Graph<std::string, int> g;

    g.addNode("A");
    g.addNode("B");
    g.addNode("C");
    g.addNode("D");
    g.addNode("E");
    g.addNode("F");

    g.addEdge("A", "C", 1);
    g.addEdge("B", "C", 4);
    g.addEdge("C", "D", 1);
    g.addEdge("D", "E", 2);
    g.addEdge("F", "E", 1);

    std::vector<std::string> result = g.topologicalSort();

    for (const auto& node : result) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

    return 0;
}
