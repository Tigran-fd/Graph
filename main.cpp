#include <iostream>
#include "GraphClass/Graph.h""

int main() {
    Graph<char, int> g;

    g.addNode('A');
    g.addNode('B');
    g.addNode('C');
    g.addNode('D');
    g.addNode('E');

    g.addEdge('A', 'B', 1);
    g.addEdge('B', 'A', 1);
    g.addEdge('A', 'C', 5);
    g.addEdge('C', 'A', 5);
    g.addEdge('B', 'C', 3);
    g.addEdge('C', 'B', 3);
    g.addEdge('B', 'D', 4);
    g.addEdge('D', 'B', 4);
    g.addEdge('C', 'D', 2);
    g.addEdge('D', 'C', 2);
    g.addEdge('D', 'E', 7);
    g.addEdge('E', 'D', 7);

    for (const auto mst = g.mstKruskal(); const auto& edge : mst) {
        std::cout << edge.source->value << " - " << edge.destination->value
                  << " : " << edge.label << "\n";
    }

    return 0;
}
