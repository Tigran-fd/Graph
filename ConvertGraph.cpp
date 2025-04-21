#include "GraphClass/Graph.h"
#include <vector>

Graph<int, int> ConvertMatrix(const std::vector<std::vector<int>>& matrix)
{
    Graph<int, int> graph;

    for (int i = 0; i < matrix.size(); i++) {
        graph.addNode(i);
    }

    for (int i = 0; i < matrix.size(); i++)
    {
        for (int j = 0; j < matrix[i].size(); j++)
        {
            if (matrix[i][j] != 0)
            {
                graph.addEdge(i, j, matrix[i][j]);
            }
        }
    }

    return graph;
}


Graph<int, int> ConvertAdjacencyList(const std::vector<std::pair<int, int>>& adjacencylist)
{
    Graph<int, int> graph;
    for (const auto& [a, b] : adjacencylist)
    {
        graph.addNode(a);
        graph.addNode(b);
    }
    for (const auto& [a, b] : adjacencylist)
    {
        graph.addEdge(a, b, 0);
    }

    return graph;
}