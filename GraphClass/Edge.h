#ifndef EDGE_H
#define EDGE_H

#include "Node.h"

template <class T, class L>
class Edge {
public:
    Node<T>* source;
    Node<T>* destination;
    L label;

    Edge(Node<T>* source, Node<T>* destination, const L& label)
        : source(source), destination(destination), label(label) {}

    bool operator==(const Edge<T, L>& other) const {
        return source == other.source
            && destination == other.destination
            && label == other.label;
    }
};

template <class T, class L>
bool operator<(const Edge<T, L>& a, const Edge<T, L>& b) {
    return a.label < b.label;
}

template <class T, class L>
struct EdgeHash {
    std::size_t operator()(const Edge<T, L>& edge) const {
        return std::hash<T>()(edge.source->value) ^
               std::hash<T>()(edge.destination->value) ^
               std::hash<L>()(edge.label);
    }
};

#endif // EDGE_H
