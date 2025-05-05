#ifndef NODE_H
#define NODE_H
#include <climits>

template <class T>
class Node {
public:
    T value;
    Node* parent;
    int distance = INT_MAX;
    explicit Node(const T& value) : value(value), parent(nullptr) {}
};

#endif // NODE_H
