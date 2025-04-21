#ifndef NODE_H
#define NODE_H

template <class T>
class Node {
public:
    T value;
    explicit Node(const T& value) : value(value) {}
};

#endif // NODE_H
