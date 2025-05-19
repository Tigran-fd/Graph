#ifndef DISJOINT_SET_H
#define DISJOINT_SET_H

#include "Node.h"

template <class T>
class DisjointSet {
public:
    static Node<T> *makeSet(Node<T> *root) {
        root->parent = root;
        root->rank = 0;
        return root;
    }
    static Node<T> *findSet(Node<T>* root) {
        while (root->parent != root) {
            root = root->parent;
        }
        return root;
    }
    static void unionSet(Node<T>* x, Node<T>* y) {
        Node<T>* a = findSet(x);
        Node<T>* b = findSet(y);

        if (a == b) return;

        if (a->rank > b->rank) {
            b->parent = a;
        } else if (a->rank < b->rank) {
            a->parent = b;
        } else {
            b->parent = a;
            ++a->rank;
        }
    }
};
#endif //DISJOINT_SET_H
