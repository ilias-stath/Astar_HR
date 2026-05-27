#include <iostream>
#include "Node.h"

using namespace std;


Node::Node(unsigned short x, uint8_t y, float c, float h, Node* parent) {
    this->x = x;
    this->y = y;
    totalCost = c + h;
    this->parent = parent;
}

void Node::setParameters(unsigned short x, uint8_t y, float c, float h, Node* parent) {
    this->x = x;
    this->y = y;
    totalCost = c + h;
    this->parent = parent;
}

unsigned short Node::getX() {
    return x;
}

uint8_t Node::getY() {
    return y;
}

float Node::getTotalCost() {
    return totalCost;
}

Node* Node::getParent() {
    return this->parent;
}

void Node::printNode() {
    cout << "x = " << x << " ,  y = " << static_cast<int>(y) << " ,  tot_cost = " << totalCost << " ,  parent = " << parent << endl;
}
