#ifndef NODE_H
#define NODE_H


class Node {
private:
private:

    unsigned short x;
    uint8_t y;
    float totalCost;
    Node* parent;

public:
    Node(unsigned short x, uint8_t y, float c, float h, Node* parent);

    void setParameters(unsigned short x, uint8_t y, float c, float h, Node* parent);

    unsigned short getX();

    uint8_t getY();

    float getTotalCost();

    Node* getParent();

    void printNode();
};


#endif