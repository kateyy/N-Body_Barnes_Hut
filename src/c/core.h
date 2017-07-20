#pragma once

#include <stdint.h>

int node_quantity;
int roots_quantity;

struct coord
{
    double x;
    double y;
    double z;
};

struct body
{
    struct coord position;
    struct coord force;
    struct coord speed;
    double acel;
    double mass;
};

struct node
{
    struct coord start;
    struct coord end;
    int bodies_quantity;
    int initialized;
    int has_center_of_mass;
    int deep;
    struct body **bodies;
    struct node *UNE;
    struct node *USE;
    struct node *USW;
    struct node *UNW;
    struct node *DNE;
    struct node *DSE;
    struct node *DSW;
    struct node *DNW;
    struct node *UP;
    struct body centerOfMass;
};

struct node *nodes;
struct node **roots;
struct body *bodies;

void initializeNode(struct node *node, struct node *up, double sx, double sy,
    double sz, double ex, double ey, double ez,
    int bodies_quantity, int deep);
void addBodyInNode(struct body *body, struct node *node);
void resetNodes(void);
void divideNode(struct node *node);
void setCenterOfMass(struct node *node);
void applyForceBetweenBodies(struct body *b1, struct body *b2);
int applyForce(struct node *node, struct body *body);
void forceOverNode(struct node *node, struct node *down, struct body *body,
    int inverse);
void init(void);
void benchMode(void);
