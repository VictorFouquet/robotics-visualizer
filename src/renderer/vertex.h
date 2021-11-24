#pragma once

struct  Vertex
{
    bool operator==(Vertex v) { return x == v.x && y == v.y; };
    int x, y;
};
