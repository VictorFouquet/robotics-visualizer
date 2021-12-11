#include "vector3d.h"

int Vector3d::orientation(Vector3d q, Vector3d r)
{
    // Returns 0 if colinear, 1 if clockwise, -1 if counter clockwise
    long int a = (q.y - y) * (r.x - q.x);
    long int b = (q.x - x) * (r.y - q.y);
    long int ori = a - b;

    return ori == 0 ? ori : (ori > 0) ? 1 : -1;
}