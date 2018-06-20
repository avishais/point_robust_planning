
#ifndef GEN_SYSTEM_
#define GEN_SYSTEM_

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>

#define PI 3.14159265359

using namespace std;

typedef vector< double > Vector;
typedef vector<vector< double >> Matrix;

class gen_system
{
private:

    // This is the f(.) function in \dot{x}=f(x,u)
    // Implemented for a 2D-state point
    Vector f_func(Vector x, Vector u);

public:
    // Constructor
    gen_system();

    // Propogate state x with random u and dt
    Vector prop(Vector x, Vector u, double dt);

    void test1();
};

#endif /*GEN_SYSTEM_*/