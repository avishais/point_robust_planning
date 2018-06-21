#include "sys.h"

// Constructor for the robots
gen_system::gen_system() {}

Vector gen_system::f_func(Vector x, Vector u) {
    Vector f(2);

    f[0] = u[0] * cos(u[1]);
    f[1] = u[0] * sin(u[1]);

    return f;
}

Vector gen_system::prop(Vector x, Vector u, double dt) {
    
    Vector x_next(2);

    Vector f = f_func(x, u);

    x_next[0] = x[0] + f[0] * dt;
    x_next[1] = x[1] + f[1] * dt;

    return x_next;
}

void gen_system::printVector(Vector x) {

    cout << "[ " << x[0] << ", " << x[1] << " ]" << endl;
}