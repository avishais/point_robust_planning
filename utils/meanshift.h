#include <iostream>
#include <vector>

using namespace std;

typedef vector< double > Vector;
typedef vector<vector< double >> Matrix;

class MeanShift {
public:

    MeanShift() { set_kernel(NULL); }
    MeanShift(double (*_kernel_func)(double,double)) { set_kernel(kernel_func); }
    Vector meanshift(Matrix points, double r, double eps = 0.0001);

private:
    double (*kernel_func)(double,double);
    void set_kernel(double (*_kernel_func)(double,double));
    Vector shift_point(Vector point, Matrix points);
    Matrix naiveNN_radius(Vector, Matrix, double);
    Vector mean(Matrix points);

    double kernel_bandwidth = 3;
    int dim;
};