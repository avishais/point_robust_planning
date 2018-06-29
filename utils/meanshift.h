#include <iostream>
#include <vector>

#define Ln cout << __LINE__ << endl;

using namespace std;

typedef vector< double > Vector;
typedef vector<vector< double >> Matrix;

typedef struct Cluster  {
    Cluster() {};
    void init( int dim, int num) {
        points.resize(num);
        num_points = num;
        centroid.resize(dim);
    }

    Matrix points;
    Vector centroid;
    double stddev;
    int num_points;

} cluster;

class MeanShift {
public:

    MeanShift() { set_kernel(NULL); }
    MeanShift(double (*_kernel_func)(double,double)) { set_kernel(kernel_func); }
    Vector meanshift(Matrix points);
    cluster meanshift(Matrix points, double r, double eps = 0.0001);

private:
    double (*kernel_func)(double,double);
    void set_kernel(double (*_kernel_func)(double,double));
    Vector shift_point(Vector point, Matrix points);

    // Simple NN search of a set of points within a radius from a given point
    Matrix naiveNN_radius(Vector, Matrix);

    // Returns the mean/centroid of a set of points
    Vector mean(Matrix points);
    
    // Return the L2-norm of the standard deviations along all axes
    double stddev(Matrix points, Vector mean);

    double kernel_bandwidth = 3;
    int dim;
    double eps_sqr;
    double R_sqr;
};