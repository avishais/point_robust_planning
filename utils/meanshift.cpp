#include <stdio.h>
#include <math.h>
#include "meanshift.h"

using namespace std;

#define CLUSTER_EPSILON 0.5

double euclidean_distance(const Vector &point_a, const Vector &point_b){
    double total = 0;
    for(int i=0; i<point_a.size(); i++){
        const double temp = (point_a[i] - point_b[i]);
        total += temp*temp;
    }
    return sqrt(total);
}

double euclidean_distance_sqr(const Vector &point_a, const Vector &point_b){
    double total = 0;
    for(int i=0; i<point_a.size(); i++){
        const double temp = (point_a[i] - point_b[i]);
        total += temp*temp;
    }
    return (total);
}

double gaussian_kernel(double distance, double kernel_bandwidth){
    double temp =  exp(-1.0/2.0 * (distance*distance) / (kernel_bandwidth*kernel_bandwidth));
    return temp;
}

void MeanShift::set_kernel( double (*_kernel_func)(double,double) ) {
    if(!_kernel_func){
        kernel_func = gaussian_kernel;
    } else {
        kernel_func = _kernel_func;    
    }
}

Vector MeanShift::shift_point(Vector point, Matrix points ) {

    Vector shifted_point( dim , 0 ) ;

    double total_weight = 0;
    for(int i=0; i < points.size(); i++){
        Vector temp_point = points[i];
        double distance = euclidean_distance(point, temp_point);
        double weight = kernel_func(distance, kernel_bandwidth);

        for(int j=0; j < shifted_point.size(); j++) 
            shifted_point[j] += temp_point[j] * weight;
        total_weight += weight;
    }

    for(int i=0; i < shifted_point.size(); i++)
        shifted_point[i] /= total_weight;

    return shifted_point;
}

Matrix MeanShift::naiveNN_radius(Vector point, Matrix points, double r) {
    
    Matrix P;
    double r_sqr = r * r;
    for (int i = 0; i < points.size(); i++) 
        if (euclidean_distance_sqr(point, points[i]) < r_sqr)
            P.push_back(points[i]);

    return P;
}

Vector MeanShift::mean(Matrix points) {
    Vector m(dim, 0);
    for (int i = 0; i < points.size(); i++) {
        for (int j = 0; j < dim; j++)
            m[j] += points[i][j];
    }

    for (int j = 0; j < dim; j++)
        m[j] /= points.size();

    return m;
}

Vector MeanShift::meanshift(Matrix points, double r, double eps) {
    dim = points[0].size();

    const double eps_sqr = eps * eps;

    Vector x = mean(points);
    Vector x_prev(dim);
    do {
        x_prev = x;
        // Points within a radius around the current point
        Matrix M = naiveNN_radius(x, points, r);
        // Shift the current point
        x = shift_point(x, M);

    } while (euclidean_distance_sqr(x, x_prev) > eps_sqr);

    return x;
}

// int main() {
//     Matrix M = {{10.0, 10.0},{11.0, 9.0},{1.0, 1.0},{5.0, 4.0},{5.0, 5.0},{5.0, 6.0},{2.0, 1.0},{2.0, 2.0},{4.0, 5.0},{6.0, 5.0},{9.0, 9.0},{10.0, 9.0}};

//     MeanShift ms;
//     Vector x = ms.meanshift(M, 3, 0.00001);

//     cout << x[0] << " " << x[1] << endl;


// }

