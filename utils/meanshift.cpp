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

Matrix MeanShift::naiveNN_radius(Vector point, Matrix points) {
    
    Matrix P;
    for (int i = 0; i < points.size(); i++) 
        if (euclidean_distance_sqr(point, points[i]) < R_sqr)
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

double MeanShift::stddev(Matrix points, Vector mean) {
    Vector sd(dim, 0);
    for (int i = 0; i < points.size(); i++)
        for (int j = 0; j < dim; j++)
            sd[j] += (points[i][j] - mean[j])*(points[i][j] - mean[j]);
    double sum = 0;
    for (int j = 0; j < dim; j++)
        sum += sd[j] / points.size();

    return sqrt(sum);
}

Vector MeanShift::meanshift(Matrix points) {

    Vector x = mean(points);
    Vector x_prev(dim);
    do {
        x_prev = x;
        // Points within a radius around the current point
        Matrix M = naiveNN_radius(x, points);

        // Shift the current point
        x = shift_point(x, M);

    } while (euclidean_distance_sqr(x, x_prev) > eps_sqr);

    return x;
}

cluster MeanShift::meanshift(Matrix points, double r, double eps) {

    dim = points[0].size();
    eps_sqr = eps * eps;
    R_sqr = r * r;

    Vector x = meanshift(points);

    Matrix M = naiveNN_radius(x, points);
    if (M.size()==0) {
        cout << "po: " << points.size() << endl;
        cout << "Ms: " << M.size() << endl;
        Vector m = mean(points);
        cout << m[0] << " " << m[1] << endl;
        cout << x[0] << " " << x[1] << endl;
    }

    cluster C;
    C.init(M[0].size(), M.size());
    C.points = M;
    C.centroid = mean(C.points);
    C.stddev = stddev(C.points, C.centroid);

    // Vector m = mean(points);
    // Matrix M = naiveNN_radius(m, points);
    // cluster C;
    // C.init(M[0].size(), M.size());
    // C.points = M;
    // C.centroid = m;
    // C.stddev = stddev(C.points, C.centroid);

    return C;
}

// int main() {
//     Matrix M = {{10.0, 10.0},{11.0, 9.0},{1.0, 1.0},{5.0, 4.0},{5.0, 5.0},{5.0, 6.0},{2.0, 1.0},{2.0, 2.0},{4.0, 5.0},{6.0, 5.0},{9.0, 9.0},{10.0, 9.0}};

//     MeanShift ms;
//     Vector x = ms.meanshift(M, 3, 0.00001);

//     cout << x[0] << " " << x[1] << endl;
// }

