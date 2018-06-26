
#ifndef CLUS_
#define CLUS_

#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include <algorithm>

using namespace std;
using namespace cv;

typedef vector< double > Vector;
typedef vector<vector< double >> Matrix;

typedef struct  {
    Matrix points;
    Vector centroid;
    int num_points;

} cluster;

class kmeans_clustering {
    private:
        vector<Point2f> load_data(Matrix);

        int elbow(vector<Point2f> points, int num_points);

        int silhouette(vector<Point2f> points, int num_points);

        double sqEucDistance(Point2f p1, Point2f p2);

        
    public:

        vector<cluster> getClusters(Matrix P);

        void printClusters(vector<cluster>);   
};


#endif