
#include "clus.h"

vector<Point2f> kmeans_clus::load_data(Matrix M) {
    vector<Point2f> points;

    for (int i = 0; i < (int)M.size(); i++)
        points.push_back(Point2f(M[i][0],M[i][1]));

    return points;
}

int kmeans_clus::elbow(vector<Point2f> points, int num_points) {
    Vector sse(num_points);
    Mat labels, centers;
    int attempts=3, flags=cv::KMEANS_RANDOM_CENTERS; // hey, just guessing here
    TermCriteria tc;

    for (int k = 1; k < 7; k++) {
        sse[k-1] = 0;
        kmeans(points,k,labels,tc,attempts,flags, centers);   // points will get cast implicitly to Mat
        
        // For each cluster
        for (int i = 0; i < k; i++) {
            double xc = centers.at<float>( i, 0 );
            double yc = centers.at<float>( i, 1 );
            for (int j = 0; j < num_points; j++) {
                if (labels.at<int>(j)==i) {
                    sse[k-1] += (points[j].x-xc)*(points[j].x-xc) + (points[j].y-yc)*(points[j].y-yc);
                }
            }
        }
    }
   
    for (int k = 1; k < (int)sse.size() && sse[k]; k++)
        if ( (sse[k-1]-sse[k]) / sse[k-1] < 0.3)
            return k;
    return 1;
}

vector<cluster> kmeans_clus::getClusters(Matrix P) {
    vector<Point2f> points = load_data(P);
    int num_points = P.size();

    int K = elbow(points, num_points);

    Mat labels,centers;
    int attempts=3, flags=cv::KMEANS_RANDOM_CENTERS; // hey, just guessing here
    TermCriteria tc;
    kmeans(points, K, labels, tc, attempts, flags, centers);   // points will get cast implicitly to Mat

    vector<cluster> C;
    for (int i = 0; i < K; i++) {
        cluster c;
        c.centroid.push_back(centers.at<float>(i,0));
        c.centroid.push_back(centers.at<float>(i,1));
        c.num_points = 0;
        for (int j = 0; j < num_points; j++) {
            if (labels.at<int>(j)==i) {
                c.points.push_back({points[j].x, points[j].y});
                c.num_points++;
            }
        }
        C.push_back(c);
    }

    return C;
}

void kmeans_clus::printClusters(vector<cluster> C) {

    cout << "There are " << C.size() << " clusters." << endl;

    for (int i = 0; i < (int)C.size(); i++) {
        cout << "Cluster " << i+1 << ": " << endl;
        cout << "Centroid: < " << C[i].centroid[0] << ", " << C[i].centroid[1] << " >\n";
        cout << "Number of points: " << C[i].num_points << endl;
        cout << "Points: \n";
        for (int j = 0; j < C[i].num_points; j++) 
            cout << "     [ " << C[i].points[j][0] << ", " << C[i].points[j][1] << " ]" << endl;
    }
}

// int main() {

//     Matrix M = {{10.0, 10.0},{11.0, 9.0},{1.0, 1.0},{5.0, 4.0},{5.0, 5.0},{5.0, 6.0},{2.0, 1.0},{2.0, 2.0},{4.0, 5.0},{6.0, 5.0},{9.0, 9.0},{10.0, 9.0},{3,3},{4,3},{3.5,4}};

//     kmeans_clus KM;

//     vector<cluster> C = KM.getClusters(M);

//     KM.printClusters(C);



//     return 0;
// }