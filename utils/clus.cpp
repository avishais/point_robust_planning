
#include "clus.h"

vector<Point2f> kmeans_clustering::load_data(Matrix M) {
    vector<Point2f> points;

    for (int i = 0; i < (int)M.size(); i++)
        points.push_back(Point2f(M[i][0],M[i][1]));

    return points;
}

int kmeans_clustering::elbow(vector<Point2f> points, int num_points) {
    Vector sse(num_points);
    Mat labels, centers;
    int attempts=3, flags=cv::KMEANS_RANDOM_CENTERS; // hey, just guessing here
    TermCriteria tc;

    for (int k = 1; k < min(7, num_points); k++) {
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
        cout << sse[k] << " ";
    cout << endl;
 
    for (int k = 1; k < (int)sse.size() && sse[k]; k++)
        if ( (sse[k-1]-sse[k]) / sse[k-1] < 0.3)
            return k;
    return 1;
}

double kmeans_clustering::sqEucDistance(Point2f p1, Point2f p2) {
    return (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y);
}

int kmeans_clustering::silhouette(vector<Point2f> points, int num_points) {
    Vector sse(num_points);
    Mat labels, centers;
    int attempts=3, flags=cv::KMEANS_RANDOM_CENTERS; // hey, just guessing here
    TermCriteria tc;

    int K = min(7, num_points);
    Vector S(K);
    for (int k = 2; k < K; k++) {
        kmeans(points,k,labels,tc,attempts,flags, centers);   // points will get cast implicitly to Mat
        
        double s = 0;
        for (int i = 0; i < num_points; i++) {
            double a = 0;
            int a_c = 0;
            Matrix b;
            for (int j = 0; j < k; j++)
                b.push_back({0,0});

            int curr_cluster = labels.at<int>(i);

            for (int j = 0; j < num_points; j++) {
                if (i==j)
                    continue;
                if (labels.at<int>(j) == curr_cluster) {
                    a += sqEucDistance(points[i], points[j]);
                    a_c++;
                }
                else { // if (labels.at<int>(j) ~= curr_cluster)
                    b[labels.at<int>(j)][0] += sqEucDistance(points[i], points[j]);
                    b[labels.at<int>(j)][1]++;
                }
            }

            // cout << "i: " << i << endl;
            // for (int j = 0; j < 2; j++)
            //     cout << b[j][0] << " ";
            // cout << endl;
            // for (int j = 0; j < 2; j++)
            //     cout << b[j][1] << " ";
            // cout << endl;
            // cin.ignore();
            
            // if there is only one point in the cluster
            if (a_c==0) {
                s += 1;
                continue;
            }

            a = a/a_c;
            double min_b = 1e9;
            for (int j = 0; j < k; j++) {
                if (b[j][1]) {
                    b[j][0] /= b[j][1];
                    if (b[j][0] < min_b)
                        min_b = b[j][0];
                }
            }
            // cout << a << " " << min_b << " " << (min_b - a) / std::max(a, min_b) << endl;
            // Silhoutte cumsum
            s += (min_b - a) / std::max(a, min_b);
        }
        S[k-1] = s / num_points;    
    }
    double maxS = 0;
    for (int k = 0; k < (int)S.size(); k++) {
        if (S[k] > maxS) {
            maxS = S[k];
            K = k+1;
        }
    }

    if (K < 0.7)
        return 1;

    return K;
}

vector<cluster> kmeans_clustering::getClusters(Matrix P) {
    vector<Point2f> points = load_data(P);
    int num_points = P.size();

    // int K = elbow(points, num_points);
    int K = silhouette(points, num_points);


    Mat labels, centers;
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

    // printClusters(C);
    // cin.ignore();

    return C;
}

void kmeans_clustering::printClusters(vector<cluster> C) {

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

//     kmeans_clustering KM;

//     vector<cluster> C = KM.getClusters(M);

//     KM.printClusters(C);



//     return 0;
// }