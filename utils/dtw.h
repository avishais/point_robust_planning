/*
 *	Author: Kan Ouivirach
 *	email: zkan at cs dot ait dot ac dot th
 *
 */

#include <iostream>
#include <fstream>
#include <iostream>
#include <vector>
#include <math.h>


using namespace std;

typedef vector< double > Vector;
typedef vector<vector< double >> Matrix;

class DTW {
public:
	vector< vector<int> > mGamma;
	vector< vector<double> > mGamma_cont;
	
	enum {
		INF = 100000000
	};
	int min( int x, int y, int z );
	double min( double x, double y, double z );
	double normSq(Vector p1, Vector p2);
	double norm(Vector p1, Vector p2);

	DTW() { set_dl(0.2); };
	~DTW() {};

	double dtwDist( Matrix, Matrix );
	double dtwDist( Matrix );
	void getOptPath(Matrix, Matrix, Matrix);
	Matrix oversampling(Matrix s);

	void setReferencePath(Matrix r) {
		r_ = oversampling(r);
	}

private:
	double dl_;
	void set_dl(double dl) {
		dl_ = dl;
	}
	
	Matrix r_; // Reference path to be used if set


};

