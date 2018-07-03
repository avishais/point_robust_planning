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

#define LN cout << __LINE__ << endl;

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
	int min( int x, int y, int z ) const;
	double min( double x, double y, double z ) const;
	double normSq(Vector p1, Vector p2) const;
	double norm(Vector p1, Vector p2) const;

	DTW() { set_dl(0.3); };
	~DTW() {};

	double dtwDist( Matrix, Matrix );
	double dtwDist( Matrix ) const;
	void getOptPath(Matrix, Matrix, Matrix);
	Matrix oversampling(Matrix) const;
	int trim(Matrix) const;

	void setReferencePath(Matrix r) {
		r_ = oversampling(r);
	}

	Matrix r_; // Reference path to be used if set

private:
	double dl_;
	void set_dl(double dl) {
		dl_ = dl;
	}
	
};

