/*
 *	Author: Kan Ouivirach
 *	email: zkan at cs dot ait dot ac dot th
 *
 */

#include <iostream>
#include <fstream>
#include <iostream>
#include <vector>


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

	DTW() {};
	~DTW() {};

	double dtwDist( Matrix s1, Matrix s2 );

	void getOptPath(Matrix, Matrix, Matrix);

};

