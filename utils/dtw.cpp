#include "dtw.h"

int DTW::min( int x, int y, int z ) {
	if( ( x <= y ) && ( x <= z ) ) return x;
	if( ( y <= x ) && ( y <= z ) ) return y;
	if( ( z <= x ) && ( z <= y ) ) return z;
}

double DTW::min( double x, double y, double z ) {
	if( ( x <= y ) && ( x <= z ) ) return x;
	if( ( y <= x ) && ( y <= z ) ) return y;
	if( ( z <= x ) && ( z <= y ) ) return z;
}

double DTW::normSq(Vector p1, Vector p2) {
	double sum = 0;
	
	for (int i = 0; i < p1.size(); i++)
		sum += (p1[i] - p2[i]) * (p1[i] - p2[i]);

	return sum;
}

void DTW::getOptPath(Matrix D, Matrix r, Matrix t) {
	int n = D[0].size()-1;
	int m = D.size()-1;

	// There is a numeric error that must be solved. However, this function is nnot important for the DTW implementation as we only need the final distance.
	vector <vector<int> > W;
	W.push_back( {m, n} );
	while (n+m != 0) {
		if (n-1 == 0)
			m -= 1;
		else if (m-1 == 0)
				n -= 1;
		else
			if ( D[m-1][n] <= D[m][n-1] && D[m-1][n] <= D[m-1][n-1] ) {
				m -= 1;
				cout << 1 << endl;
			}
			else if ( D[m][n-1] <= D[m-1][n] && D[m][n-1] <= D[m-1][n-1] ) {
					n -= 1;
					cout << 2 << endl;
			}
				else if ( D[m-1][n-1] <= D[m-1][n] && D[m-1][n-1] <= D[m][n-1] ) {
					m -= 1;
					n -= 1;
					cout << 3 << endl;
				}

		W.push_back( {m, n} );
	}

	// Matrix S;
	// for (int i = W.size()-1; i >= 0; i--) {
	// 	m = W[i][0]; n = W[i][0];
	// 	S.push_back( {r[W[m]][0], r[W[m]][1], t[W[n]][0], t[W[n]][1]} );
	// }

	for (int i = 0; i < W.size(); i++) {
		for (int j = 0; j < W[i].size(); j++)
			cout << W[i][j] << " ";
		cout << endl;
	}

	// return S;
}

void initMatrix(Matrix &M, int m, int n) {
	M.resize(m);
	for (int i = 0; i < m; i++)
		M[i].resize(n);
} 

double DTW::dtwDist( Matrix r, Matrix t ) {

	int M = r.size();
	int N = t.size();

	Matrix d;
	initMatrix(d, M, N);
	for (int i = 0; i < M; i++)
		for (int j = 0; j < N; j++)
			d[i][j] = normSq(r[i], t[j]);

	Matrix D;
	initMatrix(D, M, N);
	D[0][0] = d[0][0];
	for (int i = 1; i < M; i++)
		D[i][0] = d[i][0] + D[i-1][0];
	for (int i = 1; i < N; i++)
		D[0][i] = d[0][i] + D[0][i-1];

	for( int i = 1; i < M; i++ ) 
		for( int j = 1; j < N; j++ ) 
				D[i][j] = d[i][j] + min( D[i-1][j], D[i][j-1], D[i-1][j-1] );

	// getOptPath(D, r, t);

	return D[M-1][N-1];

}


int main() {
	Matrix s1 = {{0, 0},{1, 1},{2, 1.5},{3, 1.5},{8, 1.5},{10, 1.75},{12, 0}};
	Matrix s2 = {{0, 0},{0.5, 0.5},{1, 1},{2, 1.2},{3, 1.5},{4, 1.5},{5, 1.5},{6.2, 1.5},{8, 1.5},{10, 1.75},{12, 0}}; 

	DTW T;
	cout << T.dtwDist( s1, s2) << endl;

	return 0;
}





