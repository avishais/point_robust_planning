#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GL/glut.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <unistd.h>
#include <string.h>
#include "../utils/point_sys_def.h"

using namespace std;

typedef vector< double > Vector;
typedef vector< vector<double> > Matrix;

const char* robot_pfile = "../path/path.txt";
const char* tree_pfile  = "../path/tree.txt";
Matrix path, tree;
Vector start(2), goal(2), reached(2);
double fmax_x = 10., fmin_x = -10., fmax_y = 10., fmin_y = -10.;

Vector obs1 = OBS1;
Vector obs2 = OBS2;
Vector obs3 = OBS3;

Vector convert2window(Vector v) {

    int i = 0;
    do {
        v[i]   = (v[i]-fmin_x)/(fmax_x-fmin_x) * 2. - 1;
        v[i+1] = (v[i+1]-fmin_y)/(fmax_y-fmin_y) * 2. - 1;
        i += 2;
    } while (i < v.size());
    return v;
}

void get_path_data() {
    ifstream inFile;
    inFile.open(robot_pfile);
	if (!inFile) {
        cout << "\nError opening file.\n";
        return;
    }
	
    Vector v(2);
    while (inFile >> v[0] && inFile >> v[1]) {
        v = convert2window(v);
        path.push_back(v);
    }

    start = path[0];
    goal = path[path.size()-1];
    path.pop_back();
    reached =  path[path.size()-1];
}

void get_tree_data() {
    ifstream inFile;
    inFile.open(tree_pfile);
	if (!inFile) {
        cout << "\nError opening file.\n";
        return;
    }
	
    Vector v(4);
    while (inFile >> v[0] && inFile >> v[1] && inFile >> v[2] && inFile >> v[3]) {
        v = convert2window(v);
        tree.push_back(v);
    }
}

void DrawCircle(Vector c, int num_segments) {
    glBegin(GL_POLYGON);
    glColor3f(0.3f, 0.2f, 0.0f); // Red
    for (int ii = 0; ii < num_segments; ii++)   {
        float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);//get the current angle
        Vector p =  convert2window({c[2] * cosf(theta) + c[0], c[2] * sinf(theta) + c[1]});
        glVertex2f(p[0], p[1]);//output vertex 
    }
    glEnd();
}

void KeyboardCB(unsigned char key, int x, int y) 
{
	switch(key)
	{
	case 'q':
		exit(0);
	}

	glutPostRedisplay();
}

void display() {
   glClearColor(1.0f, 1.0f, 1.0f, 0.0f); // Set background color to white and not opaque
   glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer (background)
 
    // Draw tree
    glLineWidth(1);
    for (int i = 0; i < tree.size(); i++) {
        glBegin(GL_LINES);              
            glColor3f(0.0f, 0.0f, 0.0f); 
            glVertex2f(tree[i][0], tree[i][1]);    // x, y
            glVertex2f(tree[i][2], tree[i][3]);
        glEnd();
    }

    // Draw path
    glLineWidth(5);
    for (int i = 0; i < path.size()-1; i++) {
        glBegin(GL_LINES);              
            glColor3f(0.0f, 0.0f, 0.0f); 
            glVertex2f(path[i][0], path[i][1]);    // x, y
            glVertex2f(path[i+1][0], path[i+1][1]);
        glEnd();
    }

    glPointSize(6);
    glBegin(GL_POINTS);
        // Start point
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex2f(start[0], start[1]);    // x, y
        // Goal point
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex2f(goal[0], goal[1]);    // x, y
        // Reached point
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex2f(reached[0], reached[1]);    // x, y
    glEnd();

    DrawCircle(obs1, 20);
    DrawCircle(obs2, 20);
    DrawCircle(obs3, 20);
 
    glFlush();  // Render now
}


int main(int argc, char **argv)
{
    get_path_data();
    get_tree_data();
    // for (int i = 0; i < tree.size(); i++)
    //     cout << tree[i][0] << " " << tree[i][1] << endl;

	glutInit(&argc, argv);
	// glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);

	// set the callbacks
    glutInitWindowSize(500, 500);   // Set the window's initial width & height
    glutInitWindowPosition(50, 50); // Position the window's initial top-left corner

    // create the window
	glutCreateWindow("Robot View");

    glutDisplayFunc(display); // Register display callback handler for window re-paint
   	glutKeyboardFunc(KeyboardCB);

	glutMainLoop();

    return 0;
}