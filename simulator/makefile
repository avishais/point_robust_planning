#compiler
CXX= g++

GL_INCPATH = -I/usr/include/
GL_LIBPATH = -L/usr/lib/ #-L/usr/X11R6/lib/
GL_LIBS = -lGLU -lGL -lXext -lX11 -lglut #-lXmu -lXi 

all:
	$(CXX) main.cpp -o viz $(GL_INCPATH) $(GL_LIBS) -std=c++11
