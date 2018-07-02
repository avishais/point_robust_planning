#compiler
OMPL_DIR = /usr
INC_CLASSES = ./utils/
INC_PLANNERS = ./planners/
# INC_VALIDITY = ./validity_checkers/
INC_RUN = ./run/

GL_INCPATH = -I/usr/include/
GL_LIBPATH = -L/usr/lib/ -L/usr/X11R6/lib/
GL_LIBS = -lGLU -lGL -lXext -lX11 -lglut

# OPENCV = `pkg-config opencv --cflags --libs`

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/local/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${INC_RUN} -I${INC_CLASSES} -I${INC_PLANNERS} $(GL_INCPATH)  
LDFLAGS=  -L${OMPL_DIR}/local/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OMPL_DIR}/lib/x86_64-linux-gnu -Wl,-rpath ${OMPL_DIR}/local/lib/ $(GL_LIBS)

CPP_P = ${INC_RUN}plan.cpp ${INC_PLANNERS}SSTbelief.cpp ${INC_CLASSES}meanshift.cpp ${INC_CLASSES}dtw.cpp #${INC_PLANNERS}RRT.cpp 

all:
	$(CXX) ${CPP_P} -o p $(CXXFLAGS) $(LDFLAGS) -DSSTB -std=c++11





