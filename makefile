#compiler
OMPL_DIR = /usr
INC_CLASSES = ./proj_classes/
INC_PLANNERS = ./planners/
INC_VALIDITY = ./validity_checkers/
INC_RUN = ./run/

GL_INCPATH = -I/usr/include/
GL_LIBPATH = -L/usr/lib/ -L/usr/X11R6/lib/
GL_LIBS = -lGLU -lGL -lXext -lX11 -lglut

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/local/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${INC_CLASSES} -I${INC_PLANNERS} -I${PQP_DIR}/include $(GL_INCPATH)  
LDFLAGS=  -L${OMPL_DIR}/local/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OMPL_DIR}/lib/x86_64-linux-gnu -lm $(GL_LIBS)
LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system

CPPPQP = ${INC_VALIDITY}collisionDetection.cpp ${INC_VALIDITY}model.cpp

CPP_P = ${INC_RUN}plan.cpp # ${INC_PLANNERS}CBiRRT_PCS.cpp ${INC_PLANNERS}RRT_PCS.cpp ${INC_PLANNERS}SBL_PCS.cpp 

all:
	$(CXX) ${CPP_P} -o p $(CXXFLAGS) $(LDFLAGS) -std=c++11




