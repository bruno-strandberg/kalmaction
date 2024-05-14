CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17

SRCDIR = src
INCDIR = inc

BOOST_INCLUDE = -I/usr/include/boost
OPENCV_INCLUDE = -I/usr/include/opencv4
EIGEN_INCLUDE = -I/usr/include/eigen3
INCLUDE = -I$(INCDIR) $(BOOST_INCLUDE) $(OPENCV_INCLUDE) $(EIGEN_INCLUDE)

BOOST_LIBS = -lboost_program_options
OPENCV_LIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs

SRCS = $(wildcard $(SRCDIR)/*.cpp)
OBJS = $(SRCS:.cpp=.o)
DEPS = $(wildcard $(INCDIR)/*.h)

EXEC = kalmaction

all: $(EXEC)

$(EXEC): $(EXEC).cpp $(OBJS)
	@echo "  Building $@ with pre-requisities $^..."  
	$(CXX) $(CXXFLAGS) $(INCLUDE) $^ -o $@ $(OPENCV_LIBS) $(BOOST_LIBS)

$(SRCDIR)/%.o: $(SRCDIR)/%.cpp $(DEPS)
	@echo "  Building $@ with pre-requisities $^..."  
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

clean:
	rm -f $(SRCDIR)/*.o $(EXEC) $(EXEC).o $(EXEC).cpp~ $(SRCDIR)/*~ $(INCDIR)/*~ Makefile~

.PHONY: print

print:
	@echo $(SRCS)
	@echo $(OBJS)
	@echo $(DEPS)
	@echo $(CXXFLAGS)

