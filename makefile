# Compiler and flags
CXX = g++
# Add Eigen include path - adjust this path based on your system:
# For Windows (typical vcpkg location):
EIGEN_PATH = /usr/include/eigen3
# For Linux: EIGEN_PATH = /usr/include/eigen3

CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -g -I$(EIGEN_PATH)

# Target executable
TARGET = imu_fusion

# Source files
SRCS = main.cpp IMUFusion.cpp

# Object files
OBJS = $(SRCS:.cpp=.o)

# Build target
$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ 

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -f $(OBJS) $(TARGET) $(TARGET).exe
