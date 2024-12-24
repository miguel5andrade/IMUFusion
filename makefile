# Compiler and flags
CXX = g++

CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -g 
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
