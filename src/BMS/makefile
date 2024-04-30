# Makefile for compiling uart.cpp

# Compiler settings
CXX = g++
CXXFLAGS = -std=c++11

# Source file and output executable
SOURCE = uart.cpp
EXECUTABLE = uart

# Build rule
all: $(EXECUTABLE)

$(EXECUTABLE): $(SOURCE)
	$(CXX) $(CXXFLAGS) $(SOURCE) -o $(EXECUTABLE) -lCppLinuxSerial

# Clean rule
clean:
	rm -f $(EXECUTABLE)
