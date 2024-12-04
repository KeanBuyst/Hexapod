# Compiler
CXX = g++

# Directories
SRC_DIR = src
HEADER_DIR = headers
BIN_DIR = bin

# Libraries
LIBS = -lpigpio -lrt -lpthread -lbluetooth

# Compiler flags
CXXFLAGS = -I$(HEADER_DIR) -std=c++17

# Find all source files and corresponding object files
SRCS = $(shell find $(SRC_DIR) -name '*.cpp')
OBJS = $(patsubst $(SRC_DIR)/%.cpp,$(BIN_DIR)/%.o,$(SRCS))

# Target executable
TARGET = $(BIN_DIR)/run

# Default target
all: $(TARGET)

# Link the target executable
$(TARGET): $(OBJS) | $(BIN_DIR)
	$(CXX) $(OBJS) -o $@ $(LIBS)

# Compile source files into object files
$(BIN_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BIN_DIR)
	@mkdir -p $(dir $@)  # Create the subdirectory in obj if it doesn't exist
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# Clean up
clean:
	rm -rf $(BIN_DIR)

# Phony targets
.PHONY: all clean