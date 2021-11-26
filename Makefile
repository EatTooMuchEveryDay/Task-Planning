Z3_DIR = /usr
CXX_FLAGS=-std=c++17 -O2 -Wall -Wextra
INCLUDE_FLAGS=-I${Z3}/include `pkg-config --cflags opencv`
# Linker options
LD_FLAGS=-L${Z3}/lib -lz3 `pkg-config --libs opencv`

# The c++ compiler to invoke 
CXX=c++
all: Project5

clean:
	rm -f *.o
	rm -f Project5

%.o: ./%.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@

Project5: Project5.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -o $@ $^ $(LD_FLAGS)

