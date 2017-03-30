#makefile for mifareCrack
exe=crack
C=gcc
CXX = g++
CXXFLAGS = --std=c++11
objects=build/main.o build/RC522.o build/MFrec.o build/crapto1.o build/crypto1.o

exe: $(objects)
	g++ $(objects) -o $(exe) -lpthread

build/%.o: src/%.cpp 
	$(CXX) -c $(CXXFLAGS) $^ -o $@
build/%.o: src/%.c
	$(C) -c $^ -o $@

clean:
	rm $(exe) $(objects)
