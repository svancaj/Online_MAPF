CC=g++
CPPFLAGS=-std=c++11 -Wall -Wextra -pedantic 

all: intersection

intersection: intersection.cpp graphs.hpp
	$(CC) $(CPPFLAGS) -o $@ intersection.cpp
	cp $@ ../

clean:
	rm -f *.o intersection

	