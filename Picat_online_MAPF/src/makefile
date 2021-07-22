CC=g++
CPPFLAGS=-O3 -std=c++11 -Wall -Wextra -pedantic -Wno-unused-result

DEPS = *.h

OBJ = main.o instance.o replan_all.o replan_single.o independence_detection.o a_star.o

all: olmapf

%.o: %.cpp $(DEPS)
	$(CC) $(CPPFLAGS) -c -o $@ $<

olmapf: $(OBJ)
	$(CC) $(CPPFLAGS) -o $@ $^
	cp $@ ../bin/

clean:
	rm -f *.o olmapf gmon.out instance.pi tmp.out results_*
	rm -f ../bin/olmapf ../bin/gmon.out ../bin/instance.pi ../bin/tmp.out ../bin/results_* ../bin/single_plan.tmp

	