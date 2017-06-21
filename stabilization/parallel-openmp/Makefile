CXX=g++
OBJECTS=par.o
INCLUDES = -I/usr/local/include/opencv -I/usr/local/include -L/usr/local/lib
LIBS = -lopencv_highgui -lopencv_videoio -lopencv_video -lopencv_imgproc -lopencv_core -lm -fopenmp
CFLAGS= -pedantic -Wall -O2

par: par.o
	$(CXX) $(CFLAGS) $(OBJECTS) -o par $(INCLUDES) $(LIBS)

par.o: par.cpp
	$(CXX) $(CFLAGS) -c par.cpp

clean:
	rm -f par par.o

run: par
	./par


