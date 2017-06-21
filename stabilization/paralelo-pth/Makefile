CXX=g++
OBJECTS=par.o
INCLUDES = -I/usr/local/include/opencv -I/usr/local/include -L/usr/local/lib
LIBS = -lopencv_highgui -lopencv_videoio -lopencv_video -lopencv_imgproc -lopencv_core -lm -lpthread
CFLAGS= -pedantic -Wall -O2

par: par.o
	#$(CXX) $(OBJECTS) -o seq `pkg-config --cflags --libs opencv`
	$(CXX)  $(CFLAGS) $(OBJECTS)  -o par $(INCLUDES) $(LIBS)

par.o: #por aqui seus .h

clean:
	rm -f par par.o

run: par
	./par


