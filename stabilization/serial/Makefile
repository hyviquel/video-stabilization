CXX=g++
OBJECTS=ser.o
INCLUDES = -I/usr/local/include/opencv -I/usr/local/include -L/usr/local/lib
LIBS = -lopencv_highgui -lopencv_videoio -lopencv_video -lopencv_imgproc -lopencv_core -lm
CFLAGS= -pedantic -Wall -g -O2

ser: ser.o
	#$(CXX) $(OBJECTS) -o seq `pkg-config --cflags --libs opencv`
	$(CXX) $(CFLAGS) $(OBJECTS) -o ser $(INCLUDES) $(LIBS)

ser.o: #por aqui seus .h

clean:
	rm -f ser ser.o

run: ser
	./ser


