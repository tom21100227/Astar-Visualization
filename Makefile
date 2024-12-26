PLATFORM = $(shell uname)

## Compiler flags
CFLAGS = -std=c++20

## Compilation flags
##comment out one or the other
##debugging
# CFLAGS += -g -DDEBUG
## verbpse flag
# CFLAGS += -DVERBOSE
##release
CFLAGS += -O3 -DNDEBUG
LDFLAGS=

CFLAGS += -Wall

ifeq ($(PLATFORM),Darwin)
## Mac OS X
CFLAGS += -m64  -Wno-deprecated -Wunused-function
# INCLUDEPATH=-I/System/Library/Frameworks/OpenGL.framework/Headers -I/usr/local/include -I/opt/homebrew/include
# LDFLAGS+= -m64 -lc -framework AGL -framework OpenGL -framework GLUT -framework Foundation -lsfml-audio -lsfml-system
INCLUDEPATH=-I/System/Library/Frameworks/OpenGL.framework/Headers -I/usr/local/include -I/opt/homebrew/include -I/opt/homebrew/Cellar/sfml/2.6.2/include
LIBPATH=-DAUDIO -L/usr/local/lib -L/opt/homebrew/lib -L/opt/homebrew/Cellar/sfml/2.6.2/lib
LDFLAGS+= -m64 -lc -framework AGL -framework OpenGL -framework GLUT -framework Foundation
else
## Linux
CFLAGS += -m64
INCLUDEPATH  = -I/usr/include/GL/
LIBPATH = -L/usr/lib64 -L/usr/X11R6/lib
LDFLAGS+=  -lGL -lglut -lrt -lGLU -lX11 -lm  -lXmu -lXext -lXi
endif


CC = g++ -Wall $(INCLUDEPATH)


PROGS = motion

default: $(PROGS)

audio:
	$(CC) $(LIBPATH) -o motion_with_audio motion.cpp geom.cpp $(CFLAGS) $(LDFLAGS) -lsfml-audio -lsfml-system

motion: motion.o geom.o
	$(CC) -o $@ motion.o geom.o $(LDFLAGS)

motion.o: motion.cpp  geom.h
	$(CC) -c $(CFLAGS)   motion.cpp  -o $@

geom.o: geom.cpp geom.h
	$(CC) -c $(CFLAGS)  geom.cpp -o $@

clean:
	rm *.o
	rm motion
	rm motion_with_audio
