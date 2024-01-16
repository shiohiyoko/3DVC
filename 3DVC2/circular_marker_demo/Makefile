CC		= g++ -O2

INSTALL		= /usr/bin/install

PREFIX		= .

CFLAGS          = -Wall \
		  `pkg-config --cflags opencv4 eigen3 libpng`

LDFLAGS         = `pkg-config --libs opencv4 eigen3 libjpeg libpng glfw3`

LIBS            = `pkg-config --libs opencv4 libjpeg libpng glfw3` -lGLU -lGL

SRCS		= main.c \
		  camera.c \
		  application.c \
		  glfw_window.c \
		  ellipse.c \
		  ellipse_detection.c \
		  ellipse_fitting.c \
		  circular_marker.c \
		  circular_marker_detection.c \
		  GLMetaseq.c \
		  metasequoia.c

HDRS		= camera.h \
		  application.h \
		  glfw_window.h \
		  ellipse.h \
		  ellipse_detection.h \
		  ellipse_fitting.h \
		  circular_marker.h \
		  circular_marker_detection.h \
		  GLMetaseq.h \
		  metasequoia.h

OBJS		= $(SRCS:.c=.o)

PROGRAM		= circular_marker_demo

all:		$(PROGRAM)

$(PROGRAM):	$(OBJS) $(HDRS) 
		$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $(PROGRAM)

clean:;		rm -f *.o *~ $(PROGRAM)

###							End of Makefile
