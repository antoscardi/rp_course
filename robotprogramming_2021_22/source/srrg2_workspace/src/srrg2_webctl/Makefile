CC=gcc
CCOPTS=-Wall -std=gnu99 -O3
CXX=g++
CXXOPTS=-Wall -std=gnu++11 -O3

COBJS=proc_control.o
HEADERS= proc_control.h proc_shell.h proc_websocket.h env.h
OBJS=proc_shell.o env.o

.phony=clean all

BINS= proc_webctl proc_shell simple
all:	$(BINS)


%.o:	%.c $(HEADERS)
	$(CC) -c $(CCOPTS) $<

%.o:	%.cpp $(HEADERS)
	$(CXX) -c $(CXXOPTS) $<

proc_webctl:	proc_webctl_main.cpp $(COBJS) $(OBJS) proc_websocket.o 
	$(CXX) -o $@ $^ $(CXXOPTS) -lpthread -lwebsockets

proc_shell:	proc_shell_main.cpp $(COBJS) $(OBJS)
	$(CXX) -o $@  $^ $(CXXOPTS) -lpthread 

simple:  simple.c
	$(CC) -o $@ $^

clean:
	rm -f *~ *.o $(BINS)
