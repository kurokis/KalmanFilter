TARGET := $(notdir $(shell pwd))

CCFLAGS    := -std=gnu99
CPPFLAGS   := -std=c++11 -w
LDLIBS     := -lm
LDFLAGS    := -g

CC         := gcc
CPP        := g++

sim: main.cpp attitude.c kalman_filter.c matrix.c quaternion.c vector.c
	$(CPP) -o sim main.cpp attitude.c kalman_filter.c matrix.c quaternion.c vector.c $(CPPFLAGS)
