TARGET := $(notdir $(shell pwd))

CCFLAGS    := -std=gnu99
CPPFLAGS   := -std=c++11
LDLIBS     := -lm
LDFLAGS    := -g

CC         := gcc
CPP        := g++

sim: main.c attitude.c kalman_filter.c matrix.c quaternion.c vector.c
	$(CC) -o sim main.c attitude.c kalman_filter.c matrix.c quaternion.c vector.c $(CCFLAGS)
