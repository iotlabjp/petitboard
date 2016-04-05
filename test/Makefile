CC = gcc
MRAA_LIBS = -lmraa

all: test

test: test.o
	$(CC) -lm $(MRAA_LIBS) -o $@ $^

%.o: %.c
	$(CC) -Wall --std=c99 -c $<

clean :
	rm test test.o
