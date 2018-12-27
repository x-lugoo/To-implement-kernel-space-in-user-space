
CC=gcc
CFLAGS=-g 
all: dma-user 

dma-user: dma-user.c hw-addresses.h
	$(CC) $< $(CFLAGS) -o $@  

clean:
	rm -rf dma-user dma-example 

.PHONY: clean 
