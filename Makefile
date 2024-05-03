all: clean sim

clean:
	rm -f *.o sim

userapp: 
	gcc -o sim sim.c -pthread
	

