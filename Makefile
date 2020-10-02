.PHONY: clean all

CC = g++
CFLAGS = -g -c -Wall -pedantic -Wextra -Wno-unused-parameter
LDFLAGS = -std=c++11

BIN = raytrace
SRC = main.cpp \
      parser.cpp \
      raytracer.cpp \
      object.cpp

OBJ = main.o parser.o raytracer.o object.o

all: $(BIN)

%.o: $(SRC)
	$(CC) $(LDFLAGS) $(CFLAGS) $(SRC)

$(BIN): $(OBJ)
	$(CC) $(OBJ) -o $(BIN)

clean:
	rm *.o $(BIN)

test: $(BIN)
	./$(BIN) scenes/basic.ray