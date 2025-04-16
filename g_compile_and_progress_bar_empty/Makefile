BIN=mystudy.exe
C=g++
SRC=$(wildcard *.c)
OBJ=$(SRC:.c=.o)
LFLAGS=-o
FLAGS=-c
RM=rm -f

$(BIN):$(OBJ)
        @g++ -o $@ $^
        @echo "bian yi cheng gong"
%.o:%.c
        @$(C) $(FLAGS) $<
        @echo "zhuan huan cheng gong"
.PHONY:clean
clean:
        @rm -f $(BIN) $(OBJ)

.PHONY:test
test:
        @echo $(OBJ)
        @echo $(BIN)

.PHONY:cache
cache:
        @rm -f $(OBJ)
        @echo "clear cache"
