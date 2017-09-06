INC = -I ../include
CXXFLAGS += -DDEBUG -g

all: serial_com 

# $^ - The names of all the prerequisites, with spaces between them.
# $@ - The file name of the target of the rule. 
serial_com: communication.cpp serial.cpp motorController.cpp motion.cpp
	g++ $(INC) $^ -std=c++11 -o $@ $(CXXFLAGS)

clean:
	rm serial_com
