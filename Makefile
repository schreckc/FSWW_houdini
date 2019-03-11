SRC=src/
HOUDINI_DIR=/opt/hfs17.0.459/
SHELL := /bin/bash
EIGEN=/usr/include/eigen3

all: sources solve_FS

env:
	pushd $(HOUDINI_DIR);source houdini_setup;popd

sources: $(SRC)SOP_Source.cpp $(SRC)SOP_Source.hpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_Source.cpp -I$(EIGEN)

solve_FS: $(SRC)SOP_Solve_FS.cpp $(SRC)SOP_Solve_FS.hpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_Solve_FS.cpp -I$(EIGEN)

clean:
	rm ($SRC)*.o
