SRC=src/
HOUDINI_DIR=/opt/hfs17.0.459/
SHELL := /bin/bash
EIGEN=/usr/include/eigen3

all: inter freq obstacles

env:
	pushd $(HOUDINI_DIR);source houdini_setup;popd

inter: create_source solve_FS_inter deform_surface_inter
freq: create_source solve_FS deform_surface

obstacles: circle square texture


create_source: $(SRC)SOP_Create_Source.cpp $(SRC)SOP_Create_Source.hpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_Create_Source.cpp -I$(EIGEN) -g

solve_FS_inter: $(SRC)SOP_Solve_FS_inter.cpp $(SRC)SOP_Solve_FS_inter.hpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_Solve_FS_inter.cpp -I$(EIGEN) -g

deform_surface_inter: $(SRC)SOP_Deform_Surface_inter.cpp $(SRC)SOP_Deform_Surface_inter.cpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_Deform_Surface_inter.cpp -I$(EIGEN) -g

solve_FS: $(SRC)SOP_Solve_FS.cpp $(SRC)SOP_Solve_FS.hpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_Solve_FS.cpp -I$(EIGEN) -g

deform_surface: $(SRC)SOP_Deform_Surface.cpp $(SRC)SOP_Deform_Surface.cpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_Deform_Surface.cpp -I$(EIGEN) -g

texture: $(SRC)Grid.hpp  $(SRC)SOP_TextureObstacle_Src.cpp $(SRC)SOP_TextureObstacle_Src.hpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_TextureObstacle_Src.cpp -I$(EIGEN) -I/usr/local/include/SDL2 -lgomp -lSDL2_image -g

circle: $(SRC)SOP_CircleObstacle_Src.cpp $(SRC)SOP_CircleObstacle_Src.hpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_CircleObstacle_Src.cpp -I$(EIGEN) -g

square: $(SRC)SOP_SquareObstacle_Src.cpp $(SRC)SOP_SquareObstacle_Src.hpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_SquareObstacle_Src.cpp -I$(EIGEN) -g

clean:
	rm ($SRC)*.o
