SRC=src/
HOUDINI_DIR=/opt/hfs17.0.459/
SHELL := /bin/bash
EIGEN=/usr/include/eigen3

all: inter freq

env:
	pushd $(HOUDINI_DIR);source houdini_setup;popd

inter: create_source solve_FS_inter deform_surface_inter
freq: create_source solve_FS deform_surface

create_source: $(SRC)SOP_Create_Source.cpp $(SRC)SOP_Create_Source.hpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_Create_Source.cpp -I$(EIGEN)

solve_FS_inter: $(SRC)SOP_Solve_FS_inter.cpp $(SRC)SOP_Solve_FS_inter.hpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_Solve_FS_inter.cpp -I$(EIGEN)

deform_surface_inter: $(SRC)SOP_Deform_Surface_inter.cpp $(SRC)SOP_Deform_Surface_inter.cpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_Deform_Surface_inter.cpp -I$(EIGEN)

solve_FS: $(SRC)SOP_Solve_FS.cpp $(SRC)SOP_Solve_FS.hpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_Solve_FS.cpp -I$(EIGEN)

deform_surface: $(SRC)SOP_Deform_Surface.cpp $(SRC)SOP_Deform_Surface.cpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_Deform_Surface.cpp -I$(EIGEN)

obstacle: $(SRC)Grid.hpp  $(SRC)SOP_TextureObstacle_Src.cpp $(SRC)SOP_TextureObstacle_Src.hpp $(SRC)definitions.hpp
	hcustom $(SRC)SOP_TextureObstacle_Src.cpp -I$(EIGEN) -I/usr/local/include/SDL2 -lgomp -lSDL2_image -g

clean:
	rm ($SRC)*.o
