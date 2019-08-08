/*
 * MIT License
 * 
 * Copyright (c) 2019 Camille Schreck
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *----------------------------------------------------------------------------
 * Texture Obstacle SOP
 *----------------------------------------------------------------------------
 * Create set of point sources along an offset surface of the obstacle.
 * Range of wavelength (and time step per wl), and is copied from input geometry, as well as
 *    the detail attibute.
 * Create on primitve and subset of sources for each wl.
 * Spacing depends on the wavelength and the parameter "density".
 * I use this node also to create a set of point sampling the border of the obstacle (offset=0)
 *    for the boundary conditions.
 * Note: the density of the boundary points should be at least twice the density of the
 *    sources.
 * Note 2: for the aperiodic version, do not forget to check the "interactive sources" box in
 *   the parameter of the node creating the sources of the obstacle.
 * Note 3: the density is also limited by the resolution of the
 *   texture (cannot have more than one source per pixel).
 */


#include "SOP_TextureObstacle_Src.hpp"

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>
#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Director.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <UT/UT_DSOVersion.h>
#include <SYS/SYS_Math.h>
#include <IMG/IMG_File.h>
#include <UT/UT_PtrArray.h>
#include <PXL/PXL_Common.h>
#include <PXL/PXL_Raster.h>

#include "definitions.hpp"
#include <vector>
#include <string>


void newSopOperator(OP_OperatorTable *table) {
  table->addOperator(new OP_Operator("texture_obstacle_src_fs",
				     "Texture Obstacle_Src Sources FS",
				     SOP_Texture_Obstacle_Src::myConstructor,
				     SOP_Texture_Obstacle_Src::myTemplateList,
				     1,
				     1,
				     nullptr,  
				     OP_FLAG_GENERATOR));
}
static PRM_Name names[] = {
  PRM_Name("center",  "Center"),
  PRM_Name("off",     "Offset distance"),
  PRM_Name("density",   "Density"),
  PRM_Name("file",   "Texture File"),
  PRM_Name("inter_src",   "Interactive sources"),
};

PRM_Default* off_default = new PRM_Default(0.3);
PRM_Default* dens_default = new PRM_Default(3);

PRM_Template SOP_Texture_Obstacle_Src::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
  	       0, 0, SOP_Node::getGroupSelectButton(GA_GROUP_POINT)),
  PRM_Template(PRM_XYZ_J,     3, &names[0], PRMzeroDefaults),
  PRM_Template(PRM_FLT_J,     1, &names[1], off_default),
  PRM_Template(PRM_FLT_J,     1, &names[2], dens_default),
  PRM_Template(PRM_PICFILE,   1, &names[3], PRMzeroDefaults),
  PRM_Template(PRM_TOGGLE_J,    1, &names[4]),
  PRM_Template(),
};


OP_Node *SOP_Texture_Obstacle_Src::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
  return new SOP_Texture_Obstacle_Src(net, name, op);
}

SOP_Texture_Obstacle_Src::SOP_Texture_Obstacle_Src(OP_Network *net, const char *name, OP_Operator *op)
  : SOP_Node(net, name, op) {
  mySopFlags.setManagesDataIDs(true);
}

SOP_Texture_Obstacle_Src::~SOP_Texture_Obstacle_Src() {
}

OP_ERROR SOP_Texture_Obstacle_Src::cookInputGroups(OP_Context &context, int alone) {
  return cookInputPointGroups(context,
			      myGroup,
			      alone,
			      true,
			      0,
			      -1,
			      true,
			      false,
			      true,
			      0);
}

void SOP_Texture_Obstacle_Src::loadTexture() {
  int n_rows = 600;
  int n_cols = 600;  
  gr = Grid(n_rows, n_cols, 0.05);
  UT_String str;
  FILE(str, 0);
  UT_PtrArray<PXL_Raster *> images;
  IMG_File *file = IMG_File::open(str);
  if(file) {
    bool success = file->readImages(images);
    file->close();
    delete file;
    if(success) {
      uint w = images[0]->getXres();
      uint h = images[0]->getYres();
      FLOAT pix_per_cell_x = 3*(FLOAT)h/(FLOAT)n_rows;
      FLOAT pix_per_cell_y = 3*(FLOAT)w/(FLOAT)n_cols;
      unsigned char* pixels = (unsigned char*) images[0]->getPixels();
    
#pragma omp parallel for
      for (uint i = 0; i < n_rows; ++i) {
      	uint x = i*pix_per_cell_x;
      	for (uint j = 0; j < n_cols; ++j) {
      	  uint y = j*pix_per_cell_y;
	  gr(i, j) = (uint)pixels[(uint)(x*w+y)]/256.0;
	  if (gr(i,j) < 0.1) {
	    gr(i,j) = 0;
	  } else {
	    gr(i, j) = 1;
	  }
	}
		
      }
      delete images[0];
    }
  }
  
}


OP_ERROR SOP_Texture_Obstacle_Src::cookMySop(OP_Context &context) {
  // Flag the SOP as being time dependent (i.e. cook on time changes)
  flags().timeDep <= 0;
  float t = context.getTime();
  bool is_inter = INTER_SRC(t);
  
  OP_AutoLockInputs inputs(this);
  if (inputs.lock(context) >= UT_ERROR_ABORT)
    return error();
  
  gdp->clearAndDestroy();

  const GU_Detail *is = inputGeo(0); //input sources (used to get wavelength)
  int nb_wl = is->getPrimitiveRange().getEntries();

  std::vector<float> wave_lengths(nb_wl);
  std::vector<int> ampli_steps(nb_wl);

  GA_ROHandleF w_handle(is->findAttribute(GA_ATTRIB_PRIMITIVE, "wavelengths"));
  GA_ROHandleF as_handle(is->findAttribute(GA_ATTRIB_PRIMITIVE, "ampli_steps"));
  if (!w_handle.isValid()) {
    addError(SOP_ATTRIBUTE_INVALID, "wavelengths input sources");
    return error();
  }
  if (!as_handle.isValid()) {
    addError(SOP_ATTRIBUTE_INVALID, "amplis_steps input sources");
    return error();
  }
  GA_ROHandleI bs_handle(is->findAttribute(GA_ATTRIB_DETAIL, "buffer_size"));
  GA_ROHandleF damping_handle(is->findAttribute(GA_ATTRIB_DETAIL, "damping"));
  if (!bs_handle.isValid()) {
    addError(SOP_ATTRIBUTE_INVALID, "buffer sizes input sources");
    return error();
  }
  if (!damping_handle.isValid()) {
    addError(SOP_ATTRIBUTE_INVALID, "damping input sources");
    return error();
  }
  int buffer_size = 2;
  if (is_inter) {
    buffer_size = bs_handle.get(0);
  }
  float damping = damping_handle.get(0);
   
  loadTexture();
  int w = 0;
  GA_Range range_is = is->getPrimitiveRange();
  for(GA_Iterator itis = range_is.begin(); itis != range_is.end(); ++itis, ++w) {
    GA_Offset prim_off = *itis;
    float wl = w_handle.get(prim_off);
    wave_lengths[w] = wl;
    ampli_steps[w]= as_handle.get(prim_off);
  }

  for (int w = 0; w < nb_wl; ++w) {
    float wl = wave_lengths[w];
    float density = DENSITY(t)/wl;
    float off = OFF(t);

    std::list<VEC2> sources;
    int nb_points = 0;
    int k_max = 0 + wl*off/gr.getCellSize();
    // if (k_max > 25) {
    //   k_max = 25;
    // }
    Grid tmp_prev = gr;
    Grid tmp = gr ;
    for (int k = 0; k < k_max; ++k) {
      tmp = tmp_prev;
      for (int i = 0; i < gr.getNbRows(); ++i) {
    	for (int j = 0; j < gr.getNbCols(); ++j) {
    	  if (tmp_prev(i, j) != 0 && (tmp_prev(i-1, j) == 0 || tmp_prev(i, j-1) == 0 || tmp_prev(i+1, j) == 0 || tmp_prev(i, j+1) == 0)) {
	    tmp(i, j) = 0;
    	  }
    	}
      }
      tmp_prev = tmp;
    }
    float os = 1.0/gr.getCellSize()/density;
    if (wl >= 1) {
      os /= 2;
    }
    int offset = floor(os)+1;
    for (int i = 0; i < gr.getNbRows(); ++i) {
      for (int j = 0; j < gr.getNbCols(); ++j) {
	if (tmp(i, j) != 0 && (tmp(i-1, j) == 0 || tmp(i, j-1) == 0 || tmp(i+1, j) == 0 || tmp(i, j+1) == 0)) {
	  bool no_neigh = true;
	  for (int k = -offset; k <= offset; ++k) {
	    for (int h = -offset; h <= offset; ++h) {
    	      if (tmp(i+k, j+h) == 0.5) {
    		no_neigh = false;
    		break;
    	      }
	    }
	  }
	
	  if (no_neigh) {
	    VEC2 pos = VEC2((i - gr.getNbRows()/2)*gr.getCellSize(), (j-gr.getNbCols()/2)*gr.getCellSize());
	    sources.push_back(pos);
	    ++nb_points;
	    tmp(i, j) = 0.5;
	  }
	}
      }
    }
    GA_Offset ptoff = gdp->appendPointBlock(nb_points);
    GA_Offset vtxoff;
    GA_Offset prim_off = gdp->appendPrimitivesAndVertices(GA_PRIMPOLY, 1, nb_points, vtxoff, true);
    std::list<VEC2>::iterator it = sources.begin();
    for (int i = 0; i < nb_points;  ++i, ++it) {
      VEC2 pos = *it;
      gdp->getTopology().wireVertexPoint(vtxoff+i,ptoff+i);
      gdp->setPos3(ptoff+i, UT_Vector3(pos(0)+CX(t), CY(t), pos(1)+CZ(t)));
    }
  }

  GA_RWHandleF wl_attrib(gdp->findFloatTuple(GA_ATTRIB_PRIMITIVE, "wavelengths", 1));
  GA_RWHandleF as_attrib(gdp->findFloatTuple(GA_ATTRIB_PRIMITIVE, "ampli_steps", 1));
  GA_RWHandleF bs_attrib(gdp->findFloatTuple(GA_ATTRIB_DETAIL, "buffer_size", 1));
  GA_RWHandleF damping_attrib(gdp->findFloatTuple(GA_ATTRIB_DETAIL, "damping", 1));
  if (!wl_attrib.isValid()) {
    wl_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "wavelengths", 1));
  }
  if (!wl_attrib.isValid()) {
    addError(SOP_MESSAGE, "Failed to create attribute wavelengths");
    return error();
  }
  if (!as_attrib.isValid()) {
    as_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "ampli_steps", 1));
  }
  if (!as_attrib.isValid()) {
    addError(SOP_MESSAGE, "Failed to create attribute ampli_steps");
    return error();
  }
  if (!bs_attrib.isValid()) {
    bs_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_DETAIL, "buffer_size", 1));
  }
  if (!bs_attrib.isValid()) {
    addError(SOP_MESSAGE, "Failed to create attribute buffer_size");
    return error();
  }
  if (!damping_attrib.isValid()) {
    damping_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_DETAIL, "damping", 1));
  }
  if (!damping_attrib.isValid()) {
    addError(SOP_MESSAGE, "Failed to create attribute damping");
    return error();
  }

  bs_attrib.set(0, buffer_size);
  damping_attrib.set(0, damping);

  w = 0;
  GA_Offset prim_off, lcl_start, lcl_end;
  for (GA_Iterator lcl_it((gdp)->getPrimitiveRange()); lcl_it.blockAdvance(lcl_start, lcl_end); ) {
    for (prim_off = lcl_start; prim_off < lcl_end; ++prim_off) {
      wl_attrib.set(prim_off, wave_lengths[w]);
      as_attrib.set(prim_off, ampli_steps[w]);
      ++w;
    }
  }
  GA_RWHandleF ampli_attrib(gdp->findFloatTuple(GA_ATTRIB_POINT, "ampli", buffer_size));
  if (!ampli_attrib.isValid()) {
    ampli_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_POINT, "ampli", buffer_size));
  }
  if (!ampli_attrib.isValid()) {
    addError(SOP_MESSAGE, "Failed to create attribute ampli");
    return error();
  }
  {
    GA_Offset ptoff; 
    GA_FOR_ALL_PTOFF(gdp, ptoff) {
      for (uint i = 0; i < buffer_size; ++i) {
	ampli_attrib.set(ptoff, i, 0);
      }
    }
  }
  gdp->bumpDataIdsForAddOrRemove(true, true, true);

  return error();
}
