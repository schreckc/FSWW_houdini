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
 *
 *----------------------------------------------------------------------------
 * Circle Obstacle SOP
 *---------------------------------------------------------------------------
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
 */

#include "SOP_CircleObstacle_Src.hpp"

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>
#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Director.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <UT/UT_DSOVersion.h>
#include <SYS/SYS_Math.h>

#include "definitions.hpp"
#include <vector>


void newSopOperator(OP_OperatorTable *table) {
  table->addOperator(new OP_Operator("circle_obstacle_src_fs",
				     "Circle Obstacle Sources FS",
				     SOP_Circle_Obstacle_Src::myConstructor,
				     SOP_Circle_Obstacle_Src::myTemplateList,
				     1,
				     2,
				     nullptr,  
				     OP_FLAG_GENERATOR));
}
static PRM_Name names[] = {
  PRM_Name("center",  "Center"),
  PRM_Name("off",     "Offset distance"),
  PRM_Name("density",   "Density"),
  PRM_Name("radius",   "Radius"),
  PRM_Name("inter_src",   "Interactive sources"),
};

PRM_Default* off_default = new PRM_Default(0.3);
PRM_Default* dens_default = new PRM_Default(3);

PRM_Template
SOP_Circle_Obstacle_Src::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
	       0, 0, SOP_Node::getGroupSelectButton(GA_GROUP_POINT)),
  PRM_Template(PRM_XYZ_J,     3, &names[0], PRMzeroDefaults),
  PRM_Template(PRM_FLT_J,     1, &names[1], off_default),
  PRM_Template(PRM_FLT_J,     1, &names[2], dens_default),
  PRM_Template(PRM_FLT_J,     1, &names[3], PRMoneDefaults),
  PRM_Template(PRM_TOGGLE_J,  1, &names[4]),
  PRM_Template(),
};


OP_Node *SOP_Circle_Obstacle_Src::myConstructor(OP_Network *net, const char *name, OP_Operator *op) {
  return new SOP_Circle_Obstacle_Src(net, name, op);
}

SOP_Circle_Obstacle_Src::SOP_Circle_Obstacle_Src(OP_Network *net, const char *name, OP_Operator *op)
  : SOP_Node(net, name, op) {
  mySopFlags.setManagesDataIDs(true);
}

SOP_Circle_Obstacle_Src::~SOP_Circle_Obstacle_Src()
{
}
OP_ERROR
SOP_Circle_Obstacle_Src::cookInputGroups(OP_Context &context, int alone)
{
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


OP_ERROR SOP_Circle_Obstacle_Src::cookMySop(OP_Context &context) {

  flags().timeDep = 0;
  float t = context.getTime();
  bool is_inter = INTER_SRC(t);
  OP_AutoLockInputs inputs(this);
  if (inputs.lock(context) >= UT_ERROR_ABORT)
    return error();
  
  gdp->clearAndDestroy();

  // get details and primitives attibutes from the input sources
  const GU_Detail *is = inputGeo(0); //input sources

  int nb_inputs = getInputsArraySize();
  int nb_wl = 0;
  std::vector<float> wave_lengths;
  std::vector<int> ampli_steps;
  if (nb_inputs == 2) {
     const GU_Detail *bp = inputGeo(1);
    const GA_ROHandleI ws_attrib(bp->findIntTuple(GA_ATTRIB_DETAIL, "winsize", 1));
    if (!ws_attrib.isValid()) {
      addError(SOP_ATTRIBUTE_INVALID, "winsize");
      return error();
    }
    int winsize = ws_attrib.get(0);
    nb_wl = winsize/2;
    wave_lengths = std::vector<float>(winsize);
    ampli_steps = std::vector<int>(winsize);
    const GA_ROHandleF wl_attrib(bp->findFloatTuple(GA_ATTRIB_DETAIL, "wavelengths", winsize/2));
    if (!wl_attrib.isValid()) {
      addError(SOP_MESSAGE, "Cannot find attribute wavelengths");
      return error();
    }
    const GA_ROHandleF as_attrib(bp->findFloatTuple(GA_ATTRIB_DETAIL, "ampli_steps", winsize/2));
    if (!as_attrib.isValid()) {
      addError(SOP_MESSAGE, "Cannot find attribute ampli_steps");
      return error();
    }
    for (uint w = 0; w < nb_wl; ++w) {
      wave_lengths[w] = wl_attrib.get(0, w);
      ampli_steps[w] = as_attrib.get(0, w);
    }
  } else {
    nb_wl = is->getPrimitiveRange().getEntries();
    wave_lengths = std::vector<float>(nb_wl);
    ampli_steps = std::vector<int>(nb_wl);
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
     int w = 0;
     GA_Range range_is = is->getPrimitiveRange();
     for(GA_Iterator itis = range_is.begin(); itis != range_is.end(); ++itis, ++w) {
       GA_Offset prim_off = *itis;
       float wl = w_handle.get(prim_off);
       wave_lengths[w] = wl;
       ampli_steps[w]= as_handle.get(prim_off);
     }
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
    buffer_size= bs_handle.get(0);
  }
  float damping = damping_handle.get(0);
   
 

  //create set of points for each wavelength, and liked them to their corresponding primitve
  for (int w = 0; w < nb_wl; ++w) {
    float wl = wave_lengths[w];
    float density = DENSITY(t)/wl;
    float off = OFF(t);
    float radius = RADIUS(t) - off;;
    int nb_points = 2*M_PI*radius*density;
    float angle_step = 2*M_PI/nb_points;

    VEC3 dir(1, 0, 0);
    VEC3 center(CX(t), CY(t), CZ(t));
    MAT3 rotation;
    rotation <<
      cos(angle_step), 0, -sin(angle_step),
      0, 0, 0, 
      sin(angle_step), 0, cos(angle_step);

    GA_Offset ptoff = gdp->appendPointBlock(nb_points);
    GA_Offset vtxoff;
    GA_Offset prim_off = gdp->appendPrimitivesAndVertices(GA_PRIMPOLY, 1, nb_points, vtxoff, true);
    for (int i = 0; i < nb_points;  ++i) {
      VEC3 pos = center + radius*dir;
      gdp->getTopology().wireVertexPoint(vtxoff+i,ptoff+i);
      gdp->setPos3(ptoff+i, UT_Vector3(pos(0), pos(1), pos(2)));
      dir = rotation*dir;
      dir.normalize();
    }

  }

  // create and set attibutes for primitves and detail
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
  
  int w = 0;
  GA_Offset prim_off, lcl_start, lcl_end;
  for (GA_Iterator lcl_it((gdp)->getPrimitiveRange()); lcl_it.blockAdvance(lcl_start, lcl_end); ) {
    for (prim_off = lcl_start; prim_off < lcl_end; ++prim_off) {
      wl_attrib.set(prim_off, wave_lengths[w]);
      as_attrib.set(prim_off, ampli_steps[w]);
      ++w;
    }
  }

  //create amplitude buffer attributes for the point sources and set them to zero  
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
