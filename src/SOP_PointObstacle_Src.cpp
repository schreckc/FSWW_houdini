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
 * Point Obstacle SOP
 *---------------------------------------------------------------------------
 * Create set of point sources (one per wavelennght) at the user-defined position.
 * Range of wavelength (and time step per wl), and is copied from input geometry, as well as
 *    the detail attibute.
 * Create on primitve and subset of sources for each wl.
 * Spacing depends on the wavelength and the parameter "density".
 * Note: for the aperiodic version, do not forget to check the "interactive sources" box in
 *   the parameter of the node creating the sources of the obstacle.
 */

#include "SOP_PointObstacle_Src.hpp"

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
  table->addOperator(new OP_Operator("point_obstacle_src_fs",
				     "Point Obstacle Sources FS",
				     SOP_Point_Obstacle_Src::myConstructor,
				     SOP_Point_Obstacle_Src::myTemplateList,
				     1,
				     2,
				     nullptr,  
				     OP_FLAG_GENERATOR));
}
static PRM_Name names[] = {
  PRM_Name("position",  "Position"),
  PRM_Name("inter_src",   "Interactive sources"),
};

PRM_Template
SOP_Point_Obstacle_Src::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
	       0, 0, SOP_Node::getGroupSelectButton(GA_GROUP_POINT)),
  PRM_Template(PRM_XYZ_J,     3, &names[0], PRMzeroDefaults),
  PRM_Template(PRM_TOGGLE_J,  1, &names[1]),
  PRM_Template(),
};


OP_Node *SOP_Point_Obstacle_Src::myConstructor(OP_Network *net, const char *name, OP_Operator *op) {
  return new SOP_Point_Obstacle_Src(net, name, op);
}

SOP_Point_Obstacle_Src::SOP_Point_Obstacle_Src(OP_Network *net, const char *name, OP_Operator *op)
  : SOP_Node(net, name, op) {
  mySopFlags.setManagesDataIDs(true);
}

SOP_Point_Obstacle_Src::~SOP_Point_Obstacle_Src()
{
}
OP_ERROR
SOP_Point_Obstacle_Src::cookInputGroups(OP_Context &context, int alone)
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


OP_ERROR SOP_Point_Obstacle_Src::cookMySop(OP_Context &context) {


   OP_AutoLockInputs inputs(this);
  if (inputs.lock(context) >= UT_ERROR_ABORT)
     return error();

  flags().timeDep = 0;
  float t = context.getTime();
  bool is_inter = INTER_SRC(t);
  
   gdp->clearAndDestroy();

  // get details and primitives attibutes from the input sources
  const GU_Detail *is = inputGeo(0); //input sources

  int nb_inputs = getInputsArraySize();
  int nb_wl = 0;
  std::vector<float> wave_lengths;
  std::vector<int> ampli_steps;
  if (nb_inputs == 2) {
     const GU_Detail *bp = inputGeo(1);
     nb_wl =  bp->getPrimitiveRange().getEntries();
  
    wave_lengths = std::vector<float>(nb_wl);
    ampli_steps = std::vector<int>(nb_wl);
    const GA_ROHandleF wl_attrib(bp->findFloatTuple(GA_ATTRIB_PRIMITIVE, "wavelength", 1));
    if (!wl_attrib.isValid()) {
      addError(SOP_MESSAGE, "Cannot find attribute wavelength");
      return error();
    }
    const GA_ROHandleI as_attrib(bp->findIntTuple(GA_ATTRIB_PRIMITIVE, "ampli_step", 1));
    if (!as_attrib.isValid()) {
      addError(SOP_MESSAGE, "Cannot find attribute ampli_step");
      return error();
    }
    uint w = 0;
    GA_Range range_prim = bp->getPrimitiveRange();
    for(GA_Iterator itp = range_prim.begin(); itp != range_prim.end(); ++itp, ++w) {
      wave_lengths[w] = wl_attrib.get(*itp);
      ampli_steps[w] = as_attrib.get(*itp);
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
   
 

  //create one point for each wavelength, and link them to their corresponding primitve
  for (int w = 0; w < nb_wl; ++w) {
    float wl = wave_lengths[w];
    VEC3 pos(X(t), Y(t), Z(t));

    GA_Offset ptoff = gdp->appendPointBlock(1);
    GA_Offset vtxoff;
    GA_Offset prim_off = gdp->appendPrimitivesAndVertices(GA_PRIMPOLY, 1, 1, vtxoff, true);
    gdp->getTopology().wireVertexPoint(vtxoff,ptoff);
    gdp->setPos3(ptoff, UT_Vector3(pos(0), pos(1), pos(2)));
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
