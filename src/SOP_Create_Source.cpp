/*
 * Copyright (c) 2019
 *	Side Effects Software Inc.  All rights reserved.
 *
 * Redistribution and use of Houdini Development Kit samples in source and
 * binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. The name of Side Effects Software may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE `AS IS' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *----------------------------------------------------------------------------
 * Create_Source SOP
 *----------------------------------------------------------------------------
 * create one source that can be used as input.
 * Parameters:
 *    -Position (X, Y, Z): position of the source (X, Z: coordintate on the water surface, Y=0)
 *    -Amplitude
 *    -Phase (not used yet)
 *    -Minimum/maximum wavelength, wavelength multiplicative step: used to compute the range of wl
 *    -Type (not used yet, keep at 0)
 *    -Size of the buffer: size of the buffer recording past amplitude
 *    -Damping
 */


#include "SOP_Create_Source.hpp"

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

void newSopOperator(OP_OperatorTable *table)
{
  table->addOperator(new OP_Operator(
				     "create_source_fs",
				     "Create Source FS",
				     SOP_Create_Source::myConstructor,
				     SOP_Create_Source::myTemplateList,
				     0,
				     0,
				     nullptr,  
				     OP_FLAG_GENERATOR));
}
static PRM_Name names[] = {
  PRM_Name("pos",  "Position"),
  PRM_Name("amp",     "Amplitude"),
  PRM_Name("phase",   "Phase"),
  PRM_Name("wl_min",  "Minimum Wavelength"),
  PRM_Name("wl_max",  "Maximum Wavelength"),
  PRM_Name("wl_step",  "Wavelength Multiplicative Step"),
  PRM_Name("type",   "Type (point or line)"),
  PRM_Name("buffer_size",   "Size of the buffer containing past amplitudes"),
  PRM_Name("damping",   "Damping"),
};

PRM_Template
SOP_Create_Source::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
	       0, 0, SOP_Node::getGroupSelectButton(
						    GA_GROUP_POINT)),
  PRM_Template(PRM_XYZ_J,     3, &names[0], PRMzeroDefaults),
  PRM_Template(PRM_FLT_J,     1, &names[1], PRMoneDefaults, 0,
	       &PRMscaleRange),
  PRM_Template(PRM_FLT_J,     1, &names[2], PRMzeroDefaults, 0,
	       &PRMscaleRange),
  PRM_Template(PRM_FLT_J,     1, &names[3], PRMoneDefaults),
  PRM_Template(PRM_FLT_J,     1, &names[4], PRMoneDefaults),
  PRM_Template(PRM_FLT_J,     1, &names[5], PRMoneDefaults),
  PRM_Template(PRM_INT_J,     1, &names[6], PRMzeroDefaults),
  PRM_Template(PRM_INT_J,     1, &names[7], new PRM_Default(500)),
  PRM_Template(PRM_FLT_J,     1, &names[8], PRMzeroDefaults),
  PRM_Template(),
};


OP_Node *
SOP_Create_Source::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
  return new SOP_Create_Source(net, name, op);
}

SOP_Create_Source::SOP_Create_Source(OP_Network *net, const char *name, OP_Operator *op)
  : SOP_Node(net, name, op)
{
  // This indicates that this SOP manually manages its data IDs,
  // so that Houdini can identify what attributes may have changed,
  // e.g. to reduce work for the viewport, or other SOPs that
  // check whether data IDs have changed.
  // By default, (i.e. if this line weren't here), all data IDs
  // would be bumped after the SOP cook, to indicate that
  // everything might have changed.
  // If some data IDs don't get bumped properly, the viewport
  // may not update, or SOPs that check data IDs
  // may not cook correctly, so be *very* careful!
  mySopFlags.setManagesDataIDs(true);
}

SOP_Create_Source::~SOP_Create_Source()
{
}
OP_ERROR
SOP_Create_Source::cookInputGroups(OP_Context &context, int alone)
{
  // The SOP_Node::cookInputPointGroups() provides a good default
  // implementation for just handling a point selection.
  return cookInputPointGroups(
			      context, // This is needed for cooking the group parameter, and cooking the input if alone.
			      myGroup, // The group (or NULL) is written to myGroup if not alone.
			      alone,   // This is true iff called outside of cookMySop to update handles.
			      // true means the group will be for the input geometry.
			      // false means the group will be for gdp (the working/output geometry).
			      true,    // (default) true means to set the selection to the group if not alone and the highlight flag is on.
			      0,       // (default) Parameter index of the group field
			      -1,      // (default) Parameter index of the group type field (-1 since there isn't one)
			      true,    // (default) true means that a pointer to an existing group is okay; false means group is always new.
			      false,   // (default) false means new groups should be unordered; true means new groups should be ordered.
			      true,    // (default) true means that all new groups should be detached, so not owned by the detail;
			      //           false means that new point and primitive groups on gdp will be owned by gdp.
			      0        // (default) Index of the input whose geometry the group will be made for if alone.
			      );
}


OP_ERROR
SOP_Create_Source::cookMySop(OP_Context &context)
{
  // Flag the SOP as being time dependent (i.e. cook on time changes)
  flags().timeDep = 0;

  float t = context.getTime();
  int fr = context.getFrame();

  // TODO: user def time step
  float dt_ = 0.1;
  // if (fr != 0) {
  //   dt_ = t/fr;
  // }
  float phase = PHASE(t);
  float amp = AMP(t);
  uint buffer_size = BUFFER_SIZE(0);
  
  std::vector<float> wave_lengths;

  // Compute the range of wavelength we want to use between WL_MIN and WL_MAX(t)
  // Note: multiplicative step between wl
  FLOAT wl = WL_MIN(0);
  int nb_wl = 1;
  wave_lengths.push_back(wl);

  float max_wl = WL_MAX(t);
  float step_wl = 1+WL_STEP(t);
  while (wl < max_wl) {
    wl *= step_wl;
    wave_lengths.push_back(wl);
    ++nb_wl;
  }
  
  // compute the number of time step between updates for each wavelength
  std::vector<int> ampli_steps(nb_wl);
  wl = wave_lengths[0];
  FLOAT period = 0.25*wl/velocity(2*M_PI/wl);
  int d_period = period/(dt_);
      
  if (d_period == 0) {
    ampli_steps[0] = 1;
  } else {
    ampli_steps[0] = d_period;
  }
  for (int w = 1; w < nb_wl; ++w) {
    wl = wave_lengths[w];
    period = 0.25*wl/velocity(2*M_PI/wl);
    d_period = period/(dt_*ampli_steps[0]);
    if (d_period == 0) {
      ampli_steps[w] = 1;
    } else {
      ampli_steps[w] = d_period*ampli_steps[0];
    }
  }

  // begin creation of geometry
  gdp->clearAndDestroy();
  GA_Offset start_ptoff;
  // for now we create one source (one point) per wl
  start_ptoff = gdp->appendPointBlock(nb_wl);
  UT_Vector3 Pvalue(X(t), Y(t), Z(t)); //user-defined position
  GA_Offset ptoff;
  {
    GA_FOR_ALL_PTOFF(gdp, ptoff) {
      gdp->setPos3(ptoff, Pvalue);
    }
  }
  if (TYPE(t) == 0) { // TYPE == 0 means point source (!! line sources note implemented)

    // creation of the primitive (one per wl), and link one source to each
    for (int w = 0; w < nb_wl; ++w) {
      GA_Offset vtxoff;
      GA_Offset prim_off = gdp->appendPrimitivesAndVertices(GA_PRIMPOLY, 1, 1, vtxoff, true);
      gdp->getTopology().wireVertexPoint(vtxoff, start_ptoff+w);
    }
  }

  //create amplitude buffer for each source and fill it
  GA_RWHandleF ampli_attrib(gdp->findFloatTuple(GA_ATTRIB_POINT, "ampli", buffer_size));
  if (!ampli_attrib.isValid()) {
    ampli_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_POINT, "ampli", buffer_size));
  }
  if (!ampli_attrib.isValid()) {
    addError(SOP_MESSAGE, "Failed to create attribute ampli");
    return error();
  }
 
  if (buffer_size > 100) {
    GA_FOR_ALL_PTOFF(gdp, ptoff) {
      for (uint i = 0; i < 100; i+=2) {
	ampli_attrib.set(ptoff, i, amp);
	ampli_attrib.set(ptoff, i+1, 0);
      }
      for (uint i = 100; i < buffer_size; i+=2) {
	ampli_attrib.set(ptoff, i, 0);
	ampli_attrib.set(ptoff, i+1, 0);
      }
    }
  } else {
    GA_FOR_ALL_PTOFF(gdp, ptoff) {
      for (uint i = 0; i < buffer_size; i+=2) {
	ampli_attrib.set(ptoff, i, amp);
	ampli_attrib.set(ptoff, i+1, 0);
      }
    
    }
  }

  // creation of the primitve attibutes
  GA_RWHandleF wl_attrib(gdp->findFloatTuple(GA_ATTRIB_PRIMITIVE, "wavelengths", 1));
  GA_RWHandleI as_attrib(gdp->findIntTuple(GA_ATTRIB_PRIMITIVE, "ampli_steps", 1));
  if (!wl_attrib.isValid()) {
    wl_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "wavelengths", 1));
  }
  if (!wl_attrib.isValid()) {
    addError(SOP_MESSAGE, "Failed to create attribute wavelengths");
    return error();
  }
  if (!as_attrib.isValid()) {
    as_attrib = GA_RWHandleI(gdp->addIntTuple(GA_ATTRIB_PRIMITIVE, "ampli_steps", 1));
  }
  if (!as_attrib.isValid()) {
    addError(SOP_MESSAGE, "Failed to create attribute ampli_steps");
    return error();
  }
  // setting primitive attibutes
  GA_Offset prim_off;
  GA_Offset lcl_start, lcl_end;
  int w = 0;
  for (GA_Iterator lcl_it((gdp)->getPrimitiveRange()); lcl_it.blockAdvance(lcl_start, lcl_end); ) {
    for (prim_off = lcl_start; prim_off < lcl_end; ++prim_off) {
      wl_attrib.set(prim_off, wave_lengths[w]);
      as_attrib.set(prim_off, ampli_steps[w]);
      ++w;
    }
  }

  // creation of the detail attibutes
  GA_RWHandleI bs_attrib(gdp->findIntTuple(GA_ATTRIB_DETAIL, "buffer_size", 1));
  GA_RWHandleI damping_attrib(gdp->findIntTuple(GA_ATTRIB_DETAIL, "damping", 1));
  if (!bs_attrib.isValid()) {
    bs_attrib = GA_RWHandleI(gdp->addIntTuple(GA_ATTRIB_DETAIL, "buffer_size", 1));
  }
  if (!bs_attrib.isValid()) {
    addError(SOP_MESSAGE, "Failed to create attribute buffer_size");
    return error();
  }
  bs_attrib.set(0,buffer_size);
  if (!damping_attrib.isValid()) {
    damping_attrib = GA_RWHandleI(gdp->addIntTuple(GA_ATTRIB_DETAIL, "damping", 1));
  }
  if (!damping_attrib.isValid()) {
    addError(SOP_MESSAGE, "Failed to create attribute damping");
    return error();
  }
  // setting detail attibutes
  bs_attrib.set(0,buffer_size);
  damping_attrib.set(0,DAMPING(t));
 
  wl_attrib->bumpDataId();
  as_attrib->bumpDataId();
  bs_attrib->bumpDataId();
  damping_attrib->bumpDataId();
     
  return error();
}
