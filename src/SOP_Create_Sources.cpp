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
 * Create_Sources SOP
 *----------------------------------------------------------------------------
 * create a set of sources from an input set of points that can be used as input.
 * Parameters:
 *    -Amplitude
 *    -Phase (not used yet)
 *    -Minimum/maximum wavelength, wavelength multiplicative step: used to compute the range of wl
 *    -Type (not used yet, keep at 0)
 *    -Size of the buffer: size of the buffer recording past amplitude
 *    -Damping
 */


#include "SOP_Create_Sources.hpp"

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
  table->addOperator(new OP_Operator("create_sources_fs",
				     "Create Sources FS",
				     SOP_Create_Sources::myConstructor,
				     SOP_Create_Sources::myTemplateList,
				     1,
				     1,
				     nullptr,  
				     OP_FLAG_GENERATOR));
}
static PRM_Name names[] = {
  PRM_Name("amp",     "Amplitude"),
  PRM_Name("phase",   "Phase"),
  PRM_Name("wl_min",  "Minimum Wavelength"),
  PRM_Name("wl_max",  "Maximum Wavelength"),
  PRM_Name("wl_step",  "Wavelength Multiplicative Step"),
  PRM_Name("type",   "Type (point or line)"),
  PRM_Name("buffer_size",   "Size of the buffer containing past amplitudes"),
  PRM_Name("damping",   "Damping"),
  PRM_Name("inter_src",   "Interactive sources"),
};

PRM_Template
SOP_Create_Sources::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
	       0, 0, SOP_Node::getGroupSelectButton(GA_GROUP_POINT)),
  PRM_Template(PRM_FLT_J,     1, &names[0], PRMoneDefaults, 0,
	       &PRMscaleRange),
  PRM_Template(PRM_FLT_J,     1, &names[1], PRMzeroDefaults, 0,
	       &PRMscaleRange),
  PRM_Template(PRM_FLT_J,     1, &names[2], PRMoneDefaults),
  PRM_Template(PRM_FLT_J,     1, &names[3], PRMoneDefaults),
  PRM_Template(PRM_FLT_J,     1, &names[4], PRMoneDefaults),
  PRM_Template(PRM_INT_J,     1, &names[5], PRMzeroDefaults),
  PRM_Template(PRM_INT_J,     1, &names[6], new PRM_Default(500)),
  PRM_Template(PRM_FLT_J,     1, &names[7], PRMzeroDefaults),
  PRM_Template(PRM_TOGGLE_J,  1, &names[8]),
  PRM_Template(),
};


OP_Node *
SOP_Create_Sources::myConstructor(OP_Network *net, const char *name, OP_Operator *op) {
  return new SOP_Create_Sources(net, name, op);
}

SOP_Create_Sources::SOP_Create_Sources(OP_Network *net, const char *name, OP_Operator *op)
  : SOP_Node(net, name, op) {
  mySopFlags.setManagesDataIDs(true);
}

SOP_Create_Sources::~SOP_Create_Sources()
{
}
OP_ERROR
SOP_Create_Sources::cookInputGroups(OP_Context &context, int alone)
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


OP_ERROR SOP_Create_Sources::cookMySop(OP_Context &context) {
  flags().timeDep = 0;

  float t = context.getTime();
  int fr = context.getFrame();
  
  // TODO: user def time step
  float dt_ = 0.1;
  // if (fr != 0) {
  //   dt_ = t/fr;
  // }
  float phase = PHASE(t);
  COMPLEX amp = (float)AMP(t)*exp(COMPLEX(0,1)*phase);
  bool is_inter = INTER_SRC(t);
  uint buffer_size = 2;
  if (is_inter) {
    buffer_size = BUFFER_SIZE(0);
  }
  std::vector<float> wave_lengths(0);

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

    OP_AutoLockInputs inputs(this);
  if (inputs.lock(context) >= UT_ERROR_ABORT)
    return error();
  // begin creation of geometry
  gdp->clearAndDestroy();
  const GU_Detail *fs = inputGeo(0);
  uint nb_pts = fs->getPointRange().getEntries();
  for (int w = 0; w < nb_wl; ++w) {
    GA_Offset ptoff = gdp->appendPointBlock(nb_pts);
    GA_Offset vtxoff;
    GA_Offset prim_off = gdp->appendPrimitivesAndVertices(GA_PRIMPOLY, 1, nb_pts, vtxoff, true);
    GA_Range range = fs->getPointRange();
    uint i = 0;
    for(GA_Iterator itfs = range.begin(); itfs != range.end(); ++itfs) {
      UT_Vector3 pos_fs = fs->getPos3(*itfs);
      gdp->getTopology().wireVertexPoint(vtxoff+i,ptoff+i);
      gdp->setPos3(ptoff+i, pos_fs);
      ++i;
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

  GA_Offset ptoff;
  if (buffer_size > 100) {
    GA_FOR_ALL_PTOFF(gdp, ptoff) {
      for (uint i = 0; i < 100; i+=2) {
	ampli_attrib.set(ptoff, i, real(amp));
	ampli_attrib.set(ptoff, i+1, imag(amp));
      }
      for (uint i = 100; i < buffer_size; i+=2) {
	ampli_attrib.set(ptoff, i, 0);
	ampli_attrib.set(ptoff, i+1, 0);
      }
    }
  } else {
    GA_FOR_ALL_PTOFF(gdp, ptoff) {
      for (uint i = 0; i < buffer_size; i+=2) {
	ampli_attrib.set(ptoff, i, real(amp));
	ampli_attrib.set(ptoff, i+1, imag(amp));
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
  uint w = 0;
  GA_Range range_i = gdp->getPrimitiveRange();
  for(GA_Iterator itp = range_i.begin(); itp != range_i.end(); ++itp, ++w) {
    wl_attrib.set(*itp, wave_lengths[w]);
    as_attrib.set(*itp, ampli_steps[w]);
  }

  // creation of the detail attibutes
  GA_RWHandleI bs_attrib(gdp->findIntTuple(GA_ATTRIB_DETAIL, "buffer_size", 1));
  GA_RWHandleF damping_attrib(gdp->findFloatTuple(GA_ATTRIB_DETAIL, "damping", 1));
  if (!bs_attrib.isValid()) {
    bs_attrib = GA_RWHandleI(gdp->addIntTuple(GA_ATTRIB_DETAIL, "buffer_size", 1));
  }
  if (!bs_attrib.isValid()) {
    addError(SOP_MESSAGE, "Failed to create attribute buffer_size");
    return error();
  }
  bs_attrib.set(0,buffer_size);
  if (!damping_attrib.isValid()) {
    damping_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_DETAIL, "damping", 1));
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
