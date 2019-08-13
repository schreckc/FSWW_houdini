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
 * Merge_Sources SOP
 *----------------------------------------------------------------------------
 * Take any number (at least one) of set of sources and merge them together.
 * For now, we assume that all the sets have the same parameter (range of wavelength, 
 * buffer_size etc...)
 */


#include "SOP_Merge_Sources.hpp"

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
  table->addOperator(new OP_Operator("merge_sources_fs",
				     "Merge Sources FS",
				     SOP_Merge_Sources::myConstructor,
				     SOP_Merge_Sources::myTemplateList,
				     1,
				     OP_MULTI_INPUT_MAX,
				     nullptr,  
				     OP_FLAG_GENERATOR));
}
static PRM_Name names[] = {
  PRM_Name("inter_src",   "Interactive sources"),
};

PRM_Template
SOP_Merge_Sources::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
	       0, 0, SOP_Node::getGroupSelectButton(GA_GROUP_POINT)),
  PRM_Template(PRM_TOGGLE_J,  1, &names[0]),
  PRM_Template(),
};


OP_Node *
SOP_Merge_Sources::myConstructor(OP_Network *net, const char *name, OP_Operator *op) {
  return new SOP_Merge_Sources(net, name, op);
}

SOP_Merge_Sources::SOP_Merge_Sources(OP_Network *net, const char *name, OP_Operator *op)
  : SOP_Node(net, name, op) {
  mySopFlags.setManagesDataIDs(true);
}

SOP_Merge_Sources::~SOP_Merge_Sources()
{
}
OP_ERROR
SOP_Merge_Sources::cookInputGroups(OP_Context &context, int alone)
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


OP_ERROR SOP_Merge_Sources::cookMySop(OP_Context &context) {
  // for now I am assuming that the different set of sources have the same parameters

  OP_AutoLockInputs inputs(this);
  if (inputs.lock(context) >= UT_ERROR_ABORT)
    return error();

  flags().timeDep = 0;
  
  float t = context.getTime();
  int fr = context.getFrame();

  float dt_ = 0.1;
  bool is_inter = INTER_SRC(t);
  
    
  // get wavelenght range and time step for each wavelength from input sources
  int nb_wl = inputGeo(0)->getPrimitiveRange().getEntries();
  std::vector<float> wave_lengths(nb_wl);
  std::vector<int> ampli_steps(nb_wl);
  
  GA_ROHandleF w_handle(inputGeo(0)->findAttribute(GA_ATTRIB_PRIMITIVE, "wavelengths"));
  GA_ROHandleF as_handle(inputGeo(0)->findAttribute(GA_ATTRIB_PRIMITIVE, "ampli_steps"));
  if (!w_handle.isValid()) {
    addError(SOP_ATTRIBUTE_INVALID, "wavelengths input merge sources");
    return error();
  }
  if (!as_handle.isValid()) {
    addError(SOP_ATTRIBUTE_INVALID, "ampli step merge sources");
    return error();
  }
 
  int w = 0;
  GA_Range range_is = inputGeo(0)->getPrimitiveRange();
  for(GA_Iterator itis = range_is.begin(); itis != range_is.end(); ++itis, ++w) {
    GA_Offset prim_off = *itis;
    float wl = w_handle.get(prim_off);
    wave_lengths[w] = wl;
    ampli_steps[w]= as_handle.get(prim_off);
  }

  // get details attibutes from the input sources
  GA_ROHandleI bs_handle(inputGeo(0)->findAttribute(GA_ATTRIB_DETAIL, "buffer_size"));
  GA_ROHandleF damping_handle(inputGeo(0)->findAttribute(GA_ATTRIB_DETAIL, "damping"));
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
  

  
  // begin creation of geometry
  gdp->clearAndDestroy();
  int nb_inputs = getInputsArraySize();
  for (int w = 0; w < nb_wl; ++w) {
    float wl = wave_lengths[w];
    uint nb_pts = 0;
    for (int input = 0; input < nb_inputs; ++input) { //all these inputs should be sets of sources
      const GU_Detail *fs = inputGeo(input);
      const GA_Primitive* prim = fs->getPrimitiveByIndex(w);
      GA_Range range = prim->getPointRange();
      nb_pts += range.getEntries();
    }
    GA_Offset ptoff = gdp->appendPointBlock(nb_pts);
    GA_Offset vtxoff;
    GA_Offset prim_off = gdp->appendPrimitivesAndVertices(GA_PRIMPOLY, 1, nb_pts, vtxoff, true);
    uint i = 0;

    //create amplitude buffer attributes for the point sources
    GA_RWHandleF ampli_attrib(gdp->findFloatTuple(GA_ATTRIB_POINT, "ampli", buffer_size));
    if (!ampli_attrib.isValid()) {
      ampli_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_POINT, "ampli", buffer_size));
    }
    if (!ampli_attrib.isValid()) {
      addError(SOP_MESSAGE, "Failed to create attribute ampli");
      return error();
    }
    const GA_Attribute *afs;
    const GA_AIFTuple *tuple; 
    for (int input = 0; input < nb_inputs; ++input) { //all these inputs should be sets of sources
      const GU_Detail *fs = inputGeo(input);
      const GA_Primitive* prim = fs->getPrimitiveByIndex(w);
      GA_Range range = prim->getPointRange();
      afs = fs->findFloatTuple(GA_ATTRIB_POINT, "ampli", buffer_size);
      tuple = afs->getAIFTuple();
      for(GA_Iterator itfs = range.begin(); itfs != range.end(); ++itfs) {
  	UT_Vector3 pos_fs = fs->getPos3(*itfs);
  	gdp->getTopology().wireVertexPoint(vtxoff+i,ptoff+i);
  	gdp->setPos3(ptoff+i, pos_fs);
	for (uint f = 0; f < buffer_size; ++f) {
	  FLOAT a;
	  tuple->get(afs, *itfs, a, f);
	  ampli_attrib.set(ptoff+i, f, a);
	}
  	++i;
      }
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
  
  w = 0;
  GA_Offset prim_off, lcl_start, lcl_end;
  for (GA_Iterator lcl_it((gdp)->getPrimitiveRange()); lcl_it.blockAdvance(lcl_start, lcl_end); ) {
    for (prim_off = lcl_start; prim_off < lcl_end; ++prim_off) {
      wl_attrib.set(prim_off, wave_lengths[w]);
      as_attrib.set(prim_off, ampli_steps[w]);
      ++w;
    }
  }
  gdp->bumpDataIdsForAddOrRemove(true, true, true);
  return error();
}
