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
 * Deform_Surface interactive (aperiodic) SOP
 *----------------------------------------------------------------------------
 * At each frame, sum the contribution of all the inputs for all wavelengths
 * at each point to get the height.
 */


#include "SOP_Deform_Surface_inter.hpp"

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

using namespace HDK_Sample;

void
newSopOperator(OP_OperatorTable *table)
{
  table->addOperator(new OP_Operator(
				     "def_surf_fs_inter",
				     "Deform Surface_inter FS",
				     SOP_Deform_Surface_inter::myConstructor,
				     SOP_Deform_Surface_inter::myTemplateList,
				     2,
				     OP_MULTI_INPUT_MAX,
				     0));
}
static PRM_Name names[] = {
  PRM_Name("amp",     "Amplitude"),
};

PRM_Template
SOP_Deform_Surface_inter::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
	       0, 0, SOP_Node::getGroupSelectButton(
						    GA_GROUP_POINT)),
  PRM_Template(PRM_FLT_J,     1, &names[0], PRMoneDefaults, 0,
	       &PRMscaleRange),
  PRM_Template(),
};


OP_Node *
SOP_Deform_Surface_inter::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
  return new SOP_Deform_Surface_inter(net, name, op);
}

SOP_Deform_Surface_inter::SOP_Deform_Surface_inter(OP_Network *net, const char *name, OP_Operator *op)
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

SOP_Deform_Surface_inter::~SOP_Deform_Surface_inter() {
}
OP_ERROR
SOP_Deform_Surface_inter::cookInputGroups(OP_Context &context, int alone)
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
SOP_Deform_Surface_inter::cookMySop(OP_Context &context)
{
  OP_AutoLockInputs inputs(this);
  if (inputs.lock(context) >= UT_ERROR_ABORT)
    return error();
    
  flags().timeDep = 1;
  float t = context.getTime();
  int fr = context.getFrame();
  float dt = 0.1;
  t = dt*fr;

  float amp = AMP(t);
  int nb_inputs = getInputsArraySize();
  
  duplicateSource(0, context); //grid
  GA_RWHandleV3 Phandle(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
  
  int nb_wl = inputGeo(1)->getPrimitiveRange().getEntries();
  GA_Offset ptoff;
  GA_FOR_ALL_PTOFF(gdp, ptoff) {
    UT_Vector3 Pvalue = gdp->getPos3(ptoff);
    Pvalue.y() = 0;
    gdp->setPos3(ptoff, Pvalue);
  }

  
  for (int input = 1; input < nb_inputs; ++input) { //all these inputs should be sets of sources
    const GU_Detail *fs = inputGeo(input);
    GA_ROHandleF w_handle(fs->findAttribute(GA_ATTRIB_PRIMITIVE, "wavelengths"));
    GA_ROHandleI as_handle(fs->findAttribute(GA_ATTRIB_PRIMITIVE, "ampli_steps"));
    if (!w_handle.isValid()) {
      addError(SOP_ATTRIBUTE_INVALID, "wavelengths");
      return error();
    }
    if (!as_handle.isValid()) {
      addError(SOP_ATTRIBUTE_INVALID, "ampli_steps");
      return error();
    }
    GA_ROHandleI bs_handle(fs->findAttribute(GA_ATTRIB_DETAIL, "buffer_size"));
    GA_ROHandleF damping_handle(fs->findAttribute(GA_ATTRIB_DETAIL, "damping"));
    if (!bs_handle.isValid()) {
      addError(SOP_ATTRIBUTE_INVALID, "buffer sizes input sources");
      return error();
    }
    if (!damping_handle.isValid()) {
      addError(SOP_ATTRIBUTE_INVALID, "damping input sources");
      return error();
    }
    int buffer_size = bs_handle.get(0);
    float damping_coef = damping_handle.get(0);
          
    const GA_Attribute *afs = fs->findFloatTuple(GA_ATTRIB_POINT, "ampli", buffer_size);
    if (!afs)	{
      addError(SOP_ATTRIBUTE_INVALID, "ampli");
      return error();
    }
    const GA_AIFTuple *tuple = afs->getAIFTuple();
    GA_Offset ptoff;
    if (!tuple) {
      addError(SOP_ATTRIBUTE_INVALID, "ampli tuple");
      return error();
    }
      
    GA_Offset prim_off;
    GA_Offset lcl_start, lcl_end;
    int w = 0;
    for (GA_Iterator lcl_it((fs)->getPrimitiveRange()); lcl_it.blockAdvance(lcl_start, lcl_end); ) {
      for (prim_off = lcl_start; prim_off < lcl_end; ++prim_off) {
	float wl = w_handle.get(prim_off);
	int as = as_handle.get(prim_off);
	float k = M_PI*2.0/wl;
	float om = omega(k);//sqrtf(9.81*k + 0.074/1000*pow(k, 3));
	const GA_Primitive* prim = fs->getPrimitive(prim_off);
	GA_Range range = prim->getPointRange();

	GA_Offset ptoff;
	GA_FOR_ALL_PTOFF(gdp, ptoff) {
	  UT_Vector3 Pvalue = gdp->getPos3(ptoff);
	  COMPLEX a = 0;//exp(std::complex<float>(0, 1)*(k*Pvalue.x()));
	  for(GA_Iterator it = range.begin(); it != range.end(); ++it) {
	    UT_Vector3 P_fs = fs->getPos3((*it));
	    float r = sqrt(pow(Pvalue.x() - P_fs.x(), 2) + pow(Pvalue.z() - P_fs.z(), 2));
	    float v = velocity(k, om);//0.5*omega/k;
	    float ar = 0, ai = 0;
	    fpreal t_ret = t - r/v;
	    OP_Context c_ret(t_ret);
	    int f_ret = floor(t_ret/(dt)) - 1;
	    f_ret = floor((float)f_ret/(float)as); 
	    if (t_ret < 0) {
	      --f_ret;
	    }
	    if (f_ret >= 0) {
	      afs = fs->findFloatTuple(GA_ATTRIB_POINT, "ampli", buffer_size);
	      tuple = afs->getAIFTuple();
	      tuple->get(afs, *it, ar, 2*f_ret);
	      tuple->get(afs, *it, ai, 2*f_ret+1);
	    }
	    std::complex<float> ampli(ar, ai);
	    a += ampli*fund_solution(k*r)*damping(damping_coef, r, k);
	  }
	  Pvalue.y() += real(amp*a*exp(-COMPLEX(0, 1)*(om*(float)t)));
	  gdp->setPos3(ptoff, Pvalue);
	}
	++w;
      }
    }
  }
  Phandle.bumpDataId();
  return error();
}
