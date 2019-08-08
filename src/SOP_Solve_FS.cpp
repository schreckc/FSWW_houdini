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
 * Solve FS SOP
 *----------------------------------------------------------------------------
 * Compute amplitude of the obstacle sources (input 0) such that the boundary condition are
 * respected (as well as possible, least square) at the boundary points (input 1) given the
 * incoming waves (input 2).
 * 
 */

#include "SOP_Solve_FS.hpp"

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>
#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Director.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <UT/UT_DSOVersion.h>
#include <SYS/SYS_Math.h>

void newSopOperator(OP_OperatorTable *table)
{
  table->addOperator(new OP_Operator(
				     "solve_fs",
				     "Solve FS",
				     SOP_Solve_FS::myConstructor,
				     SOP_Solve_FS::myTemplateList,
				     3,
				     3,
				     nullptr,  
				     OP_FLAG_GENERATOR));
}

PRM_Template
SOP_Solve_FS::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
	       0, 0, SOP_Node::getGroupSelectButton(
						    GA_GROUP_POINT)),
  PRM_Template(),
};


OP_Node *
SOP_Solve_FS::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
  return new SOP_Solve_FS(net, name, op);
}

SOP_Solve_FS::SOP_Solve_FS(OP_Network *net, const char *name, OP_Operator *op)
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

SOP_Solve_FS::~SOP_Solve_FS()
{
}
OP_ERROR
SOP_Solve_FS::cookInputGroups(OP_Context &context, int alone)
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
SOP_Solve_FS::cookMySop(OP_Context &context)
{
  OP_AutoLockInputs inputs(this);
  if (inputs.lock(context) >= UT_ERROR_ABORT)
    return error();

  float t = context.getTime();

  duplicateSource(0, context);
  
  const GU_Detail *bp = inputGeo(1); //boundary points
  const GU_Detail *is = inputGeo(2); //input sources

  int nb_wl = is->getPrimitiveRange().getEntries();

  std::vector<VectorXcf> p_in(nb_wl);
  std::vector<Eigen::BDCSVD<MatrixXcf> > svd(nb_wl);

  // get wavelenght range and time step for each wavelength from input sources
  std::vector<float> wave_lengths(nb_wl);
  std::vector<int> ampli_steps(nb_wl);
  
  GA_ROHandleF w_handle(is->findAttribute(GA_ATTRIB_PRIMITIVE, "wavelengths"));
  GA_ROHandleF as_handle(is->findAttribute(GA_ATTRIB_PRIMITIVE, "ampli_steps"));
  if (!w_handle.isValid()) {
    addError(SOP_ATTRIBUTE_INVALID, "wavelengths input sources");
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
  
  // create transfer matrix
  GA_Offset fsoff;
  GA_Offset bpoff;

  for (int w = 0; w < nb_wl; ++w) {
    float wl = wave_lengths[w];
    float k = M_PI*2.0/wl;
    const GA_Primitive* prim_fs = gdp->getPrimitiveByIndex(w);
    GA_Range range_fs = prim_fs->getPointRange();
    int nb_fs = range_fs.getEntries();
    const GA_Primitive* prim_bp = bp->getPrimitiveByIndex(w);
    GA_Range range_bp = prim_bp->getPointRange();
    int nb_bp = range_bp.getEntries();
    MatrixXcf T = MatrixXcf(nb_bp, nb_fs);
    uint i = 0, j = 0;
    for(GA_Iterator itfs = range_fs.begin(); itfs != range_fs.end(); ++itfs) {
      UT_Vector3 pos_fs = gdp->getPos3(*itfs);
      i = 0;
      for(GA_Iterator itbp = range_bp.begin(); itbp != range_bp.end(); ++itbp) {
  	UT_Vector3 pos_b = bp->getPos3(*itbp);
  	float r = sqrt(pow(pos_b.x() - pos_fs.x(), 2) + pow(pos_b.z() - pos_fs.z(), 2));
  	T(i, j) = fund_solution(k*r);
  	++i;
      }
      ++j;
    }
    svd[w] =  BDCSVD<MatrixXcf>(T,ComputeThinU | ComputeThinV);
  }

  // fill the p_in for each wavelength
  GA_ROHandleF a_handle(is->findFloatTuple(GA_ATTRIB_POINT, "ampli", 2));
  
  for (int w = 0; w < nb_wl; ++w) {
    float wl = wave_lengths[w];
    float k = M_PI*2.0/wl;
    int i = 0;
    const GA_Primitive* prim_bp = bp->getPrimitiveByIndex(w);
    GA_Range range_bp = prim_bp->getPointRange();
    int nb_bp = range_bp.getEntries();
    p_in[w] = VectorXcf(nb_bp);
    for(GA_Iterator itbp = range_bp.begin(); itbp != range_bp.end(); ++itbp) {
      UT_Vector3 pos_b = bp->getPos3(*itbp);
      p_in[w](i) = 0;
      const GA_Primitive* prim = is->getPrimitiveByIndex(w);
      GA_Range range = prim->getPointRange();
      for(GA_Iterator it = range.begin(); it != range.end(); ++it) {
  	UT_Vector3 pos_is = is->getPos3(*it);
  	float r = sqrt(pow(pos_b.x() - pos_is.x(), 2) + pow(pos_b.z() - pos_is.z(), 2));
  	float ar = 0, ai = 0;
  	ar = a_handle.get(*it, 0);
  	ai = a_handle.get(*it, 1);
  	std::complex<float> ampli(ar, ai);
  	p_in[w](i) -= ampli*fund_solution(k*r);
      }
      ++i;
    }
  }

  // // solve for each wavelength (and set new ampli)
  GA_RWHandleF ampli_attrib(gdp->findFloatTuple(GA_ATTRIB_POINT, "ampli", 2));
  for (int w = 0; w < nb_wl; ++w) {
    float wl = wave_lengths[w];
    float k = M_PI*2.0/wl;
    const GA_Primitive* prim = gdp->getPrimitiveByIndex(w);
    GA_Range range = prim->getPointRange();
    int nb_es = range.getEntries();
    VectorXcf c(nb_es);
    c = svd[w].solve(p_in[w]);
    int i = 0;
    for(GA_Iterator it = range.begin(); it != range.end(); ++it, ++i) {
      ampli_attrib.set(*it, 0, real(c[i]));
      ampli_attrib.set(*it, 1, imag(c[i]));
    }
  }
  ampli_attrib.bumpDataId();

  return error();
}

