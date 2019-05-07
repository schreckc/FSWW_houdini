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
 */

/// This is the pure C++ implementation of the wave SOP.
/// @see @ref HOM/SOP_HOMWave.py, @ref HOM/SOP_HOMWaveNumpy.py, @ref HOM/SOP_HOMWaveInlinecpp.py, @ref HOM/SOP_HOMWave.C, @ref SOP/SOP_VEXWave.vfl

#include "SOP_Solve_FS_inter.hpp"

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>
#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Director.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <UT/UT_DSOVersion.h>
#include <SYS/SYS_Math.h>

using namespace HDK_Sample;


void
newSopOperator(OP_OperatorTable *table)
{
  table->addOperator(new OP_Operator(
				     "solve_fs_inter",
				     "Solve FS Interactif",
				     SOP_Solve_FS_inter::myConstructor,
				     SOP_Solve_FS_inter::myTemplateList,
				     3,
				     3,
				     nullptr,  
				     OP_FLAG_GENERATOR));
}

PRM_Template
SOP_Solve_FS_inter::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
	       0, 0, SOP_Node::getGroupSelectButton(
						    GA_GROUP_POINT)),
  PRM_Template(),
};


OP_Node *
SOP_Solve_FS_inter::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
  return new SOP_Solve_FS_inter(net, name, op);
}

SOP_Solve_FS_inter::SOP_Solve_FS_inter(OP_Network *net, const char *name, OP_Operator *op)
  : SOP_Node(net, name, op)
{
  int size_buffer = 500;
  myGDPLists = UT_Array<const GU_Detail *>(size_buffer);
  myISLists = UT_Array<const GU_Detail *>(size_buffer);
  gdp_count = 0;
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

SOP_Solve_FS_inter::~SOP_Solve_FS_inter()
{
}
OP_ERROR
SOP_Solve_FS_inter::cookInputGroups(OP_Context &context, int alone)
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
SOP_Solve_FS_inter::cookMySop(OP_Context &context)
{
  // We must lock our inputs before we try to access their geometry.
  // OP_AutoLockInputs will automatically unlock our inputs when we return.
  // NOTE: Don't call unlockInputs yourself when using this!
  OP_AutoLockInputs inputs(this);
  if (inputs.lock(context) >= UT_ERROR_ABORT)
    return error();

  //
  // Flag the SOP as being time dependent (i.e. cook on time changes)
  flags().timeDep = 1;
  
  //   gdp->clearAndDestroy();
  duplicateSource(0, context);
  fpreal frame = OPgetDirector()->getChannelManager()->getSample(context.getTime());
  frame *= 0.03;
  float t = context.getTime();
  int fr = context.getFrame();
  float dt = 0.25;
  t = dt*fr;
  // if (fr != 0) {
  //   dt = t/fr;
  // }
  
  //const GU_Detail *fsi = inputGeo(0); //sources
  const GU_Detail *bp = inputGeo(1); //boundary points
  const GU_Detail *is = inputGeo(2); //input sources

  // int nb_bp= bp->getNumPoints(); // nb of boundary points, should be diff for each freq
  int nb_wl = is->getPrimitiveRange().getEntries();
  //  int nb_fs = fsi->getNumPoints(); //should be diff for each freq

  //
  myISLists[gdp_count] = new GU_Detail(is);
  // myGDPLists[gdp_count] = new GU_Detail(gdp);
  // ++gdp_count;  //moved at the end of the function

  
  if (fr == 1) {
    wave_lengths = std::vector<float>(nb_wl);
    ampli_steps = std::vector<int>(nb_wl);
    p_in = std::vector<VectorXcf>(nb_wl);
    svd = std::vector<Eigen::BDCSVD<MatrixXcf> >(nb_wl);
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
  for (int w = 0; w < nb_wl; ++w) {
    float wl = wave_lengths[w];
    float k = M_PI*2.0/wl;
    float v = velocity(k);
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
      uint nb_bp = 0;
      for(GA_Iterator itbp = range_bp.begin(); itbp != range_bp.end(); ++itbp) {
	UT_Vector3 pos_b = bp->getPos3(*itbp);
	float r = sqrt(pow(pos_b.x() - pos_fs.x(), 2) + pow(pos_b.z() - pos_fs.z(), 2));
	float ret = r/v;
	float q = interpolation(ret, 0, dt);
	//	std::cout<<"i j "<<i<<" "<<j<<" "<<nb_fs<<" "<<nb_bp<<" "<<k<<" "<<r<<" "<<pos_b(0)<<" "<<pos_fs(0)<<std::endl;
	if (q > 0) {
	  q = 1;
	  T(i, j) = q*fund_solution(k*r);
	  ++nb_bp;
	} else {
	  T(i, j) = 0;
	}
	++i;
      }
      std::cout<<"nb bp infuenced "<<nb_bp<<" "<<j<<std::endl;
      ++j;
    }
    //    std::cout<<T<<std::endl;
    svd[w] =  BDCSVD<MatrixXcf>(T,ComputeThinU | ComputeThinV);
  }
  }

  const GA_Attribute *afs;
  const GA_AIFTuple *tuple; 
    // fill the p_in for each frequencies
  GA_ROHandleF a_handle(is->findFloatTuple(GA_ATTRIB_POINT, "ampli", 2));
  for (int w = 0; w < nb_wl; ++w) {
    int as = ampli_steps[w];
    // std::cout<<"frame as "<<fr<<" "<<as<<std::endl;
    // if (fr%as == 0) {
      
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
       // add contribution from input sources
      const GA_Primitive* prim_is = is->getPrimitiveByIndex(w);
      GA_Range range_is = prim_is->getPointRange();
      for(GA_Iterator it = range_is.begin(); it != range_is.end(); ++it) {
      	UT_Vector3 pos_is = is->getPos3(*it);
      	 float r = sqrt(pow(pos_b.x() - pos_is.x(), 2) + pow(pos_b.z() - pos_is.z(), 2));
      	 float v = velocity(k);
      	 fpreal t_ret = t - r/v;
      	 OP_Context c_ret(t_ret);
      	 int f_ret = floor(t_ret/dt);
      	 float ar = 0, ai = 0;
      	  if (f_ret >= 1) {
      	   const GU_Detail *fs_ret = myISLists[f_ret]; 
      	   afs = fs_ret->findFloatTuple(GA_ATTRIB_POINT, "ampli", 2);
      	   tuple = afs->getAIFTuple();
      	   tuple->get(afs, *it, ar, 0);
      	   tuple->get(afs, *it, ai, 1);
      	 }
      	  std::complex<float> ampli(ar, ai);
	  p_in[w](i) -= ampli*fund_solution(k*r);
      }
      // //      add contribution from other sources of the obstacle
      const GA_Primitive* prim = gdp->getPrimitiveByIndex(w);
      GA_Range range = prim->getPointRange();
      for(GA_Iterator it = range.begin(); it != range.end(); ++it) {
      	UT_Vector3 pos_fs = gdp->getPos3(*it);
      	 float r = sqrt(pow(pos_b.x() - pos_fs.x(), 2) + pow(pos_b.z() - pos_fs.z(), 2));
      	float ret = r/velocity(k);
      	int f_ret = floor((t - ret)/(dt));
	if (ret > t) {
	  --f_ret;
	}
      	float q = interpolation(ret, 0, (dt));
      	float ar = 0, ai = 0;
      	if (q <= 0) {
      	   if (f_ret >= 0) {
	     const GU_Detail *fs_ret = myGDPLists[f_ret]; 
      	      afs = fs_ret->findFloatTuple(GA_ATTRIB_POINT, "ampli", 2);
      	      tuple = afs->getAIFTuple();
      	      tuple->get(afs, *it, ar, 0);
      	      tuple->get(afs, *it, ai, 1);
	      //      std::cout<<"f_ret "<<f_ret<<"  "<<*it<<" "<<*itbp<<" "<<t<<" "<<ret<<" ("<<ar<<","<<ai<<")"<<std::endl;
      	    }
      	  COMPLEX a(ar, ai);
	    p_in[w](i) -= 0.8f*a*fund_solution(k*r);
      	} else {
      	  if (f_ret >= 1) {
      	    const GU_Detail *fs_ret_prev = myGDPLists[f_ret-1]; 
      	    afs = fs_ret_prev->findFloatTuple(GA_ATTRIB_POINT, "ampli", 2);
      	    tuple = afs->getAIFTuple();
      	    tuple->get(afs, *it, ar, 0);
      	    tuple->get(afs, *it, ai, 1);
      	  }
      	  COMPLEX a_prev(ar, ai);
	  //COMPLEX ampli = (1-q)*a_prev;
	  //COMPLEX ampli = a_prev;
	  // p_in[w](i) -= ampli*fund_solution(k*r);
      	}
      }
       ++i;
  }
    //    std::cout<<p_in[w]<<std::endl;
    // }
  }
  
  // solve for each frequencies
  GA_RWHandleF ampli_attrib(gdp->findFloatTuple(GA_ATTRIB_POINT, "ampli", 2));
  for (int w = 0; w < nb_wl; ++w) {
    int as = ampli_steps[w];
    // if (fr%as == 0) {
    // float wl = wave_lengths[w];
    // float k = M_PI*2.0/wl;
    const GA_Primitive* prim = gdp->getPrimitiveByIndex(w);
    GA_Range range = prim->getPointRange();
    int nb_es = range.getEntries();
    VectorXcf c(nb_es);
    //     std::cout<<"pin\n"<<p_in[w]<<std::endl;
    c = svd[w].solve(p_in[w]);
    //    std::cout<<"c\n"<<c<<std::endl;
    int i = 0;
    for(GA_Iterator it = range.begin(); it != range.end(); ++it, ++i) {
      ampli_attrib.set(*it, 0, real(c[i]));
      ampli_attrib.set(*it, 1, imag(c[i]));
    }
    // }
  }
 
  myGDPLists[gdp_count] = new GU_Detail(gdp);
  ++gdp_count;
  ampli_attrib.bumpDataId();
  
  return error();
}

