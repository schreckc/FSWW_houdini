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

#include "SOP_Solve_FS_static.hpp"

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
        "solve_fs_static",
        "Static Solve FS",
        SOP_Solve_FS_static::myConstructor,
        SOP_Solve_FS_static::myTemplateList,
        3,
        3,
        0));
}

PRM_Template
SOP_Solve_FS_static::myTemplateList[] = {
    PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
                                0, 0, SOP_Node::getGroupSelectButton(
                                                GA_GROUP_POINT)),
    PRM_Template(),
};


OP_Node *
SOP_Solve_FS_static::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_Solve_FS_static(net, name, op);
}

SOP_Solve_FS_static::SOP_Solve_FS_static(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
  int size_buffer = 250;
  myGDPLists = UT_Array<const GU_Detail *>(size_buffer);
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

SOP_Solve_FS_static::~SOP_Solve_FS_static()
{
}
OP_ERROR
SOP_Solve_FS_static::cookInputGroups(OP_Context &context, int alone)
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
SOP_Solve_FS_static::cookMySop(OP_Context &context)
{
    // We must lock our inputs before we try to access their geometry.
    // OP_AutoLockInputs will automatically unlock our inputs when we return.
    // NOTE: Don't call unlockInputs yourself when using this!
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    // Duplicate input geometry
    duplicateSource(0, context);

    const GU_Detail *bp = inputGeo(1); //boundary points
    const GU_Detail *is = inputGeo(2); //input sources

     GA_ROHandleI d_handle(gdp->findAttribute(GA_ATTRIB_DETAIL, "is_dynamic"));
    const int is_dynamic = d_handle.get(GA_Offset(0));
    // Flag the SOP as being time dependent (i.e. cook on time changes)
    //flags().timeDep = 1;
       float t = context.getTime();
    
    GA_ROHandleF w_handle(gdp->findAttribute(GA_ATTRIB_DETAIL, "wavelength"));
    if (!d_handle.isValid())
      {
	addError(SOP_ATTRIBUTE_INVALID, "wavelength");
	return error();
      }
    const float period = w_handle.get(GA_Offset(0));
    float k = M_PI*2.0/period;

    int nb_fs = gdp->getNumPoints(); //nb of fundamental sources
    int nb_bp= bp->getNumPoints(); // nb of boundary points
    
      MatrixXcf T = MatrixXcf(nb_bp, nb_fs);

      // const GA_Attribute *afs = gdp->findFloatTuple(GA_ATTRIB_POINT, "v", 2);
     const GA_Attribute *ais = is->findFloatTuple(GA_ATTRIB_POINT, "v", 2);
     const GA_AIFTuple *tuple = ais->getAIFTuple();

     GA_Offset fsoff;
     GA_Offset bpoff;
      
      uint i = 0, j = 0;
      {
      GA_FOR_ALL_PTOFF(bp, bpoff) {
      	UT_Vector3 pos_b = bp->getPos3(bpoff);
	j = 0;
	GA_FOR_ALL_PTOFF(gdp, fsoff) {
	  //if (j < nb_fs) {
	  //   addError(SOP_ATTRIBUTE_INVALID, "too many sources");
	  //   return error();
      	    // }
	  UT_Vector3 pos_fs = gdp->getPos3(fsoff);
	  float r = sqrt(pow(pos_b.x() - pos_fs.x(), 2) + pow(pos_b.z() - pos_fs.z(), 2));
	  T(i, j) = std::complex<float>(0, -1)/(FLOAT)4.0*sqrtf(2.0f/(M_PI*k*r))*exp(std::complex<float>(0, 1)*(k*r - (float)M_PI/4.0f));
	  ++j;
	  //   } 
	}
      	++i;
      }
      }
  Eigen::BDCSVD<MatrixXcf> svd_T = BDCSVD<MatrixXcf>(T,ComputeThinU | ComputeThinV);

      VectorXcf c(nb_fs);
      VectorXcf p_in(nb_bp);

      i = 0; j = 0;
      GA_Offset isoff;
      {
      GA_FOR_ALL_PTOFF(bp, bpoff) {
      	UT_Vector3 pos_b = bp->getPos3(bpoff);
      	p_in(i) = -exp(std::complex<float>(0, 1)*(k*pos_b.x()));
	j = 0;
      	// GA_FOR_ALL_PTOFF(is, isoff) {
      	//   UT_Vector3 pos_is = is->getPos3(isoff);
      	//   float r = sqrt(pow(pos_b.x() - pos_is.x(), 2) + pow(pos_b.z() - pos_is.z(), 2));
      	//   float ar = 0, ai = 0;
      	//   tuple->get(ais, isoff, ar, 0);
      	//   tuple->get(ais, isoff, ai, 1);
      	//   std::complex<float> ampli(ar, ai);
      	//   p_in(i) -= ampli*std::complex<float>(0, -1)/(FLOAT)4.0*sqrtf(2.0f/(M_PI*k*r))*exp(std::complex<float>(0, 1)*(k*r - (float)M_PI/4.0f));
      	//   ++j;
      	// }
      	++i;
      }
      }
       c = svd_T.solve(p_in);
       i = 0;
       GA_RWHandleV3 vhandle(gdp->findAttribute(GA_ATTRIB_POINT, "v"));
       {
	 GA_FOR_ALL_PTOFF(gdp, fsoff) {
	   //if (i < nb_fs) {
	   UT_Vector3 nampli(real(c(i)), imag(c(i)), 0);
	   vhandle.set(fsoff, nampli);
	   ++i;
	   //}
	 }
       }
       vhandle.bumpDataId();
       
    return error();
}
