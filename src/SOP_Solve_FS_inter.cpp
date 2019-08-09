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
 * Solve FS interactive (aperiodic) SOP
 *----------------------------------------------------------------------------
 * Compute amplitude of the obstacle sources (input 0) such that the boundary condition are
 * respected (as well as possible, least square) at the boundary points (input 1) given the
 * incoming waves (input 2).
 */

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


void newSopOperator(OP_OperatorTable *table) {
  table->addOperator(new OP_Operator("solve_fs_inter",
				     "Solve FS Interactif",
				     SOP_Solve_FS_inter::myConstructor,
				     SOP_Solve_FS_inter::myTemplateList,
				     3,
				     3,
				     nullptr,  
				     OP_FLAG_GENERATOR));
}

PRM_Template SOP_Solve_FS_inter::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
	       0, 0, SOP_Node::getGroupSelectButton(
						    GA_GROUP_POINT)),
  PRM_Template(),
};


OP_Node *SOP_Solve_FS_inter::myConstructor(OP_Network *net, const char *name, OP_Operator *op) {
  return new SOP_Solve_FS_inter(net, name, op);
}

SOP_Solve_FS_inter::SOP_Solve_FS_inter(OP_Network *net, const char *name, OP_Operator *op)
  : SOP_Node(net, name, op) {
  mySopFlags.setManagesDataIDs(true);
}

SOP_Solve_FS_inter::~SOP_Solve_FS_inter() {

}
OP_ERROR
SOP_Solve_FS_inter::cookInputGroups(OP_Context &context, int alone) {
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


OP_ERROR SOP_Solve_FS_inter::cookMySop(OP_Context &context) {
  OP_AutoLockInputs inputs(this);
  if (inputs.lock(context) >= UT_ERROR_ABORT)
    return error();

  flags().timeDep = 1;
  
  duplicateSource(0, context);
  float t = context.getTime();
  int fr = context.getFrame();
  float dt = 0.1;
  t = dt*fr;
  
  const GU_Detail *bp = inputGeo(1); //boundary points
  const GU_Detail *is = inputGeo(2); //input sources

  int nb_wl = is->getPrimitiveRange().getEntries();
  
  if (fr == 1) {
    // get wavelenght range and time step for each wavelength from input sources
    wave_lengths = std::vector<float>(nb_wl);
    ampli_steps = std::vector<int>(nb_wl);
    p_in = std::vector<VectorXcf>(nb_wl);
    svd = std::vector<Eigen::BDCSVD<MatrixXcf> >(nb_wl);
    GA_ROHandleF w_handle(is->findAttribute(GA_ATTRIB_PRIMITIVE, "wavelengths"));
    GA_ROHandleI as_handle(is->findAttribute(GA_ATTRIB_PRIMITIVE, "ampli_steps"));
    if (!w_handle.isValid()) {
      addError(SOP_ATTRIBUTE_INVALID, "wavelengths input sources");
      return error();
    }
    if (!as_handle.isValid()) {
      addError(SOP_ATTRIBUTE_INVALID, "ampli_steps input sources");
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
    buffer_size = bs_handle.get(0);
    damping_coef = damping_handle.get(0);

   

    // create transfer matrix
    for (int w = 0; w < nb_wl; ++w) {
      float wl = wave_lengths[w];
      int as = ampli_steps[w];
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
	  float q = interpolation(ret, 0, dt*as);
	  if (q > 0 && r != 0) {
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
      svd[w] =  BDCSVD<MatrixXcf>(T,ComputeThinU | ComputeThinV);
    }
  }

  GA_ROHandleI bs_es_handle(gdp->findAttribute(GA_ATTRIB_DETAIL, "buffer_size"));
  if (!bs_es_handle.isValid()) {
    addError(SOP_ATTRIBUTE_INVALID, "buffer sizes obsacle sources");
    return error();
  }
  if (bs_es_handle.get(0) != buffer_size) {
    addError(SOP_ATTRIBUTE_INVALID, "Buffer size of obstacle sources do not match buffer size of input sources. Did you forget to check the interactive sources box ?");
    return error();
  }

  const GA_Attribute *afs;
  const GA_AIFTuple *tuple; 
  // fill the p_in for each wavelength
  GA_ROHandleF a_handle(is->findFloatTuple(GA_ATTRIB_POINT, "ampli", buffer_size));
  for (int w = 0; w < nb_wl; ++w) {
    int as = ampli_steps[w];
    if (fr%as == 0) {
      
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
	  int f_ret = floor((t_ret)/(dt*as))+1;
	  float ar = 0, ai = 0;
      	  if (f_ret >= 1) {
	    afs = is->findFloatTuple(GA_ATTRIB_POINT, "ampli", buffer_size);
	    tuple = afs->getAIFTuple();
	    tuple->get(afs, *it, ar, 2*f_ret);
	    tuple->get(afs, *it, ai, 2*f_ret+1);
	  }
      	  std::complex<float> ampli(ar, ai);
      	  p_in[w](i) -= ampli*fund_solution(k*r)*damping(damping_coef, r, k);
	}
	// add contribution from other sources of the obstacle
	const GA_Primitive* prim = gdp->getPrimitiveByIndex(w);
	GA_Range range = prim->getPointRange();
	for(GA_Iterator it = range.begin(); it != range.end(); ++it) {
	  UT_Vector3 pos_fs = gdp->getPos3(*it);
	  float r = sqrt(pow(pos_b.x() - pos_fs.x(), 2) + pow(pos_b.z() - pos_fs.z(), 2));
	  float ret = r/velocity(k);
	  int f_ret = floor((t - ret)/(dt*as))+1;
	  if (ret > t) {
	    --f_ret;
	  }
	  float q = interpolation(ret, 0, (dt*as));
	  float ar = 0, ai = 0;
	  if (q <= 0) {
	    if (f_ret >= 0) {
      	      afs = gdp->findFloatTuple(GA_ATTRIB_POINT, "ampli", buffer_size);
      	      tuple = afs->getAIFTuple();
      	      tuple->get(afs, *it, ar, 2*f_ret);
      	      tuple->get(afs, *it, ai, 2*f_ret+1);
      	    }
	    COMPLEX a(ar, ai);
	    p_in[w](i) -= 0.8f*a*fund_solution(k*r)*damping(damping_coef, r, k);
	  } else {
	    // if (f_ret >= 1) {
	    //   const GU_Detail *fs_ret_prev = myGDPLists[f_ret-1]; 
	    //   afs = fs_ret_prev->findFloatTuple(GA_ATTRIB_POINT, "ampli", 2);
	    //   tuple = afs->getAIFTuple();
	    //   tuple->get(afs, *it, ar, 0);
	    //   tuple->get(afs, *it, ai, 1);
	    // }
	    // COMPLEX a_prev(ar, ai);
	    // COMPLEX ampli = (1-q)*a_prev;
	    // //COMPLEX ampli = a_prev;
	    //  p_in[w](i) -= ampli*fund_solution(k*r);
	  }
	}
	++i;
      }
    }
  }
  
  // solve for each wavelength and set new anpli
  GA_RWHandleF ampli_attrib(gdp->findFloatTuple(GA_ATTRIB_POINT, "ampli", buffer_size));
  for (int w = 0; w < nb_wl; ++w) {
    int as = ampli_steps[w];
    if (fr%as == 0) {
      const GA_Primitive* prim = gdp->getPrimitiveByIndex(w);
      GA_Range range = prim->getPointRange();
      int nb_es = range.getEntries();
      VectorXcf c(nb_es);
      c = svd[w].solve(p_in[w]);
      int i = 0;
      for(GA_Iterator it = range.begin(); it != range.end(); ++it, ++i) {
	// if (real(c[i]) != 0 || imag(c[i]) != 0) {
	//   std::cout<<"ampli c[i] "<<c[i]<<std::endl;
	//   }
	ampli_attrib.set(*it, 2*(fr/as), real(c[i]));
	ampli_attrib.set(*it, 2*(fr/as)+1, imag(c[i]));
      }
      ampli_attrib.bumpDataId();
    }
  }
 
  return error();
}

