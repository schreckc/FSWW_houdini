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
 * Deform_Surface SOP
 *----------------------------------------------------------------------------
 * At first frame, compute the spectrum a each point of the grid (input 0) by 
 * summing the contribution of all sources (each other input is a set of sources).
 * At each frame, sum the contribution of each wavelength at each point to get 
 * the height (according to the spectrum of the point).
 */


#include "SOP_Deform_Surface.hpp"

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

void newSopOperator(OP_OperatorTable *table) {
  table->addOperator(new OP_Operator("def_surf_fs",
				     "Deform Surface FS",
				     SOP_Deform_Surface::myConstructor,
				     SOP_Deform_Surface::myTemplateList,
				     2,
				     OP_MULTI_INPUT_MAX,
				     0));
}
static PRM_Name names[] = {
  PRM_Name("amp",     "Amplitude"),
};

PRM_Template SOP_Deform_Surface::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
	       0, 0, SOP_Node::getGroupSelectButton(GA_GROUP_POINT)),
  PRM_Template(PRM_FLT_J,     1, &names[0], PRMoneDefaults, 0,
	       &PRMscaleRange),
  PRM_Template(),
};


OP_Node *SOP_Deform_Surface::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
  return new SOP_Deform_Surface(net, name, op);
}

SOP_Deform_Surface::SOP_Deform_Surface(OP_Network *net, const char *name, OP_Operator *op)
  : SOP_Node(net, name, op) {
  mySopFlags.setManagesDataIDs(true);
}

SOP_Deform_Surface::~SOP_Deform_Surface()
{
}
OP_ERROR SOP_Deform_Surface::cookInputGroups(OP_Context &context, int alone) {
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


OP_ERROR SOP_Deform_Surface::cookMySop(OP_Context &context) {
  OP_AutoLockInputs inputs(this);
  if (inputs.lock(context) >= UT_ERROR_ABORT)
    return error();
    
  flags().timeDep = 1;
  fpreal frame = OPgetDirector()->getChannelManager()->getSample(context.getTime());
  frame *= 0.03;
  float t = context.getTime();
  int fr = context.getFrame();
  float dt = 0.025;
  if (fr != 0) {
    dt = t/fr;
  }

  float amp = AMP(t);
    
  duplicateSource(0, context); //grid

  GA_RWHandleV3 Phandle(gdp->findAttribute(GA_ATTRIB_POINT, "P"));

  // we assume that all input (1,..) have the same number of wavelength (same number of primitive)
  int nb_wl = inputGeo(1)->getPrimitiveRange().getEntries();
  GA_Offset ptoff;
  GA_FOR_ALL_PTOFF(gdp, ptoff) {
    UT_Vector3 Pvalue = gdp->getPos3(ptoff);
    Pvalue.y() = 0;
    gdp->setPos3(ptoff, Pvalue);
  }

  if (fr <= 1) {
    // each point of the grid get list of complex ampli (one for each wavelength) (spectrum of the wave)
    GA_RWHandleF amplir_attrib(gdp->findFloatTuple(GA_ATTRIB_POINT, "ampli_r", nb_wl));
    GA_RWHandleF amplii_attrib(gdp->findFloatTuple(GA_ATTRIB_POINT, "ampli_i", nb_wl));
    if (!amplir_attrib.isValid()) {
      amplir_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_POINT, "ampli_r", nb_wl));
    }
    if (!amplii_attrib.isValid()) {
      amplii_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_POINT, "ampli_i", nb_wl));
    }
    GA_FOR_ALL_PTOFF(gdp, ptoff) {
      for (int w = 0; w < nb_wl; ++w) {
	amplir_attrib.set(ptoff, w, 0);
	amplii_attrib.set(ptoff, w, 0);
      }
    }
     
    //compute the spectrum at each point of the grid
    int nb_inputs = getInputsArraySize();
    for (int input = 1; input < nb_inputs; ++input) { //all these inputs should be sets of sources
      const GU_Detail *fs = inputGeo(input);
      GA_ROHandleF w_handle(fs->findAttribute(GA_ATTRIB_PRIMITIVE, "wavelengths"));
      if (!w_handle.isValid()) {
	addError(SOP_ATTRIBUTE_INVALID, "wavelengths");
	return error();
      }
   
      const GA_Attribute *afs = fs->findFloatTuple(GA_ATTRIB_POINT, "ampli", 2);
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
      // add contribution of each wavelength
      for (GA_Iterator lcl_it((fs)->getPrimitiveRange()); lcl_it.blockAdvance(lcl_start, lcl_end); ) {
	for (prim_off = lcl_start; prim_off < lcl_end; ++prim_off) {
	  float wl = w_handle.get(prim_off);
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
	      //	      float v = velocity(k, om);//0.5*omega/k;
	      float ar = 0, ai = 0;
	      tuple->get(afs, *it, ar, 0);
	      tuple->get(afs, *it, ai, 1);
	      std::complex<float> ampli(ar, ai);
	      a += ampli*fund_solution(k*r);
	    }
	    float tmpr = amplir_attrib.get(ptoff, w);
	    float tmpi = amplii_attrib.get(ptoff, w);
	    amplir_attrib.set(ptoff, w, tmpr + real(a));
	    amplii_attrib.set(ptoff, w, tmpi + imag(a));
	    Pvalue.y() += real(amp*a*exp(-COMPLEX(0, 1)*(om*(float)t)));
	    gdp->setPos3(ptoff, Pvalue);
	  }
	  ++w;
	}
      }
    }
    amplir_attrib->bumpDataId();
    amplii_attrib->bumpDataId();

  } else {  //sum the contribution of each wavelength to get the height at each point

    GA_Offset prim_off;
    GA_FOR_ALL_PTOFF(gdp, ptoff) {
      UT_Vector3 Pvalue = gdp->getPos3(ptoff);
    
      const GU_Detail *fs = inputGeo(1);
      GA_ROHandleF w_handle(fs->findAttribute(GA_ATTRIB_PRIMITIVE, "wavelengths"));
      GA_Offset lcl_start, lcl_end;
      GA_RWHandleF amplir_attrib(gdp->findFloatTuple(GA_ATTRIB_POINT, "ampli_r", nb_wl));
      GA_RWHandleF amplii_attrib(gdp->findFloatTuple(GA_ATTRIB_POINT, "ampli_i", nb_wl));
      if (!amplir_attrib.isValid()) {
	addError(SOP_ATTRIBUTE_INVALID, "ampli real");
	return error();
      }
      if (!amplii_attrib.isValid()) {
	addError(SOP_ATTRIBUTE_INVALID, "ampli imag");
	return error();
      }
      
      int w = 0;
      for (GA_Iterator lcl_it((fs)->getPrimitiveRange()); lcl_it.blockAdvance(lcl_start, lcl_end); ) {
	for (prim_off = lcl_start; prim_off < lcl_end; ++prim_off) {
	  float wl = w_handle.get(prim_off);
	  float k = M_PI*2.0/wl;
	  float om = omega(k);//sqrtf(9.81*k + 0.074/1000*pow(k, 3));
	  float ar = 1, ai = 0;
	  ar = amplir_attrib.get(ptoff, w);
	  ai = amplii_attrib.get(ptoff, w);
	  std::complex<float> a(ar, ai);
	  Pvalue.y() += real(amp*a*exp(-COMPLEX(0, 1)*(om*(float)t)));
	  gdp->setPos3(ptoff, Pvalue);
	  ++w;						 
	}
      }

    }
  }
  Phandle.bumpDataId();
  return error();
}
