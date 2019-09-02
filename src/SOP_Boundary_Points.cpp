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
 * Boundary_Points SOP
 *----------------------------------------------------------------------------
 */


#include "SOP_Boundary_Points.hpp"

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
  table->addOperator(new OP_Operator("boundary_points_fs",
				     "Boundary Points FS",
				     SOP_Boundary_Points::myConstructor,
				     SOP_Boundary_Points::myTemplateList,
				     1,
				     1,
				     nullptr,  
				     OP_FLAG_GENERATOR));
}
static PRM_Name names[] = {
  PRM_Name("dt_",     "Time Step"),
  PRM_Name("win_size",   "Window Size"),
  PRM_Name("shift_b",   "Shift B"),
  PRM_Name("shift_u",   "Shift U")
};

PRM_Template
SOP_Boundary_Points::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
	       0, 0, SOP_Node::getGroupSelectButton(GA_GROUP_POINT)),
  PRM_Template(PRM_FLT_J,     1, &names[0], PRMoneDefaults, 0,
	       &PRMscaleRange),
  PRM_Template(PRM_INT_J,  1, &names[1], new PRM_Default(256)),
  PRM_Template(PRM_INT_J,  1, &names[2], new PRM_Default(124)),
  PRM_Template(PRM_INT_J,  1, &names[3], new PRM_Default(2)),
  PRM_Template(),
};


OP_Node *
SOP_Boundary_Points::myConstructor(OP_Network *net, const char *name, OP_Operator *op) {
  return new SOP_Boundary_Points(net, name, op);
}

SOP_Boundary_Points::SOP_Boundary_Points(OP_Network *net, const char *name, OP_Operator *op)
  : SOP_Node(net, name, op) {
  mySopFlags.setManagesDataIDs(true);
}

SOP_Boundary_Points::~SOP_Boundary_Points()
{
}
OP_ERROR
SOP_Boundary_Points::cookInputGroups(OP_Context &context, int alone)
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

std::ofstream & SOP_Boundary_Points::record(std::ofstream & file) {
  std::list<InputPoint>::iterator it;
  for (it = inputPoints.begin(); it !=inputPoints.end(); ++it) {
    (*it).record_samples(file);
  }
  return file;
}

std::ifstream & SOP_Boundary_Points::read(std::ifstream & file) {
  std::list<InputPoint>::iterator it;
  for (it = inputPoints.begin(); it !=inputPoints.end(); ++it) {
    FLOAT next_sample;
    file >> next_sample;
    (*it).update(next_sample);
  }
  return file;
}

OP_ERROR SOP_Boundary_Points::cookMySop(OP_Context &context) {
  flags().timeDep = 1;

 float t = context.getTime();
  int fr = context.getFrame();
  
  // TODO: user def time step
  float dt_ = 0.1/3.0;
  t = dt_*fr;
   // if (fr != 0) {
   //   dt_ = t/fr;
   // }
   // std::cout<<"TIME STEP "<<dt_<<" "<<t<<" "<<fr<<std::endl;
   OP_AutoLockInputs inputs(this);
   if (inputs.lock(context) >= UT_ERROR_ABORT)
     return error();
  const GU_Detail *fs = inputGeo(0);
  int winsize = WIN_SIZE(0);
   uint shift_b = SHIFT_B(0);
   uint shift_u = SHIFT_U(0);
  
  if (fr == 1) {
  
    float sample_rate = 1.0/(dt_);
    float freq_step = 1.0/(winsize*dt_);

  float f_min = freq_step;
  float f_max = sample_rate;
  float k_min = pow((2*M_PI*f_min), 2)/9.81;
  float k_max = pow((2*M_PI*f_max), 2)/9.81;
  float w_min = 2*M_PI/k_max;
  float w_max = 2*M_PI/k_min;
  std::cout<<"wl min-max "<<w_min<<" - "<<w_max<<std::endl;
  std::cout<<"freq min-max "<<f_min<<" - "<<f_max<<std::endl;

  uint nb_wl = winsize/2;
  nb_wl -= shift_b + shift_u;

     //record
    std::string str = "test_record";
    std::ofstream file;
    file.open(str);
    file << nb_wl<<"\n";
  
  // Compute the range of wavelength we want to use between WL_MIN and WL_MAX(t)
  // Note: multiplicative step between wl
   wave_lengths = std::vector<float>(winsize);
  float f_cur = f_min;
  for (int i = 0; i < winsize; ++i) {
    float k_cur = pow((2*M_PI*f_cur), 2)/9.81;
    float w_cur = 2*M_PI/k_cur;
    wave_lengths[/*winsize - 1 - */i] = w_cur;
    f_cur += freq_step;
    //  std::cout<<i<<" "<<wave_lengths[i]<<std::endl;
  }
  
  // compute the number of time step between updates for each wavelength
  std::vector<int> ampli_steps(winsize);
  float wl = wave_lengths[winsize - nb_wl - 1];
  FLOAT period = 0.5*wl/velocity(2*M_PI/wl);
  int d_period = period/(dt_);

  std::cout<<"nb wl "<<nb_wl<<" " <<winsize - nb_wl - 1<<std::endl;
  if (d_period == 0) {
    ampli_steps[winsize - nb_wl - 1] = 1;
  } else {
    ampli_steps[winsize - nb_wl - 1] = d_period;
  }
  for (int w = winsize-2; w >= 0; --w) {
    wl = wave_lengths[w];
    period = 0.5*wl/velocity(2*M_PI/wl);
    d_period = period/(dt_*ampli_steps[winsize - nb_wl - 1]);
    if (d_period == 0) {
      ampli_steps[w] = 1;
    } else {
      ampli_steps[w] = d_period*ampli_steps[winsize - nb_wl - 1];
    }
  }

  uint nb_pts = fs->getPointRange().getEntries();
  //GA_Offset ptoff = gdp->appendPointBlock(nb_pts);
    GA_Range range = fs->getPointRange();

    // create InputPoint that record height of input simu
    uint i = 0;
    inputPoints.clear();
   for(GA_Iterator itfs = range.begin(); itfs != range.end(); ++itfs) {
     UT_Vector3 pos_fs = fs->getPos3(*itfs);
     //  gdp->setPos3(ptoff+i, pos_fs);
      InputPoint ip(winsize, dt_);
      ip.setPos(pos_fs(0), pos_fs(1));
      inputPoints.push_back(ip);
      ++i;
   }
   inputPoints.front().setName("bp_sop_test");

  
   //  begin creation of geometry
   gdp->clearAndDestroy();
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
  GA_RWHandleF ampli_attrib(gdp->findFloatTuple(GA_ATTRIB_POINT, "ampli", 2));
  if (!ampli_attrib.isValid()) {
    ampli_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_POINT, "ampli", 2));
  }
  if (!ampli_attrib.isValid()) {
    addError(SOP_MESSAGE, "Failed to create attribute ampli");
    return error();
  }

  GA_Offset ptoff;
  GA_FOR_ALL_PTOFF(gdp, ptoff) {
    ampli_attrib.set(ptoff, 0, 0);
    ampli_attrib.set(ptoff, 1, 0);
  }

  
   // creation of the primitve attibutes
  GA_RWHandleF wl_attrib(gdp->findFloatTuple(GA_ATTRIB_PRIMITIVE, "wavelength", 1));
  GA_RWHandleI as_attrib(gdp->findIntTuple(GA_ATTRIB_PRIMITIVE, "ampli_step", 1));
  if (!wl_attrib.isValid()) {
    wl_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "wavelength", 1));
  }
  if (!wl_attrib.isValid()) {
    addError(SOP_MESSAGE, "Failed to create attribute wavelength");
    return error();
  }
  if (!as_attrib.isValid()) {
    as_attrib = GA_RWHandleI(gdp->addIntTuple(GA_ATTRIB_PRIMITIVE, "ampli_step", 1));
  }
  if (!as_attrib.isValid()) {
    addError(SOP_MESSAGE, "Failed to create attribute ampli_step");
    return error();
  }
  // setting primitive attibutes
  uint w = 0;
  GA_Range range_i = gdp->getPrimitiveRange();
  for(GA_Iterator itp = range_i.begin(); itp != range_i.end(); ++itp, ++w) {
     wl_attrib.set(*itp, wave_lengths[w+shift_u]);
     as_attrib.set(*itp, ampli_steps[w+shift_u]);
     std::cout<<"w "<<w<<"  wl "<<wave_lengths[w+shift_u]<<std::endl;
    // const GA_Primitive* prim_fs = gdp->getPrimitiveByIndex(w);
    // GA_Range range_fs = prim_fs->getPointRange();
    //  std::list<InputPoint>::iterator it = inputPoints.begin();
    //  for(GA_Iterator itfs = range_fs.begin(); itfs != range_fs.end(); ++itfs, ++it) {
    //   ampli_attrib->set(itfs, 0, it.spectrum_re[i]);
    //   ampli_attrib->set(itfs, 1, it.spectrum_im[i]);
    // }
  }

  
  
   
  // //create window and spectrum for each bp
  // GA_RWHandleF spectrum_attrib(gdp->findFloatTuple(GA_ATTRIB_POINT, "spectrum", winsize));
  // if (!spectrum_attrib.isValid()) {
  //   spectrum_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_POINT, "spectrum", winsize));
  // }
  // if (!spectrum_attrib.isValid()) {
  //   addError(SOP_MESSAGE, "Failed to create attribute spectrum");
  //   return error();
  // }

  // GA_FOR_ALL_PTOFF(gdp, ptoff) {
  //   for (uint i = 0; i < winsize; ++i) {
  //     spectrum_attrib.set(ptoff, i, 0);
  //   }
  // }

  // // creation of the detail attibutes
  //  GA_RWHandleI ws_attrib(gdp->findIntTuple(GA_ATTRIB_DETAIL, "winsize", 1));
  // // GA_RWHandleF damping_attrib(gdp->findFloatTuple(GA_ATTRIB_DETAIL, "damping", 1));
  //  if (!ws_attrib.isValid()) {
  //    ws_attrib = GA_RWHandleI(gdp->addIntTuple(GA_ATTRIB_DETAIL, "winsize", 1));
  //  }
  // if (!ws_attrib.isValid()) {
  //   addError(SOP_MESSAGE, "Failed to create attribute winsize");
  //   return error();
  //  }
  //  ws_attrib.set(0, winsize);
   
  //  GA_RWHandleF wl_attrib(gdp->findFloatTuple(GA_ATTRIB_DETAIL, "wavelengths", winsize/2.0));
  //  if (!wl_attrib.isValid()) {
  //    wl_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_DETAIL, "wavelengths", winsize/2.0));
  //  }
  //  if (!wl_attrib.isValid()) {
  //    addError(SOP_MESSAGE, "Failed to create attribute wavelengths");
  //    return error();
  //  }
  //  GA_RWHandleF as_attrib(gdp->findFloatTuple(GA_ATTRIB_DETAIL, "ampli_steps", winsize/2.0));
  //  if (!as_attrib.isValid()) {
  //    as_attrib = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_DETAIL, "ampli_steps", winsize/2.0));
  //  }
  //  if (!as_attrib.isValid()) {
  //    addError(SOP_MESSAGE, "Failed to create attribute ampli_steps");
  //    return error();
  //  }
  //  for (uint i = 0; i < winsize/2; ++i) {
  //    wl_attrib.set(0, i, wave_lengths[i + winsize/2]);
  //    as_attrib.set(0, i, ampli_steps[i + winsize/2]);
  //  }
  //  spectrum_attrib->bumpDataId();
  //  ws_attrib->bumpDataId();
  //  wl_attrib->bumpDataId();
  //  as_attrib->bumpDataId();
  }
  
  // GA_RWHandleF spectrum_attrib(gdp->findFloatTuple(GA_ATTRIB_POINT, "spectrum", winsize));
  // if (!spectrum_attrib.isValid()) {
  //   addError(SOP_MESSAGE, "Attribute spectrum missing");
  //   return error();
  // }

  GA_Range rangefs = fs->getPointRange();
  GA_Range rangegdp = gdp->getPointRange();
  GA_Iterator itfs = rangefs.begin();
  GA_Iterator itgdp = rangegdp.begin();
  std::list<InputPoint>::iterator it = inputPoints.begin();
  for(; itfs != rangefs.end(); ++itfs, ++itgdp, ++it) {
    UT_Vector3 pos_fs = fs->getPos3(*itfs);
    (*it).update(pos_fs(1)-3.0);
    
  }
 
  
  
  if (fr %/*winsize/2.0*/8 == 0) {
    GA_RWHandleF ampli_attrib(gdp->findFloatTuple(GA_ATTRIB_POINT, "ampli", 2));
    if (!ampli_attrib.isValid()) {
      addError(SOP_MESSAGE, "cannot find attribute ampli");
      return error();
    }
    
    GA_Offset ptoff;
    GA_Range range_i = gdp->getPrimitiveRange();
    int w = 0;
    for(GA_Iterator itp = range_i.begin(); itp != range_i.end(); ++itp, ++w) {
      FLOAT wl = wave_lengths[w];
      float k = M_PI*2.0/wl;
      float om = omega(k);
      
      const GA_Primitive* prim = gdp->getPrimitive(*itp);
      GA_Range range_bp = prim->getPointRange();
      it = inputPoints.begin();
      for(GA_Iterator itbp = range_bp.begin(); itbp != range_bp.end(); ++itbp, ++it) {
	COMPLEX a = 0.5f*COMPLEX((*it).spectrum_re[w+shift_u], (*it).spectrum_im[w+shift_u]);
	a /= sqrt((FLOAT)winsize);
	// if (itbp == range_bp.begin()) {
	//   std::cout<<"ampli0  "<<a<<std::endl;
	// }
	//	a*= exp(COMPLEX(0, 1)*(om*(float)t));
	ampli_attrib.set(*itbp, 0, real(a));
	ampli_attrib.set(*itbp, 1, imag(a));
	 // if (itbp == range_bp.begin()) {
	 //   std::cout<<"ampli "<<a<<std::endl;
	 // }
	// if (fr < 200) {
	//   ampli_attrib.set(*itbp, 0, 1);
	//   ampli_attrib.set(*itbp, 1, 0);
	// } else {
	//   ampli_attrib.set(*itbp, 0, 0);
	//   ampli_attrib.set(*itbp, 1, 0);
	// }
	//	    std::cout<<"ampli bp "<<w<<" "<<*itbp<<" "<<(*it).spectrum_re[w]<<" "<<(*it).spectrum_im[w]<<std::endl;
      }
      // for (uint i = 0; i < winsize/2; ++i) {
      //   spectrum_attrib.set(*itgdp, 2*i, (*it).spectrum_re[i]);
      //   spectrum_attrib.set(*itgdp, 2*i+1, (*it).spectrum_im[i]);
      // }
    }
    //  std::cout<<"plotting spectrum "<<fr<<std::endl;
    inputPoints.front().plotSpectrum();
    inputPoints.front().plotSpectrogram();
    inputPoints.front().plotSamples();
  }
  return error();
}
