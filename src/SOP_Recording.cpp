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
 * Recording SOP
 *----------------------------------------------------------------------------
 */


#include "SOP_Recording.hpp"

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
  table->addOperator(new OP_Operator("recording_fs",
				     "Recording FS",
				     SOP_Recording::myConstructor,
				     SOP_Recording::myTemplateList,
				     1,
				     1,
				     nullptr,  
				     OP_FLAG_GENERATOR));
}
static PRM_Name names[] = {
  PRM_Name("dt_",     "Time Step"),
  PRM_Name("win_size",   "Window Size"),
  PRM_Name("shift_b",   "Shift B"),
  PRM_Name("shift_u",   "Shift U"),
   PRM_Name("name",   "Name")
};

PRM_Template
SOP_Recording::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
	       0, 0, SOP_Node::getGroupSelectButton(GA_GROUP_POINT)),
  PRM_Template(PRM_FLT_J,     1, &names[0], PRMoneDefaults, 0,
	       &PRMscaleRange),
  PRM_Template(PRM_INT_J,  1, &names[1], new PRM_Default(256)),
  PRM_Template(PRM_INT_J,  1, &names[2], new PRM_Default(124)),
  PRM_Template(PRM_INT_J,  1, &names[3], new PRM_Default(2)),
  PRM_Template(PRM_STRING,  1, &names[4]),
  PRM_Template(),
};


OP_Node *
SOP_Recording::myConstructor(OP_Network *net, const char *name, OP_Operator *op) {
  return new SOP_Recording(net, name, op);
}

SOP_Recording::SOP_Recording(OP_Network *net, const char *name, OP_Operator *op)
  : SOP_Node(net, name, op) {
  mySopFlags.setManagesDataIDs(true);
}

SOP_Recording::~SOP_Recording()
{
}
OP_ERROR
SOP_Recording::cookInputGroups(OP_Context &context, int alone)
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

std::ofstream & SOP_Recording::record(std::ofstream & file) {
  std::list<InputPoint>::iterator it;
  for (it = inputPoints.begin(); it !=inputPoints.end(); ++it) {
    (*it).record_samples(file);
  }
  return file;
}

std::ifstream & SOP_Recording::read(std::ifstream & file) {
  std::list<InputPoint>::iterator it;
  for (it = inputPoints.begin(); it !=inputPoints.end(); ++it) {
    FLOAT next_sample;
    file >> next_sample;
    (*it).update(next_sample);
  }
  return file;
}

OP_ERROR SOP_Recording::cookMySop(OP_Context &context) {
 //  flags().timeDep = 1;

 // float t = context.getTime();
 //  int fr = context.getFrame();
  
 //  // TODO: user def time step
 //  float dt_ = 0.1/3.0;
 //  t = dt_*fr;
 //   // if (fr != 0) {
 //   //   dt_ = t/fr;
 //   // }
 //   // std::cout<<"TIME STEP "<<dt_<<" "<<t<<" "<<fr<<std::endl;
 //   OP_AutoLockInputs inputs(this);
 //   if (inputs.lock(context) >= UT_ERROR_ABORT)
 //     return error();
 //  const GU_Detail *fs = inputGeo(0);
 //  int winsize = WIN_SIZE(0);
 //   uint shift_b = SHIFT_B(0);
 //   uint shift_u = SHIFT_U(0);
  
 //  if (fr == 1) {
  
 //    float sample_rate = 1.0/(dt_);
 //    float freq_step = 1.0/(winsize*dt_);

 //  float f_min = freq_step;
 //  float f_max = sample_rate;
 //  float k_min = pow((2*M_PI*f_min), 2)/9.81;
 //  float k_max = pow((2*M_PI*f_max), 2)/9.81;
 //  float w_min = 2*M_PI/k_max;
 //  float w_max = 2*M_PI/k_min;
 //  std::cout<<"wl min-max "<<w_min<<" - "<<w_max<<std::endl;
 //  std::cout<<"freq min-max "<<f_min<<" - "<<f_max<<std::endl;

 //  int nb_wl = winsize/2;
 //  nb_wl -= shift_b + shift_u;
 //  if (nb_wl <= 0) {
 //    nb_wl = 1;
 //    shift_u = 0;
 //  }
 //    //  //record
 //    // std::string str = "test_record";
 //    // std::ofstream file;
 //    // file.open(str);
 //    // file << nb_wl<<"\n";
  
 //  // Compute the range of wavelength we want to use between WL_MIN and WL_MAX(t)
 //  // Note: multiplicative step between wl
 //   wave_lengths = std::vector<float>(winsize);
 //  float f_cur = f_min;
 //  for (int i = 0; i < winsize; ++i) {
 //    float k_cur = pow((2*M_PI*f_cur), 2)/9.81;
 //    float w_cur = 2*M_PI/k_cur;
 //    wave_lengths[/*winsize - 1 - */i] = w_cur;
 //    f_cur += freq_step;
 //    //  std::cout<<i<<" "<<wave_lengths[i]<<std::endl;
 //  }
  
 //  // compute the number of time step between updates for each wavelength
 //  ampli_steps = std::vector<int>(winsize);
 //  float wl = wave_lengths[winsize - nb_wl - 1];
 //  FLOAT period = 0.5*wl/velocity(2*M_PI/wl);
 //  int d_period = period/(dt_);

 //  if (d_period == 0) {
 //    ampli_steps[winsize - nb_wl - 1] = 1;
 //  } else {
 //    ampli_steps[winsize - nb_wl - 1] = d_period;
 //  }
 //  for (int w = winsize-2; w >= 0; --w) {
 //    wl = wave_lengths[w];
 //    period = 0.5*wl/velocity(2*M_PI/wl);
 //    d_period = period/(dt_*ampli_steps[winsize - nb_wl - 1]);
 //    if (d_period == 0) {
 //      ampli_steps[w] = 1;
 //    } else {
 //      ampli_steps[w] = d_period*ampli_steps[winsize - nb_wl - 1];
 //    }
 //  }

 //  uint nb_pts = fs->getPointRange().getEntries();
 //  //GA_Offset ptoff = gdp->appendPointBlock(nb_pts);
 //    GA_Range range = fs->getPointRange();

 //    // create InputPoint that record height of input simu
 //    uint i = 0;
 //    inputPoints.clear();
 //    uint nb_ip = range.getEntries();
 //   for(GA_Iterator itfs = range.begin(); itfs != range.end(); ++itfs) {
 //     UT_Vector3 pos_fs = fs->getPos3(*itfs);
 //     //  gdp->setPos3(ptoff+i, pos_fs);
 //      InputPoint ip(winsize, dt_);
 //      ip.setPos(pos_fs(0), pos_fs(2));
 //      UT_String name_str;
 //       NAME(name_str);
 //       std::stringstream ss; 
 //       ss <<name_str<<i;
 //       std::string str(ss.str());
 //       ip.setName(str);

 //      inputPoints.push_back(ip);
 //      ++i;
 //   }
 //  }
 //  gdp->clearAndDestroy();
    
 //  GA_Range rangefs = fs->getPointRange();
 //  GA_Iterator itfs = rangefs.begin();
 //  std::list<InputPoint>::iterator it = inputPoints.begin();
 //  for(; itfs != rangefs.end(); ++itfs, ++it) {
 //    UT_Vector3 pos_fs = fs->getPos3(*itfs);
 //    (*it).update(pos_fs(1));

 //     (*it).plotSpectrum();
 //     (*it).plotSpectrogram();
 //     (*it).plotSamples();
 //   }
 
  
  

  return error();
}
