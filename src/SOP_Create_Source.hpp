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
 * Create_Source SOP
 *----------------------------------------------------------------------------
 * create one source that can be used as input.
 * Parameters:
 *    -Position (X, Y, Z): position of the source (X, Z: coordintate on the water surface, Y=0)
 *    -Amplitude
 *    -Phase
 *    -Minimum/maximum wavelength, wavelength multiplicative step: used to compute the range of wl
 *    -Type (not used yet, keep at 0)
 *    -Size of the buffer: size of the buffer recording past amplitude
 *    -Damping
 */


#ifndef __SOP_Create_Source_h__
#define __SOP_Create_Source_h__

#include <SOP/SOP_Node.h>

class SOP_Create_Source : public SOP_Node {
  
public:
  SOP_Create_Source(OP_Network *net, const char *name, OP_Operator *op);
  virtual ~SOP_Create_Source();

  static PRM_Template myTemplateList[];
  static OP_Node *myConstructor(OP_Network*, const char *, OP_Operator *);

  virtual OP_ERROR             cookInputGroups(OP_Context &context, 
					       int alone = 0);

protected:
  virtual OP_ERROR cookMySop(OP_Context &context);
private:
  void        getGroups(UT_String &str){ evalString(str, "group", 0, 0); }
  fpreal      AMP(fpreal t)            { return evalFloat("amp", 0, t); }
  fpreal      PHASE(fpreal t)          { return evalFloat("phase", 0, t); }
  fpreal      WL_MIN(fpreal t)             { return evalFloat("wl_min", 0, t); }
  fpreal      WL_MAX(fpreal t)             { return evalFloat("wl_max", 0, t); }
  fpreal      WL_STEP(fpreal t)             { return evalFloat("wl_step", 0, t); }
  int         TYPE(fpreal t) {return evalInt("type", 0, t);}
  int         BUFFER_SIZE(fpreal t) {return evalInt("buffer_size", 0, t);}
  fpreal      DAMPING(fpreal t) { return evalFloat("damping", 0, t); }
  fpreal      X(fpreal t) { return evalFloat("pos", 0, t); }
  fpreal      Y(fpreal t) { return evalFloat("pos", 1, t); }
  fpreal      Z(fpreal t) { return evalFloat("pos", 2, t); }
  int        INTER_SRC(fpreal t) {return evalInt("inter_src", 0, t);}
  
  const GA_PointGroup *myGroup;
};

#endif
