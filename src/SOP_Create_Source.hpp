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
 * Create_Source SOP
 *----------------------------------------------------------------------------
 * create one source that can be used as input.
 * Parameters:
 *    -Position (X, Y, Z): position of the source (X, Z: coordintate on the water surface, Y=0)
 *    -Amplitude
 *    -Phase (not used yet)
 *    -Minimum/maximum wavelength, wavelength multiplicative step: used to compute the range of wl
 *    -Type (not used yet, keep at 0)
 *    -Size of the buffer: size of the buffer recording past amplitude
 *    -Damping
 */


#ifndef __SOP_Create_Source_h__
#define __SOP_Create_Source_h__

#include <SOP/SOP_Node.h>

class SOP_Create_Source : public SOP_Node
{
public:
  SOP_Create_Source(OP_Network *net, const char *name, OP_Operator *op);
  virtual ~SOP_Create_Source();

  static PRM_Template myTemplateList[];
  static OP_Node *myConstructor(OP_Network*, const char *, OP_Operator *);

  /// This method is created so that it can be called by handles.  It only
  /// cooks the input group of this SOP.  The geometry in this group is
  /// the only geometry manipulated by this SOP.
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

  /// This is the group of geometry to be manipulated by this SOP and cooked
  /// by the method "cookInputGroups".
  const GA_PointGroup *myGroup;

  int gdp_count;
  UT_Array<const GU_Detail*> myGDPLists;
};

#endif
