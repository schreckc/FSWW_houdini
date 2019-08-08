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
 * Deform_Surface SOP
 *----------------------------------------------------------------------------
 * At first frame, compute the spectrum a each point of the grid (input 0) by 
 * summing the contribution of all sources (each other input is a set of sources).
 * At each frame, sum the contribution of each wavelength at each point to get 
 * the height (according to the spectrum of the point).
 */


#ifndef __SOP_Deform_Surface_h__
#define __SOP_Deform_Surface_h__

#include <SOP/SOP_Node.h>

class SOP_Deform_Surface : public SOP_Node
{
public:
  SOP_Deform_Surface(OP_Network *net, const char *name, OP_Operator *op);
  virtual ~SOP_Deform_Surface();

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
  fpreal      AMP(fpreal t)           { return evalFloat("amp", 0, t); }

  /// This is the group of geometry to be manipulated by this SOP and cooked
  /// by the method "cookInputGroups".
  const GA_PointGroup *myGroup;

  int gdp_count;
  UT_Array<const GU_Detail*> myGDPLists;
};


#endif
