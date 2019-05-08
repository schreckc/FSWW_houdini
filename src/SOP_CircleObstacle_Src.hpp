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
 * Circle Obstalce SOP
 */


#ifndef __SOP_Circle_Obstacle_Src_h__
#define __SOP_Circle_Obstacle_Src_h__

#include <SOP/SOP_Node.h>
#include "definitions.hpp"
#include <Eigen/SVD>

namespace HDK_Sample {

/// Pure C++ implementation of @ref SOP_HOMWave
/// @see SOP_HOMWave, vex_wave(), @ref HOM/SOP_HOMWave.py
class SOP_Circle_Obstacle_Src : public SOP_Node
{
public:
    SOP_Circle_Obstacle_Src(OP_Network *net, const char *name, OP_Operator *op);
    virtual ~SOP_Circle_Obstacle_Src();

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
  fpreal      OFF(fpreal t)            { return evalFloat("off", 0, t); }
  fpreal      DENSITY(fpreal t)          { return evalFloat("density", 0, t); }
  fpreal      RADIUS(fpreal t)          { return evalFloat("radius", 0, t); }
  fpreal      CX(fpreal t) { return evalFloat("center", 0, t); }
  fpreal      CY(fpreal t) { return evalFloat("center", 1, t); }
  fpreal      CZ(fpreal t) { return evalFloat("center", 2, t); }
  int        INTER_SRC(fpreal t) {return evalInt("inter_src", 0, t);}
  /// This is the group of geometry to be manipulated by this SOP and cooked
    /// by the method "cookInputGroups".
    const GA_PointGroup *myGroup;
  
};
} // End of HDK_Sample namespace

#endif
