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
 *
 *----------------------------------------------------------------------------
 * Write Grid SOP
 *---------------------------------------------------------------------------
 */


#ifndef __SOP_Write_Grid_h__
#define __SOP_Write_Grid_h__

#include <SOP/SOP_Node.h>
#include "definitions.hpp"
#include <Eigen/SVD>

class SOP_Write_Grid : public SOP_Node {
  
public:
  SOP_Write_Grid(OP_Network *net, const char *name, OP_Operator *op);
  virtual ~SOP_Write_Grid();

  static PRM_Template myTemplateList[];
  static OP_Node *myConstructor(OP_Network*, const char *, OP_Operator *);

  virtual OP_ERROR             cookInputGroups(OP_Context &context, 
					       int alone = 0);

protected:
  virtual OP_ERROR cookMySop(OP_Context &context);
private:
  void        getGroups(UT_String &str){ evalString(str, "group", 0, 0); }
  void        FILE(UT_String &str){ evalString(str, "file", 0, 0); }

  const GA_PointGroup *myGroup;
  
};

#endif
