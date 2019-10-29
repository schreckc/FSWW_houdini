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

#include "SOP_WriteGrid.hpp"

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
  table->addOperator(new OP_Operator("write_grid",
				     "Write Grid",
				     SOP_Write_Grid::myConstructor,
				     SOP_Write_Grid::myTemplateList,
				     1,
				     1,
				     nullptr,  
				     OP_FLAG_GENERATOR));
}
static PRM_Name names[] = {
  PRM_Name("file",   "File"),
};

PRM_Default* off_default = new PRM_Default(0.3);
PRM_Default* dens_default = new PRM_Default(3);

PRM_Template
SOP_Write_Grid::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
	       0, 0, SOP_Node::getGroupSelectButton(GA_GROUP_POINT)),
  PRM_Template(PRM_STRING,  1, &names[0], PRMzeroDefaults),
  PRM_Template(),
};


OP_Node *SOP_Write_Grid::myConstructor(OP_Network *net, const char *name, OP_Operator *op) {
  return new SOP_Write_Grid(net, name, op);
}

SOP_Write_Grid::SOP_Write_Grid(OP_Network *net, const char *name, OP_Operator *op)
  : SOP_Node(net, name, op) {
  mySopFlags.setManagesDataIDs(true);
}

SOP_Write_Grid::~SOP_Write_Grid()
{
}
OP_ERROR
SOP_Write_Grid::cookInputGroups(OP_Context &context, int alone)
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


OP_ERROR SOP_Write_Grid::cookMySop(OP_Context &context) {
   OP_AutoLockInputs inputs(this);
  if (inputs.lock(context) >= UT_ERROR_ABORT)
     return error();

  flags().timeDep = 1;
  float t = context.getTime();

  duplicateSource(0, context);

  int fr = context.getFrame();
    
  UT_String file_name;
  FILE(file_name);
  std::stringstream ss; 
  ss <<file_name<<fr<<".obj";
  std::string str(ss.str());
  
  std::ofstream file;
  file.open(str.c_str());

  GA_Range range = gdp->getPointRange();
  int nb_pts = range.getEntries();

  file << "#Grid from simulation frame "<<fr<<"\n";
  file << "o "<<nb_pts<<"\n";
    
  for(GA_Iterator it = range.begin(); it != range.end(); ++it) {
    UT_Vector3 pos = gdp->getPos3((*it));
    file << "v "<< pos(0) <<" "<<pos(1)<<" "<<pos(2)<<"\n"; 
  }

  GA_Range range_prim = gdp->getPrimitiveRange();
  //  std::cout<<"nb prim "<< range_prim.getEntries()<<std::endl;
   for(GA_Iterator it = range_prim.begin(); it != range_prim.end(); ++it) {
     const GA_Primitive* face = gdp->getPrimitive((*it));
     GA_Range range_face = face->getVertexRange();
     file<<"f";
     int nv = range_face.getEntries();
     file<<nv<<" ";
     for(GA_Iterator itv = range_face.begin(); itv != range_face.end(); ++itv) {
       int ind = gdp->vertexPoint(*itv) - (*range.begin());
       file<<ind<<" ";
     }
     file<<"\n";
   }
  return error();
}
