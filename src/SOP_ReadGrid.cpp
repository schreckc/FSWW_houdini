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
 * Read Grid SOP
 *---------------------------------------------------------------------------
 */

#include "SOP_ReadGrid.hpp"

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
  table->addOperator(new OP_Operator("read_grid",
				     "Read Grid",
				     SOP_Read_Grid::myConstructor,
				     SOP_Read_Grid::myTemplateList,
				     0,
				     0,
				     nullptr,  
				     OP_FLAG_GENERATOR));
}
static PRM_Name names[] = {
  PRM_Name("file",   "File"),
};

PRM_Default* off_default = new PRM_Default(0.3);
PRM_Default* dens_default = new PRM_Default(3);

PRM_Template
SOP_Read_Grid::myTemplateList[] = {
  PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu,
	       0, 0, SOP_Node::getGroupSelectButton(GA_GROUP_POINT)),
  PRM_Template(PRM_STRING,  1, &names[0], PRMzeroDefaults),
  PRM_Template(),
};


OP_Node *SOP_Read_Grid::myConstructor(OP_Network *net, const char *name, OP_Operator *op) {
  return new SOP_Read_Grid(net, name, op);
}

SOP_Read_Grid::SOP_Read_Grid(OP_Network *net, const char *name, OP_Operator *op)
  : SOP_Node(net, name, op) {
  mySopFlags.setManagesDataIDs(true);
}

SOP_Read_Grid::~SOP_Read_Grid()
{
}
OP_ERROR
SOP_Read_Grid::cookInputGroups(OP_Context &context, int alone)
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


OP_ERROR SOP_Read_Grid::cookMySop(OP_Context &context) {
   OP_AutoLockInputs inputs(this);
  if (inputs.lock(context) >= UT_ERROR_ABORT)
     return error();

  flags().timeDep = 1;
  float t = context.getTime();

  int fr = context.getFrame();
    
  UT_String file_name;
  FILE(file_name);
  std::stringstream ss; 
  ss <<file_name<<fr<<".obj";
  std::string str(ss.str());

  std::cout<<"reading "<<str<<std::endl;
  
  std::ifstream file;
  file.open(str.c_str());

  gdp->clearAndDestroy();
  
  std::string line;
  while (getline(file,line)) {
    if (line.substr(0,1) != "#") {
      break;
    }
  }
  
  int nb_pts = 0;
  if (line.substr(0,2) == "o ") {
    std::istringstream s(line.substr(2));
    s >> nb_pts;
  } else {
    std::cout<<line<<std::endl;
    addError(SOP_ATTRIBUTE_INVALID, "incorrect file (no number of points specified)");
    return error();
  }
  GA_Offset ptoff = gdp->appendPointBlock(nb_pts);

  uint i = 0;
  while (getline(file, line)) {
    if (line.substr(0,2) == "v ") {
      std::istringstream s(line.substr(2));
      UT_Vector3 v;
      s >> v(0); 
      s >> v(1); 
      s >> v(2);
      gdp->setPos3(ptoff+i, v);
      ++i;
    } else  if (line.substr(0,1) == "f") {
      std::cout<<line<<std::endl;
      std::istringstream s(line.substr(1));
      GA_Offset vtxoff;
      int nv = 0;
      s >> nv;
      GA_Offset prim_off = gdp->appendPrimitivesAndVertices(GA_PRIMPOLY, 1, nv, vtxoff, true);
      for (int f = 0; f < nv; ++f) {
	int ind = 0;
	s >> ind;
	gdp->getTopology().wireVertexPoint(vtxoff+f,ptoff+ind);
      }
      
    } else  if (line.substr(0,1) == "#") {
      // ignore comment
    } else {
      addError(SOP_ATTRIBUTE_INVALID, "incorrect file");
      return error();
    }
  }
  gdp->bumpDataIdsForAddOrRemove(true, true, true);
  return error();
}
