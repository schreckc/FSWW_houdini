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
 * Texture Obstacle SOP
 *----------------------------------------------------------------------------
 * Create set of point sources along an offset surface of the obstacle.
 * Range of wavelength (and time step per wl), and is copied from input geometry, as well as
 *    the detail attibute.
 * Create on primitve and subset of sources for each wl.
 * Spacing depends on the wavelength and the parameter "density".
 * I use this node also to create a set of point sampling the border of the obstacle (offset=0)
 *    for the boundary conditions.
 * Note: the density of the boundary points should be at least twice the density of the
 *    sources.
 * Note 2: for the aperiodic version, do not forget to check the "interactive sources" box in
 *   the parameter of the node creating the sources of the obstacle.
 * Note 3: the density is also limited by the resolution of the
 *   texture (cannot have more than one source per pixel).
 */


#ifndef __SOP_Texture_Obstacle_Src_h__
#define __SOP_Texture_Obstacle_Src_h__

#include <SOP/SOP_Node.h>
#include "definitions.hpp"
#include "Grid.hpp"
#include <Eigen/SVD>
#include <SDL2/SDL_image.h>


class SOP_Texture_Obstacle_Src : public SOP_Node
{
public:
  SOP_Texture_Obstacle_Src(OP_Network *net, const char *name, OP_Operator *op);
  virtual ~SOP_Texture_Obstacle_Src();

  static PRM_Template myTemplateList[];
  static OP_Node *myConstructor(OP_Network*, const char *, OP_Operator *);

  virtual OP_ERROR             cookInputGroups(OP_Context &context, 
					       int alone = 0);

protected:
  virtual OP_ERROR cookMySop(OP_Context &context);
private:
  void        getGroups(UT_String &str){ evalString(str, "group", 0, 0); }
  fpreal      OFF(fpreal t)            { return evalFloat("off", 0, t); }
  fpreal      DENSITY(fpreal t)          { return evalFloat("density", 0, t); }
  void        FILE(UT_String &str, fpreal t) { evalString(str, "file", 0, t); }
  fpreal      CX(fpreal t) { return evalFloat("center", 0, t); }
  fpreal      CY(fpreal t) { return evalFloat("center", 1, t); }
  fpreal      CZ(fpreal t) { return evalFloat("center", 2, t); }
  int        INTER_SRC(fpreal t) {return evalInt("inter_src", 0, t);}
  
  void loadTexture();
  
  const GA_PointGroup *myGroup;
  Grid gr;
};

#endif
