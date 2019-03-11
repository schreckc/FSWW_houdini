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
 * The Sources SOP
 */

#include "SOP_Sources.hpp"
#include "GEO_source.hpp"

#include <GU/GU_Detail.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <OP/OP_AutoLockInputs.h>
#include <CH/CH_LocalVariable.h>
#include <PRM/PRM_Include.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Vector3.h>
#include <SYS/SYS_Types.h>
#include <limits.h>
#include <stddef.h>

using namespace HDK_Sample;

//
// Help is stored in a "wiki" style text file.  The text file should be copied
// to $HOUDINI_PATH/help/nodes/sop/tetra.txt
//
// See the sample_install.sh file for an example.
//


///
/// newSopOperator is the hook that Houdini grabs from this dll
/// and invokes to register the SOP.  In this case we add ourselves
/// to the specified operator table.
///
void
newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(new OP_Operator(
        "fs_sources",                // Internal name
        "FS Sources",                    // UI name
        SOP_Sources::myConstructor,   // How to build the SOP
        SOP_Sources::myTemplateList,  // My parameters
        1,                          // Min # of sources
        1,                          // Max # of sources
        0,                          // Local variables
        OP_FLAG_GENERATOR));        // Flag it as generator
}

static PRM_Name PRMwavelengthName("wavelength", "Wavelength");

PRM_Template
SOP_Sources::myTemplateList[] = {
    PRM_Template(PRM_FLT, 1, &PRMwavelengthName, PRMoneDefaults),
    PRM_Template()
};


OP_Node *
SOP_Sources::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_Sources(net, name, op);
}

SOP_Sources::SOP_Sources(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
    // This indicates that this SOP manually manages its data IDs,
    // so that Houdini can identify what attributes may have changed,
    // e.g. to reduce work for the viewport, or other SOPs that
    // check whether data IDs have changed.
    // By default, (i.e. if this line weren't here), all data IDs
    // would be bumped after the SOP cook, to indicate that
    // everything might have changed.
    // If some data IDs don't get bumped properly, the viewport
    // may not update, or SOPs that check data IDs
    // may not cook correctly, so be *very* careful!
    mySopFlags.setManagesDataIDs(true);
}

SOP_Sources::~SOP_Sources() {}

OP_ERROR
SOP_Sources::cookMySop(OP_Context &context)
{
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();
    fpreal now = context.getTime();

    // Since we don't have inputs, we don't need to lock them.

    float wl = WAVELENGTH(now);
    gdp->clearAndDestroy();
    const GU_Detail *fs = inputGeo(0);
    GA_Offset ptoff;
    GA_FOR_ALL_PTOFF(fs, ptoff) {
      const GA_Offset startptoff = gdp->appendPointBlock(1);
      UT_Vector3 pos = fs->getPos3(ptoff);
      gdp->setPos3(startptoff, pos);
      GEO_Source *prim_src = GEO_Source::build(gdp, false);
           // GA_Offset soff = prim_src->getPointOffset(0);

    }
    gdp->getP()->bumpDataId();
    select(GA_GROUP_PRIMITIVE);

    
    // // Try to reuse the existing tetrahedron, if the detail
    // // hasn't been cleared.
    // GEO_PrimSources *tet = NULL;
    // if (gdp->getNumPrimitives() == 1)
    // {
    //     GA_Primitive *prim = gdp->getPrimitiveByIndex(0);

    //     // This type check is not strictly necessary, since
    //     // this SOP only ever makes a GEO_PrimSources, but it's
    //     // here just in case, and as an example.
    //     if (prim->getTypeId() == GEO_PrimSources::theTypeId())
    //         tet = (GEO_PrimSources *)prim;
    // } g

    // if (tet == NULL)
    // {
    //     // In addition to destroying everything except the empty P
    //     // and topology attributes, this bumps the data IDs for
    //     // those remaining attributes, as well as the primitive list
    //     // data ID.
    //     gdp->clearAndDestroy();

    //     // Build a tetrahedron
    //     tet = GEO_PrimSources::build(gdp, true);
    // }

    // for (int i = 0; i < 4; i++)
    // {
    //     UT_Vector3 pos(i == 1, i == 2, i == 3);
    //     pos *= rad;
    //     pos += translate;
    //     GA_Offset ptoff = tet->getPointOffset(i);
    //     gdp->setPos3(ptoff, pos);
    // }

    // // We've modified P, so we need to bump the data ID,
    // // (though in the case where we did clearAndDestroy(), it's
    // // redundant, because that already bumped it).
    // gdp->getP()->bumpDataId();

    // // Set the node selection for this primitive. This will highlight all
    // // the tets generated by the node, but only if the highlight flag for this 
    // // node is on and the node is selected.
    // select(GA_GROUP_PRIMITIVE);

    return error();
}
