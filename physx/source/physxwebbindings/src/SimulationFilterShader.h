//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PX_PHYSICS_EXTENSIONS_SIMULATIONFILTERSHADER_H
#define PX_PHYSICS_EXTENSIONS_SIMULATIONFILTERSHADER_H
/** \addtogroup extensions
  @{
*/

#include "PxFiltering.h"
#include "PxPhysXConfig.h"

namespace physx {

class PxActor;
class PxScene;
class PxShape;

PxFilterFlags simulationFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
                                     PxFilterObjectAttributes attributes1, PxFilterData filterData1,
                                     PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize);

PxU16 getGroup(const PxActor& actor);
void setGroup(PxActor& actor, PxU32 collisionGroup);
bool setShapeGroup(PxShape& shape, PxU32 collisionGroup);
void setShapeLayerOverrides(PxShape& shape, PxU32 includeLayers, PxU32 excludeLayers);
bool getGroupCollisionFlag(const PxScene& scene, PxU32 group1, PxU32 group2);
void setGroupCollisionFlag(PxScene& scene, PxU32 group1, PxU32 group2, bool enable);
bool shouldCollide(const PxScene& scene, const PxFilterData& first, const PxFilterData& second);
void setPairIgnored(PxScene& scene, PxU32 first, PxU32 second, bool ignored);
void removeIgnoredPairs(PxScene& scene, PxU32 shapeId);

PxU32 getCollisionFilterRevision();
void bumpCollisionFilterRevision();

} // namespace physx
#endif
