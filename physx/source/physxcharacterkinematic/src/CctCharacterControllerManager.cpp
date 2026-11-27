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

#include "CctCharacterControllerManager.h"
#include "CctBoxController.h"
#include "CctCapsuleController.h"
#include "CctObstacleContext.h"
#include "geometry/PxGeometryHelpers.h"
#include "geometry/PxGeometryQuery.h"
#include "PsMathUtils.h"
#include "PxRigidDynamic.h"
#include "PxScene.h"
#include "PxPhysics.h"
#include "PsFoundation.h"
#include "CmRadixSortBuffered.h"

using namespace physx;
using namespace Cct;

static const PxF32 gMaxOverlapRecover = 4.0f;	// PT: TODO: expose this

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CharacterControllerManager::CharacterControllerManager(PxScene& scene, bool lockingEnabled) :
	mScene									(scene),
	mRenderBuffer							(NULL),
	mDebugRenderingFlags					(0),
	mMaxEdgeLength							(1.0f),
	mTessellation							(false),
	mOverlapRecovery						(true),
	mPreciseSweeps							(true),
	mPreventVerticalSlidingAgainstCeiling	(false),
	mLockingEnabled							(lockingEnabled)
{
	// PT: register ourself as a deletion listener, to be called by the SDK whenever an object is deleted	
	PxPhysics& physics = scene.getPhysics();
	physics.registerDeletionListener(*this, PxDeletionEventFlag::eUSER_RELEASE);
}

CharacterControllerManager::~CharacterControllerManager()
{
	PX_DELETE_AND_RESET(mRenderBuffer);
}

void CharacterControllerManager::release() 
{
	// PT: TODO: use non virtual calls & move to dtor
	while(getNbControllers()!= 0)
		releaseController(*getController(0));

	while(getNbObstacleContexts()!= 0)
		mObstacleContexts[0]->release();

	PxPhysics& physics = mScene.getPhysics();
	physics.unregisterDeletionListener(*this);

	delete this;

	Ps::Foundation::decRefCount();
}

PxScene& CharacterControllerManager::getScene() const
{
	return mScene;
}

PxRenderBuffer& CharacterControllerManager::getRenderBuffer()
{
	if(!mRenderBuffer)
		mRenderBuffer = PX_NEW(Cm::RenderBuffer); 

	return *mRenderBuffer;
}

void CharacterControllerManager::setDebugRenderingFlags(PxControllerDebugRenderFlags flags)
{
	mDebugRenderingFlags = flags;

	if(!flags)
	{
		PX_DELETE_AND_RESET(mRenderBuffer);
	}
}

PxU32 CharacterControllerManager::getNbControllers() const
{
	return mControllers.size();
}

Controller** CharacterControllerManager::getControllers() 
{
	return mControllers.begin();
}

PxController* CharacterControllerManager::getController(PxU32 index) 
{
	if(index>=mControllers.size())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PxControllerManager::getController(): out-of-range index");
		return NULL;
	}

	PX_ASSERT(mControllers[index]);
	return mControllers[index]->getPxController();
}

PxController* CharacterControllerManager::createController(const PxControllerDesc& desc)
{
	if(!desc.isValid())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PxControllerManager::createController(): desc.isValid() fails.");
		return NULL;
	}

	Controller* newController = NULL;

	PxController* N = NULL;
	if(desc.getType()==PxControllerShapeType::eBOX)
	{
		BoxController* boxController = PX_NEW(BoxController)(desc, mScene.getPhysics(), &mScene);
		newController = boxController;
		N = boxController;
	}
	else if(desc.getType()==PxControllerShapeType::eCAPSULE)
	{
		CapsuleController* capsuleController = PX_NEW(CapsuleController)(desc, mScene.getPhysics(), &mScene);
		newController = capsuleController;
		N = capsuleController;
	}
	else PX_ALWAYS_ASSERT_MESSAGE( "INTERNAL ERROR - invalid CCT type, should have been caught by isValid().");

	if(newController)
	{
		mControllers.pushBack(newController);		
		newController->setCctManager(this);

		PxShape* shape = NULL;
		PxU32 nb = N->getActor()->getShapes(&shape, 1);
		PX_ASSERT(nb==1);
		PX_UNUSED(nb);
		mCCTShapes.insert(shape);
	}

	return N;
}

void CharacterControllerManager::releaseController(PxController& controller)
{
	for(PxU32 i = 0; i<mControllers.size(); i++)
	{
		if(mControllers[i]->getPxController() == &controller)
		{
			mControllers.replaceWithLast(i);
			break;
		}
	}

	PxShape* shape = NULL;
	PxU32 nb = controller.getActor()->getShapes(&shape, 1);
	PX_ASSERT(nb==1);
	PX_UNUSED(nb);
	mCCTShapes.erase(shape);

	if(controller.getType() == PxControllerShapeType::eCAPSULE) 
	{
		CapsuleController* cc = static_cast<CapsuleController*>(&controller);
		PX_DELETE(cc);
	} 
	else if(controller.getType() == PxControllerShapeType::eBOX) 
	{
		BoxController* bc = static_cast<BoxController*>(&controller);
		PX_DELETE(bc);
	} 
	else PX_ASSERT(0);
}

void CharacterControllerManager::purgeControllers()
{
	while(mControllers.size())
		releaseController(*mControllers[0]->getPxController());
}

void CharacterControllerManager::onRelease(const PxBase* observed, void* , PxDeletionEventFlag::Enum deletionEvent)
{
	PX_ASSERT(deletionEvent == PxDeletionEventFlag::eUSER_RELEASE);  // the only type we registered for
	PX_UNUSED(deletionEvent);

	if(!(observed->getConcreteType()==PxConcreteType:: eRIGID_DYNAMIC || observed->getConcreteType()==PxConcreteType:: eRIGID_STATIC ||
		observed->getConcreteType()==PxConcreteType::eSHAPE))
		return;

	// check if object was registered
	if(mLockingEnabled)
		mWriteLock.lock();

	const ObservedRefCountMap::Entry* releaseEntry = mObservedRefCountMap.find(observed);
	if(mLockingEnabled)
		mWriteLock.unlock();

	if(releaseEntry)
	{
		for(PxU32 i = 0; i < mControllers.size(); i++)
		{
			Controller* controller = mControllers[i];
			if(mLockingEnabled)
				controller->mWriteLock.lock();

			controller->onRelease(*observed);

			if(mLockingEnabled)
				controller->mWriteLock.unlock();
		}
	}
}

void CharacterControllerManager::registerObservedObject(const PxBase* obj)
{	
	if(mLockingEnabled)
		mWriteLock.lock();

	mObservedRefCountMap[obj].refCount++;	

	if(mLockingEnabled)
		mWriteLock.unlock();
}

void CharacterControllerManager::unregisterObservedObject(const PxBase* obj)
{
	if(mLockingEnabled)
		mWriteLock.lock();

	ObservedRefCounter& refCounter = mObservedRefCountMap[obj];
	PX_ASSERT(refCounter.refCount);
	refCounter.refCount--;
	if(!refCounter.refCount)
		mObservedRefCountMap.erase(obj);

	if(mLockingEnabled)
		mWriteLock.unlock();
}

PxU32 CharacterControllerManager::getNbObstacleContexts() const
{
	return mObstacleContexts.size();
}

PxObstacleContext* CharacterControllerManager::getObstacleContext(PxU32 index)
{
	if(index>=mObstacleContexts.size())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PxControllerManager::getObstacleContext(): out-of-range index");
		return NULL;
	}

	PX_ASSERT(mObstacleContexts[index]);
	return mObstacleContexts[index];
}

PxObstacleContext* CharacterControllerManager::createObstacleContext()
{
	ObstacleContext* oc = PX_NEW(ObstacleContext)(*this);

	mObstacleContexts.pushBack(oc);

	return oc;
}

void CharacterControllerManager::releaseObstacleContext(ObstacleContext& oc)
{
	PX_ASSERT(mObstacleContexts.find(&oc) != mObstacleContexts.end());
	mObstacleContexts.findAndReplaceWithLast(&oc);

	PX_DELETE(&oc);
}

void CharacterControllerManager::onObstacleRemoved(ObstacleHandle index) const
{
	for(PxU32 i = 0; i<mControllers.size(); i++)
	{
		mControllers[i]->mCctModule.onObstacleRemoved(index);
	}
}

void CharacterControllerManager::onObstacleUpdated(ObstacleHandle index, const PxObstacleContext* context) const
{
	for(PxU32 i = 0; i<mControllers.size(); i++)
	{
		mControllers[i]->mCctModule.onObstacleUpdated(index,context, toVec3(mControllers[i]->mPosition), -mControllers[i]->mUserParams.mUpDirection, mControllers[i]->getHalfHeightInternal());
	}
}

void CharacterControllerManager::onObstacleAdded(ObstacleHandle index, const PxObstacleContext* context) const
{
	for(PxU32 i = 0; i<mControllers.size(); i++)
	{
		mControllers[i]->mCctModule.onObstacleAdded(index,context, toVec3(mControllers[i]->mPosition), -mControllers[i]->mUserParams.mUpDirection, mControllers[i]->getHalfHeightInternal());
	}
}

// PT: TODO: move to array class?
template <class T> 
void resetOrClear(T& a)
{
	const PxU32 c = a.capacity();
	if(!c)
		return;
	const PxU32 s = a.size();
	if(s>c/2)
		a.clear();
	else
		a.reset();
}

void CharacterControllerManager::resetObstaclesBuffers()
{
	resetOrClear(mBoxUserData);
	resetOrClear(mBoxes);
	resetOrClear(mCapsuleUserData);
	resetOrClear(mCapsules);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CharacterControllerManager::setTessellation(bool flag, float maxEdgeLength)
{
	mTessellation = flag;
	mMaxEdgeLength = maxEdgeLength;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CharacterControllerManager::setOverlapRecoveryModule(bool flag)
{
	mOverlapRecovery = flag;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CharacterControllerManager::setPreciseSweeps(bool flag)
{
	mPreciseSweeps = flag;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CharacterControllerManager::setPreventVerticalSlidingAgainstCeiling(bool flag)
{
	mPreventVerticalSlidingAgainstCeiling = flag;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CharacterControllerManager::shiftOrigin(const PxVec3& shift)
{
	for(PxU32 i=0; i < mControllers.size(); i++)
	{
		mControllers[i]->onOriginShift(shift);
	}

	for(PxU32 i=0; i < mObstacleContexts.size(); i++)
	{
		mObstacleContexts[i]->onOriginShift(shift);
	}

	if(mRenderBuffer)
		mRenderBuffer->shift(-shift);

	// assumption is that these are just used for temporary stuff
	PX_ASSERT(!mBoxes.size());
	PX_ASSERT(!mCapsules.size());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static PxGeometryHolder controllerGeometry(const Controller& controller)
{
	if(controller.mType == PxControllerShapeType::eBOX)
	{
		const BoxController& box = static_cast<const BoxController&>(controller);
		return PxGeometryHolder(PxBoxGeometry(box.getHalfHeight(), box.getHalfSideExtent(), box.getHalfForwardExtent()));
	}

	const CapsuleController& capsule = static_cast<const CapsuleController&>(controller);
	return PxGeometryHolder(PxCapsuleGeometry(capsule.getRadius(), capsule.getHeight()*0.5f));
}

static PxVec3 planarDirection(const PxVec3& direction, const PxVec3& centerDifference, const PxVec3& up)
{
	const PxVec3 tangent = up.cross(direction).cross(up);
	if(!Ps::isAlmostZero(tangent))
		return tangent.getNormalized();

	// A near-vertical penetration normal cannot give a reliable direction in the movement plane.
	return up.cross(centerDifference).cross(up).getNormalized();
}

static void resolveControllerOverlap(Controller& first, Controller& second, PxF32 elapsedTime)
{
	const PxGeometryHolder firstGeometry = controllerGeometry(first);
	const PxGeometryHolder secondGeometry = controllerGeometry(second);
	// Subtract the extended positions before converting to Float32.
	const PxTransform firstPose(PxVec3(0.0f), first.mUserParams.mQuatFromUp);
	const PxTransform secondPose(second.mPosition - first.mPosition, second.mUserParams.mQuatFromUp);
	PxVec3 direction;
	PxF32 depth;
	if(PxGeometryQuery::computePenetration(direction, depth, firstGeometry.any(), firstPose, secondGeometry.any(), secondPose))
	{
		const PxF32 distance = PxMin(depth, gMaxOverlapRecover*elapsedTime)*0.5f;
		const PxVec3 centerDifference = -secondPose.p;
		first.mOverlapRecover += planarDirection(direction, centerDifference, first.mUserParams.mUpDirection)*distance;
		second.mOverlapRecover -= planarDirection(direction, centerDifference, second.mUserParams.mUpDirection)*distance;
	}
}

template <typename Callback>
static void forEachOverlappingBounds(const PxBounds3* bounds, PxU32 nb, const Callback& callback)
{
	float* minima = reinterpret_cast<float*>(PX_ALLOC_TEMP(sizeof(float)*nb, "forEachOverlappingBounds"));
	for(PxU32 i=0; i<nb; i++)
		minima[i] = bounds[i].minimum.x;

	Cm::RadixSortBuffered sorter;
	const PxU32* sorted = sorter.Sort(minima, nb).GetRanks();
	for(PxU32 i=0; i<nb; i++)
	{
		const PxU32 first = sorted[i];
		for(PxU32 j=i+1; j<nb && minima[sorted[j]]<=bounds[first].maximum.x; j++)
		{
			const PxU32 second = sorted[j];
			if(bounds[first].intersects(bounds[second]))
				callback(first, second);
		}
	}

	PX_FREE(minima);
}

void CharacterControllerManager::computeInteractions(PxF32 elapsedTime, PxControllerFilterCallback* cctFilterCb)
{
	const PxU32 nbControllers = mControllers.size();
	if(nbControllers < 2 || elapsedTime == 0.0f)
	{
		for(PxU32 i=0; i<nbControllers; i++)
			mControllers[i]->mOverlapRecover = PxVec3(0.0f);
		return;
	}

	PxBounds3* boxes = reinterpret_cast<PxBounds3*>(PX_ALLOC_TEMP(sizeof(PxBounds3)*nbControllers, "CharacterControllerManager::computeInteractions"));
	for(PxU32 i=0; i<nbControllers; i++)
	{
		Controller& current = *mControllers[i];
		current.mOverlapRecover = PxVec3(0.0f);

		PxExtendedBounds3 bounds;
		current.getWorldBox(bounds);
		boxes[i] = PxBounds3(toVec3(bounds.minimum), toVec3(bounds.maximum));
	}

	forEachOverlappingBounds(boxes, nbControllers, [this, cctFilterCb, elapsedTime](PxU32 index0, PxU32 index1)
	{
		Controller& first = *mControllers[index0];
		Controller& second = *mControllers[index1];
		if(!cctFilterCb || cctFilterCb->filter(*first.getPxController(), *second.getPxController()))
			resolveControllerOverlap(first, second, elapsedTime);
	});

	PX_FREE(boxes);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public factory methods

PX_C_EXPORT PxControllerManager* PX_CALL_CONV PxCreateControllerManager(PxScene& scene, bool lockingEnabled)
{
	Ps::Foundation::incRefCount();
	return PX_NEW(CharacterControllerManager)(scene, lockingEnabled);
}
