#ifndef AVATAR_WORLD_H
#define AVATAR_WORLD_H

#include "Character/bCharacter.h"
#include "Visualizer/gBdOsgSystem.h"

#include <osg/Group>

#define MG 0

#if MG == 1
class mgPoseTransfer;
#else
class PoseTransfer;
#endif

class AvatarWorld
{
public:
	struct IK_NODE
	{
		// link id
		int id;
		// patch id
		int pid;
		// target position
		gVec3 trn;
		// target rotation
		gVec3 rot;
		//
		bool useRot;
		// weight
		double w;
		IK_NODE()
		{
			id = -1;
			trn.setZero();
			rot.setZero();
			useRot = false;
			w = 1.0;
		}

		gVec3 localP;
		btVector3 normal;
		double halfSize;
	};

	struct IK_PARAM
	{
		std::vector<IK_NODE> targets;
		arma::mat* refPose;
	};
	AvatarWorld();
private:
	

public:
	static AvatarWorld* getInstance()
	{
		if (!pInstance)
			pInstance = new AvatarWorld;
		return pInstance;
	}
	virtual ~AvatarWorld();

	// init BD
	void initWorld();
	void loadAvatarModelFromFile(const char* filename, const double scale = 1.);
	void loadReferenceAvatarModel(const char* filename, const double scale = 1.);

	void doStep();

	void showAvatar();
	void hideAvatar();
	void showRefAvatar();
	void hideRefAvatar();

	void removeAvatar();

	osg::ref_ptr<osg::Group> getSceneData() const { return scene; }

	osg::ref_ptr<osg::Group> getSceneDebugData() const { return scene_debug; }
	void removeSceneDebugData() { scene_debug->removeChildren(0, scene_debug->getNumChildren()); }

	//static void setAvatarScale(const double scale) { avatarScale = scale; }

public:

//protected:
	
	bCharacter*			avatar;
	bCharacterSim*		avatarSim;
		
	bCharacter*			refAvatar;
	bCharacterSim*		refAvatarSim;

	btRigidBody* m_groundBody;

	// dynamics
	btBroadphaseInterface				*m_broadphase;
	btCollisionDispatcher				*m_dispatcher;
	btConstraintSolver					*m_solver;
	btDefaultCollisionConfiguration		*m_collisionConfiguration;
	btDynamicsWorld						*m_dynamicsWorld;

	// OSG
	osg::ref_ptr<osg::Group> scene;
	osg::ref_ptr<osg::Group> scene_debug;
	osg::ref_ptr<osg::Group> avatarGroup;
	osg::ref_ptr<osg::Group> refAvatarGroup;
	gBDOSGSystem visSys;
	
	const float simulationTimeStep;// = 1./(60.*10.);
	const float displayTimeStep;// = simulationTimeStep;

	//static double avatarScale;
	//static double refAvatarScale;

#if MG == 1
	mgPoseTransfer *poseTrans;
#else
	PoseTransfer *poseTrans;
#endif

private:
	static AvatarWorld* pInstance;
};

#endif
