#include "AvatarWorld.h"

#include "Character/bCharacterLoader.h"
#include "Visualizer/gOSGShape.h"
#include <osg/BlendFunc>
#include <osg/Geode>

//#if MG==1
//#include "MocapProcessor/mgPoseTransfer.h"
//#else
//#include "PoseTransfer.h"
//#endif

extern gVec3 MW_GRAVITY_VECTOR;
AvatarWorld* AvatarWorld::pInstance = NULL;

#define SAFE_RELEASE(pPtr) { if(pPtr) {delete pPtr; pPtr = NULL;} }

AvatarWorld::AvatarWorld()
	:simulationTimeStep(1./(60.*10.)),
	displayTimeStep(simulationTimeStep),
	avatar(NULL),
	avatarSim(NULL),
	refAvatar(NULL),
	refAvatarSim(NULL),
	m_groundBody(NULL),
	m_broadphase(NULL),
	m_dispatcher(NULL),
	m_solver(NULL),
	m_collisionConfiguration(NULL),
	m_dynamicsWorld(NULL)
{
	scene = new osg::Group;
	scene_debug = new osg::Group;
	scene->addChild(scene_debug);

	//visSys.setDebugMode(gBDVisSystem::ALL);
	//visSys.setRenderMode(gBDVisSystem::WIREFRAME);

	initWorld();
}

AvatarWorld::~AvatarWorld()
{
	SAFE_RELEASE(avatar);
	SAFE_RELEASE(avatarSim);

	// dynamics
	SAFE_RELEASE(m_solver);
	SAFE_RELEASE(m_collisionConfiguration);
	SAFE_RELEASE(m_dispatcher);
	SAFE_RELEASE(m_broadphase);
	SAFE_RELEASE(m_dynamicsWorld);
}

void AvatarWorld::initWorld()
{
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);
	m_solver = new btSequentialImpulseConstraintSolver;
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = 0.001f;
	m_dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = 0.01f;
	m_dynamicsWorld->getDispatchInfo().m_useContinuous=true;

	m_dynamicsWorld->getSolverInfo().m_splitImpulse=true;
	m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
	m_dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_USE_2_FRICTION_DIRECTIONS | SOLVER_USE_WARMSTARTING | SOLVER_CACHE_FRIENDLY;
	m_dynamicsWorld->setGravity(btVector3(MW_GRAVITY_VECTOR.x(), MW_GRAVITY_VECTOR.y(), MW_GRAVITY_VECTOR.z()));
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(20.),btScalar(10.),btScalar(20.)));
		//m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		//groundTransform.setRotation(btQuaternion(0, -10*gDTR, 0 ));
		//groundTransform.setOrigin(btVector3(0.0,-10.0,0));
		groundTransform.setOrigin(btVector3(0.0,-10.0,0));
		//#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
		btCollisionObject* fixedGround = new btCollisionObject();
		fixedGround->setCollisionShape(groundShape);
		fixedGround->setWorldTransform(groundTransform);
		fixedGround->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
		fixedGround->setFriction(0.9);
		fixedGround->setRestitution(0.1);
		m_dynamicsWorld->addCollisionObject(fixedGround);
#else
		//localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
		m_groundBody = new btRigidBody(0.0,0,groundShape);	
		m_groundBody->setWorldTransform(groundTransform);
		m_groundBody->setContactProcessingThreshold(BT_LARGE_FLOAT);
		m_groundBody->setFriction(0.9);
		m_dynamicsWorld->addRigidBody(m_groundBody);

		//visSys->addVisBody(groundBody);
#endif //CREATE_GROUND_COLLISION_OBJECT

	}
}

void AvatarWorld::loadAvatarModelFromFile(const char* filename, const double scale)
{
	SAFE_RELEASE(avatar);
	SAFE_RELEASE(avatarSim);

	avatar = new bCharacter();
	avatarSim = new bCharacterSim(avatar);

	bCharacterLoader loader;

	if( !loader.loadModel(filename, avatar, scale) )	printf("fail to load!\n");
	avatar->postLoading();

	//double lowerBodyLength = 
	//	fabs(m_avatar->baseLink()->pos().y() - m_avatar->getLFootLink()->pos().y()) //root to ankle
	//	+ fabs( (m_avatar->getLFootLink()->frame().multVec3(m_avatar->getLFootGeom().sole())).y() ); //ankle to ground
	//m_avatar->setBasePosition(gVec3(0,lowerBodyLength + 10.0 ,0)); //set initial position 10.0 centimeter off ground
	//avatar->updateKinematicsUptoPos();

	//create avatarSim
	avatar->setupCharacterSim(avatarSim,m_dynamicsWorld, btVector3(0,0,0));
	avatarSim->postLoading();
	
	// kinematic 
	avatarSim->setBtBodiesDynamicOrKinematic(btCollisionObject::CF_KINEMATIC_OBJECT);

	visSys.setDebugMode(gBDVisSystem::ALL);
	
	double shapeWidth = gOSGShape::_width;
	osg::Vec4 color = gOSGShape::color;
	color.a() = 0.4;
	gOSGShape::_width = 0.5;
	gOSGShape::setColor( color );
	visSys.setCharacter(avatarSim, "avatar");
	avatarGroup = visSys.getOSGGroup("avatar");

	gOSGShape::_width = shapeWidth;

	//visSys.setRenderMode(gBDVisSystem::POLYGON, "avatar");
	visSys.setRenderMode(gBDVisSystem::WIREFRAME, "avatar");

	osg::ref_ptr<osg::Group> group = visSys.getOSGGroup("avatar");
	group->getStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
	group->getStateSet()->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );

	osg::ref_ptr<osg::BlendFunc> bf = new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA );
	group->getStateSet()->setAttributeAndModes(bf);

	scene->addChild(avatarGroup);
}

void AvatarWorld::loadReferenceAvatarModel(const char* filename, const double scale)
{
	SAFE_RELEASE(refAvatar);
	SAFE_RELEASE(refAvatarSim);

	refAvatar = new bCharacter();
	refAvatarSim = new bCharacterSim(refAvatar);

	bCharacterLoader loader;

	if( !loader.loadModel(filename, refAvatar, scale) )	printf("fail to load!\n");
	refAvatar->postLoading();

	//double lowerBodyLength = 
	//	fabs(m_avatar->baseLink()->pos().y() - m_avatar->getLFootLink()->pos().y()) //root to ankle
	//	+ fabs( (m_avatar->getLFootLink()->frame().multVec3(m_avatar->getLFootGeom().sole())).y() ); //ankle to ground
	//m_avatar->setBasePosition(gVec3(0,lowerBodyLength + 10.0 ,0)); //set initial position 10.0 centimeter off ground
	//avatar->updateKinematicsUptoPos();

	//create avatarSim
	refAvatar->setupCharacterSim(refAvatarSim,m_dynamicsWorld, btVector3(0,0,0));
	refAvatarSim->postLoading();
	
	// kinematic 
	refAvatarSim->setBtBodiesDynamicOrKinematic(btCollisionObject::CF_KINEMATIC_OBJECT);


	double shapeWidth = gOSGShape::_width;
	gOSGShape::_width = 0.5;

	visSys.setCharacter(refAvatarSim, "refAvatar");

	gOSGShape::_width = shapeWidth;

	refAvatarGroup = visSys.getOSGGroup("refAvatar");
	visSys.setRenderMode(gBDVisSystem::WIREFRAME, "refAvatar");
	scene->addChild(refAvatarGroup);
}

void AvatarWorld::showAvatar()
{
	if( scene->getChildIndex(avatarGroup) == scene->getNumChildren() )
		scene->addChild(avatarGroup);

	//if( scene->getChildIndex(refAvatarGroup) == scene->getNumChildren() )
	//	scene->addChild(refAvatarGroup);
}

void AvatarWorld::hideAvatar()
{
	scene->removeChild(avatarGroup);
	//scene->removeChild(refAvatarGroup);
}

void AvatarWorld::showRefAvatar()
{
	if( scene->getChildIndex(refAvatarGroup) == scene->getNumChildren() )
		scene->addChild(refAvatarGroup);
}

void AvatarWorld::hideRefAvatar()
{
	scene->removeChild(refAvatarGroup);
}

void AvatarWorld::removeAvatar() {
	SAFE_RELEASE(avatar);
	SAFE_RELEASE(avatarSim);

	scene->removeChildren(0, scene->getNumChildren());
};
