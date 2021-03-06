
#include <Windows.h>
#include <iostream>
#include <algorithm>

#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/LineWidth>
#include <osgDB/ReadFile>

#include "Visualizer/gOSGShape.h"
#include "Loader/MotionLoader.h"
#include "Loader/mgSkeletonToBCharacter.h"
#include "AvatarWorld.h"

#include "MocapProcessor/mgMBSUtil.h"
#include "MocapProcessor/mgUtility.h"
#include "Visualizer/gEventHandler.h"

#include "Visualizer/gOSGSkin.h"

#include <osgAnimation/AnimationManagerBase>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/Animation>
#include <osgAnimation/Skeleton>
#include <osgAnimation/Bone>
#include <osgAnimation/UpdateBone>
#include <osgAnimation/StackedRotateAxisElement>
#include <osgAnimation/StackedMatrixElement>
#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/StackedQuaternionElement>
#include <osgAnimation/StackedScaleElement>
#include <osg/TriangleIndexFunctor>
#include <osgDB/Options>

#include "..\alglib-3.15.0.cpp.gpl\cpp\src\dataanalysis.h"
#include "igl/readOBJ.h"
#include "mgPoseTransfer.h"

#define NCPtoNSP(n) (n+2) // convert # of control point to # of spline control points, which used in BSpline
#define NCPtoNSE(n) (n-1) // convert # of control point to # of spline segment, which used in BSpline


double DEBUG_DRAW_CONSTRAINT_SIZE = 2;
gVec3 MW_GRAVITY_VECTOR(0, -9.8, 0);
gVec3 MW_GROUND_NORMAL(0, 1, 0);


// test
arma::mat refCoord;

osg::ref_ptr<osg::Group> debugGroup = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup2 = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup3 = new osg::Group;

const static std::string kTaeilPath = "D:/Taeil_Jin/Development/feature_MWTaeil/Projects/Env_Retargeting/Data/demo/taeil/";
const static std::string kTaeilPath_TaskFBX = "D:/development/DATA/";
const static std::string kTaeilPath_TaskBVH = "D:/development/DATA/demo/taeil/Environment/";
const static std::string kTaeilPath_TaskOBJ = "D:/development/DATA/furniture/";

const static std::string kSourceObjectFileName = kTaeilPath_TaskOBJ + "vitra_noarm_blonde.obj"; //"VItra_noArm_tri.obj";// "source.obj";//"source.obj";
const static std::string kSourceObjectFileName2 = kTaeilPath_TaskOBJ + "test_Desk_tri_blonde.obj";// "test_desk_tri.obj";

void initializeVisualization()
{
	osg::ref_ptr<osgDB::Options> options = new osgDB::Options;
	options->setOptionString("noRotation");
	osg::ref_ptr<osg::Node> src_obj_node = osgDB::readNodeFile(kSourceObjectFileName, options.get());
	//osg::ref_ptr<osg::Node> tar_obj_node = osgDB::readNodeFile(kTargetObjectFileName, options.get());
	src_obj_node->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	//tar_obj_node->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

	osg::Vec3 src_p = src_obj_node->getWorldMatrices().begin()->getTrans();
	std::cout << " x " << src_p.x() << " y " << src_p.y() << " z " << src_p.z() << std::endl;


	debugGroup2->addChild(src_obj_node);
	//debugGroup3->addChild(tar_obj_node);

	osg::ref_ptr<osg::Node> src_obj_node2 = osgDB::readNodeFile(kSourceObjectFileName2, options.get());
	//osg::ref_ptr<osg::Node> tar_obj_node2 = osgDB::readNodeFile(kTargetObjectFileName2, options.get());
	src_obj_node2->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	//tar_obj_node2->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

	src_p = src_obj_node2->getWorldMatrices().begin()->getTrans();
	std::cout << " x " << src_p.x() << " y " << src_p.y() << " z " << src_p.z() << std::endl;

	debugGroup2->addChild(src_obj_node2);
	//debugGroup3->addChild(tar_obj_node2);
}
//
osg::Vec3 drawBone(osg::ref_ptr<osg::Node> node, osg::ref_ptr<osg::Group> view_group)
{
	osg::ref_ptr<osgAnimation::Bone> b = dynamic_cast<osgAnimation::Bone*>(node.get());
	if (!b)
	{
		osg::ref_ptr<osg::Group> group = dynamic_cast<osg::Group*>(node.get());
		if (group)
		{
			for (int i = 0; i < group->getNumChildren(); ++i)
			{
				drawBone(group->getChild(i), view_group);
			}
		}
		return osg::Vec3(0, 0, 0);
	}

	osg::Matrix wMat = b->getWorldMatrices()[0];
	//osg::Matrix wMat = b->getMatrixInSkeletonSpace();
	//osg::Vec3 pos = b->getMatrix().getTrans();
	osg::Vec3 pos = wMat.getTrans();

	gOSGShape::setColor(osg::Vec4(1, 0, 0, 1));
	view_group->addChild(gOSGShape::createPoint(pos, 3.));

	for (int i = 0; i < b->getNumChildren(); ++i)
	{
		osg::Vec3 c_pos = drawBone(b->getChild(i), view_group);

		gOSGShape::setColor(osg::Vec4(0, 0, 0, 1));
		view_group->addChild(gOSGShape::createLineShape(pos, c_pos, 1.));

	}

	return pos;
}

osg::Vec3 drawBone(mgBone* bone, double* data, osg::ref_ptr<osg::Group> view_group)
{
	gXMat wMat;
	bone->skeleton->getWMatrixAt(bone->id, data, wMat);

	gVec3 pos_g = wMat.trn();
	osg::Vec3 pos(pos_g.x(), pos_g.y(), pos_g.z());

	gOSGShape::setColor(osg::Vec4(1, 0, 0, 1));
	view_group->addChild(gOSGShape::createPoint(pos, 3.));

	for (int i = 0; i < bone->children.size(); ++i)
	{
		osg::Vec3 c_pos = drawBone(bone->children[i], data, view_group);

		gOSGShape::setColor(osg::Vec4(0, 0, 0, 1));
		view_group->addChild(gOSGShape::createLineShape(pos, c_pos, 1.));
	}

	return pos;
}

#include "Loader/FBXLoader.h"

osg::ref_ptr<osg::Node> getNodeFromName(osg::ref_ptr<osg::Node> node, std::string& name)
{
	osg::Group* group = node->asGroup();
	if (!group) return NULL;

	if (node->getName() == name)
		return node;

	for (int i = 0; i < group->getNumChildren(); ++i)
	{
		osg::ref_ptr<osg::Node> n = getNodeFromName(group->getChild(i), name);
		if (n) return n;
	}
	return NULL;
}

gXMat getNodeTransform(osg::ref_ptr<osg::Node> node, FBXLoader* fbx)
{
	return gXMat();
}

//get gVec3 to OSGVec
osg::Vec3 gVec3_2_OsgVec(gVec3 gVec) {
	osg::Vec3 p(gVec.x(), gVec.y(), gVec.z());
	return p;
};
//get gVec3 to OSGVec
gVec3 OsgVec_2_gVec3(osg::Vec3 oVec) {
	gVec3 p(oVec.x(), oVec.y(), oVec.z());
	return p;
};
//print gVec3
void printgVec3(char* txt, gVec3 gVec) {
	std::cout << txt << " " << " x " << gVec.x() << " y " << gVec.y() << " z " << gVec.z() << std::endl;
}

//event handler
int iter = 0;
double simulationTime = 0;

bool bool_go = false;
bool bool_go_back = false;
inline bool toggleUpdateMotion() { bool_go = !bool_go; return bool_go; }
inline bool toggleUpdateMotionBack() { bool_go_back = !bool_go_back; return bool_go_back; }

// event handler
void keyEventToggleAnimation(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (toggleUpdateMotion())
		std::cout << "Activate animation mode.\n";
	else
		std::cout << "Deactivate animation mode.\n";
}
void keyEventToggleAnimationBack(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (toggleUpdateMotionBack())
		std::cout << "Activate animation mode.\n";
	else
		std::cout << "Deactivate animation mode.\n";
}
void keyEventOneFrameGo(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	simulationTime += 1. / 30.;
	iter++;
}
void keyEventOneFrameBack(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	simulationTime -= 1. / 30.;
	iter--;
}

void drawMesh(Eigen::MatrixXd V) {
	for (int r = 0; r < V.rows(); r++) {
		debugGroup->addChild(gOSGShape::createPoint(osg::Vec3(V(r, 0), V(r, 1), V(r, 2)), 5.0));
	}
}
//----------------------------------------------------you may don't need to check upper coding lines (core and individual's functions are mixed. )

#include "Character/bCharacterLoader.h"
#include <osg/BlendFunc>
btBroadphaseInterface				*m_broadphase;
btCollisionDispatcher				*m_dispatcher;
btConstraintSolver					*m_solver;
btDefaultCollisionConfiguration		*m_collisionConfiguration;
btDynamicsWorld						*m_dynamicsWorld;
btRigidBody* m_groundBody;

void initWorld()
{
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000, -10000, -10000);
	btVector3 worldAabbMax(10000, 10000, 10000);
	m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax);
	m_solver = new btSequentialImpulseConstraintSolver;
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	//m_dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = 0.001f;
	m_dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = 0.01f;
	m_dynamicsWorld->getDispatchInfo().m_useContinuous = true;

	m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;
	m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
	m_dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_USE_2_FRICTION_DIRECTIONS | SOLVER_USE_WARMSTARTING | SOLVER_CACHE_FRIENDLY;
	m_dynamicsWorld->setGravity(btVector3(MW_GRAVITY_VECTOR.x(), MW_GRAVITY_VECTOR.y(), MW_GRAVITY_VECTOR.z()));
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(20.), btScalar(10.), btScalar(20.)));
		//m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		//groundTransform.setRotation(btQuaternion(0, -10*gDTR, 0 ));
		//groundTransform.setOrigin(btVector3(0.0,-10.0,0));
		groundTransform.setOrigin(btVector3(0.0, -10.0, 0));
		//#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
		btCollisionObject* fixedGround = new btCollisionObject();
		fixedGround->setCollisionShape(groundShape);
		fixedGround->setWorldTransform(groundTransform);
		fixedGround->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
		fixedGround->setFriction(0.9);
		fixedGround->setRestitution(0.1);
		m_dynamicsWorld->addCollisionObject(fixedGround);
#else
		//localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
		m_groundBody = new btRigidBody(0.0, 0, groundShape);
		m_groundBody->setWorldTransform(groundTransform);
		m_groundBody->setContactProcessingThreshold(BT_LARGE_FLOAT);
		m_groundBody->setFriction(0.9);
		m_dynamicsWorld->addRigidBody(m_groundBody);

		//visSys->addVisBody(groundBody);
#endif //CREATE_GROUND_COLLISION_OBJECT

	}
}
double loadAvatarModelFromFile(bCharacter* in_avatar, bCharacterSim* in_avatarSim, gBDOSGSystem* visSys, const char* filename, const double scale)
{

	bCharacterLoader loader;
	double exist = -1;
	if (!loader.loadModel(filename, in_avatar, scale)) { printf("fail to load!\n"); exist = -1; }
	else
	{
		exist = 0;
		in_avatar->postLoading();

		//double lowerBodyLength = 
		//	fabs(m_avatar->baseLink()->pos().y() - m_avatar->getLFootLink()->pos().y()) //root to ankle
		//	+ fabs( (m_avatar->getLFootLink()->frame().multVec3(m_avatar->getLFootGeom().sole())).y() ); //ankle to ground
		//m_avatar->setBasePosition(gVec3(0,lowerBodyLength + 10.0 ,0)); //set initial position 10.0 centimeter off ground
		//avatar->updateKinematicsUptoPos();

		//create avatarSim
		in_avatar->setupCharacterSim(in_avatarSim, m_dynamicsWorld, btVector3(0, 0, 0));
		in_avatarSim->postLoading();

		// kinematic 
		in_avatarSim->setBtBodiesDynamicOrKinematic(btCollisionObject::CF_KINEMATIC_OBJECT);


		visSys->setDebugMode(gBDVisSystem::ALL);

		double shapeWidth = gOSGShape::_width;
		osg::Vec4 color = gOSGShape::color;
		color.a() = 0.4;
		gOSGShape::_width = 0.5;
		gOSGShape::setColor(color);
		visSys->setCharacter(in_avatarSim, "in_avatar");
		osg::ref_ptr<osg::Group> avatarGroup;
		avatarGroup = visSys->getOSGGroup("in_avatar");

		gOSGShape::_width = shapeWidth;

		//visSys.setRenderMode(gBDVisSystem::POLYGON, "in_avatar");
		visSys->setRenderMode(gBDVisSystem::WIREFRAME, "in_avatar");

		osg::ref_ptr<osg::Group> group = visSys->getOSGGroup("in_avatar");
		group->getStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		group->getStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

		osg::ref_ptr<osg::BlendFunc> bf = new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
		group->getStateSet()->setAttributeAndModes(bf);

	}
	return exist;
}
// poseTransfer process
// poseTrans input : bCharacter* src, bCharacter* tar
// poseTrans output : retargetted tar pose ( you should update new pose using updateKinematicsUptoPos();)
// initTranfer: setting desired position, desired direction objectives
// poseTrans->transferPoseLevMar(offset); : do pose Transfer
mgPoseTransfer* poseTrans;

void SetJntRotDirOBJ(mgPoseTransfer* poseTrans, const char* txt_id, const char* src_jnt, const char* tar_jnt) {
	poseTrans->addPoint(txt_id, *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 0, 0));
	poseTrans->addPoint("t0_x", *poseTrans->src->findLink(src_jnt), gVec3(1, 0, 0), *poseTrans->tar->findLink(src_jnt), gVec3(1, 0, 0));
	poseTrans->addPoint("t0_z", *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 1), *poseTrans->tar->findLink(src_jnt), gVec3(0, 0, 1));

	//poseTrans->addDirectionObjective(txt_id, "t0_x", 1.0);
	poseTrans->addDirectionObjective(txt_id, "t0_z", 1.0);

}
void initTransfer(bCharacter* src, bCharacter* tar) {
	poseTrans = new mgPoseTransfer(src, tar);

	// you need to manually match the src chracter joint name and corresponding tar character joint name
	//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
	SetJntRotDirOBJ(poseTrans, "t0", "Hips", "Hips");
	SetJntRotDirOBJ(poseTrans, "t1", "LowerBack", "LowerBack");
	SetJntRotDirOBJ(poseTrans, "t2", "Spine", "Spine");
	SetJntRotDirOBJ(poseTrans, "t3", "Spine1", "Spine1");
	SetJntRotDirOBJ(poseTrans, "t4", "Neck", "Neck");
	SetJntRotDirOBJ(poseTrans, "t5", "Neck1", "Neck1");
	SetJntRotDirOBJ(poseTrans, "t6", "Head", "Head");

	//poseTrans->addPoint("t1", *src->findLink("LowerBack"), gVec3(0, 0, 0), *tar->findLink("LowerBack"), gVec3(0, 0, 0));
	/*poseTrans->addPoint("t2", *src->findLink("Spine"), gVec3(0, 0, 0), *tar->findLink("Spine"), gVec3(0, 0, 0));
	poseTrans->addPoint("t3", *src->findLink("Spine1"), gVec3(0, 0, 0), *tar->findLink("Spine1"), gVec3(0, 0, 0));
	poseTrans->addPoint("t4", *src->findLink("Neck"), gVec3(0, 0, 0), *tar->findLink("Neck"), gVec3(0, 0, 0));
	poseTrans->addPoint("t5", *src->findLink("Neck1"), gVec3(0, 0, 0), *tar->findLink("Neck1"), gVec3(0, 0, 0));
	poseTrans->addPoint("t6", *src->findLink("Head"), gVec3(0, 0, 0), *tar->findLink("Head"), gVec3(0, 0, 0));
*/


//left leg chain
	SetJntRotDirOBJ(poseTrans, "la0", "LHipJoint", "LHipJoint");
	SetJntRotDirOBJ(poseTrans, "la1", "LeftUpLeg", "LeftUpLeg");
	SetJntRotDirOBJ(poseTrans, "la2", "LeftLeg", "LeftLeg");
	SetJntRotDirOBJ(poseTrans, "la3", "LeftFoot", "LeftFoot");
	SetJntRotDirOBJ(poseTrans, "la4", "LeftToeBase", "LeftToeBase");

	//poseTrans->addPoint("la0", *src->findLink("LHipJoint"), gVec3(0, 0, 0), *tar->findLink("LHipJoint"), gVec3(0, 0, 0));
	//poseTrans->addPoint("la1", *src->findLink("LeftUpLeg"), gVec3(0, 0, 0), *tar->findLink("LeftUpLeg"), gVec3(0, 0, 0));
	//poseTrans->addPoint("la2", *src->findLink("LeftLeg"), gVec3(0, 0, 0), *tar->findLink("LeftLeg"), gVec3(0, 0, 0));
	//poseTrans->addPoint("la3", *src->findLink("LeftFoot"), gVec3(0, 0, 0), *tar->findLink("LeftFoot"), gVec3(0, 0, 0));
	//poseTrans->addPoint("la4", *src->findLink("LeftToeBase"), gVec3(0, 0, 0), *tar->findLink("LeftToeBase"), gVec3(0, 0, 0));

	//right leg chain
	SetJntRotDirOBJ(poseTrans, "ra0", "RHipJoint", "RHipJoint");
	SetJntRotDirOBJ(poseTrans, "ra1", "RightUpLeg", "RightUpLeg");
	SetJntRotDirOBJ(poseTrans, "ra2", "RightLeg", "RightLeg");
	SetJntRotDirOBJ(poseTrans, "ra3", "RightFoot", "RightFoot");
	SetJntRotDirOBJ(poseTrans, "ra4", "RightToeBase", "RightToeBase");

	/*poseTrans->addPoint("ra0", *src->findLink("RHipJoint"), gVec3(0, 0, 0), *tar->findLink("RHipJoint"), gVec3(0, 0, 0));
	poseTrans->addPoint("ra1", *src->findLink("RightUpLeg"), gVec3(0, 0, 0), *tar->findLink("RightUpLeg"), gVec3(0, 0, 0));
	poseTrans->addPoint("ra2", *src->findLink("RightLeg"), gVec3(0, 0, 0), *tar->findLink("RightLeg"), gVec3(0, 0, 0));
	poseTrans->addPoint("ra3", *src->findLink("RightFoot"), gVec3(0, 0, 0), *tar->findLink("RightFoot"), gVec3(0, 0, 0));
	poseTrans->addPoint("ra4", *src->findLink("RightToeBase"), gVec3(0, 0, 0), *tar->findLink("RightToeBase"), gVec3(0, 0, 0));*/

	//left arm chain
	SetJntRotDirOBJ(poseTrans, "ll0", "LeftShoulder", "LeftShoulder");
	SetJntRotDirOBJ(poseTrans, "ll1", "LeftArm", "LeftArm");
	SetJntRotDirOBJ(poseTrans, "ll2", "LeftForeArm", "LeftForeArm");
	SetJntRotDirOBJ(poseTrans, "ll3", "LeftHand", "LeftHand");
	SetJntRotDirOBJ(poseTrans, "ll4", "LeftFingerBase", "LeftFingerBase");
	SetJntRotDirOBJ(poseTrans, "ll5", "LeftHandIndex1", "LeftHandIndex1");
	SetJntRotDirOBJ(poseTrans, "ll6", "LThumb", "LThumb");

	/*poseTrans->addPoint("ll0", *src->findLink("LeftShoulder"), gVec3(0, 0, 0), *tar->findLink("LeftShoulder"), gVec3(0, 0, 0));
	poseTrans->addPoint("ll1", *src->findLink("LeftArm"), gVec3(0, 0, 0), *tar->findLink("LeftArm"), gVec3(0, 0, 0));
	poseTrans->addPoint("ll2", *src->findLink("LeftForeArm"), gVec3(0, 0, 0), *tar->findLink("LeftForeArm"), gVec3(0, 0, 0));
	poseTrans->addPoint("ll3", *src->findLink("LeftHand"), gVec3(0, 0, 0), *tar->findLink("LeftHand"), gVec3(0, 0, 0));
	poseTrans->addPoint("ll4", *src->findLink("LeftFingerBase"), gVec3(0, 0, 0), *tar->findLink("LeftFingerBase"), gVec3(0, 0, 0));
	poseTrans->addPoint("ll5", *src->findLink("LeftHandIndex1"), gVec3(0, 0, 0), *tar->findLink("LeftHandIndex1"), gVec3(0, 0, 0));
	poseTrans->addPoint("ll6", *src->findLink("LThumb"), gVec3(0, 0, 0), *tar->findLink("LThumb"), gVec3(0, 0, 0));*/

	//right arm chain
	SetJntRotDirOBJ(poseTrans, "rl0", "RightShoulder", "RightShoulder");
	SetJntRotDirOBJ(poseTrans, "rl1", "RightArm", "RightArm");
	SetJntRotDirOBJ(poseTrans, "rl2", "RightForeArm", "RightForeArm");
	SetJntRotDirOBJ(poseTrans, "rl3", "RightHand", "RightHand");
	SetJntRotDirOBJ(poseTrans, "rl4", "RightFingerBase", "RightFingerBase");
	SetJntRotDirOBJ(poseTrans, "rl5", "RightHandIndex1", "RightHandIndex1");
	SetJntRotDirOBJ(poseTrans, "rl6", "RThumb", "RThumb");

	/*poseTrans->addPoint("rl0", *src->findLink("RightShoulder"), gVec3(0, 0, 0), *tar->findLink("RightShoulder"), gVec3(0, 0, 0));
	poseTrans->addPoint("rl1", *src->findLink("RightArm"), gVec3(0, 0, 0), *tar->findLink("RightArm"), gVec3(0, 0, 0));
	poseTrans->addPoint("rl2", *src->findLink("RightForeArm"), gVec3(0, 0, 0), *tar->findLink("RightForeArm"), gVec3(0, 0, 0));
	poseTrans->addPoint("rl3", *src->findLink("RightHand"), gVec3(0, 0, 0), *tar->findLink("RightHand"), gVec3(0, 0, 0));
	poseTrans->addPoint("rl4", *src->findLink("RightFingerBase"), gVec3(0, 0, 0), *tar->findLink("RightFingerBase"), gVec3(0, 0, 0));
	poseTrans->addPoint("rl5", *src->findLink("RightHandIndex1"), gVec3(0, 0, 0), *tar->findLink("RightHandIndex1"), gVec3(0, 0, 0));
	poseTrans->addPoint("rl6", *src->findLink("RThumb"), gVec3(0, 0, 0), *tar->findLink("RThumb"), gVec3(0, 0, 0));
*/
//poseTrans->addPoint("rl3", *src->findLink("RightToeBase"),gVec3(0,0,0), *tar->findLink("Neck"),gVec3(0,0,0));


	double weightDir = 1.;//importance of direction vector in pose transfer
						  //double weightPos= 1.01;//importance of end-effector orientation in pose transfer
	double weightPos = 1.01;//importance of end-effector orientation in pose transfer


							//direction objectives
	poseTrans->addDirectionObjective("t0", "t1", weightDir);
	poseTrans->addDirectionObjective("t1", "t2", weightDir);
	poseTrans->addDirectionObjective("t2", "t3", weightDir);
	poseTrans->addDirectionObjective("t3", "t4", weightDir);
	poseTrans->addDirectionObjective("t4", "t5", weightDir);
	poseTrans->addDirectionObjective("t5", "t6", weightDir);
	poseTrans->addDirectionObjective("t0", "t0_x", weightDir);
	poseTrans->addDirectionObjective("t0", "t0_z", weightDir);

	//left leg chain 
	poseTrans->addDirectionObjective("la0", "la1", weightDir);
	poseTrans->addDirectionObjective("la1", "la2", weightDir);
	poseTrans->addDirectionObjective("la2", "la3", weightDir);
	poseTrans->addDirectionObjective("la3", "la4", weightDir);

	//right leg chain
	poseTrans->addDirectionObjective("ra0", "ra1", weightDir);
	poseTrans->addDirectionObjective("ra1", "ra2", weightDir);
	poseTrans->addDirectionObjective("ra2", "ra3", weightDir);
	poseTrans->addDirectionObjective("ra3", "ra4", weightDir);

	//right arm chain
	poseTrans->addDirectionObjective("rl0", "rl1", weightDir);
	poseTrans->addDirectionObjective("rl1", "rl2", weightDir);
	poseTrans->addDirectionObjective("rl2", "rl3", weightDir);
	poseTrans->addDirectionObjective("rl3", "rl4", weightDir);
	poseTrans->addDirectionObjective("rl4", "rl5", weightDir);
	poseTrans->addDirectionObjective("rl5", "rl6", weightDir);

	//right arm chain
	poseTrans->addDirectionObjective("ll0", "ll1", weightDir);
	poseTrans->addDirectionObjective("ll1", "ll2", weightDir);
	poseTrans->addDirectionObjective("ll2", "ll3", weightDir);
	poseTrans->addDirectionObjective("ll3", "ll4", weightDir);
	poseTrans->addDirectionObjective("ll4", "ll5", weightDir);
	poseTrans->addDirectionObjective("ll5", "ll6", weightDir);

	//position objectives
	poseTrans->addPositionObjective("t0", weightPos * 100);
	//poseTrans->addPositionObjective("la3", weightPos);
	//poseTrans->addPositionObjective("ra3", weightPos);
	//poseTrans->addPositionObjective("rl2", weightPos);
	//poseTrans->addPositionObjective("ll2", weightPos);

}



int main(int argc, char **argv)
{
	// construct the viewer.
	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;

	osg::ref_ptr<osg::Group> scene = new osg::Group;
	osg::ref_ptr<osg::MatrixTransform> rootTrans = new osg::MatrixTransform;
	osg::ref_ptr<osg::MatrixTransform> refTrans = new osg::MatrixTransform;

	//initializeVisualization();

	scene->addChild(debugGroup);
	scene->addChild(debugGroup2);
	scene->addChild(debugGroup3);

	viewer->setSceneData(scene);


	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = 10;
	traits->y = 10;
	traits->width = 1024;
	traits->height = 768;
	traits->windowDecoration = true;
	traits->supportsResize = false;
	traits->windowName = "test";

	osg::ref_ptr<osg::GraphicsContext> graphicscontext = osg::GraphicsContext::createGraphicsContext(traits);
	graphicscontext->realize();

	viewer->getCamera()->setGraphicsContext(graphicscontext);

	osg::Camera* camera = viewer->getCamera();

	camera->setClearColor(osg::Vec4(1., 1., 1., 1.0));
	camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
	camera->setProjectionMatrixAsPerspective(30.0f, static_cast<double>(traits->width) / static_cast<double>(traits->height), 1.0f, 10000.0f);

	osg::ref_ptr<osgGA::TrackballManipulator> manipulator = new osgGA::TrackballManipulator;
	viewer->setCameraManipulator(manipulator);
	manipulator->setAutoComputeHomePosition(true);
	//manipulator->setHomePosition(osg::Vec3(-100, 100, -100), osg::Vec3(0, 0, 0), osg::Vec3(0, 1, 0), false);
	manipulator->home(0);

	int margin = 300;
	int w = margin * 2;
	int h = margin * 2;
	double size = 10;

	double startPoint_h = 0;
	double startPoint_w = 0;

	float width = 1.0f;

	gOSGShape::setColor(osg::Vec4(0, 0, 0, 1));
	for (int i = 0; i <= (h / size + 1); i++)
		scene->addChild(gOSGShape::createLineShape(osg::Vec3(startPoint_w, 0.0, i*size + startPoint_h), osg::Vec3(1, 0, 0), w + size, width));
	for (int i = 0; i <= (w / size + 1); i++)
		scene->addChild(gOSGShape::createLineShape(osg::Vec3(i*size + startPoint_w, 0.0, startPoint_h), osg::Vec3(0, 0, 1), h + size, width));
	scene->addChild(gOSGShape::createAxis(5.0, 5.0));

	//initializeVisualization();
	initWorld(); // init btworld for viewing avatar vis system
	//--------------------- initial setup is finished. (you may not need to see upper coding lines)


	// keyboard event handler
	osg::ref_ptr<gEventHandler> handler = new gEventHandler;
	handler->assignKey(osgGA::GUIEventAdapter::KEY_Y, keyEventToggleAnimation);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_H, keyEventToggleAnimationBack);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_U, keyEventOneFrameGo);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_J, keyEventOneFrameBack);
	viewer->addEventHandler(handler);

	// get src motion file
	MotionLoader loader;
	loader.loadMotionFile("124_01.bvh");
	mgData* motion = loader.getMotion();
	mgSkeleton* skeleton = loader.getSkeleton();

	// generate src bCharacter from motion, and save as txt file
	bCharacter* src = new bCharacter();
	bCharacterSim* srcSim = new bCharacterSim(src);
	gBDOSGSystem* srcVis = new gBDOSGSystem();

	const double mass = 70.;
	mgSkeletonToBCharacter::saveToBCharacter(skeleton, "Src.txt", mass);
	// load src bCharacter from txt file
	loadAvatarModelFromFile(src,srcSim,srcVis,"Src.txt",1.0);
	// load src bCharacter visualization class from bCharcter
	scene->addChild(srcVis->getOSGGroup());
	// load src motion (pose sequence) : nMotion is a total number of frame for motion data
	arma::mat refCoord(src->sizeCompactCoordArray() + 1, motion->nMotion, arma::fill::zeros);
	for (int f = 0; f < motion->nMotion; f++)
	{
		arma::vec coord;

		mgMBSUtil::getCoordArrayFromRawData(
			coord,
			src,
			skeleton,
			motion->motions[f]
		);

		//refCoord.col(f) = coord;
		refCoord.submat(0, f, arma::SizeMat(coord.n_elem, 1)) = coord;
	}

	// generate tar bCharacter from txt file
	bCharacter* tar = new bCharacter();
	bCharacterSim* tarSim = new bCharacterSim(tar);
	gBDOSGSystem* tarVis = new gBDOSGSystem();
	double check_load = loadAvatarModelFromFile(tar, tarSim, tarVis, "target_skeleton_35.txt", 1.0);
	scene->addChild(tarVis->getOSGGroup());

	// initialization of poseTransfer
	initTransfer(src, tar);

	// update src motion ( do what we need )
	int nFnt = 0;
	while (!viewer->done())
	{
		
		viewer->frame(simulationTime);

		if (iter < 0)
			iter = 0;
		if (iter >= motion->nMotion)
			iter = 0;

		std::cout << " frame " << iter << std::endl;

		src->setFromCompactCoordArray(refCoord.col(iter));
		src->updateKinematicsUptoPos();
		src->updateKinematicBodiesOfCharacterSim();


		gXMat offset;
		offset.setTrn(100, 0, 0);
		poseTrans->transferPoseLevMar(offset);
		tar->updateKinematicsUptoPos();
		tar->updateKinematicBodiesOfCharacterSim();
		
		tarVis->update();
		srcVis->update();
		

		simulationTime += 1. / 30.;
		iter++;

		if (bool_go == true) {
			simulationTime += 1. / 30.;
			iter++;
		}
		else if (bool_go_back == true) {
			simulationTime -= 1. / 30.;
			iter--;
		}

	}
	return 0;


}