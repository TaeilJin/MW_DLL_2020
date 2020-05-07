
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
#include "TAE12\TAE12IK\mw_tiUtil_IK.h"

#define NCPtoNSP(n) (n+2) // convert # of control point to # of spline control points, which used in BSpline
#define NCPtoNSE(n) (n-1) // convert # of control point to # of spline segment, which used in BSpline


double DEBUG_DRAW_CONSTRAINT_SIZE = 2;
gVec3 MW_GRAVITY_VECTOR(0, -9.8, 0);
gVec3 MW_GROUND_NORMAL(0, 1, 0);


// test

//bCharacter *avatar;
arma::mat refCoord;
mw_tiUtil_CHAIN *ti_IK;

osg::ref_ptr<osg::Group> debugGroup = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup2 = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup3 = new osg::Group;

const static std::string kTaeilPath = "D:/Taeil_Jin/Development/feature_MWTaeil/Projects/Env_Retargeting/Data/demo/taeil/";
const static std::string kTaeilPath_TaskBVH = "D:/development/DATA/BODYAGENT_BVHFILE/";// "D:/Taeil_Jin/Development/DATA/BODYAGENT_BVHFILE/";
const static std::string kTaeilPath_TaskP1 = kTaeilPath_TaskBVH + "P1/P1_1A_UP.bvh";
const static std::string kSourceObjectFileName = kTaeilPath + "vitra_noarm.obj"; //"VItra_noArm_tri.obj";// "source.obj";//"source.obj";
const static std::string kSourceObjectFileName2 = kTaeilPath + "test_desk_tri.obj";// "test_desk_tri.obj";
const static std::string kTargetObjectFileName = kTaeilPath + "vitra_noarm.obj";//"VItra_noArm_tri.obj";// "Long_woodbench.obj";//"Loft_small.obj";// Sofa_tri.obj";// "Sofa_tri.obj";// Vitra_tallChair.obj";
const static std::string kTargetObjectFileName2 = kTaeilPath + "WhiteBoard.obj"; // WhiteBoard;

//

osgAnimation::BasicAnimationManager* findFirstOsgAnimationManagerNode(osg::Node* node)
{
	osgAnimation::BasicAnimationManager* manager = dynamic_cast<osgAnimation::BasicAnimationManager*>(node->getUpdateCallback());
	if (manager)
	{
		//std::cout << "name: " << node->getName() << std::endl;
		return manager;
	}

	//return NULL if not osg::Group
	osg::Group* group = node->asGroup();
	if (!group) return NULL;

	//else, traverse children		
	for (int i = 0; i < group->getNumChildren(); ++i)
	{
		manager = findFirstOsgAnimationManagerNode(group->getChild(i));
		if (manager) return manager;
	}
	return NULL;
}

osgAnimation::Skeleton* findFirstOsgAnimationSkeletonNode(osg::Node* node)
{
	//return NULL if not osg::Group
	osg::Group* group = node->asGroup();
	if (!group) return NULL;

	//see if node is Skeleton
	osgAnimation::Skeleton* re = dynamic_cast<osgAnimation::Skeleton*>(node);
	if (re)  return re;

	//else, traverse children		
	for (int i = 0; i < group->getNumChildren(); ++i)
	{
		re = findFirstOsgAnimationSkeletonNode(group->getChild(i));
		if (re) return re;
	}
	return NULL;
}

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
void loadAvatarModelFromFile(bCharacter* in_avatar, bCharacterSim* in_avatarSim, gBDOSGSystem* visSys, const char* filename, const double scale)
{

	bCharacterLoader loader;

	if (!loader.loadModel(filename, in_avatar, scale))	printf("fail to load!\n");
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


struct bCharacterGroup {
	bCharacter* avatar;
	bCharacterSim* avatarSim;
	gBDOSGSystem* avatarVis;
	arma::mat RefCoords;

};
bCharacterGroup* MakeCharacter_ByMotionFile(string Motionfile_Path) {
	bCharacterGroup* bChaGroup = new bCharacterGroup();

	MotionLoader loader;
	if (loader.loadMotionFile(Motionfile_Path.c_str()) != -1) {
		mgData* motion = loader.getMotion();
		mgSkeleton* skeleton = loader.getSkeleton();

		const double mass = 70.;
		mgSkeletonToBCharacter::saveToBCharacter(skeleton, (Motionfile_Path + "_cha.txt").c_str(), mass);

		bCharacter* avatar_test = new bCharacter();
		bCharacterSim* avatarSim_test = new bCharacterSim(avatar_test);
		gBDOSGSystem* avatarVIS_test = new gBDOSGSystem();

		loadAvatarModelFromFile(avatar_test, avatarSim_test, avatarVIS_test, (Motionfile_Path + "_cha.txt").c_str(), 1.0);
		bChaGroup->avatar = avatar_test;
		bChaGroup->avatarSim = avatarSim_test;
		bChaGroup->avatarVis = avatarVIS_test;

		arma::mat refCoord(avatar_test->sizeCompactCoordArray() + 1, motion->nMotion, arma::fill::zeros);
		for (int f = 0; f < motion->nMotion; f++)
		{
			arma::vec coord;

			mgMBSUtil::getCoordArrayFromRawData(
				coord,
				avatar_test,
				skeleton,
				motion->motions[f]
			);

			//refCoord.col(f) = coord;
			refCoord.submat(0, f, arma::SizeMat(coord.n_elem, 1)) = coord;
		}

		bChaGroup->RefCoords = refCoord;
	}
	else {
		std::cout << " we will skip this  " << (Motionfile_Path) << std::endl;
	}

	return bChaGroup;
}

bCharacterGroup* MakeCharacter_ByTextFile(string CharacterFile_Path) {
	bCharacterGroup* bChaGroup_character;
	
	bCharacter* avatar_test = new bCharacter();
	bCharacterSim* avatarSim_test = new bCharacterSim(avatar_test);
	
	bCharacterLoader loader;

	if (!loader.loadModel(CharacterFile_Path.c_str(), avatar_test, 1.0))	printf("fail to load!\n");
	avatar_test->postLoading();

	//create avatarSim
	avatar_test->setupCharacterSim(avatarSim_test, m_dynamicsWorld, btVector3(0, 0, 0));
	avatarSim_test->postLoading();

	// kinematic 
	avatarSim_test->setBtBodiesDynamicOrKinematic(btCollisionObject::CF_KINEMATIC_OBJECT);

	bChaGroup_character->avatar = avatar_test;
	bChaGroup_character->avatarSim = avatarSim_test;
	
	return bChaGroup_character;
}


gBDOSGSystem* MakeCharacterVis(bCharacterSim* in_avatarSim) {
	gBDOSGSystem* visSys = new gBDOSGSystem();

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

	return visSys;
}

bCharacterGroup* bChaGroup_Connect;

#ifdef  UNITY_MW_DLL_TEST_EXPORTS
#define DLL_TEST_API __declspec(dllexport)
#else
#define DLL_TEST_API __declspec(dllimport)
#endif //  DLL_TEST_EXPORTS

extern "C"
{
	typedef int(*INIT_IK)(bCharacter* bc);
	typedef bCharacter*(*FULLBODY_IK)(gVec3 pos_D_RArm, gVec3 pos_D_LArm, gVec3 pos_D_base, gVec3 pos_D_RLeg, gVec3 pos_D_LLEg);

	DLL_TEST_API int INIT_bChaGroup() {

		bChaGroup_Connect = new bCharacterGroup();
		std::string CharacterFile_Path = "ybot_MW_cha.txt";
		
		AvatarWorld::getInstance()->loadAvatarModelFromFile(CharacterFile_Path.c_str(), 0.09872);

		bChaGroup_Connect->avatar = AvatarWorld::getInstance()->avatar; 
		
		//MW solver 상의 아바타도 업데이트
		HMODULE hDll = LoadLibrary(L"D://development//Unity//SamplingMesh//Assets//PoseGenerator.dll");

		INIT_IK init_MWSolver;
		init_MWSolver = (INIT_IK)GetProcAddress(hDll, "INIT_IK");

		//Init MW Solver
		return init_MWSolver(bChaGroup_Connect->avatar);


	}

	DLL_TEST_API void SET_BASE_POSITION_MW(float* input_pos) {

		gVec3 position(-1.0 * input_pos[0],input_pos[1], input_pos[2]);
		
		bChaGroup_Connect->avatar->setBasePosition(position);
		bChaGroup_Connect->avatar->updateKinematicsUptoPos();
		bChaGroup_Connect->avatar->updateKinematicBodiesOfCharacterSim();
			
	}

	DLL_TEST_API void SET_JOINT_QUATERNION_MW(LPCSTR jointname, float* input_Quat) {
		gQuat quat; quat.set(input_Quat[0], input_Quat[1], input_Quat[2], input_Quat[3]);
		gRotMat rot_LeftHanded = quat.inRotMatrix();
		gRotMat rot_RightHanded;
		double* convert_L2R = new double[9];
		convert_L2R[0] = rot_LeftHanded.e(0);    convert_L2R[3] = -1 * rot_LeftHanded.e(3); convert_L2R[6] = -1 * rot_LeftHanded.e(6);
		convert_L2R[1] = -1 * rot_LeftHanded.e(1); convert_L2R[4] = rot_LeftHanded.e(4);    convert_L2R[7] = rot_LeftHanded.e(7);
		convert_L2R[2] = -1 * rot_LeftHanded.e(2); convert_L2R[5] = rot_LeftHanded.e(5);    convert_L2R[8] = rot_LeftHanded.e(8);
		rot_RightHanded.set(convert_L2R);

		gXMat mat_G = bChaGroup_Connect->avatar->findLink(jointname)->localFrame();
		mat_G.setRot(rot_RightHanded);

		bChaGroup_Connect->avatar->findLink(jointname)->setFromSafeCoordArray(mat_G.rotInQuat().cptr());
		bChaGroup_Connect->avatar->updateKinematicsUptoPos();
		bChaGroup_Connect->avatar->updateKinematicBodiesOfCharacterSim();

	}

	DLL_TEST_API void SET_JOINT_QUATERNION_UNITY(LPCSTR jointname, float* output_Quat) {

		bChaGroup_Connect->avatar->updateKinematicsUptoPos();

		gRotMat rot_MW = bChaGroup_Connect->avatar->findLink(jointname)->localFrame().rot();

		//
		gQuat quat_rH = rot_MW.inQuat();
		gQuat quat_lH;
		//convert righthand quaternion to lefthand quaternion
		quat_lH.setX(quat_rH.x());
		quat_lH.setY(-1.0 * quat_rH.y());
		quat_lH.setZ(-1.0 * quat_rH.z());
		quat_lH.setW(quat_rH.w());

		output_Quat[0] = quat_lH.x(); output_Quat[1] = quat_lH.y(); output_Quat[2] = quat_lH.z(); output_Quat[3] = quat_lH.w();
		//


	}

	DLL_TEST_API void SET_BASE_POSITION_UNITY(float* output_POS) {

		bChaGroup_Connect->avatar->updateKinematicsUptoPos();

		gVec3 pos = bChaGroup_Connect->avatar->baseLink()->frame().trn();

		output_POS[0] = -1 * pos.x(); output_POS[1] = pos.y(); output_POS[2] = pos.z();

	}

	DLL_TEST_API void DO_FULLBODY_IK(float* input_D_RArm, float* input_D_LArm, float* input_D_base, float* input_D_RLeg, float* input_D_LLeg) {

		//MW solver 상의 아바타도 업데이트
		HMODULE hDll = LoadLibrary(L"D://development//Unity//SamplingMesh//Assets//PoseGenerator.dll");

		FULLBODY_IK FullBody_IK;
		FullBody_IK = (FULLBODY_IK)GetProcAddress(hDll, "FULLBODY_IK");

		//Init MW Solver
		gVec3 pos_D_RArm(input_D_RArm[0], input_D_RArm[1], input_D_RArm[2]);
		gVec3 pos_D_LArm(input_D_LArm[0], input_D_LArm[1], input_D_LArm[2]);
		gVec3 pos_D_base(input_D_base[0], input_D_base[1], input_D_base[2]);
		gVec3 pos_D_RLeg(input_D_RLeg[0], input_D_RLeg[1], input_D_RLeg[2]);
		gVec3 pos_D_LLeg(input_D_LLeg[0], input_D_LLeg[1], input_D_LLeg[2]);
		
		bChaGroup_Connect->avatar = FullBody_IK(pos_D_RArm, pos_D_LArm, pos_D_base, pos_D_RLeg, pos_D_LLeg);
		bChaGroup_Connect->avatar->updateKinematicsUptoPos();
	}
	DLL_TEST_API LPCSTR INIT_JOINT_LIST(int i){
		
		LPCSTR joint_name = bChaGroup_Connect->avatar->link(i)->name();
		
		return joint_name;
	}

}

int main(int argc, char **argv)
{
	// construct the viewer.
	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;

	osg::ref_ptr<osg::Group> scene = new osg::Group;
	osg::ref_ptr<osg::MatrixTransform> rootTrans = new osg::MatrixTransform;
	osg::ref_ptr<osg::MatrixTransform> refTrans = new osg::MatrixTransform;

	//scene->addChild(rootTrans);
	//scene->addChild(refTrans);

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

	bCharacterGroup* avatarGroup = new bCharacterGroup();
	std::string CharacterFile_Path = "C:/Users/TAEIL_LAB/Desktop/Github_MWDLL_2/msvc2017/ybot_MW_cha.txt";
	avatarGroup = MakeCharacter_ByTextFile(CharacterFile_Path);
	std::cout << "see " << avatarGroup->avatar->numLinks() << std::endl;;
	LPCTSTR* jointlist = new LPCTSTR[avatarGroup->avatar->numLinks()];

	for (int i = 0; i < avatarGroup->avatar->numLinks(); i++) {
		jointlist[i] = (LPCTSTR)avatarGroup->avatar->link(i)->name();
		std::cout << " jointlist " << jointlist[i] << std::endl;
	}

	avatarGroup->avatarVis = new gBDOSGSystem();
	avatarGroup->avatarVis = MakeCharacterVis(avatarGroup->avatarSim);

	scene->addChild(avatarGroup->avatarVis->getOSGGroup());
	int iter = 0;
	double simulationTime = 0;

	int nFnt = 0; int nTotalMotion = 0;
	while (!viewer->done())
	{
		viewer->frame(simulationTime);

		//gVec3 p; p.set(90 * M_PI / 180, 0, 0);
		//avatarGroup->avatar->findLink("LeftUpLeg")->setFromCompactCoordArray(p.cptr());
		//avatarGroup->avatar->updateKinematicsUptoPos();
		//avatarGroup->avatar->updateKinematicBodiesOfCharacterSim();

		avatarGroup->avatarVis->update();


		/*if (iter >= avatarGroup->RefCoords.n_cols) {
			iter = 0; nTotalMotion++;
		}
		else {

			debugGroup->removeChildren(0, debugGroup->g etNumChildren());

			avatarGroup->avatar->setFromCompactCoordArray(avatarGroup->RefCoords.col(iter));
			avatarGroup->avatar->updateKinematicsUptoPos();
			avatarGroup->avatar->updateKinematicBodiesOfCharacterSim();


			debugGroup->addChild(avatarGroup->avatarVis->getOSGGroup());
			avatarGroup->avatarVis->update();

			iter++;
		}*/


		simulationTime += 1. / 30.;


	}
	return 0;


}