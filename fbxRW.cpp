#include <Windows.h>
#include <iostream>
#include <algorithm>
#include <io.h>

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

#include "TAE12\TAE12IK\mw_tiUtil_IK.h"
#include "mgPoseTransfer.h"

#define NCPtoNSP(n) (n+2) // convert # of control point to # of spline control points, which used in BSpline
#define NCPtoNSE(n) (n-1) // convert # of control point to # of spline segment, which used in BSpline

double DEBUG_DRAW_CONSTRAINT_SIZE = 2;
gVec3 MW_GRAVITY_VECTOR(0, -9.8, 0);
gVec3 MW_GROUND_NORMAL(0, 1, 0);

//bCharacter *avatar;
arma::mat refCoord;
mw_tiUtil_CHAIN *ti_IK;

osg::ref_ptr<osg::Group> debugGroup = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup2 = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup3 = new osg::Group;

const static std::string kTaeilPath = "D:/Taeil_Jin/Development/feature_MWTaeil/Projects/Env_Retargeting/Data/demo/taeil/";
const static std::string kTaeilPath_TaskBVH = "C:/Users/TAEIL_LAB/Desktop/WalkMe_bvh/Source/";// "D:/Taeil_Jin/Development/DATA/BODYAGENT_BVHFILE/";
const static std::string kTaeilPath_TargetBVH = "C:/Users/TAEIL_LAB/Desktop/WalkMe_bvh/Target/";
const static std::string kTaeilPath_TaskP1 = kTaeilPath_TaskBVH + "P1/P1_1A_UP.bvh";
const static std::string kSourceObjectFileName = kTaeilPath + "vitra_noarm.obj"; //"VItra_noArm_tri.obj";// "source.obj";//"source.obj";
const static std::string kSourceObjectFileName2 = kTaeilPath + "test_desk_tri.obj";// "test_desk_tri.obj";
const static std::string kTargetObjectFileName = kTaeilPath + "vitra_noarm.obj";//"VItra_noArm_tri.obj";// "Long_woodbench.obj";//"Loft_small.obj";// Sofa_tri.obj";// "Sofa_tri.obj";// Vitra_tallChair.obj";
const static std::string kTargetObjectFileName2 = kTaeilPath + "WhiteBoard.obj"; // WhiteBoard;

void initializeVisualization()
{
	osg::ref_ptr<osgDB::Options> options = new osgDB::Options;
	options->setOptionString("noRotation");
	osg::ref_ptr<osg::Node> src_obj_node = osgDB::readNodeFile(kSourceObjectFileName, options.get());
	osg::ref_ptr<osg::Node> tar_obj_node = osgDB::readNodeFile(kTargetObjectFileName, options.get());
	src_obj_node->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	tar_obj_node->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

	osg::Vec3 src_p = src_obj_node->getWorldMatrices().begin()->getTrans();
	std::cout << " x " << src_p.x() << " y " << src_p.y() << " z " << src_p.z() << std::endl;


	debugGroup2->addChild(src_obj_node);
	debugGroup3->addChild(tar_obj_node);

	osg::ref_ptr<osg::Node> src_obj_node2 = osgDB::readNodeFile(kSourceObjectFileName2, options.get());
	osg::ref_ptr<osg::Node> tar_obj_node2 = osgDB::readNodeFile(kTargetObjectFileName2, options.get());
	src_obj_node2->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	tar_obj_node2->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

	src_p = src_obj_node2->getWorldMatrices().begin()->getTrans();
	std::cout << " x " << src_p.x() << " y " << src_p.y() << " z " << src_p.z() << std::endl;

	debugGroup2->addChild(src_obj_node2);
	debugGroup3->addChild(tar_obj_node2);
}
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


//save motion curve
std::ofstream myfile_TaskPlane;
bool first_bool_TaskPlane = false;
gXMat gXMat_standard_frame;
void writeCSVFile_TaskPlane(double cha_height, bCharacter* in_avatar, bool b_stop,
	gLink* gLink_rhand, gLink* gLink_lhand, gLink* gLink_rfoot, gLink* gLink_lfoot) {
	using namespace std;

	// input: position of task plane, normal of task plane, angle between normal and hand direction.
	// output: base position(x,z) foot position(x,y,z), leaning direction(spine)

	if (b_stop == false) {

		// Input
		// Task plane
		//-- 1.center position of hands / character height (midHandPos)
		gLink* gLink_rHand = gLink_rhand; gLink* gLink_lHand = gLink_lhand;
		gVec3 rHand = gLink_rHand->frame().multVec4(gLink_rHand->inertia().comInVec3());
		gVec3 lHand = gLink_lHand->frame().multVec4(gLink_lHand->inertia().comInVec3());
		// center position
		gVec3 dirR2L = lHand - rHand; double mag = dirR2L.magnitude();
		mag = mag / 2;	dirR2L.normalize();
		gVec3 pos_CenterHand = rHand + (dirR2L * mag);
		pos_CenterHand = gXMat_standard_frame.invMultVec4(pos_CenterHand);
		pos_CenterHand /= cha_height;

		// write CSV FILE
		myfile_TaskPlane << pos_CenterHand.x() << "," << pos_CenterHand.y() << "," << pos_CenterHand.z() << ",";

		//-- 2.object normal vector
		gVec3 norm_TaskPlane = gVec3(0, 1, 0);

		// write CSV FILE
		myfile_TaskPlane << norm_TaskPlane.x() << "," << norm_TaskPlane.y() << "," << norm_TaskPlane.z() << ",";


		//-- 3.angle between hand direction and object normal vector

		//right hand
		gVec3 rHandMid = gLink_rHand->frame().trn();
		gVec3 dirR2Tip = gLink_rHand->frame().rotX();
		dirR2Tip.normalize(); dirR2Tip = -1 * dirR2Tip;
		//left hand
		gVec3 lHandMid = gLink_lHand->frame().trn();
		gVec3 dirL2Tip = gLink_lHand->frame().rotX();
		dirL2Tip.normalize(); dirL2Tip = dirL2Tip;

		//get normal
		gVec3 norm_Hands = dirR2Tip % dirL2Tip;
		double angles = std::acos(norm_Hands.operator,(norm_TaskPlane));

		// write CSV FILE
		myfile_TaskPlane << angles << ",";

		//Output
		// Foot position (seen from base) / character height (pos_CenterFoot)
		// Base position (relative position with task position) / character height (pos_Base)
		// Spine direction : calculate direction relative with up-vector(world frame normal)

		//--1.Foot position
		gLink* FootR = gLink_rfoot;	gLink* FootL = gLink_lfoot;
		gVec3 rFoot = FootR->frame().multVec4(FootR->inertia().comInVec3());
		gVec3 lFoot = FootL->frame().multVec4(FootL->inertia().comInVec3());

		dirR2L = lFoot - rFoot; mag = dirR2L.magnitude();
		mag = mag / 2;	dirR2L.normalize();
		gVec3 pos_CenterFoot = rFoot + (dirR2L * mag);
		pos_CenterFoot = gXMat_standard_frame.invMultVec4(pos_CenterFoot);
		pos_CenterFoot /= (cha_height);

		// write CSV FILE
		myfile_TaskPlane << pos_CenterFoot.x() << "," << pos_CenterFoot.y() << "," << pos_CenterFoot.z() << ",";


		//--2.base position (pos_Base)
		gVec3 pos_Base = in_avatar->baseLink()->frame().trn();
		pos_Base = gXMat_standard_frame.invMultVec4(pos_Base);
		pos_Base = pos_Base /= (cha_height);

		// write CSV FILE
		myfile_TaskPlane << pos_Base.x() << "," << pos_Base.y() << "," << pos_Base.z() << ",";


		//--3.Spine direction (dir_spineJoint)
		int nSize = in_avatar->m_trunkIdx.size();
		for (int p = 0; p < nSize; p++)
		{
			int idx_j = in_avatar->m_trunkIdx[p];

			gLink* gLink_joint = in_avatar->link(idx_j);
			gVec3 dir_spineJoint = gLink_joint->frame().rotY(); // world direction of joint
			dir_spineJoint = gXMat_standard_frame.invMultVec3(dir_spineJoint);
			if (p == nSize - 1)
			{									// write CSV FILE
				myfile_TaskPlane << dir_spineJoint.x() << "," << dir_spineJoint.y() << "," << dir_spineJoint.z() << "\n";
			}
			else {
				myfile_TaskPlane << dir_spineJoint.x() << "," << dir_spineJoint.y() << "," << dir_spineJoint.z() << ",";
			}
		}

	}
	else
	{
		std::cout << " write csvFile is end " << std::endl;
		myfile_TaskPlane.close();

	}

}
void writeCSVFile_Quat(bCharacter* in_avatar, bool b_stop) {
	using namespace std;
	if (b_stop == false) {
		arma::vec quaternion;
		quaternion.resize(in_avatar->calcSizeSafeCoordArray());
		in_avatar->getSafeCoordArray(quaternion);

		for (int p = 0; p < in_avatar->numLinks(); p++) {
			if (p == 0) {
				gVec3 pos_Jnt = in_avatar->link(p)->frame().trn(); // root world position
				myfile_TaskPlane << pos_Jnt.x() << "," << pos_Jnt.y() << "," << pos_Jnt.z() << ",";
			}
			gQuat quat_Jnt = in_avatar->link(p)->frame().rotInQuat(); // root + joint local rotation
			myfile_TaskPlane << quat_Jnt.x() << "," << quat_Jnt.y() << "," << quat_Jnt.z() << "," << quat_Jnt.w() << ",";
		}

		for (int p = 1; p < in_avatar->numLinks(); p++) {
			gVec3 pos_Jnt = in_avatar->link(p)->frame().trn(); // joint world position
			myfile_TaskPlane << pos_Jnt.x() << "," << pos_Jnt.y() << "," << pos_Jnt.z() << ",";
		}

		myfile_TaskPlane << "\n";
	}
	else
	{
		std::cout << " write csvFile is end " << std::endl;
		myfile_TaskPlane.close();

	}
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

// avatar for Retargeting
std::vector<bCharacter*> avatar_Group;
std::vector<bCharacterSim*> avatarSim_Group;
std::vector<gBDOSGSystem*> avatarVis_Group;
std::vector<double> avatar_length;


double len_Src;

//
mgPoseTransfer *poseTrans;

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
void loadAvatarModelFromFile(bCharacter* in_avatar, bCharacterSim* in_avatarSim, gBDOSGSystem* visSys, const char* filename, const double scale, double& exist)
{

	bCharacterLoader loader;

	if (!loader.loadModel(filename, in_avatar, scale)) { printf("fail to avatar model load!\n"); exist = -1; }
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

}
void SetJntRotDirOBJ(mgPoseTransfer* poseTrans, const char* txt_id, const char* src_jnt, const char* tar_jnt) {
	poseTrans->addPoint(txt_id, *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 0, 0));
	poseTrans->addPoint("t0_x", *poseTrans->src->findLink(src_jnt), gVec3(1, 0, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(1, 0, 0));
	poseTrans->addPoint("t0_z", *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 1), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 0, 1));

	poseTrans->addDirectionObjective(txt_id, "t0_x", 1.0);
	poseTrans->addDirectionObjective(txt_id, "t0_z", 1.0);

}
void initTransfer(bCharacter* src, bCharacter* tar) {
	poseTrans = new mgPoseTransfer(src, tar);

	//
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

std::vector<std::string> get_files_inDirectory(const std::string& _path, const std::string& _filter)
{
	std::string searching = _path + _filter;
	std::vector<std::string> return_;

	_finddata_t fd;
	intptr_t handle = _findfirst(searching.c_str(), &fd);  //현재 폴더 내 모든 파일을 찾는다.

	if (handle == -1)    return return_;
	int result = 0;
	do
	{
		string filename_ext = fd.name;
		string delimiter = ".";
		string filename = filename_ext.substr(0, filename_ext.find(delimiter));
		return_.push_back(filename);
		result = _findnext(handle, &fd);
	} while (result != -1);

	_findclose(handle);
	return return_;
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

	vector<string>source_files = get_files_inDirectory(kTaeilPath_TaskBVH, "*.bvh");
	vector<string>target_files = get_files_inDirectory(kTaeilPath_TargetBVH, "*.txt");

	int target_skeleton_idx[80];
	for (int i = 0; i < 80; i++) {
		target_skeleton_idx[i] = i;
	}

	srand(time(NULL));
	int temp;
	int rn;
	for (int i = 0; i < 80 - 1; i++) {
		rn = rand() % (80 - i) + i; // i 부터 num-1 사이에 임의의 정수 생성
		temp = target_skeleton_idx[i];
		target_skeleton_idx[i] = target_skeleton_idx[rn];
		target_skeleton_idx[rn] = temp;
	}

	MotionLoader loader;

	//for (int i = 0; i < source_files.size(); i++) {
	while (1)
	{
		int i = 0;
		//for (int i = 2; i < 1; i++) {	
		if (loader.loadMotionFile((kTaeilPath_TaskBVH + source_files[i] + ".bvh").c_str()) == 0) {
			//get source avatar & motion data
			bCharacter* bCha_src = new bCharacter();
			bCharacterSim* bChaSim_src = new bCharacterSim(bCha_src);
			gBDOSGSystem* bChaVis_src = new gBDOSGSystem();

			mgData* motion = loader.getMotion();
			mgSkeleton* skeleton = loader.getSkeleton();
			const double mass = 70.;
			mgSkeletonToBCharacter::saveToBCharacter(skeleton, ("source_skeleton.txt"), mass); // make src specfile
			double exist = -1;
			loadAvatarModelFromFile(bCha_src, bChaSim_src, bChaVis_src, ("source_skeleton.txt"), 1.0, exist); // load src specfile																					  
			arma::mat refCoord(bCha_src->sizeCompactCoordArray() + 1, motion->nMotion, arma::fill::zeros); //load src motion
			
			for (int f = 0; f < motion->nMotion; f++)
			{
				arma::vec coord;
				mgMBSUtil::getCoordArrayFromRawData(
					coord,
					bCha_src,
					skeleton,
					motion->motions[f]
				);
				refCoord.submat(0, f, arma::SizeMat(coord.n_elem, 1)) = coord;
			}

			//for (int j = 0; j < target_files.size(); j++) { 
			//for (int j = 0; j < 10; j++) {
			//cout << "source " << i << " to target " << j << " " << endl;
			bCharacter* bCha_tar = new bCharacter();
			bCharacterSim* bChaSim_tar = new bCharacterSim(bCha_tar);
			gBDOSGSystem* bChaVis_tar = new gBDOSGSystem();

			string delimiter = "skeleton";
			string subject = target_files[target_skeleton_idx[i]].substr(target_files[target_skeleton_idx[i]].length() - 2);
			string filename = target_files[target_skeleton_idx[i]] + ".txt";

			loadAvatarModelFromFile(bCha_tar, bChaSim_tar, bChaVis_tar, (kTaeilPath_TargetBVH + filename.c_str() ).c_str(), 1.0, exist);
		
			myfile_TaskPlane.open("retarget_output/" + source_files[i] + "_retargeted_" + subject + ".csv");

			//viewer->frame(1);
			
			//// retargeting
			initTransfer(bCha_src, bCha_tar);
			for (int iter = 0; iter < refCoord.n_cols; iter++) {
				bCha_src->setFromCompactCoordArray(refCoord.col(iter));
				bCha_src->updateKinematicsUptoPos();
				bCha_src->updateKinematicBodiesOfCharacterSim();

				gXMat offset;
				poseTrans->transferPoseLevMar(offset);
				bCha_tar->updateKinematicsUptoPos();
				bCha_tar->updateKinematicBodiesOfCharacterSim();

				//// visualize
				viewer->frame(1);
				debugGroup->removeChildren(0, debugGroup->getNumChildren());
				debugGroup->addChild(bChaVis_src->getOSGGroup());
				bChaVis_src->update();

				//for (int p = 0; p < poseTrans->srcPoints.size(); p++) {
				//	gOSGShape::setColor(osg::Vec4(1.0, 0.0, 0, 1.0));
				//	debugGroup->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(poseTrans->srcPoints[p].posWorld()), 5.0));
				//	gOSGShape::setColor(osg::Vec4(0.0, 0.0, 1.0, 1.0));
				//	debugGroup->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(poseTrans->tarPoints[p].posWorld()), 5.0));

				//}
				for (int i = 0; i < bCha_tar->numLinks(); i++) {
					gOSGShape::setColor(osg::Vec4(1.0, 0.0, 0, 1.0));
					debugGroup->addChild(gOSGShape::createLineShape(gVec3_2_OsgVec(bCha_tar->link(i)->frame().trn()), gVec3_2_OsgVec(bCha_tar->link(i)->frame().rotZ()), 10.0, 1.0));

					gOSGShape::setColor(osg::Vec4(0.0, 0.0, 1.0, 1.0));
					debugGroup->addChild(gOSGShape::createLineShape(gVec3_2_OsgVec(bCha_src->link(i)->frame().trn()), gVec3_2_OsgVec(bCha_src->link(i)->frame().rotZ()), 3.0, 1.0));

				}
				//debugGroup->removeChildren(0, debugGroup->getNumChildren());
				debugGroup->addChild(bChaVis_tar->getOSGGroup());
				bChaVis_tar->update();

				// write
				writeCSVFile_Quat(bCha_tar, false);
			}

			cout << " done" << endl;
			myfile_TaskPlane.close();

			delete bCha_tar;
			delete bChaSim_tar;
			delete bChaVis_tar;

			delete bCha_src;
			delete bChaSim_src;
			delete bChaVis_src;
		}
	//}
	}
}