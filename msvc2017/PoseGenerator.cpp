
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
void writeCSVFile_TaskPlane(double cha_height, bCharacter* in_avatar, bool b_stop) {
	using namespace std;

	// input: position of task plane, normal of task plane, angle between normal and hand direction.
	// output: base position(x,z) foot position(x,y,z), leaning direction(spine)

	if (b_stop == false) {

		// Input
		// Task plane
		//-- 1.center position of hands / character height (midHandPos)
		gLink* gLink_rHand = in_avatar->findLink("RHand"); gLink* gLink_lHand = in_avatar->findLink("LHand");
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


		////-- 3.angle between hand direction and object normal vector

		////right hand
		//gVec3 rHandMid = in_avatar->findLink("RHand")->frame().trn();
		//gVec3 dirR2Tip = in_avatar->findLink("RHand")->frame().rotX();
		//dirR2Tip.normalize(); dirR2Tip = -1 * dirR2Tip;
		////left hand
		//gVec3 lHandMid = in_avatar->findLink("LHand")->frame().trn();
		//gVec3 dirL2Tip = in_avatar->findLink("LHand")->frame().rotX();
		//dirL2Tip.normalize(); dirL2Tip = dirL2Tip;

		////get normal
		//gVec3 norm_Hands = dirR2Tip % dirL2Tip;
		//double angles = std::acos(norm_Hands.operator,(norm_TaskPlane));

		//// write CSV FILE
		//myfile_TaskPlane << angles << ",";

		//Output
		// Foot position (seen from base) / character height (pos_CenterFoot)
		// Base position (relative position with task position) / character height (pos_Base)
		// Spine direction : calculate direction relative with up-vector(world frame normal)

		//--1.Foot position
		gLink* FootR = in_avatar->findLink("RFoot");	gLink* FootL = in_avatar->findLink("LFoot");
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

std::vector<bCharacter*> avatar_Group;
std::vector<bCharacterSim*> avatarSim_Group;
std::vector<gBDOSGSystem*> avatarVis_Group;
std::vector<double> avatar_length;
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

alglib::real_2d_array GetReal2dArr_ByArmaMat(arma::mat mat_ARMA) {
	using namespace alglib;

	real_2d_array Arr_TrainedData_ALG = "[[1,1,1],[1,2,2],[2,1,2],[2,2,4]]";
	
	Arr_TrainedData_ALG.setlength(mat_ARMA.n_rows, mat_ARMA.n_cols);
	for (int i = 0; i < mat_ARMA.n_rows; i++) {
		for (int j = 0; j < mat_ARMA.n_cols; j++) {
			Arr_TrainedData_ALG[i][j] = mat_ARMA.row(i)[j];
		}
	}

	return Arr_TrainedData_ALG;

}
alglib::real_2d_array GetData_to_Real2dArr(std::string TrainData_CSV) {
	using namespace alglib;

	arma::mat mat_TrainData; mat_TrainData.load(TrainData_CSV, arma::csv_ascii);

	real_2d_array Arr_TrainedData_ALG = GetReal2dArr_ByArmaMat(mat_TrainData);

	return Arr_TrainedData_ALG;
}
void TrainMLPRegressor(std::string TrainData_CSV , int n_input, int n_output) {
	
	using namespace alglib;

	arma::mat mat_TrainData; mat_TrainData.load(TrainData_CSV, arma::csv_ascii);
	
	real_2d_array Arr_TrainedData_ALG = GetReal2dArr_ByArmaMat(mat_TrainData);
	
	int n_Train_Rows = Arr_TrainedData_ALG.rows(); int n_Train_Cols = Arr_TrainedData_ALG.cols();
	std::cout << " train data " << n_Train_Rows << " all xy " << n_Train_Cols << std::endl;


	//// Trainer object is used to train network. It stores dataset, training settings,
	//// and other information which is NOT part of neural network. You should use
	//// trainer object as follows:
	//// (1) you create trainer object and specify task type (classification/regression)
	////     and number of inputs/outputs
	//// (2) you add dataset to the trainer object
	//// (3) you may change training settings (stopping criteria or weight decay)
	//// (4) finally, you may train one or more networks
	////
	//// You may interleave stages 2...4 and repeat them many times. Trainer object
	//// remembers its internal state and can be used several times after its creation
	//// and initialization.
	////
	mlptrainer trn;

	////
	//// Stage 1: object creation.
	////
	//// We have to specify number of inputs and outputs. Trainer object can be used
	//// only for problems with same number of inputs/outputs as was specified during
	//// its creation.
	////
	//// In case you want to train SOFTMAX-normalized network which solves classification
	//// problems,  you  must  use  another  function  to  create  trainer  object:
	//// mlpcreatetrainercls().
	////
	//// Below we create trainer object which can be used to train regression networks
	//// with 2 inputs and 1 output.
	////
	mlpcreatetrainer(n_input, n_output, trn);

	////
	//// Stage 2: specification of the training set
	////
	//// By default trainer object stores empty dataset. So to solve your non-empty problem
	//// you have to set dataset by passing to trainer dense or sparse matrix.
	////
	//// One row of the matrix corresponds to one record A*B=C in the multiplication table.
	//// First two columns store A and B, last column stores C
	////
	////     [1 * 1 = 1]   [ 1 1 1 ]
	////     [1 * 2 = 2]   [ 1 2 2 ]
	////     [2 * 1 = 2] = [ 2 1 2 ]
	////     [2 * 2 = 4]   [ 2 2 4 ]
	////
	

	mlpsetdataset(trn, Arr_TrainedData_ALG, n_Train_Rows);



	
	
	////
	//// Stage 3: modification of the training parameters.
	////
	//// You may modify parameters like weights decay or stopping criteria:
	//// * we set moderate weight decay
	//// * we choose iterations limit as stopping condition (another condition - step size -
	////   is zero, which means than this condition is not active)
	////
	double wstep = 0.000;
	ae_int_t maxits = 100;
	mlpsetdecay(trn, 0.01);
	mlpsetcond(trn, wstep, maxits);

	////
	//// Stage 4: training.
	////
	//// We will train several networks with different architecture using same trainer object.
	//// We may change training parameters or even dataset, so different networks are trained
	//// differently. But in this simple example we will train all networks with same settings.
	////
	//// We create and train three networks:
	//// * network 1 has 2x1 architecture     (2 inputs, no hidden neurons, 1 output)
	//// * network 2 has 2x5x1 architecture   (2 inputs, 5 hidden neurons, 1 output)
	//// * network 3 has 2x5x5x1 architecture (2 inputs, two hidden layers, 1 output)
	////
	//// NOTE: these networks solve regression problems. For classification problems you
	////       should use mlpcreatec0/c1/c2 to create neural networks which have SOFTMAX-
	////       normalized outputs.
	////
	//multilayerperceptron net1;
	//multilayerperceptron net2;
	multilayerperceptron net3;
	mlpreport rep;

	//mlpcreate0(n_input, n_output, net1);
	//mlpcreate1(n_input, 5, n_output, net2);
	mlpcreate2(n_input, 5, 5, n_output, net3);

	//mlptrainnetwork(trn, net1, 5, rep);
	//mlptrainnetwork(trn, net2, 5, rep);
	mlptrainnetwork(trn, net3, 5, rep);
	std::cout << "let me see the error " << rep.avgerror << std::endl;
	std::string see;
	mlpserialize(net3, see);
	

	std::cout << " see " << see << std::endl;
	std::ofstream out; out.open("Network_PoseGenerator.txt");
	out << see << std::endl;
	out.close();
	/*std::cin >> see;
	std::ofstream out("Network_PoseGenerator.txt");
	out << see;
	out.close();*/


	std::stringstream him;
	std::ifstream myfile; myfile.open("Network_PoseGenerator.txt");
	if (myfile.is_open())
	{
		him << myfile.rdbuf();

		myfile.close();

	}
	else {
		std::cout << " there is no file " << std::endl;
	}
	
	multilayerperceptron out_test;
	mlpunserialize(him, out_test);

	alglib::real_1d_array x = "[0.006503,	0.690238,	0.112993,	0,	1,	0 ]";// "[-0.235816,	0.682514,	0.0530888,	0,	1,	0,	1.76715]";// "[-0.230186,	0.36868,	0.0459454,	0,	1,	0,	1.73736]";
	alglib::real_1d_array y = "[- 2.84E-07, - 0.467457,	0.0226382,	3.87E-08,	0.489463,	0.0161014,	0,	1,	0,	0,	1,	0,	0,	1,	0,	0	,1,	0,	0,	1,	0,	0,	1,	0]";
	mlpprocess(out_test, x, y);
	printf("%s\n", y.tostring(1).c_str()); // EXPECTED: [4.000]
	

	//return 0;
}
alglib::multilayerperceptron LoadMLPRegressor(std::string Regressor) {
	using namespace alglib;
	std::stringstream him;
	std::ifstream myfile; myfile.open(Regressor);
	if (myfile.is_open())
	{
		him << myfile.rdbuf();

		myfile.close();

	}
	else {
		std::cout << " there is no file " << std::endl;
	}

	multilayerperceptron out_test;
	mlpunserialize(him, out_test);

	return out_test;
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
	
	// Trained Regressor
	//TrainMLPRegressor("Data_Set.csv", 6, 24);
	
	//alglib::multilayerperceptron MLP_Regressor = LoadMLPRegressor("Network_PoseGenerator.txt");

	//alglib::real_1d_array x = "[0.006503,	0.690238,	0.112993,	0,	1,	0 ]";// "[-0.235816,	0.682514,	0.0530888,	0,	1,	0,	1.76715]";// "[-0.230186,	0.36868,	0.0459454,	0,	1,	0,	1.73736]";
	//alglib::real_1d_array y = "[- 2.84E-07, - 0.467457,	0.0226382,	3.87E-08,	0.489463,	0.0161014,	0,	1,	0,	0,	1,	0,	0,	1,	0,	0	,1,	0,	0,	1,	0,	0,	1,	0]";
	//mlpprocess(MLP_Regressor, x, y);
	//printf("%s\n", y.tostring(1).c_str()); // EXPECTED: [4.000]

	//alglib::real_2d_array error_set = GetData_to_Real2dArr("Data_Set.csv");
	//std::cout<< " " << alglib::mlperror(MLP_Regressor, error_set, error_set.rows()) / error_set.rows() <<std::endl;

	//getting all motion data
	int n_actors = 10;
	int n_style = 6;
	std::vector<arma::mat> motion_refCoords;
	motion_refCoords.resize(n_actors * n_style);
	//one character for one motion
	avatar_Group.resize(n_actors * n_style); avatarSim_Group.resize(n_actors * n_style); avatarVis_Group.resize(n_actors * n_style);

	int n_TotalMotionClips = 0;
	//get all character specs and motions
	for (int i = 1; i < 11; i++) {
		std::string s = std::to_string(i);
		for (int j = 1; j < 7; j++) {
			MotionLoader loader;
			if (loader.loadMotionFile((kTaeilPath_TaskBVH + "P" + s + "/" + "P" + s + "_" + std::to_string(j) + ".bvh").c_str()) != -1) {
				mgData* motion = loader.getMotion();
				mgSkeleton* skeleton = loader.getSkeleton();

				//const double mass = 70.;
				//mgSkeletonToBCharacter::saveToBCharacter(skeleton, ("P" + s + "_cha.txt").c_str(), mass);

				bCharacter* avatar_test = new bCharacter();
				bCharacterSim* avatarSim_test = new bCharacterSim(avatar_test);
				gBDOSGSystem* avatarVIS_test = new gBDOSGSystem();

				loadAvatarModelFromFile(avatar_test, avatarSim_test, avatarVIS_test, ("P" + s + "_cha.txt").c_str(), 1.0);
				avatar_Group[n_TotalMotionClips] = avatar_test;
				avatarSim_Group[n_TotalMotionClips] = avatarSim_test;
				avatarVis_Group[n_TotalMotionClips] = avatarVIS_test;

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
				motion_refCoords[n_TotalMotionClips] = refCoord;
				n_TotalMotionClips++;
			}
			else {
				std::cout << " we will skip this  " << (kTaeilPath_TaskBVH + "P" + s + "/" + "P" + s + "_" + std::to_string(j) + ".bvh") << std::endl;
			}
		}
	}



	//
	avatar_length.resize(n_actors * n_style);
	for (int nTotalMotion = 0; nTotalMotion < n_TotalMotionClips; nTotalMotion++) {
		gVec3 HeadCom = avatar_Group[nTotalMotion]->findLink("Head")->inertia().comInVec3();
		HeadCom.setY(HeadCom.y() * 2);
		HeadCom = avatar_Group[nTotalMotion]->findLink("Head")->frame().multVec4(HeadCom);

		gVec3 FootToe = avatar_Group[nTotalMotion]->findLink("RToe")->inertia().comInVec3();
		//FootToe.setY(FootToe.y() * -2);
		FootToe = avatar_Group[nTotalMotion]->findLink("RToe")->frame().multVec4(
			FootToe);
		gOSGShape::setColor(osg::Vec4(1, 0, 0, 1));
		debugGroup->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(HeadCom), 15.0));
		debugGroup->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(FootToe), 15.0));
		double length = HeadCom.y() - FootToe.y();
		avatar_length[nTotalMotion] = length + 5;
		std::cout << "motion " << nTotalMotion << " 's length " << length << std::endl;
	}
	std::cout << " total motion number " << motion_refCoords.size() << " saved motion clips " << n_TotalMotionClips << std::endl;


	//
	myfile_TaskPlane.open("Data_Set.csv");

	//std::stringstream him;
	//std::ifstream myfile("Network1.txt");
	//if (myfile.is_open())
	//{
	//	him << myfile.rdbuf();

	//	myfile.close();

	//}
	//else {
	//	std::cout << " there is no file " << std::endl;
	//}
	//

	//alglib::multilayerperceptron network;
	//alglib::mlpunserialize(him.str(), network);
	//alglib::real_1d_array x = "[-0.212654,	0.535689,	0.116638,	0,	1,	0,	1.77906]";// "[-0.235816,	0.682514,	0.0530888,	0,	1,	0,	1.76715]";// "[-0.230186,	0.36868,	0.0459454,	0,	1,	0,	1.73736]";
	//alglib::real_1d_array y = "[- 2.84E-07, - 0.467457,	0.0226382,	3.87E-08,	0.489463,	0.0161014,	0,	1,	0,	0,	1,	0,	0,	1,	0,	0	,1,	0,	0,	1,	0,	0,	1,	0]";
	//mlpprocess(network, x, y);
	//printf("%s\n", y.tostring(1).c_str()); // EXPECTED: [4.000]

	//ti_IK = new mw_tiUtil_CHAIN(avatar);
	//ti_IK->initChainInfo(ti_IK->sR_LegChain, 3, avatar->m_rLegIdx, gVec3(0, 0, 0), avatar->findLink("RFoot")->id(), true);
	//ti_IK->initChainInfo(ti_IK->sL_LegChain, 4, avatar->m_lLegIdx, gVec3(0, 0, 0), avatar->findLink("LFoot")->id(), true);
	//ti_IK->initChainInfo(ti_IK->sT_TrunkChain, 0, avatar->m_trunkIdx, gVec3(0, 0, 0), avatar->findLink("Head")->id(), true);


	int iter = 900;
	double simulationTime = 0;

	int nFnt = 0; int nTotalMotion = 0;
	while (!viewer->done())
	{
		viewer->frame(simulationTime);

		debugGroup->removeChildren(0, debugGroup->getNumChildren());

		
		if (nTotalMotion < n_TotalMotionClips) {

			if (iter >= motion_refCoords[nTotalMotion].n_cols) {
				iter = 900; nTotalMotion++;
				std::cout << " " << nTotalMotion + 1 << " th motion will start" << std::endl;
			}
			else {
				avatar_Group[nTotalMotion]->setFromCompactCoordArray(motion_refCoords[nTotalMotion].col(iter));
				avatar_Group[nTotalMotion]->updateKinematicsUptoPos();
				avatar_Group[nTotalMotion]->updateKinematicBodiesOfCharacterSim();

				//
				// Task plane
				//-- 1.center position of hands / character height (midHandPos)
				gLink* gLink_rHand = avatar_Group[nTotalMotion]->findLink("RHand"); gLink* gLink_lHand = avatar_Group[nTotalMotion]->findLink("LHand");
				gVec3 rHand = gLink_rHand->frame().multVec4(gLink_rHand->inertia().comInVec3());
				gVec3 lHand = gLink_lHand->frame().multVec4(gLink_lHand->inertia().comInVec3());
				// center position
				gVec3 dirR2L = lHand - rHand; double mag = dirR2L.magnitude();
				mag = mag / 2;	dirR2L.normalize();
				gVec3 pos_CenterHand = rHand + (dirR2L * mag);

				// make ±âÁØ frame
				gXMat_standard_frame = avatar_Group[nTotalMotion]->baseLink()->frame();

				gVec3 gP_standard = gXMat_standard_frame.trn(); gP_standard.setY(0.0);
				gVec3 gV_upVector(0, 1, 0);
				gVec3 gV_rotXofPelvis = gXMat_standard_frame.rotX(); gV_rotXofPelvis.setY(0);
				gVec3 gV_rotZ = gV_rotXofPelvis % gV_upVector;

				gXMat_standard_frame.setTrn(gP_standard);
				gRotMat rot_standard;
				rot_standard.setColumn(0, gV_rotXofPelvis);
				rot_standard.setColumn(1, gV_upVector);
				rot_standard.setColumn(2, gV_rotZ);
				gXMat_standard_frame.setRot(rot_standard);


				debugGroup->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(gXMat_standard_frame.trn()), 5.0));

				gOSGShape::setColor(osg::Vec4(0, 1, 0, 1));
				gVec3 gP_upVector = gP_standard + gXMat_standard_frame.rotY()*5.0;
				debugGroup->addChild(gOSGShape::createLineShape(gVec3_2_OsgVec(gP_standard), gVec3_2_OsgVec(gP_upVector), 3.0));

				gOSGShape::setColor(osg::Vec4(1, 0, 0, 1));
				gVec3 gP_rotX = gP_standard + gXMat_standard_frame.rotX()*5.0;
				debugGroup->addChild(gOSGShape::createLineShape(gVec3_2_OsgVec(gP_standard), gVec3_2_OsgVec(gP_rotX), 3.0));

				gOSGShape::setColor(osg::Vec4(0, 0, 1, 1));
				gVec3 gP_rotZ = gP_standard + gXMat_standard_frame.rotZ()*5.0;
				debugGroup->addChild(gOSGShape::createLineShape(gVec3_2_OsgVec(gP_standard), gVec3_2_OsgVec(gP_rotZ), 3.0));

				pos_CenterHand = gXMat_standard_frame.invMultVec4(pos_CenterHand);

				/*if (pos_CenterHand.z() < 15) {

				std::cout << " frame from lessHand frame " << pos_CenterHand.z() << " motion " << nTotalMotion << " frame " << iter << std::endl;
				pos_CenterHand = gXMat_standard_frame.multVec4(pos_CenterHand);
				gOSGShape::setColor(osg::Vec4(1, 0, 0, 1));
				debugGroup->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(pos_CenterHand), 20.0));
				}*/
				//--1.Foot position
				gLink* FootR = avatar_Group[nTotalMotion]->findLink("RFoot");	gLink* FootL = avatar_Group[nTotalMotion]->findLink("LFoot");
				gVec3 rFoot = FootR->frame().multVec4(FootR->inertia().comInVec3());
				gVec3 lFoot = FootL->frame().multVec4(FootL->inertia().comInVec3());

				dirR2L = lFoot - rFoot; mag = dirR2L.magnitude();
				mag = mag / 2;	dirR2L.normalize();
				gVec3 pos_CenterFoot = rFoot + (dirR2L * mag);
				pos_CenterFoot = gXMat_standard_frame.invMultVec4(pos_CenterFoot);

				/*std::cout << " frame from lesscenter foot " << pos_CenterFoot.z() << " x " << pos_CenterFoot.x() << std::endl;
				pos_CenterFoot = avatar_Group[nTotalMotion]->baseLink()->frame().multVec4(pos_CenterFoot);
				gOSGShape::setColor(osg::Vec4(1, 0, 0, 1));
				debugGroup->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(pos_CenterFoot), 20.0));*/

				if (pos_CenterFoot.z() > -30 && pos_CenterHand.z() > 15) {
					writeCSVFile_TaskPlane(avatar_length[nTotalMotion], avatar_Group[nTotalMotion], first_bool_TaskPlane);
					nFnt++;
				}

				//


				debugGroup->addChild(avatarVis_Group[nTotalMotion]->getOSGGroup());
				avatarVis_Group[nTotalMotion]->update();

				iter++;
			}
		}
		else {
			first_bool_TaskPlane = true;
			std::cout << " we can't see the else " << std::endl;
			myfile_TaskPlane.close();

			

		}

		simulationTime += 1. / 30.;


	}
	return 0;


}