
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
#include "../TAE12/TAE12IK/mw_tiUtil_IK.h"

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

CoordinateType** tar;
std::vector<arma::vec> tar_motions;

std::vector<arma::vec> motions;

gOsgSkin* skin;

gReal sigmoid_f(gReal x)
{
	return 1.0 / (1.0 + exp(-0.165*(x - 10.0)));
}

std::vector<arma::vec> motionWarping(bCharacter* character, const arma::vec p0, const arma::vec p1, arma::mat refCoord)
{
	// return value
	std::vector<arma::vec> warped_motion;

	gReal n = refCoord.n_cols;
	const unsigned int dof = character->dof();
	std::cout << " n " << n << " dof " << dof <<std::endl;
	//motion is sit to stand
	for (int i = 0; i < n; i++) {
		arma::vec n_pose = refCoord(0, i, arma::SizeMat(character->dof(), 1));

		warped_motion.push_back(n_pose);
	}

	//// reverse a motion, and store as arma vector form.
	//for (int i = n - 1; i >= 0; --i) {
	//	arma::vec n_pose = refCoord(0, i, arma::SizeMat(character->dof(), 1));
	//
	//	warped_motion.push_back(n_pose);
	//}

	// displacement between two poses
	// dp0, (start pose, start pose of motion)
	// dp1, (end pose, end pose of motion)
	arma::vec dp0; dp0.resize(character->dof());
	arma::vec dp1; dp1.resize(character->dof());

	for (int i = 0; i < dof; ++i) {
		dp0.memptr()[i] = warped_motion.front().memptr()[i] - p0.memptr()[i];
		dp1.memptr()[i] = warped_motion.back().memptr()[i] - p1.memptr()[i];
	}

	for (int i = 0; i < n; ++i) {
		if (i < 30) {
			gReal interp_rate = sigmoid_f(30 - i);	// linear interp. //(60.0 - i)/60.f);

			for (int j = 0; j < p0.size(); ++j) {
				warped_motion[i].memptr()[j] = warped_motion[i].memptr()[j] - dp0.memptr()[j] * interp_rate;
			}
		}
		else if (i > n - 30 - 1) {
			gReal interp_rate = sigmoid_f(i - (n - 30 - 1));	// linear interp. //((i- (n - 60 -1)) /60.));

			for (int j = 0; j < p0.size(); ++j) {
				warped_motion[i].memptr()[j] = warped_motion[i].memptr()[j] - dp1.memptr()[j] * interp_rate;
			}
		}
	}

	//// "Temporary" post-processing
	//// To fix foot contact, IK for feet should be implemented.
	//// 
	//character->setFromCompactCoordArray(p0);
	//character->updateKinematicsUptoPos();
	//character->updateKinematicBodiesOfCharacterSim();

	//const gXMat rf0 = character->findLink("mixamorig9:RightFoot")->frame();
	//const gXMat lf0 = character->findLink("mixamorig9:LeftFoot")->frame();


	//character->setFromCompactCoordArray(p1);
	//character->updateKinematicsUptoPos();
	//character->updateKinematicBodiesOfCharacterSim();

	//const gXMat rf1 = character->findLink("mixamorig9:RightFoot")->frame();
	//const gXMat lf1 = character->findLink("mixamorig9:LeftFoot")->frame();

	//for (int i = 0; i < n; ++i) {

	//	{
	//		character->setFromCompactCoordArray(warped_motion[i]);
	//		character->updateKinematicsUptoPos();
	//		character->updateKinematicBodiesOfCharacterSim();

	//		gXMat rf = character->findLink("mixamorig9:RightFoot")->frame();
	//		gXMat lf = character->findLink("mixamorig9:LeftFoot")->frame();

	//		gReal dfy = (rf1.trn().y() - rf.trn().y()) + (lf1.trn().y() - lf.trn().y());

	//		warped_motion[i].memptr()[1] = warped_motion[i].memptr()[1] + dfy * 0.5f;
	//	}
	//}

	return warped_motion;
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

	//get warping motion file
	MotionLoader loader;
	loader.loadMotionFile("SitToStand.fbx");
	mgData* motion = loader.getMotion();
	mgSkeleton* skeleton = loader.getSkeleton();
	
	// generate bCharacter from motion, and save as txt file
	
	bCharacter* src = new bCharacter();
	bCharacterSim* srcSim = new bCharacterSim(src);
	gBDOSGSystem* srcVis = new gBDOSGSystem();
	const double mass = 70.;
	mgSkeletonToBCharacter::saveToBCharacter(skeleton, "Src.txt", mass);
	// load bCharacter from txt file
	loadAvatarModelFromFile(src, srcSim, srcVis, "Src.txt", 1.0);
	// load bCharacter visualization class from bCharcter
	scene->addChild(srcVis->getOSGGroup());

	//// skin
	skin = new gOsgSkin(src);
	skin->loadSkin(scene, "MaleSittingPose.fbx");
	skin->hideSkin();

	// load motion (pose sequence) : nMotion is a total number of frame for motion data
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

	//init pose
	loader.loadMotionFile("MaleSittingPose.fbx");
	skeleton = loader.getSkeleton();
	motion = loader.getMotion();
	arma::vec pose_init;
	mgMBSUtil::getCoordArrayFromRawData(
		pose_init,
		src,
		skeleton,
		motion->motions[0]
	);
	//arma::vec pose_init = refCoord(0,0,arma::SizeMat(src->dof(),1));

	//end pose
	loader.loadMotionFile("StandingClap.fbx");
	skeleton = loader.getSkeleton();
	motion = loader.getMotion();
	arma::vec pose_end;
	mgMBSUtil::getCoordArrayFromRawData(
		pose_end,
		src,
		skeleton,
		motion->motions[0]
	);
	//arma::vec pose_end = refCoord(0, (motion->nMotion-1), arma::SizeMat(src->dof(), 1));

	//motion warping
	std::vector<arma::vec> warped_motion = motionWarping(src, pose_init, pose_end, refCoord);
	

	// update src motion ( do what we need )
	int nFnt = 0;
	while (!viewer->done())
	{

		viewer->frame(simulationTime);

		if (iter < 0)
			iter = 0;
		if (iter >= warped_motion.size())
			iter = 0;

		std::cout << " frame " << iter << std::endl;

		skin->unhideSkin();


		arma::vec pose = warped_motion[iter];

		src->setFromCompactCoordArray(pose);
		src->updateKinematicsUptoPos();
		src->updateKinematicBodiesOfCharacterSim();
		
		skin->updateSkin();
	
		srcVis->update();


		//simulationTime += 1. / 30.;
		//iter++;

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