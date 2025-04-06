#include "Controller.hpp"
#include "utils.cpp"
#include <iostream>
#include <stdlib.h>
//#include "dart/ArrowShape.hpp"

using namespace std;
using namespace dart::dynamics;
using namespace dart::simulation;

double body_angles[] = {0,0,0};
double startWalk = 0;
int sim0 = 1;

Controller::Controller(dart::dynamics::SkeletonPtr _robot,
		dart::simulation::WorldPtr _world)
: mRobot(_robot),
  mWorld(_world)
{
	assert(_robot != nullptr);

	ikGain = 20;



	// some useful pointers to robot limbs
	leftFoot = mRobot->getBodyNode("l_sole");
	rightFoot = mRobot->getBodyNode("r_sole"); 
	mBase = mRobot->getBodyNode("base_link");
	mTorso = mRobot->getBodyNode("body");

	supportFoot = LEFT;
	swingFootStartingPosition.resize(6);
	if (supportFoot == RIGHT) {
		mSupportFoot = rightFoot;
		mSwingFoot = leftFoot;
	} else {
		mSupportFoot = leftFoot;
		mSwingFoot = rightFoot;
	}

	balancePoint = TORSO;
        beheavior = WALK;




	indInitial = startWalk + (int)((singleSupportDuration + doubleSupportDuration)/mWorld->getTimeStep());  



	if (balancePoint == TORSO) {
		comInitialPosition = mBase->getCOM(mSupportFoot);
	} else {
		comInitialPosition = mRobot->getCOM(mSupportFoot);
	}


	double comTargetHeight = 0.84; //0.85
        CTH = comTargetHeight;

	//Initialization measurements

	balanceBasePos.resize(6);
	balanceBasePos << getRPY(mBase, mSupportFoot), comInitialPosition;  
	balanceBasePos(5) = comTargetHeight;
        balanceBasePos(3) = mSupportFoot->getCOM(mSupportFoot)(0);
	balanceFootPos.resize(6);
        swingFootStartingPosition << getRPY(mSwingFoot, mSupportFoot), mSwingFoot->getCOM(mSupportFoot);
	balanceFootPos = swingFootStartingPosition;

        PreviousOptTargetFootPose << 0.036, -0.15, 0.0;  // X,Y,Theta

        comInitialPosition(0) = 0.0;
        comInitialPosition(1) = -0.0859634;
        comInitialPosition(2) = CTH;
        
        PreviousOptCom << comInitialPosition(0),comInitialPosition(1),comInitialPosition(2);
        stepHeight = 0.022;
	singleSupportDuration = 0.3;
	doubleSupportDuration = 0.2;
        bool activateTimingAdaptation = false;

        // The following choice strongly simplifies the gait generation algorithm
        double prediction_horizon = 2*(singleSupportDuration+doubleSupportDuration);

	solver = new mpcSolver::MPCSolver(0.05, mWorld->getTimeStep(), prediction_horizon, comInitialPosition, CTH, singleSupportDuration, doubleSupportDuration, 0.30,
			0.08, 0.25, 0.15, 0.25, 0.1, 0.0, activateTimingAdaptation); //0.05, 0.2, 0.1, 0.15, 0.1, 0.0   0.05, 0.3, 0.13, 0.17, 0.1, 0.0
	/**/

        f_r = Eigen::VectorXd::Zero(4);
}

Controller::~Controller()
{
}
 
void Controller::update()
{ 

        if(sim0 == 2 && footstepCounter== 6 ){ 
        //std::cout << "Sim Frames " << mWorld->getSimFrames() << std::endl;
        mSwingFoot->addExtForce(solver->push);


        auto visualShapeNodes = mSwingFoot->getShapeNodesWith<VisualAspect>();


        if(visualShapeNodes.size() == 1u)
        {
        //assert(visualShapeNodes[2]->getShape() == mArrow);
        visualShapeNodes[0]->remove();
        }
        if(visualShapeNodes.size() == 2u)
        {
        //assert(visualShapeNodes[2]->getShape() == mArrow);
        visualShapeNodes[1]->remove();
        }
        if(visualShapeNodes.size() == 3u)
        {
        //assert(visualShapeNodes[2]->getShape() == mArrow);
        visualShapeNodes[2]->remove();
        }
        

        if(solver->push(0) != 0.0 || solver->push(1) != 0.0){
        tail_counter = 30;
        }

        if (solver->push(0) != 0.0){
        std::shared_ptr<ArrowShape> mArrow;

        ArrowShape::Properties arrow_properties;
        arrow_properties.mRadius = 0.05;
        Eigen::Vector3d tail_offset = Eigen::Vector3d(0.0, 0.0, 0.0);
        if(solver->push(0) > 0) tail_offset(0) = 0.65;
        if(solver->push(0) < 0) tail_offset(0) = -0.65;
        if(solver->push(1) > 0) tail_offset(1) = 0.65;
        if(solver->push(1) < 0) tail_offset(1) = -0.65;

        Eigen::Vector3d tail_pos = Eigen::Vector3d(-0.1, 0.0, 0.15) ;
        if(solver->push(0) < 0) tail_pos = Eigen::Vector3d(0.1, 0.0, 0.15) ;
        if(solver->push(1) < 0) tail_pos = Eigen::Vector3d(0.1, +0.1, 0.15) ;
        if(solver->push(1) > 0) tail_pos = Eigen::Vector3d(0.1, -0.1, 0.15) ;

        Eigen::Vector3d head_pos = tail_pos - tail_offset;

        mArrow = std::shared_ptr<ArrowShape>(new ArrowShape(
             Eigen::Vector3d(0.0, 0.0, 0.0),
             Eigen::Vector3d(0.2, 0.05, 0.05),
             arrow_properties, dart::Color::Red(1.0)));
        mArrow->setPositions(
            head_pos,
            tail_pos);
      
        mSwingFoot->createShapeNodeWith<VisualAspect>(mArrow);

        tail_counter = tail_counter-1;
        }

       

        }

        if(sim0 == 0 || sim0 == 1){
        //std::cout << "Sim Frames " << mWorld->getSimFrames() << std::endl;
        mTorso->addExtForce(solver->push);

        auto visualShapeNodes = mTorso->getShapeNodesWith<VisualAspect>();
        if(visualShapeNodes.size() == 3u)
        {
        //assert(visualShapeNodes[2]->getShape() == mArrow);
        visualShapeNodes[2]->remove();
        }

        if(solver->push(0) != 0.0 || solver->push(1) != 0.0){
        tail_counter = 110;
        }

        if (tail_counter >0 && mWorld->getSimFrames()>100){
        std::shared_ptr<ArrowShape> mArrow;
        ArrowShape::Properties arrow_properties;
        arrow_properties.mRadius = 0.05;
        Eigen::Vector3d tail_offset = Eigen::Vector3d(0.0, 0.0, 0.0);
        if(solver->push(0) > 0) tail_offset(0) = 0.65;
        if(solver->push(0) < 0) tail_offset(0) = -0.65;
        if(solver->push(1) > 0) tail_offset(1) = 0.65;
        if(solver->push(1) < 0) tail_offset(1) = -0.65;

        Eigen::Vector3d tail_pos = Eigen::Vector3d(-0.1, 0.0, 0.15) ;
        if(solver->push(0) < 0) tail_pos = Eigen::Vector3d(0.1, 0.0, 0.15) ;
        if(solver->push(1) < 0) tail_pos = Eigen::Vector3d(0.1, +0.1, 0.15) ;
        if(solver->push(1) > 0) tail_pos = Eigen::Vector3d(0.1, -0.1, 0.15) ;

        Eigen::Vector3d head_pos = tail_pos - tail_offset;

        mArrow = std::shared_ptr<ArrowShape>(new ArrowShape(
             Eigen::Vector3d(0.0, 0.0, 0.0),
             Eigen::Vector3d(0.2, 0.05, 0.05),
             arrow_properties, dart::Color::Red(1.0)));
        mArrow->setPositions(
            head_pos,
            tail_pos);
        mTorso->createShapeNodeWith<VisualAspect>(mArrow);
        tail_counter = tail_counter-1;
        }
        }

        if(sim0 == 2 && footstepCounter>10){
        //std::cout << "Sim Frames " << mWorld->getSimFrames() << std::endl;
        mTorso->addExtForce(solver->push);

        auto visualShapeNodes = mTorso->getShapeNodesWith<VisualAspect>();
        if(visualShapeNodes.size() == 3u)
        {
        //assert(visualShapeNodes[2]->getShape() == mArrow);
        visualShapeNodes[2]->remove();
        }

        if(solver->push(0) != 0.0 || solver->push(1) != 0.0){
        tail_counter = 110;
        }

        if (tail_counter >0 && mWorld->getSimFrames()>100){
        std::shared_ptr<ArrowShape> mArrow;
        ArrowShape::Properties arrow_properties;
        arrow_properties.mRadius = 0.05;
        Eigen::Vector3d tail_offset = Eigen::Vector3d(0.0, 0.0, 0.0);
        if(solver->push(0) > 0) tail_offset(0) = 0.65;
        if(solver->push(0) < 0) tail_offset(0) = -0.65;
        if(solver->push(1) > 0) tail_offset(1) = 0.65;
        if(solver->push(1) < 0) tail_offset(1) = -0.65;

        Eigen::Vector3d tail_pos = Eigen::Vector3d(-0.1, 0.0, 0.15) ;
        if(solver->push(0) < 0) tail_pos = Eigen::Vector3d(0.1, 0.0, 0.15) ;
        if(solver->push(1) < 0) tail_pos = Eigen::Vector3d(0.1, +0.1, 0.15) ;
        if(solver->push(1) > 0) tail_pos = Eigen::Vector3d(0.1, -0.1, 0.15) ;

        Eigen::Vector3d head_pos = tail_pos - tail_offset;

        mArrow = std::shared_ptr<ArrowShape>(new ArrowShape(
             Eigen::Vector3d(0.0, 0.0, 0.0),
             Eigen::Vector3d(0.2, 0.05, 0.05),
             arrow_properties, dart::Color::Red(1.0)));
        mArrow->setPositions(
            head_pos,
            tail_pos);
        mTorso->createShapeNodeWith<VisualAspect>(mArrow);
        tail_counter = tail_counter-1;
        }
        }

	Eigen::VectorXd qDot;

	if (beheavior == WALK) qDot = generateWalking();
	else qDot = generateBalance();

	//if (mWorld->getSimFrames()>=startWalk) qDot = generateWalking();
	//else qDot = generateBalance();	

	// Set the velocity of the floating base to zero
	for (int i=0; i<6; ++i){
		mRobot->setCommand(i, 0);
	}

	// Set  the velocity of each joint as per inverse kinematics
	for (int i=0; i<50; ++i){
		mRobot->setCommand(i+6,qDot(i));

	}

	qDotOld = qDot;
        //storeData();
        ArmSwing();
        //AnkleRegulation();
} 

void Controller::storeData() {

       Eigen::VectorXd COMPOS = Eigen::VectorXd::Zero(3);
        COMPOS = solver->getOptimalCoMPosition();
        Eigen::VectorXd COMVEL = Eigen::VectorXd::Zero(3);
        COMVEL = solver->getOptimalCoMVelocity();


        Eigen::VectorXd COMPOS_meas  = Eigen::VectorXd::Zero(3);
        COMPOS_meas = mTorso->getCOM();
        Eigen::VectorXd COMVEL_meas  = Eigen::VectorXd::Zero(3);
        COMVEL_meas = mTorso->getCOMLinearVelocity();
        Eigen::VectorXd FOOT_meas  = Eigen::VectorXd::Zero(3);
        FOOT_meas = mSupportFoot->getCOM();

        Eigen::VectorXd ZMPPOS = Eigen::VectorXd::Zero(3);
        ZMPPOS = solver->getOptimalZMPPosition();


        ofstream myfile;

        myfile.open ("./Data/x_RF.txt",ios::app);
        myfile << mSupportFoot->getCOM()(0) <<endl; 
        myfile.close();
        myfile.open ("./Data/y_RF.txt",ios::app);
        myfile << mSupportFoot->getCOM()(1) <<endl; 
        myfile.close();
        myfile.open ("./Data/x.txt",ios::app);
        myfile << COMPOS(0) <<endl; 
        myfile.close();
        myfile.open ("./Data/y.txt",ios::app);
        myfile << COMPOS(1) <<endl; 
        myfile.close();
        myfile.open ("./Data/xz.txt",ios::app);
        myfile << ZMPPOS(0) <<endl; 
        myfile.close();
        myfile.open ("./Data/yz.txt",ios::app);
        myfile << ZMPPOS(1) <<endl; 
        myfile.close();
        myfile.open ("./Data/x_m.txt",ios::app);
        myfile << COMPOS_meas(0) <<endl; 
        myfile.close();
        myfile.open ("./Data/y_m.txt",ios::app);
        myfile << COMPOS_meas(1) <<endl; 
        myfile.close();

        myfile.open ("./Data/xd.txt",ios::app);
        myfile << mRobot->getCOMLinearVelocity()(0) <<endl; 
        myfile.close();
        myfile.open ("./Data/yd.txt",ios::app);
        myfile << mRobot->getCOMLinearVelocity()(1) <<endl; 
        myfile.close();       
        myfile.open ("./Data/x_u_M.txt",ios::app);
        myfile << f_r(0) <<endl; 
        myfile.close();
        myfile.open ("./Data/x_u_m.txt",ios::app);
        myfile << f_r(1) <<endl; 
        myfile.close();
        myfile.open ("./Data/y_u_M.txt",ios::app);
        myfile << f_r(2) <<endl; 
        myfile.close();
        myfile.open ("./Data/y_u_m.txt",ios::app);
        myfile << f_r(3) <<endl; 
        myfile.close();
        myfile.open ("./Data/x_u.txt",ios::app);
        myfile << COMPOS(0)+COMVEL(0)/3.41739249 <<endl; 
        myfile.close();
        myfile.open ("./Data/y_u.txt",ios::app);
        myfile << COMPOS(1)+COMVEL(1)/3.41739249 <<endl; 
        myfile.close();
        myfile.open ("./Data/x_um.txt",ios::app);
        myfile << mTorso->getCOM(mSupportFoot)(0)+COMVEL_meas(0)/3.41739249 <<endl; 
        myfile.close();
        myfile.open ("./Data/y_um.txt",ios::app);
        myfile << mTorso->getCOM(mSupportFoot)(1)+COMVEL_meas(1)/3.41739249 <<endl;
        myfile.close();
}

Eigen::VectorXd Controller::generateWalking(){

	if (solver->supportFootHasChanged()) supportFoot = !supportFoot;

	if (supportFoot == RIGHT) {
		mSupportFoot = rightFoot;
		mSwingFoot = leftFoot;
	} else {
		mSupportFoot = leftFoot;
		mSwingFoot = rightFoot;
	}

	if (solver->supportFootHasChanged()) {
		swingFootStartingPosition << getRPY(mSwingFoot, mSupportFoot), mSwingFoot->getCOM(mSupportFoot);
		if (mWorld->getSimFrames()>0) indInitial = mWorld->getSimFrames();
		footstepCounter++;
	}

	// Retrieve positions and orientations relative to support foot
	Eigen::VectorXd comCurrentPosition;
	Eigen::VectorXd comCurrentVelocity;
	Eigen::VectorXd comCurrentAcceleration;
	if (balancePoint == TORSO) {
		comCurrentPosition = mTorso->getCOM(mSupportFoot);
		comCurrentVelocity = mTorso->getCOMLinearVelocity();
		comCurrentAcceleration = mTorso->getCOMLinearAcceleration(/*mSupportFoot*/);
	} else {
		comCurrentPosition = mRobot->getCOM(/*mSupportFoot*/) - mSupportFoot->getCOM();
		comCurrentVelocity = mRobot->getCOMLinearVelocity(/*mSupportFoot*/);
		comCurrentAcceleration = mRobot->getCOMLinearAcceleration(/*mSupportFoot*/);
	}

 

	Eigen::VectorXd actPosSwingFoot(6);
	Eigen::VectorXd actPosBase(6);
	actPosSwingFoot << getRPY(mSwingFoot, mSupportFoot), mSwingFoot->getCOM(mSupportFoot);
	actPosBase << getRPY(mBase, mSupportFoot), comCurrentPosition;

	Eigen::Vector3d zmpCurrentPosition = Eigen::Vector3d::Zero();



        //OptComVel_err << 0.0*(comCurrentVelocity-solver->getOptimalCoMVelocity());  //0.01



        bool widj_ref = false;

        double vx, vy, vth;

        
        if (sim0 == 1) {
        vx = 0.0;
        vy = 0.0;
        vth = 0.0;

        }
        if (sim0 == 0){
        if (footstepCounter<=3) {
        vx = 0.0;
        vy = 0.0;
        vth = 0.0;
        }else{
        vx = -0.1;  //0.2
        vy = -0.1;
        vth = 0.0;
        }

        }

        if (sim0 == 2){
        if (footstepCounter<=1) {
        vx = 0.0;
        vy = 0.0;
        vth = 0.0;
        }else{
        vx = 0.1;  //0.2
        vy = 0.0;
        vth = 0.0;
        }

        }


/**/
	// Compute the CoM prediction using MPC
	solver->solve(comCurrentPosition, comCurrentVelocity, comCurrentAcceleration, mSwingFoot->getTransform(mSupportFoot),
			supportFoot, mWorld->getTime()-startWalk*(mWorld->getTimeStep()), vx, vy, vth, widj_ref); //max vel is 0.16, max omega is 0.1



        f_r<<solver->getFeasibilityRegion();

        //Update SS and DS durations accordin to the Timing_Manager
        if (solver->Timing_Manager(0,1) == 1) singleSupportDuration = solver->Timing_Manager(0,2);
        else singleSupportDuration = solver->Timing_Manager(3,2);

        if (solver->Timing_Manager(0,1) == 0) doubleSupportDuration = solver->Timing_Manager(0,2);
        else doubleSupportDuration = solver->Timing_Manager(1,2);

	Eigen::Affine3d temp = mSwingFoot->getTransform(mSupportFoot);


        // Filter DesposSwingFoot to avoid shaky foot motion

        OptFootPositioning = solver->getOptimalFootsteps();


if (!(solver->supportFootHasChanged())) {
        if ((OptFootPositioning(3) - PreviousOptTargetFootPose(2))*(OptFootPositioning(3) - PreviousOptTargetFootPose(2)) < n_threshold )  OptFootPositioning(3) = PreviousOptTargetFootPose(2); // theta

        if ((OptFootPositioning(0) - PreviousOptTargetFootPose(0))*(OptFootPositioning(0) - PreviousOptTargetFootPose(0)) < n_threshold )  OptFootPositioning(0) = PreviousOptTargetFootPose(0); // x

        if ((OptFootPositioning(1) - PreviousOptTargetFootPose(1))*(OptFootPositioning(1) - PreviousOptTargetFootPose(1)) < n_threshold )  OptFootPositioning(1) = PreviousOptTargetFootPose(1); // y
}

        PreviousOptTargetFootPose <<  OptFootPositioning(0),OptFootPositioning(1),OptFootPositioning(3);

	// Assemble desired tasks
	Eigen::VectorXd desPosSwingFoot = getOmnidirectionalSwingFootTrajectoryMPC(OptFootPositioning,swingFootStartingPosition, actPosSwingFoot, stepHeight, indInitial);

  

        if (footstepCounter == 0){desPosSwingFoot = balanceFootPos;
        desPosSwingFoot(4) = -0.15;}

        Eigen::VectorXd OptComPos(3);
        OptComPos << solver->getOptimalCoMPosition();

        

	Eigen::VectorXd desPosBase(6);
	desPosBase << 0.0, 0.0, OptFootPositioning(3), OptComPos(0), OptComPos(1), OptComPos(2);  

        PreviousOptCom = OptComPos;


	// Compute inverse kinematics
	//return getJointVelocitiesQp(desPosBase, actPosBase, desPosSwingFoot, actPosSwingFoot);
        return getJointVelocitiesStacked(desPosBase, actPosBase, desPosSwingFoot, actPosSwingFoot);
}


Eigen::VectorXd Controller::generateBalance(){

if (mWorld->getSimFrames()==0){
	balanceBasePos << getRPY(mBase, mSupportFoot), mTorso->getCOM(mSupportFoot);  //mBase, mSupportFoot
	balanceBasePos(5) = CTH;
        balanceBasePos(3) = mSupportFoot->getCOM(mSupportFoot)(0);

        swingFootStartingPosition << getRPY(mSwingFoot, mSupportFoot), mSwingFoot->getCOM(mSupportFoot);

}

	// Retrieve positions and orientations relative to support foot
	Eigen::VectorXd comCurrentPosition;
	if (balancePoint == TORSO) {
		comCurrentPosition = mTorso->getCOM(mSupportFoot); //mBase
	} else {
		comCurrentPosition = mRobot->getCOM(mSupportFoot);
	}

	Eigen::VectorXd actPosSwingFoot(6);
	Eigen::VectorXd actPosBase(6);
	actPosSwingFoot << getRPY(mSwingFoot, mSupportFoot), mSwingFoot->getCOM(mSupportFoot);
	actPosBase << getRPY(mBase, mSupportFoot), comCurrentPosition;

	Eigen::VectorXd desPosBase(6);
	desPosBase = balanceBasePos;

	Eigen::VectorXd desPosSwingFoot(6);

	desPosSwingFoot = balanceFootPos;
        desPosSwingFoot(4) = -0.16;


if (false) {

// A Balanced Motion to test inverse Kinematics

if (mWorld->getSimFrames()>=400 && mWorld->getSimFrames()<=900){
        desPosBase(4) = desPosBase(4) + 0.12*sin(2*3.14*0.01*(mWorld->getSimFrames()-400)/10);
        desPosBase(5) = desPosBase(5) -  0.05*sin(2*3.14*0.01*(mWorld->getSimFrames()-400)/5);
         ArmSwing();
}
if (mWorld->getSimFrames()>=650 && mWorld->getSimFrames()<=750){
        desPosSwingFoot(5) = desPosSwingFoot(5) + 0.1*sin(2*3.14*0.01*(mWorld->getSimFrames()-650)/2);
        std::cout<< "MOVE" <<std::endl;
}

}

        
	// Compute inverse kinematics
	return getJointVelocitiesQp(desPosBase, actPosBase, desPosSwingFoot, actPosSwingFoot);
	//return getJointVelocitiesStacked(desPosBase, actPosBase, desPosSwingFoot, actPosSwingFoot);
}





Eigen::MatrixXd Controller::getTorsoAndSwfJacobian(){

	Eigen::MatrixXd Jacobian_supportToBase;

	if (balancePoint == TORSO) {
		Jacobian_supportToBase =  mRobot->getJacobian(mTorso,mSupportFoot) - mRobot->getJacobian(mSupportFoot,mSupportFoot); //(mBase,mSupportFoot)
	} else {
		Jacobian_supportToBase =  (mRobot->getCOMJacobian(mSupportFoot) - mRobot->getJacobian(mSupportFoot,mSupportFoot));
	}

        for (unsigned int i=0; i<44; i++){ 
        if (i!=5 || i!=6)  Jacobian_supportToBase.col(i).setZero();
    }

	Eigen::MatrixXd Jacobian_SupportToSwing =  mRobot->getJacobian(mSwingFoot,mSupportFoot) - mRobot->getJacobian(mSupportFoot,mSupportFoot);

	Eigen::MatrixXd Jacobian_tot_(12, 56);

	Jacobian_tot_ << Jacobian_supportToBase, Jacobian_SupportToSwing;

	Eigen::MatrixXd Jacobian_tot(12, 50);

	// Remove the floating base columns
	Jacobian_tot = Jacobian_tot_.block<12,50>(0, 6);

	return Jacobian_tot;
}

Eigen::VectorXd Controller::getJointVelocitiesQp(Eigen::VectorXd desider_pos_base, Eigen::VectorXd actPosBase,
		Eigen::VectorXd desider_pos_SwingFoot, Eigen::VectorXd actPosSwingFoot){

	int nVariables = 50;



	Eigen::VectorXd refPosture = initialConfiguration.segment(6,50);
	Eigen::VectorXd currentPosture = mRobot->getPositions().segment(6,50);

	double jointVelocitiesGain = 0.00000001;
        //jointVelocitiesGain = 0.001;
        jointVelocitiesGain = 0.000001;
        jointVelocitiesGain = 0.000000001;  //ok
        jointVelocitiesGain = 0.000000000000001;
        jointVelocitiesGain = 0.000000000000000001;
        //jointVelocitiesGain = 0.000000000000000000001;
        jointVelocitiesGain = 0.000000001;  //ok


	double postureGain = 0;
	Eigen::MatrixXd taskGain = Eigen::MatrixXd::Identity(12,12);

	// Torso Orientation
	taskGain(0,0) = 0.1;
	taskGain(1,1) = 0.1;//0.001;
	taskGain(2,2) = 0;//0.001;

	// CoM Position
	taskGain(3,3) = 10; //1  
	taskGain(4,4) = 10; //1  
	taskGain(5,5) = 10;  //1 0.1 

	// Swing Foot Orientation
	taskGain(6,6) = 5;  //0.01
	taskGain(7,7) = 5; //0.01
	taskGain(8,8) = 5;    //1

	// Swing Foot Position
	taskGain(9,9) = 5;  //10 5
	taskGain(10,10) = 5; //1  
	taskGain(11,11) = 1; //10  

	//std::cout << taskGain(4,4) << std::endl;

        taskGain = taskGain;

	Eigen::VectorXd desired_pos(12);
	desired_pos << desider_pos_base, desider_pos_SwingFoot;
	Eigen::VectorXd actual_pos(12);
	actual_pos << actPosBase, actPosSwingFoot;
/*
        Eigen::VectorXd ComVref = Eigen::VectorXd::Zero(12);
        ComVref<<0.0,0.0,0.0,solver->getOptimalCoMVelocity(), 0.0,0.0,0.0,0.0,0.0,0.0;
        mTorso->getCOMLinearVelocity()/**/
	// Cost Function
	Eigen::MatrixXd Jacobian_tot = getTorsoAndSwfJacobian();
	Eigen::MatrixXd costFunctionH = mWorld->getTimeStep()*mWorld->getTimeStep()*Jacobian_tot.transpose()*taskGain*Jacobian_tot +
			postureGain*mWorld->getTimeStep()*mWorld->getTimeStep()*Eigen::MatrixXd::Identity(nVariables,nVariables) +
			jointVelocitiesGain*Eigen::MatrixXd::Identity(nVariables,nVariables);

	Eigen::VectorXd costFunctionF = mWorld->getTimeStep()*Jacobian_tot.transpose()*(taskGain*(actual_pos - desired_pos)) + postureGain*mWorld->getTimeStep()*(currentPosture - refPosture);


	// Constraint RHipYawPitch and LHipYawPitch to be at the same angle
	Eigen::MatrixXd AHip = Eigen::MatrixXd::Zero(1,nVariables);
	//AHip(0,2) = -mWorld->getTimeStep();
	//AHip(0,13) = mWorld->getTimeStep();
	//Eigen::VectorXd bHip(1);
	//bHip(0) = mRobot->getPosition(8) - mRobot->getPosition(19);

	// Constraint max joint accelerations
	double maxAcc = 5;
	Eigen::MatrixXd AAcc = Eigen::MatrixXd::Identity(nVariables,nVariables);
	Eigen::VectorXd bAccMax = 0*qDotOld + Eigen::VectorXd::Ones(nVariables)*maxAcc;
	Eigen::VectorXd bAccMin = 0*qDotOld - Eigen::VectorXd::Ones(nVariables)*maxAcc;

	// Stack Constraints
	//Eigen::MatrixXd A;
	//Eigen::VectorXd bMin;
	//Eigen::VectorXd bMax;
	//A.resize(1+nVariables,nVariables);
	//bMin.resize(1+nVariables);
	//bMax.resize(1+nVariables);
	//std::cout << "bo" << std::endl;
	//A << AHip, AAcc;
	//bMin << bHip, bAccMin;
	//bMax << bHip, bAccMax;

	// Solve the QP
	Eigen::VectorXd solution = solveQP(costFunctionH, costFunctionF, 0*AAcc, 0*bAccMax, 0*bAccMin);
	return solution;
}

Eigen::VectorXd Controller::getJointVelocitiesStacked(Eigen::VectorXd desider_pos_base, Eigen::VectorXd actPosBase,
		Eigen::VectorXd desider_pos_SwingFoot, Eigen::VectorXd actPosSwingFoot){

        Eigen::VectorXd ComVref = Eigen::VectorXd::Zero(12);
        ComVref<<0.0,0.0,0.0,solver->getOptimalCoMVelocity(), 0.0,0.0,0.0,0.0,0.0,0.0;
     

	Eigen::VectorXd desired_pos(12);
	desired_pos << desider_pos_base, desider_pos_SwingFoot;

	// Assemble actual positions and orientations
	Eigen::VectorXd actual_pos(12);
	actual_pos << actPosBase, actPosSwingFoot;

	// Get the proper jacobian and pseudoinvert it
	Eigen::MatrixXd Jacobian_tot = getTorsoAndSwfJacobian();
	Eigen::MatrixXd PseudoJacobian_tot = (Jacobian_tot.transpose())*(Jacobian_tot*Jacobian_tot.transpose()).inverse();

	Eigen::MatrixXd _taskGain = Eigen::MatrixXd::Identity(12,12);

/*
	// Torso Orientation
	_taskGain(0,0) = 0.1;
	_taskGain(1,1) = 0.1;//0.001;
	_taskGain(2,2) = 0;//0.001;

	// CoM Position
	_taskGain(3,3) = 1;  //0.1  10
	_taskGain(5,5) = 1;
	_taskGain(5,5) = 0.01;

	// Swing Foot Orientation
	_taskGain(6,6) = 10;
	_taskGain(7,7) = 10;
	_taskGain(8,8) = 10;

	// Swing Foot Position
	_taskGain(9,9) = 10;
	_taskGain(10,10) = 10;
	_taskGain(11,11) = 10;


        ikGain = 10;
/**/


        //// VERY GOOD
	// Torso Orientation
	_taskGain(0,0) = 1;
	_taskGain(1,1) = 1;//0.001;
	_taskGain(2,2) = 1;//0;

	// CoM Position
	_taskGain(3,3) = 10;  //0.1  10
	_taskGain(4,4) = 10;
	_taskGain(5,5) = 10;

	// Swing Foot Orientation
	_taskGain(6,6) = 10;
	_taskGain(7,7) = 10;
	_taskGain(8,8) = 10;

	// Swing Foot Position
	_taskGain(9,9) = 10;
	_taskGain(10,10) = 10;
	_taskGain(11,11) = 10;


        ikGain = 10;

	Eigen::VectorXd qDot(50);
	qDot = PseudoJacobian_tot*(ComVref+ikGain*_taskGain*(desired_pos - actual_pos));

	return qDot;
}

Eigen::VectorXd Controller::getJointVelocitiesStacked(Eigen::VectorXd desider_vel_base, Eigen::VectorXd actVelBase,
		Eigen::VectorXd desider_pos_base, Eigen::VectorXd actPosBase,
		Eigen::VectorXd desider_vel_SwingFoot, Eigen::VectorXd actVelSwingFoot,
		Eigen::VectorXd desider_pos_SwingFoot, Eigen::VectorXd actPosSwingFoot){

	Eigen::VectorXd desired_pos(12);
	Eigen::VectorXd desired_vel(12);
	desired_pos << desider_pos_base, desider_pos_SwingFoot;
	desired_vel << desider_vel_base, desider_vel_SwingFoot;

	// Assemble actual positions and orientations
	Eigen::VectorXd actual_pos(12);
	actual_pos << actPosBase, actPosSwingFoot;

	// Get the proper jacobian and pseudoinvert it
	Eigen::MatrixXd Jacobian_tot = getTorsoAndSwfJacobian();
	Eigen::MatrixXd PseudoJacobian_tot = (Jacobian_tot.transpose())*(Jacobian_tot*Jacobian_tot.transpose()).inverse();

	Eigen::VectorXd qDot(24);
	qDot = PseudoJacobian_tot*(desired_vel + ikGain*(desired_pos - actual_pos));

	return qDot;
}

Eigen::VectorXd Controller::getJointVelocitiesPrioritized(Eigen::VectorXd desider_vel_base, Eigen::VectorXd actVelBase,
		Eigen::VectorXd desider_pos_base, Eigen::VectorXd actPosBase,
		Eigen::VectorXd desider_vel_SwingFoot, Eigen::VectorXd actVelSwingFoot,
		Eigen::VectorXd desider_pos_SwingFoot, Eigen::VectorXd actPosSwingFoot){

	Eigen::MatrixXd JacobianStS_ = -(mRobot->getJacobian(mSupportFoot) - mRobot->getJacobian(mSwingFoot));
	Eigen::MatrixXd JacobianStB_ = (mRobot->getJacobian(mRobot->getBodyNode("base_link")) - mRobot->getJacobian(mSupportFoot));
	Eigen::MatrixXd JacobianStS = JacobianStS_.block<6,24>(0,6);
	Eigen::MatrixXd JacobianStB = JacobianStB_.block<6,24>(0,6);

	Eigen::MatrixXd PinvStS = (JacobianStS.transpose())*(JacobianStS*JacobianStS.transpose()).inverse();
	Eigen::MatrixXd PinvStB = (JacobianStB.transpose())*(JacobianStB*JacobianStB.transpose()).inverse();

	Eigen::VectorXd qDot_base(24);
	Eigen::VectorXd qDot(24);

	double KP = 10;
	double KD = 0.5;

	qDot_base = PinvStB*(desider_vel_base+KP*(desider_pos_base-actPosBase)+KD*(desider_vel_base-actVelBase));
	qDot = PinvStS*(desider_vel_SwingFoot+KP*(desider_pos_SwingFoot-actPosSwingFoot)+KD*(desider_vel_SwingFoot-actVelSwingFoot))
					+(Eigen::MatrixXd::Identity(24,24)-PinvStS*JacobianStS)*qDot_base;

	return qDot;
}

Eigen::Vector3d Controller::getRPY(dart::dynamics::BodyNode* body, dart::dynamics::BodyNode* referenceFrame) {

	Eigen::MatrixXd rotMatrix = body->getTransform(referenceFrame).rotation();

	Eigen::Vector3d RPY;
	RPY << atan2(rotMatrix(2,1),rotMatrix(2,2)),
			atan2(-rotMatrix(2,0),sqrt(rotMatrix(2,1)*rotMatrix(2,1)+rotMatrix(2,2)*rotMatrix(2,2))),
			atan2(rotMatrix(1,0),rotMatrix(0,0));

	return RPY;
	//return body->getTransform(referenceFrame).rotation().eulerAngles(0,1,2);
}

Eigen::Vector3d Controller::getZmpFromExternalForces()
{
	// external forces block

	std::vector<double> zmp_v;
	bool left_contact=false;
	bool right_contact =false;
	dart::dynamics::BodyNode* left_foot = mRobot->getBodyNode("l_sole");
	Eigen::Vector3d left_cop;



	if(abs(left_foot->getConstraintImpulse()[5])>0.01){
		left_cop << -(left_foot)->getConstraintImpulse()(1)/(left_foot)->getConstraintImpulse()(5)  ,  (left_foot)->getConstraintImpulse()(0)/(left_foot)->getConstraintImpulse()(5),0.0;
		Eigen::Matrix3d iRotation = left_foot->getWorldTransform().rotation();
		Eigen::Vector3d iTransl   = left_foot->getWorldTransform().translation();
		left_cop = iTransl + iRotation*left_cop;
		left_contact = true;
	}

	dart::dynamics::BodyNode* right_foot = mRobot->getBodyNode("r_sole");
	Eigen::Vector3d right_cop;

	Eigen::Vector3d right_torque;
	right_torque = right_foot->getConstraintImpulse().segment(0,3);
	right_torque = right_foot->getWorldTransform().rotation()*right_torque;


	if(abs(right_foot->getConstraintImpulse()[5])>0.01){
		right_cop << -(right_foot)->getConstraintImpulse()(1)/(right_foot)->getConstraintImpulse()(5),(right_foot)->getConstraintImpulse()(0)/(right_foot)->getConstraintImpulse()(5),0.0;
		Eigen::Matrix3d iRotation = right_foot->getWorldTransform().rotation();
		Eigen::Vector3d iTransl   = right_foot->getWorldTransform().translation();
		right_cop = iTransl + iRotation*right_cop;
		right_contact = true;
	}

	if(left_contact && right_contact){
		zmp_v.push_back((left_cop(0)*left_foot->getConstraintImpulse()[5] + right_cop(0)*right_foot->getConstraintImpulse()[5])/(left_foot->getConstraintImpulse()[5] + right_foot->getConstraintImpulse()[5]) );
		zmp_v.push_back((left_cop(1)*left_foot->getConstraintImpulse()[5] + right_cop(1)*right_foot->getConstraintImpulse()[5])/(left_foot->getConstraintImpulse()[5] + right_foot->getConstraintImpulse()[5]) );
		zmp_v.push_back(0.0);
	}else if(left_contact){
		zmp_v.push_back(left_cop(0));
		zmp_v.push_back(left_cop(1));
		zmp_v.push_back(0.0);
	}else if(right_contact){
		zmp_v.push_back(right_cop(0));
		zmp_v.push_back(right_cop(1));
		zmp_v.push_back(0.0);
	}else{
		// No contact detected
		zmp_v.push_back(0.0);
		zmp_v.push_back(0.0);
		zmp_v.push_back(0.0);
	}

        std::cout << "right_foot->getConstraintImpulse() "<< right_foot->getConstraintImpulse() <<std::endl;
        std::cout << "left_foot->getConstraintImpulse() "<< right_foot->getConstraintImpulse() <<std::endl;

	Eigen::Vector3d returnZmp;
	returnZmp << zmp_v[0], zmp_v[1], zmp_v[2];

	return returnZmp;

}

Eigen::Vector3d Controller::getZmpFromWrench(){
	std::vector<double> zmp_est;
	const dart::collision::CollisionResult& col_res = mWorld->getLastCollisionResult();
	std::vector<dart::collision::Contact> contacts = col_res.getContacts();

	int contact_counter = 1;
	double x_zmp =0.0;
	double y_zmp =0.0;
	double z_zmp =0.0;
	double total_fz = 0;
	if (contacts.size() > 0) {
		for (auto cont : contacts) {

			contact_counter = contact_counter + 1;

			double contactForce;
			if (cont.force.z() < 0) contactForce = 0;
			else contactForce = cont.force.z();

			x_zmp = x_zmp + cont.point.x()*contactForce;
			y_zmp = y_zmp + cont.point.y()*contactForce;
			z_zmp = z_zmp + cont.point.z()*contactForce;
			total_fz = total_fz + contactForce;

		}
	}
	if(total_fz!=0){
		x_zmp = x_zmp/total_fz;
		y_zmp = y_zmp/total_fz;
		z_zmp = z_zmp/total_fz;
	}else{
		x_zmp = 0.0;
		y_zmp = 0.0;
		z_zmp = 0.0;
	}

	zmp_est.push_back(x_zmp);
	zmp_est.push_back(y_zmp);
	zmp_est.push_back(z_zmp);

	Eigen::Vector3d streamable;
	streamable << zmp_est[0], zmp_est[1], zmp_est[2];

	return streamable;
}

Eigen::Vector3d Controller::getZmpFromAngularMomentum(){
	Eigen::Vector3d totalMom;
	Eigen::Vector3d realZMP;

	totalMom.resize(3);
	totalMom.setZero();

	realZMP.resize(3);
	realZMP.setZero();

	Eigen::Vector3d CoMPosition = mRobot->getCOM();
	Eigen::Vector3d CoMAcceleration = mRobot->getCOMLinearAcceleration();

	Eigen::VectorXd oldMom = totalMom;

	totalMom.setZero();

	for (unsigned int i=0; i<mRobot->getNumBodyNodes(); i++){
		Eigen::Vector3d LinearMom = (mRobot->getBodyNode(i))->getLinearMomentum();
		Eigen::Vector3d AngularMom = (mRobot->getBodyNode(i))->getAngularMomentum();

		Eigen::Vector3d iCoMPosition = (mRobot->getBodyNode(i))->getCOM();
		Eigen::Matrix3d iRotation = (mRobot->getBodyNode(i))->getWorldTransform().rotation();

		// transform the momentums in world frame
		LinearMom = iRotation*LinearMom;
		AngularMom = iRotation*AngularMom;

		Eigen::Vector3d d2;
		d2 << (iCoMPosition(0) - CoMPosition(0))*(iCoMPosition(0) - CoMPosition(0)),
				(iCoMPosition(1) - CoMPosition(1))*(iCoMPosition(1) - CoMPosition(1)),
				(iCoMPosition(2) - CoMPosition(2))*(iCoMPosition(2) - CoMPosition(2));

		totalMom = totalMom + (iCoMPosition-CoMPosition).cross(LinearMom);// + AngularMom + (mRobot->getBodyNode(i)->getMass())*d2;
	}

	Eigen::VectorXd momDot = (totalMom - oldMom)/0.01;

	realZMP << CoMPosition(0) - (CoMPosition(2)/(9.8-0*CoMAcceleration(2))) * CoMAcceleration(0)
			    		 + (1/(5*(9.8-0*CoMAcceleration(2)))) * momDot(1),
						 CoMPosition(1) - (CoMPosition(2)/(9.8-0*CoMAcceleration(2))) * CoMAcceleration(1)
						 + (1/(5*(9.8-0*CoMAcceleration(2)))) * momDot(0),
						 0;

	return realZMP;
}

dart::dynamics::SkeletonPtr Controller::getRobot() const {
	return mRobot;
}

dart::dynamics::BodyNode* Controller::getSupportFoot() {
	return mSupportFoot;
}

mpcSolver::MPCSolver* Controller::getSolver() {
	return solver;
}

Eigen::VectorXd Controller::getEndEffector() const {
	return mRobot->getCOM();
}

void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {

}


Eigen::VectorXd Controller::getOmnidirectionalSwingFootTrajectoryMPC(Eigen::VectorXd targetFootstepPosition, Eigen::VectorXd swingFootStartingPosition,
		Eigen::VectorXd swingFootActualPosition, double stepHeight, int indInitial){

	double delta = mWorld->getTimeStep();
	int ind = mWorld->getSimFrames();

	double time=ind*delta;
	double ti=indInitial*delta;

	int S = (int)(singleSupportDuration/delta);
	int D = (int)(doubleSupportDuration/delta);

	// Generate XY polynomial for single support
	Eigen::VectorXd boundaryConditions(6);
	Eigen::MatrixXd polyMatrix(6,6);
	double dur = delta*S;

	polyMatrix << 0,0,0,0,0,1,
			pow(dur,5),pow(dur,4),pow(dur,3),pow(dur,2),pow(dur,1),1,
			0,0,0,0,1,0,
			5*pow(dur,4),4*pow(dur,3),3*pow(dur,2),2*pow(dur,1),1,0,
			0,0,0,2,0,0,
			5*4*pow(dur,3),4*3*pow(dur,2),3*2*pow(dur,1),2,0,0;

	boundaryConditions << 0,1,0,0,0,0;
	Eigen::VectorXd coeffs = polyMatrix.inverse()*boundaryConditions;

	double t = (ind-indInitial)*delta;
	double polyInT = coeffs(0)*pow(t,5) + coeffs(1)*pow(t,4) + coeffs(2)*pow(t,3) + coeffs(3)*pow(t,2) + coeffs(4)*pow(t,1) + coeffs(5);
//	double polyDotInT = 5*coeffs(0)*pow(t,4) + 4*coeffs(1)*pow(t,3) + 3*coeffs(2)*pow(t,2) + 2*coeffs(3)*pow(t,1) + coeffs(4);

	Eigen::VectorXd swingFootPosition = Eigen::VectorXd::Zero(6);

	
	if (ind<indInitial){
		swingFootPosition(3) = swingFootStartingPosition(3);
		swingFootPosition(4) = swingFootStartingPosition(4);

	} else if (ind>=indInitial+S || (ind>=indInitial+S/2 && swingFootActualPosition(5)<=0) ){
		swingFootPosition(3) = targetFootstepPosition(0);
		swingFootPosition(4) = targetFootstepPosition(1);

		swingFootPosition(2) = targetFootstepPosition(3);

	} else {
		swingFootPosition(3) = swingFootStartingPosition(3) + (targetFootstepPosition(0) - swingFootStartingPosition(3))*polyInT;
		swingFootPosition(4) = swingFootStartingPosition(4) + (targetFootstepPosition(1) - swingFootStartingPosition(4))*polyInT;
		swingFootPosition(5) = -(4*stepHeight/pow((S*delta),2))*(time-ti)*(time-ti-S*delta);

		swingFootPosition(2) = targetFootstepPosition(3)*polyInT;

		/*swingFootPosition(3) = swingFootActualPosition(3) + (targetFootstepPosition(0) - swingFootActualPosition(3))*0.5;
		swingFootPosition(4) = swingFootActualPosition(4) + (targetFootstepPosition(1) - swingFootActualPosition(4))*0.5;
		swingFootPosition(5) = -(4*stepHeight/pow((S*delta),2))*(time-ti)*(time-ti-S*delta);

		swingFootPosition(2) = targetFootstepPosition(3)*polyInT;*/
	}

	return swingFootPosition;
}

void Controller::setInitialConfiguration() {

/*
  std::cout <<"L_HIP_Y " << mRobot->getDof("L_HIP_Y")->getIndexInSkeleton()<< std::endl;
  std::cout <<"L_HIP_R " << mRobot->getDof("L_HIP_R")->getIndexInSkeleton()<< std::endl;
  std::cout <<"L_HIP_P " << mRobot->getDof("L_HIP_P")->getIndexInSkeleton()<< std::endl;
  std::cout <<"L_KNEE_P " << mRobot->getDof("L_KNEE_P")->getIndexInSkeleton()<< std::endl;
  std::cout <<"L_ANKLE_P " << mRobot->getDof("L_ANKLE_P")->getIndexInSkeleton()<< std::endl;
  std::cout <<"L_ANKLE_R " << mRobot->getDof("L_ANKLE_R")->getIndexInSkeleton()<< std::endl;
  std::cout <<"R_HIP_Y " << mRobot->getDof("R_HIP_Y")->getIndexInSkeleton()<< std::endl;
  std::cout <<"R_HIP_R " << mRobot->getDof("R_HIP_R")->getIndexInSkeleton()<< std::endl;
  std::cout <<"R_HIP_P " << mRobot->getDof("R_HIP_P")->getIndexInSkeleton()<< std::endl;
  std::cout <<"R_KNEE_P " << mRobot->getDof("R_KNEE_P")->getIndexInSkeleton()<< std::endl;
  std::cout <<"R_ANKLE_P " << mRobot->getDof("R_ANKLE_P")->getIndexInSkeleton()<< std::endl;
  std::cout <<"R_ANKLE_R " << mRobot->getDof("R_ANKLE_R")->getIndexInSkeleton()<< std::endl;
std::cout <<"CHEST_P " << mRobot->getDof("CHEST_P")->getIndexInSkeleton()<< std::endl;
std::cout <<"CHEST_Y " << mRobot->getDof("CHEST_Y")->getIndexInSkeleton()<< std::endl;/**/

  Eigen::VectorXd q = mRobot->getPositions();
  //std::cout <<q.size()<< std::endl;

// Floating Base
  q[0] = 0.0;
  q[1] = 4*M_PI/180;
  q[2] = 0.0;
  q[3] = 0.0;
  q[4] = 0.0;
  q[5] = 0.753;

// Right Leg
  q[44] = 0.0;            // hip yaw
  q[45] = 3*M_PI/180;    // hip roll
  q[46] = -25*M_PI/180;   // hip pitch
  q[47] = 50*M_PI/180;   // knee pitch
  q[48] = -30*M_PI/180; // ankle pitch
  q[49] = -4*M_PI/180;   // ankle roll         
// Left Leg
  q[50] = 0.0;            // hip yaw
  q[51] = -3*M_PI/180;    // hip roll
  q[52] = -25*M_PI/180;   // hip pitch
  q[53] = 50*M_PI/180;   // knee pitch
  q[54] = -30*M_PI/180; // ankle pitch
  q[55] = 4*M_PI/180;   // ankle roll       
/**/


  mRobot->setPositions(q);

mRobot->setPosition(mRobot->getDof("R_SHOULDER_P")->getIndexInSkeleton(), (4)*M_PI/180 );
mRobot->setPosition(mRobot->getDof("R_SHOULDER_R")->getIndexInSkeleton(), -8*M_PI/180  );
mRobot->setPosition(mRobot->getDof("R_SHOULDER_Y")->getIndexInSkeleton(), 0 );

mRobot->setPosition(mRobot->getDof("R_ELBOW_P")->getIndexInSkeleton(), -25*M_PI/180 );

mRobot->setPosition(mRobot->getDof("L_SHOULDER_P")->getIndexInSkeleton(), (4)*M_PI/180  );
mRobot->setPosition(mRobot->getDof("L_SHOULDER_R")->getIndexInSkeleton(), 8*M_PI/180  );
mRobot->setPosition(mRobot->getDof("L_SHOULDER_Y")->getIndexInSkeleton(), 0 );

mRobot->setPosition(mRobot->getDof("L_ELBOW_P")->getIndexInSkeleton(), -25*M_PI/180 ); 


}

void Controller::ArmSwing() {

// TO DO: add variable period to the swing trajecotry
mRobot->setPosition(mRobot->getDof("R_SHOULDER_P")->getIndexInSkeleton(), (4+5*sin(2*M_PI*0.01*(mWorld->getSimFrames())))*M_PI/180 );
mRobot->setPosition(mRobot->getDof("L_SHOULDER_P")->getIndexInSkeleton(), (4-5*sin(2*M_PI*0.01*(mWorld->getSimFrames())))*M_PI/180  );


}

void Controller::AnkleRegulation() {

// right ankle pitch
mRobot->setPosition(mRobot->getDof("R_ANKLE_P")->getIndexInSkeleton(), (mRobot->getPosition(mRobot->getDof("R_ANKLE_P")->getIndexInSkeleton())));
// right ankle roll
mRobot->setPosition(mRobot->getDof("R_ANKLE_R")->getIndexInSkeleton(), (mRobot->getPosition(mRobot->getDof("R_ANKLE_R")->getIndexInSkeleton())));
// left ankle pitch
mRobot->setPosition(mRobot->getDof("L_ANKLE_P")->getIndexInSkeleton(), (mRobot->getPosition(mRobot->getDof("L_ANKLE_P")->getIndexInSkeleton())));
// left ankle roll
mRobot->setPosition(mRobot->getDof("L_ANKLE_R")->getIndexInSkeleton(), (mRobot->getPosition(mRobot->getDof("L_ANKLE_R")->getIndexInSkeleton())));
}


void Controller::setComTargetHeight(double height) {
	solver->setComTargetHeight(height);
}

void Controller::setReferenceVelocityX(double ref) {
	solver->setReferenceVelocityX(ref);
}

void Controller::setReferenceVelocityY(double ref) {
	solver->setReferenceVelocityY(ref);
}

void Controller::setReferenceVelocityOmega(double ref) {
	solver->setReferenceVelocityOmega(ref);
}

void Controller::setBalancePointCom() {
	balancePoint = COM;
}

void Controller::setBalancePointTorso() {
	balancePoint = TORSO;
}

void Controller::setBeheaviorBalance() {
	beheavior = BALANCE;
}

void Controller::setBeheaviorWalk() {
	beheavior = WALK;
}

void Controller::setVelGain(double value) {
	solver->setVelGain(value);
}

void Controller::setZmpGain(double value) {
	solver->setZmpGain(value);
}

Eigen::VectorXd Controller::getBalanceBasePos() {
	return balanceBasePos;
}

Eigen::VectorXd Controller::getBalanceFootPos() {
	return balanceFootPos;
}

void Controller::setBalanceBasePos(Eigen::VectorXd pos) {
	balanceBasePos = pos;
}

void Controller::setBalanceFootPos(Eigen::VectorXd pos) {
	balanceFootPos = pos;
}

