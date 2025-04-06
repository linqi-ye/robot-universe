#ifndef MPCSOLVER_HPP
#define MPCSOLVER_HPP
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "qpOASES/qpOASES.hpp"
#include <dart/dart.hpp>

namespace mpcSolver{

    class MPCSolver{
	public:
        MPCSolver(double, double, double, Eigen::Vector3d, double, double, double, double, double, double, double, double, double, double, bool);

        // Main method
        void solve(Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Affine3d, bool, double, double, double, double, bool);

        // Get stuff
        Eigen::VectorXd getOptimalCoMPosition();
        Eigen::VectorXd getOptimalCoMVelocity();
        Eigen::VectorXd getOptimalZMPPosition();
        Eigen::VectorXd getOptimalFootsteps();
        Eigen::Vector3d push;
        Eigen::MatrixXd getPredictedZmp();
        Eigen::VectorXd getFeasibilityRegion();
        bool supportFootHasChanged();
	double getOmega();

	// Set stuff
	void setComTargetHeight(double);
	void setReferenceVelocityX(double);
	void setReferenceVelocityY(double);
	void setReferenceVelocityOmega(double);
	void setVelGain(double);
	void setZmpGain(double);

        // Generate matrices
        void genCostFunction();
        void genStabilityConstraint();
        void genBalanceConstraint();
        void genFeasibilityConstraint();
        void genSwingFootConstraint(Eigen::Affine3d);
        void genUsefulMatrices();
        void ComputeFeasibilityRegion();
        void TimingAdaptation();
        void TimingAdaptation_euristics();

        // Solve
        void computeOrientations();
        Eigen::VectorXd solveQP();
        Eigen::VectorXd solveQPdart();

        // Update the state
        Eigen::Vector3d updateState(double,int,double);
        void changeReferenceFrame(Eigen::Affine3d);


        Eigen::MatrixXd Timing_Manager;

	private:

        // Constant parameters
        int N,S,D,M, footstepCounter;
        int CountDown = -100;
        double singleSupportDuration, doubleSupportDuration, thetaMax;
        double footConstraintSquareWidth;
        double deltaXMax;
        double deltaYIn;
        double deltaYOut;
        double mpcTimeStep;
        double controlTimeStep;
        double comTargetHeight;
        double omega;
        double measuredComWeight_x = 0.0;
        double measuredComWeight_y = 0.0;
        double measuredZmpWeight = 0;
        double measuredComWeight_v_x = 0.4;
        double measuredComWeight_v_y = 0.4;
        double v_x,v_y,v_th;
        double w_x, w_y;
        bool trig_x = true;
        bool trig_y = true;
        double InitCom = 0;
        int singlesupport = 1;
        int doublesupport = 0;
        bool activate_timing_adaptation;
        double ss_d, ds_d;
        bool widgetReference;
        double x_u_M, x_u_m, y_u_M, y_u_m;
        double c_k_x, c_k_y, d;
        double lambda_0, lambda_1, lambda_tot;
        double new_timing, new_timing_x, new_timing_y, t_MIN, t_MAX;
        double margin_x, margin_y;
        double xu_state, yu_state;

        // Parameters for the current iteration
        bool supportFoot;
        double simulationTime;
        double vRefX=0;
        double vRefY=0;
        double omegaRef=0;
        int mpcIter,controlIter;
        bool adaptSim;

        // useful matrices
	Eigen::MatrixXd Icf;
	Eigen::MatrixXd Ic;
	Eigen::MatrixXd Cc;
	Eigen::VectorXd Ccf;
	Eigen::MatrixXd rCosZmp;
	Eigen::MatrixXd rSinZmp;
	Eigen::MatrixXd _rCosZmp;
	Eigen::MatrixXd _rSinZmp;
	Eigen::MatrixXd zmpRotationMatrix;
        Eigen::VectorXd zDotOptimalX;
        Eigen::VectorXd zDotOptimalY;
        Eigen::VectorXd footstepsOptimalX;
        Eigen::VectorXd footstepsOptimalY;
        Eigen::MatrixXd A_timing;
        Eigen::VectorXd b_timing_max;
        Eigen::VectorXd b_timing_min;

        // Matrices for prediction
        Eigen::VectorXd p;
        Eigen::MatrixXd P;
        Eigen::MatrixXd Vu;
        Eigen::MatrixXd Vs;

        // Matrices for cost function
        Eigen::MatrixXd costFunctionH;
        Eigen::VectorXd costFunctionF;

        // Matrices for stability constraint
        Eigen::MatrixXd Aeq;
        Eigen::VectorXd beq;

        //Matrices for balance constraint
        Eigen::MatrixXd AZmp;
        Eigen::VectorXd bZmpMax;
        Eigen::VectorXd bZmpMin;

        // Matrices for feasibility constraints
        Eigen::MatrixXd AFootsteps;
        Eigen::VectorXd bFootstepsMax;
        Eigen::VectorXd bFootstepsMin;

        // Matrices for swing foot constraints
        Eigen::MatrixXd ASwingFoot;
        Eigen::VectorXd bSwingFoot;

        // Matrices for the stacked constraints
        Eigen::MatrixXd AConstraint;
        Eigen::VectorXd bConstraintMax;
        Eigen::VectorXd bConstraintMin;

        // Solution of the QP for determining orientations
        Eigen::VectorXd predictedOrientations;

        double TailIntegral = 0;

        // Cost function weights
        double qZd = 1;
        double qVx = 100;//100;
        double qVy = 100;//100;
        double qZ = 1000;

        // State
        Eigen::Vector3d comPos;
        Eigen::Vector3d comVel;
        Eigen::Vector3d zmpPos;
        Eigen::Vector4d predictedFootstep;

	// Quadratic problem
	qpOASES::QProblem qp;
	//qpOASES::QProblem TimingQP;

        // Timing QP matrices
        Eigen::MatrixXd H_Timing;
        Eigen::VectorXd f_Timing;
        Eigen::MatrixXd b_lower_timing;        
        Eigen::MatrixXd b_upper_timing;        

	// Some vectors to plot
	Eigen::MatrixXd predictedZmp;
   };

}

#endif
