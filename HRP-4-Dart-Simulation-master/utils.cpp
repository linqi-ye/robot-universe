#include <Eigen/Geometry>
#include "qpOASES/qpOASES.hpp"
#include <iostream>
inline double wrapToPi(double angle){
  double ret=angle;
  while(ret>M_PI){
    ret-=2*M_PI;
  }

  while(ret<=-M_PI){
    ret+=2*M_PI;
  }

  return ret;
}

inline double angDiff(double thetaD, double theta){
  double alpha=0;
  Eigen::Vector2d nD,n;

  nD<<cos(thetaD), sin(thetaD);
  n<<cos(theta), sin(theta);

  double alphaAbs = acos(nD.transpose()*n);

  Eigen::Vector3d n3,nD3;

  n3<<n(0),n(1),0;
  nD3<<nD(0),nD(1),0;

  Eigen::Vector3d nC3;

  nC3=n3.cross(nD3);

  if(nC3(2)>0){
    alpha=alphaAbs;
  }
  else{
    alpha=-alphaAbs;
  }

  return alpha;

}

inline Eigen::MatrixXd matrixPower(Eigen::MatrixXd& A, int exp){

	Eigen::MatrixXd result = Eigen::MatrixXd::Identity(A.rows(),A.cols());

	for (int i=0; i<exp;++i)
        	result *= A;

	return result;
}

inline double sign(double x){
	if(x>0) return +1;
	if(x<0) return -1;
	return -1;
}

inline Eigen::VectorXd solveQP(Eigen::MatrixXd costFunctionH, Eigen::VectorXd costFunctionF, 
		Eigen::MatrixXd AConstraint, Eigen::VectorXd bConstraintMin, Eigen::VectorXd bConstraintMax) {

	int nVariables = costFunctionH.rows();
	int nConstraints = AConstraint.rows();

	qpOASES::QProblem qp;

	qpOASES::real_t H[nVariables*nVariables];
	qpOASES::real_t g[nVariables];

	qpOASES::real_t A[nConstraints*nVariables];
	qpOASES::real_t lb[nConstraints];
	qpOASES::real_t ub[nConstraints];

	for(int i=0;i<nVariables;++i){
		for(int j=0;j<nVariables;++j){
			H[i*nVariables+j] = costFunctionH(i,j);
			//std::cout << "H" <<  H[i*nVariables+j] << std::endl;
		}
		g[i] = costFunctionF(i);
		//std::cout << "g" <<  g[i] << std::endl;
	}

	for(int i=0;i<nConstraints;++i){
		for(int j=0;j<nVariables;++j){
			A[i*nVariables+j] = AConstraint(i,j);
			//std::cout << "A " <<  A[i*nVariables+j] << std::endl;
		}
		lb[i] = bConstraintMin(i);
		//std::cout << "lb" <<  lb[i] << std::endl;
		ub[i] = bConstraintMax(i);
		//std::cout << "ub" <<  ub[i] << std::endl;
	}

	qpOASES::real_t xOpt[nVariables];

	qpOASES::Options options;
	options.setToMPC();
	options.printLevel=qpOASES::PL_NONE;
	qpOASES::int_t nWSR = 300;

	qp = qpOASES::QProblem(nVariables, nConstraints);
	qp.setOptions(options);
	
	//std::cout << "nVar" <<  nVariables << std::endl;
	//std::cout << "nConst" << nConstraints << std::endl;
	//std::cout << "H" << *H << std::endl;
	//std::cout << "H.size" << H->rows << " and" << H->cols << std::endl;
	//std::cout << "A" << *A << std::endl;
	//std::cout << "g" << *g << std::endl;
	//std::cout << "lb" << *lb << std::endl;
	//std::cout << "ub" << *ub << std::endl;
	qp.init(H,g,A,NULL,NULL,lb,ub,nWSR,NULL,NULL,NULL,NULL,NULL,NULL);

	qp.getPrimalSolution(xOpt);

	Eigen::VectorXd decisionVariables(nVariables);
	
	for(int i=0;i<nVariables;++i){
		decisionVariables(i) = xOpt[i];
		//std::cout << "velocity" << xOpt[i] << std::endl;
	}

	return decisionVariables;
}
