/*
There are mainly three variations of SPKF:

1. The Unscented Kalman Filter (UKF) is the more known
member of a family of Kalman filters, known as Sigma Point
Kalman Filters. The variants that have been developed
for this filter are intended to improve its performance
in several areas. 

2. The additive UKF was set out in order to
reduce the number of mathematical calculations performed
in each iteration, without using the augmented states of the
traditional UKF. This reduces significantly the computational
load of the filter that is mainly used in the calculation
and propagation of the sigma points, and makes it more
appropriate to be executed in real-time systems. 

3.The square root UKF was developed to prevent numerical instabilities to
which the algorithm is exposed, being necessary to conserve
the covariance matrix of the state errors as semidefined
positive. In addition to the numerical robustness reached,
a reduction in computational cost is obtained.

Comparing with the above, the code will try to implement the Algorithm for the square root UKF version
*/

#ifndef SPKF_HPP_

#include "state.hpp"
#include "squareRootBase.hpp"
#include "UKFBase.hpp"

namespace SPKF_namespace {
	
	template<typename T>
	class SPKF : public SquareRootBase<T>, public UKFBase<T>
	{
		public:
			SPKF() {}
			virtual ~SPKF() {}
			
			void init(){
				UKFBase<T>::init();
				P_.setIdentity();
			}

			bool computeSigmaPoints(const State<T>& x){
				Cholesky<Eigen::Matrix<T, 3, 3>> llt;
				llt.compute(P_);
				if(llt.info() != Eigen::Success)
				{
					return false;
				}
				
				Eigen::Matrix<T, 3, 3> _S = llt.matrixL().toDenseMatrix();

				UKFBase<T>::sigmaStatePoints.template leftCols<1>() = x.get_mat();

				UKFBase<T>::sigmaStatePoints.template block<3, 3>(0,1)
						= ( SPKF_namespace::gamma * _S).colwise() + x.get_mat();

				UKFBase<T>::sigmaStatePoints.template rightCols<3>()
						= (-SPKF_namespace::gamma * _S).colwise() + x.get_mat();
				
				std::cout << UKFBase<T>::sigmaStatePoints << std::endl;
				
				return true;
			}

			const State<T> predict( const MotionModel& s, const Control<T>& u )
			{
				State<T> x_;

				x_ = UKFBase<T>::computeStatePrediction(s, u);

				// TODO
				// computeCovarianceSquareRootFromSigmaPoints()
				
				return x_;
			}

			// TODO, chandle with compatible interface
			const State<T> update(OrientationMeasureModel<SPKF_namespace::T, Eigen::Matrix<T, 1, 1>> &o, const Eigen::Matrix<T, 1, 1>& om)
			{
				State<T> x_;
				// TODO
				// Compute kalman gain
				// update state and covariance
				// x += K * ( z - measurement prediction );
				return x_;
			}
			const State<T> update(PositionMeasureModel<SPKF_namespace::T, Eigen::Matrix<T, 2, 1>> &p, const Eigen::Matrix<T, 2, 1>& pm)
			{
				State<T> x_;
				// TODO
				// Compute kalman gain
				// update state and covariance
				// x += K * ( z - measurement prediction );
				return x_;
			}
		
		private:
			Eigen::Matrix<T, 3, 3> P_;
	};
}

#endif  // SPKF_HPP_
