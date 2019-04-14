#ifndef MOTIONMODEL_HPP_
#define MOTIONMODEL_HPP_

#include "state.hpp"
#include "control.hpp"

namespace SPKF_namespace
{

	class MotionModel
	{
		public:
			MotionModel() {}
			virtual ~MotionModel() {}
			
			void init(){
				Q_ = Q_.setIdentity() * q;
				this->F_.setIdentity();
				this->W_.setIdentity();
			}
			
			State<T> f(const State<T>& x, const Control<T>& u) const{
				State<T> x_;
	
				Eigen::Matrix<T, 3, 1> x_m = x.get_mat();
				Eigen::Matrix<T, 2, 1> u_m = u.get_mat();
				
				auto newOrientation = x_m(2,0) + u_m(1,0);
				T theta = newOrientation;
				
				x_.set_mat(x_m(0,0) + std::cos( newOrientation ) * u_m(0,0), x_m(1,0) + std::sin( newOrientation ) * u_m(0,0), theta);
				
				return x_;
			}

			void updateJacobians( const State<T>& x, const Control<T>& u )
			{
				// F = df/dx (Jacobian of state transition w.r.t. the state)
				this->F_.setZero();
				
				Eigen::Matrix<T, 3, 1> x_m = x.get_mat();
				Eigen::Matrix<T, 2, 1> u_m = u.get_mat();
				
				// partial derivative of x.x() w.r.t. x.x()
				this->F_(x_m(0,0), x_m(0,0)) = 1;
				// partial derivative of x.x() w.r.t. x.theta()
				this->F_(x_m(0,0), x_m(2,0) ) = -std::sin(x_m(2,0) + u_m(1,0)) * u_m(0,0);
				
				// partial derivative of x.y() w.r.t. x.y()
				this->F_(x_m(1,0), x_m(1,0)) = 1;
				// partial derivative of x.y() w.r.t. x.theta()
				this->F_(x_m(1,0), x_m(2,0)) = std::cos(x_m(2,0) + u_m(1,0)) * u_m(0,0);
				
				// partial derivative of x.theta() w.r.t. x.theta()
				this->F_(x_m(2,0), x_m(2,0)) = 1;
				
				// W = df/dw (Jacobian of state transition w.r.t. the noise)
				this->W_.setIdentity();
				// TODO: more sophisticated noise modelling
				//       i.e. The noise affects the the direction in which we move as 
				//       well as the velocity (i.e. the distance we move)
			}
		
		private:
			Eigen::Matrix<T, 3, 3> Q_;
			Eigen::Matrix<typename Eigen::Matrix<T, 3, 1>::Scalar,3,3> F_;
			Eigen::Matrix<typename Eigen::Matrix<T, 3, 1>::Scalar,3,3> W_;
	};

}

#endif	// MOTIONMODEL_HPP_
