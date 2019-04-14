#ifndef CONTROL_HPP_
#define CONTROL_HPP_

#include "configs.hpp"

namespace SPKF_namespace
{

	template<typename T>
	class Control
	{
		public:
			Control(){}
			virtual ~Control(){}
			
			Eigen::Matrix<T, 2, 1> get_mat() const { return control_mat_; }

			
			void set_mat(T v, T dtheta){ 
				control_mat_(0,0) = v;
				control_mat_(1,0) = dtheta;
			}
		
		private:			
			Eigen::Matrix<T, 2, 1> control_mat_;
	};
}

#endif	// CONTROL_HPP_
