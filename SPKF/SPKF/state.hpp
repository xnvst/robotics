#ifndef STATE_HPP_
#define STATE_HPP_

#include "configs.hpp"

namespace SPKF_namespace
{

	template<typename T>
	class State
	{
		public:
			State(){}
			virtual ~State(){}
			
			Eigen::Matrix<T, 3, 1> get_mat() const { return state_mat_; }

			
			void set_mat(T x, T y, T theta){ 
				state_mat_(0,0) = x;
				state_mat_(1,0) = y;
				state_mat_(2,0) = theta;
			}
		
		private:			
			Eigen::Matrix<T, 3, 1> state_mat_;
	};
}

#endif	// STATE_HPP_
