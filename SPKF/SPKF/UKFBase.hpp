#ifndef UKFBASE_HPP_
#define UKFBASE_HPP_

#include "configs.hpp"
#include "state.hpp"
#include "control.hpp"
#include "motionModel.hpp"

namespace SPKF_namespace {
    
    template<typename T>
    class UKFBase
    {
		public:
			UKFBase() {}
			virtual ~UKFBase() {}
			
			void init(){
				computeWeights();
			}
				
		protected:
			static constexpr int SigmaPointCount =  2 * 3 + 1;
			Eigen::Matrix<T, SigmaPointCount, 1> sigmaWeights_m;
			Eigen::Matrix<T, SigmaPointCount, 1> sigmaWeights_c;
			Eigen::Matrix<T, 3, SigmaPointCount> sigmaStatePoints;
				
			void computeWeights()
			{
				T L = 3;
				lambda = alpha * alpha * ( L + kappa ) - L;
				gamma = std::sqrt( L + lambda );
				
				// Make sure L != -lambda to avoid division by zero
				assert( std::abs(L + lambda) > 1e-6 );
				
				// Make sure L != -kappa to avoid division by zero
				assert( std::abs(L + kappa) > 1e-6 );
				
				T W_m_0 = lambda / ( L + lambda );
				T W_c_0 = W_m_0 + (T(1) - alpha*alpha + beta);
				T W_i   = T(1) / ( T(2) * alpha*alpha * (L + kappa) );
				
				// Make sure W_i > 0 to avoid square-root of negative number
				assert( W_i > T(0) );
				
				sigmaWeights_m[0] = W_m_0;
				sigmaWeights_c[0] = W_c_0;
				
				for(int i = 1; i < SigmaPointCount; ++i)
				{
					sigmaWeights_m[i] = W_i;
					sigmaWeights_c[i] = W_i;
				}
			}

			State<T> computeStatePrediction(const MotionModel& s, const Control<T>& u)
			{
				State<T> x_;
				for( int i = 0; i < SigmaPointCount; ++i )
				{
					State<T> tmp;
					tmp.set_mat((sigmaStatePoints.col(i))(0,0),(sigmaStatePoints.col(i))(1,0),(sigmaStatePoints.col(i))(2,0));
					tmp = s.f( tmp, u );
					(sigmaStatePoints.col(i))(0,0) = (tmp.get_mat())(0,0);
					(sigmaStatePoints.col(i))(1,0) = (tmp.get_mat())(1,0);
					(sigmaStatePoints.col(i))(2,0) = (tmp.get_mat())(2,0);
				}
				
				auto ret = sigmaStatePoints * sigmaWeights_m;
				x_.set_mat(ret(0,0),ret(1,0),ret(2,0));
				return x_;
			}
    };
}

#endif	// UKFBASE_HPP_
