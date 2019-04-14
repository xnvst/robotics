#ifndef SQUAREROOTBASE_HPP_
#define SQUAREROOTBASE_HPP_

#include "configs.hpp"
#include "cholesky.hpp"

namespace SPKF_namespace {
    
    template<typename T>
    class SquareRootBase
    {
		protected:
			Cholesky<Eigen::Matrix<T, 3, 3>> S;

		public:
			const Cholesky<Eigen::Matrix<T, 3, 3>>& getCovarianceSquareRoot() const
			{
				return S;
			}
			
			Eigen::Matrix<T, 3, 3> getCovariance() const
			{
				return S.reconstructedMatrix();
			}
			
			bool setCovariance(const Eigen::Matrix<T, 3, 3>& covariance)
			{
				S.compute(covariance);
				return (S.info() == Eigen::Success);
			}

			bool setCovarianceSquareRoot(const Eigen::Matrix<T, 3, 3>& covSquareRoot)
			{
				S.setL(covSquareRoot);
				return true;
			}
				
		protected:		
			SquareRootBase()
			{
				S.setIdentity();
			}
    };
}

#endif
