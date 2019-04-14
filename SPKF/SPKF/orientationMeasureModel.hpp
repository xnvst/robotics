#ifndef ORIENTATIONMEASUREMODEL_HPP_
#define ORIENTATIONMEASUREMODEL_HPP_

#include "measureModel.hpp"

namespace SPKF_namespace
{
	template<typename T, class MeasurementType>
	class OrientationMeasureModel : MeasureModel<MeasurementType>
	{
		public:
			OrientationMeasureModel() {}
			virtual ~OrientationMeasureModel() {}
			
			virtual void init(){
				R_ = R_.setIdentity() * r;
			}
			
			virtual MeasurementType h(const State<T>& x) const{
				MeasurementType m_;
				m_(0,0) = (x.get_mat())(2,0) + R_(0,0);
				return m_;
			}
		
		private:
			Eigen::Matrix<T, 1, 1> R_;
	};
}

#endif	// ORIENTATIONMEASUREMODEL_HPP_
