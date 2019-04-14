#ifndef POSITIONMEASUREMODEL_HPP_
#define POSITIONMEASUREMODEL_HPP_

#include "measureModel.hpp"

namespace SPKF_namespace
{
	template<typename T, class MeasurementType>
	class PositionMeasureModel : MeasureModel<MeasurementType>
	{
		public:
			PositionMeasureModel(T landmark1x, T landmark1y, T landmark2x, T landmark2y) {
				landmark1 << landmark1x, landmark1y;
        landmark2 << landmark2x, landmark2y;
			}
			virtual ~PositionMeasureModel() {}
			
			virtual void init(){
				R_ = R_.setIdentity() * r;
			}
			
			virtual MeasurementType h(const State<T>& x) const{
				MeasurementType m_;
				Eigen::Matrix<T, 2, 1> position = x.get_mat().template head<2>();
				Eigen::Matrix<T, 2, 1> delta1 = position - landmark1;
				m_(0,0) = std::sqrt( delta1.dot(delta1) ) + R_(0,0);
				Eigen::Matrix<T, 2, 1> delta2 = position - landmark2;
				m_(1,0) = std::sqrt( delta2.dot(delta2) ) + R_(1,0);
				return m_;
			}
		
		private:
			Eigen::Matrix<T, 2, 1> R_;
			Eigen::Matrix<T, 2, 1> landmark1;
			Eigen::Matrix<T, 2, 1> landmark2;
	};
}

#endif	// POSITIONMEASUREMODEL_HPP_
