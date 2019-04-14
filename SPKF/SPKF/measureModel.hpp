#ifndef MEASUREMODEL_HPP_
#define MEASUREMODEL_HPP_

#include "state.hpp"

namespace SPKF_namespace
{

	template<class MeasurementType>
	class MeasureModel
	{
		public:
			virtual void init() = 0;
			
			virtual MeasurementType h(const State<SPKF_namespace::T>& x) const = 0;
		
		protected:
			MeasureModel() {}
			virtual ~MeasureModel() {}
	};
}

#endif	// MEASUREMODEL_HPP_
