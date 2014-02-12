// Copyright (C) 2008 Wim Meeussen <meeussen at willowgarage com>
//  
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//  
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//  
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//  


#ifndef __NON_LINEAR_SYSTEM_CONDITIONAL_GAUSSIAN_ODO__
#define __NON_LINEAR_SYSTEM_CONDITIONAL_GAUSSIAN_ODO__

#include <pdf/analyticconditionalgaussian_additivenoise.h>

namespace BFL
{
  /// Non Linear Conditional Gaussian
  /**
     - \f$ \mu = Matrix[1] . ConditionalArguments[0] +
     Matrix[2]. ConditionalArguments[1]  + ... + Noise.\mu \f$
     - Covariance is independent of the ConditionalArguments, and is
     the covariance of the Noise pdf
  */
  class NonLinearAnalyticConditionalGaussianOdo : public AnalyticConditionalGaussianAdditiveNoise
  {
    public:
      /// Constructor
      /** @pre:  Every Matrix should have the same amount of rows!
	  This is currently not checked.  The same goes for the number
	  of columns, which should be equal to the number of rows of
	  the corresponding conditional argument!
	  @param additiveNoise Pdf representing the additive Gaussian uncertainty
      */
      NonLinearAnalyticConditionalGaussianOdo( const Gaussian& additiveNoise);

      /// Destructor
      virtual ~NonLinearAnalyticConditionalGaussianOdo();

      // redefine virtual functions
      virtual MatrixWrapper::ColumnVector    ExpectedValueGet() const;
      virtual MatrixWrapper::Matrix          dfGet(unsigned int i)       const;

    private:
      mutable MatrixWrapper::Matrix df;
    };

} // End namespace BFL
 
#endif //  
