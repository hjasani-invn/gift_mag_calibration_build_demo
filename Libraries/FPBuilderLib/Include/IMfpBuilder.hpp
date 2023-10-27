#ifndef I_MFP_BUILDER_HPP
#define I_MFP_BUILDER_HPP

#include "MfpBuilder.hpp"

namespace Fpbl
{
    class IMfpBuilder
    {
        public:
            virtual ~IMfpBuilder() {};
            virtual void setStdEstimatorType(eStdEstimatorType stdEstimatorType) = 0;
            virtual void setDefaultMagdata(double mag_x, double mag_y, double mag_z) = 0;
            virtual Fpbl::ReturnStatus buildFingerprint( const Fpbl::MagneticGrid &mfpGrid, Fpbl::MfpBuilder::LocalDB *mfpFp ) = 0;
            virtual Fpbl::ReturnStatus updateFingerprint(const Fpbl::PortalsGrid &portalGrid, Fpbl::MfpBuilder::LocalDB *mfpFp) = 0;
            virtual Fpbl::ReturnStatus updateFingerprintForCrowdsourced(const Fpbl::CSGrid &csGrid, Fpbl::MfpBuilder::LocalDB *mfpFp) = 0;
    };
}
#endif //I_MFP_BUILDER_HPP
