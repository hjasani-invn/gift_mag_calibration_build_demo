#include "Mfp3Builder.hpp"

using namespace Fpbl;

MfpBuilder::MfpBuilder()
{
    mMfpBuilder = new Mfp3BuilderRobustRef();
}

MfpBuilder::~MfpBuilder()
{
    delete mMfpBuilder;
}

void MfpBuilder::setStdEstimatorType(eStdEstimatorType stdEstimatorType)
{
    mMfpBuilder->setStdEstimatorType(stdEstimatorType);
}

void MfpBuilder::setDefaultMagdata(double mag_x, double mag_y, double mag_z)
{
    mMfpBuilder->setDefaultMagdata(mag_x, mag_y, mag_z);
}

ReturnStatus MfpBuilder::buildFingerprint(const MagneticGrid &mfpGrid, LocalDB *mfpFp)
{
    ReturnStatus result = mMfpBuilder->buildFingerprint(mfpGrid, mfpFp);
    return result;
}

ReturnStatus MfpBuilder::updateFingerprint(const PortalsGrid &portalGrid, LocalDB *mfpFp)
{
    ReturnStatus result = mMfpBuilder->updateFingerprint(portalGrid, mfpFp);
    return result;
}

ReturnStatus MfpBuilder::updateFingerprintForCrowdsourced(const CSGrid &csGrid, LocalDB *mfpFp)
{
	ReturnStatus result = mMfpBuilder->updateFingerprintForCrowdsourced(csGrid, mfpFp);
	return result;
}
