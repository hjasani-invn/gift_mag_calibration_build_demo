#define _CRT_SECURE_NO_WARNINGS
#include "Fppe.hpp"
#include "CFppe.h"
#include "version.h"
#include "Venue.h"

void CTpnPosition_print(CTpnPosition p, FILE * out)
{
	fprintf(out, "; %f", p.lattitude);				///< lattitude, [deg] [-90..90]
	fprintf(out, "; %f", p.longitude);				///< longitude, [deg] [-180..180]
	fprintf(out, "; %f", p.user_heading);			///< speed vector dicrection [deg] [-180..180] (user heading)

	fprintf(out, "; %f", p.sigma_north);			///< [m] position standard deviation in north direction
	fprintf(out, "; %f", p.sigma_east);				///< [m] position standard deviation in east direction
	fprintf(out, "; %f", p.sigma_user_heading);		///< user heading uncertanties,[deg]; this field also indicates validity of itself, user_heading and misalignment parameters
	/// < these fields are invalid when sigma_user_heading <= 0

	fprintf(out, "; %f", p.misalignment);			///< user/device heading misalignment angle [deg]
	fprintf(out, "; %f", p.sigma_misalignment);		///< user/device heading misalignment std [deg]; reserved

	fprintf(out, "; %d", p.floor);					///< floor number
	fprintf(out, "; %f", p.altitude);				///< altitude above sea level [m]
	fprintf(out, "; %f", p.sigma_altitude);			///< altitude standard deviation [m]

	fprintf(out, "; %d", (int)p.navigation_phase);	///< navigation phase flag; position is avaliable for navigation_phase > 0
	fprintf(out, "; %d", (int)p.fidgeting_flag);	///< fidjeting flag

	fprintf(out, "; %d", (int)p.is_valid);			///< position validity flag
}

void CTpnAttitude_print(CTpnAttitude a, FILE * out)
{
	fprintf(out, "; %f", a.roll);	                ///< device roll, [deg] [-180..180]
	fprintf(out, "; %f", a.pitch);					///< device pitch, [deg] [-90..90]
	fprintf(out, "; %f", a.heading);				///< device heading, [deg] [-180..180]

	fprintf(out, "; %f", a.sigma_roll);				///< device roll standard deviation
	fprintf(out, "; %f", a.sigma_pitch);			///< device pitch standard deviation
	fprintf(out, "; %f", a.sigma_heading);			///< device heading standard deviation

	fprintf(out, "; %d", (int)a.is_valid);			///< attitude validity flag
}

void CTpnMagneticMeasurement_print(CTpnMagneticMeasurement m, FILE * out)
{
	fprintf(out, "; %f", m.mX);									///< magnetic field on x-axis [mG]
	fprintf(out, "; %f", m.mY);									///< magnetic field on y-axis [mG]
	fprintf(out, "; %f", m.mZ);									///< magnetic field on z-axis [mG]

	fprintf(out, "; %f", m.sigma_mX);							///<  x-axis magnetometer meassurement noise [mG]
	fprintf(out, "; %f", m.sigma_mY);							///<  y-axis magnetometer meassurement noise [mG]
	fprintf(out, "; %f", m.sigma_mZ);							///<  z-axis magnetometer meassurement noise [mG]

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			fprintf(out, "; %f", m.covarianceMatrix[i][j]);      ///< mag bias error covariance matrix [mG^2], column order

	fprintf(out, "; %d", (int)m.level_of_calibration);			///< this parameter describes a consistency class of bias estimation and bias covariance matrix estimation
	fprintf(out, "; %d", (int)m.is_valid);						///< attitude validity flag
}

void CTpnPdr_print(CTpnPdr p, FILE * out)
{
	fprintf(out, "; %f", p.stride_length);			///< stride length [m]
	fprintf(out, "; %d", p.is_valid);				///< stride length validity flag

}

void CTpnOutput_print(const CTpnOutput* o, FILE * out)
{
	fprintf(out, "%f", o->timestamp);
	fprintf(out, "; ");        CTpnPosition_print(o->position, out);
	fprintf(out, "; ");        CTpnAttitude_print(o->attitude, out);
	fprintf(out, "; ");        CTpnPdr_print(o->pdr, out);
	fprintf(out, "; ");        CTpnMagneticMeasurement_print(o->mag_meas, out);
	fprintf(out, "\n");
}

CMagneticData CMagneticData_clone(const CMagneticData c)
{
	CMagneticData cmagneticdata;
	cmagneticdata.timestamp = c.timestamp;
	cmagneticdata.mX = c.mX;
	cmagneticdata.mY = c.mY;
	cmagneticdata.mZ = c.mZ;
	memcpy(cmagneticdata.covarianceMatrix, c.covarianceMatrix, sizeof(c.covarianceMatrix));

	return cmagneticdata;
}

class CPositionUpdate : public Fppe::IPositionUpdate
{
public:
    explicit CPositionUpdate(CIPositionUpdate* pPosCbk)
		: cpositionupdate(*pPosCbk)
	{

	}

    explicit CPositionUpdate(const CIPositionUpdate pPosCbk)
		: cpositionupdate(pPosCbk)
	{
		
	}

	virtual ~CPositionUpdate()
	{
		;
	}

	/**
	* this prototype is used in PC model to visualize particle cloud
	* \param[in] X,Y coordinates in a local frame [m]
	* \param[in] Floor floor level
	* \param[in] H heading [rad]
	* \param[in] Sig estimated position deviation
	* \param[in] t timestamp [ms]
	* \param[in] state pointer to the particles array
	* \param[in] N particles count
	*/
	virtual void update(double X, double Y, double Floor, double H, double Sig, double t, const Fppe::Particle *state, int N)
	{
		cpositionupdate.update_by_vars(cpositionupdate.object_handle, X, Y, Floor, H, Sig, t, (CParticle*)state, N);
	}

	/**
	* This method is called when new position estimation available
	* \param[in]  position navigation solution
	*/
	virtual void update(const Fppe::Position &position)
	{
		cpositionupdate.update_by_struct(cpositionupdate.object_handle, reinterpret_cast<CPosition*>(const_cast<Fppe::Position*>(&position)));
	}

protected:
	CIPositionUpdate cpositionupdate;
};

class CVenueDetectionUpdate : public Fppe::IVenueDetectionUpdate
{
public:
    explicit CVenueDetectionUpdate(CIVenueDetectionUpdate* pVenueDetectCbk)
        : c_venue_detection_update(*pVenueDetectCbk)
    {

    }

    explicit CVenueDetectionUpdate(const CIVenueDetectionUpdate pVenueDetectCbk)
        : c_venue_detection_update(pVenueDetectCbk)
    {

    }

    virtual ~CVenueDetectionUpdate()
    {
        ;
    }

    virtual void update(bool pos_inside_venue, double t)
    {
        c_venue_detection_update.update_by_vars(c_venue_detection_update.object_handle, pos_inside_venue, t);
    }

    protected:
    CIVenueDetectionUpdate c_venue_detection_update;
};

class CExtendedProximityUpdate : public Fppe::IExtendedProximityUpdate
{
public:
    explicit CExtendedProximityUpdate(CIExtendedProximityUpdate* pCbk)
        : c_extended_proxiity_update(*pCbk)
    {

    }

    explicit CExtendedProximityUpdate(const CIExtendedProximityUpdate pCbk)
        : c_extended_proxiity_update(pCbk)
    {

    }

    virtual ~CExtendedProximityUpdate()
    {
        ;
    }

    virtual void update(double t, const Fppe::ProximityBeaconData & ExtendedProximityData)
    {
        c_extended_proxiity_update.update_by_struct(c_extended_proxiity_update.object_handle, t, static_cast<CProximityBeaconData*>(const_cast<Fppe::ProximityBeaconData*>(&ExtendedProximityData)));
    }

protected:
    CIExtendedProximityUpdate c_extended_proxiity_update;
};

class foutbuf : public std::streambuf
{
public:
    foutbuf(FILE *f) : fout(f)
    { 
        ; 
    }
    ~foutbuf()
    {
        if (fout != NULL)
        {
            fflush(fout);
        }
    }
protected:
    //virtual std::streamsize xsputn(const char* s, std::streamsize n)
    //virtual int sync()
    //{
    //    if (fout != NULL)
    //    {
    //        return fflush(fout);
    //    }
    //    return EOF;
    //}

    virtual int overflow(int c)
    {
        if (c != EOF && fout !=NULL) {
            // write the character to the file
            if (fputc(c, fout) == EOF) {
                return EOF;
            }
        }
        else
            return EOF;
        return c;
    }
    FILE *fout;
};

class CLogger : public std::ostream 
{
public:
    CLogger(FILE *f) : fbuf(f), std::ostream(&fbuf)
    {
        ;
    }
protected:
    foutbuf fbuf;
    
private:
    CLogger(const CLogger &); //disable copy constructor
    const CLogger &operator=(const CLogger &); //disable copy
};

CILogger *CLogger_new(FILE *f)
{
    CLogger* mCLogger = new CLogger(f);
    return (CILogger*)mCLogger;
}

void CLogger_free(CILogger *logger)
{
    delete (CLogger *)logger;
}


CIFPEngine* FPEngine_new()
{
	Fppe::FPEngine* mFPEngine = new Fppe::FPEngine();
	return (CIFPEngine*)mFPEngine;
}

void FPEngine_free(CIFPEngine* fpEngine)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	delete mFPEngine;
}


/** \return version info*/
CVersionNumber FPEngine_getVersionNumber(CIFPEngine* fpEngine)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    VersionNumber ver = mFPEngine->getVersionNumber();
    CVersionNumber result = static_cast<CVersionNumber>(ver);
    return result;
}


///**
//* resets internal object state
//* \param[in] filename output log filename
//*/
//void FPEngine_setLogFile(CIFPEngine* fpEngine, const char* filename)
//{
//	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
//	mFPEngine->setLogFile(filename);
//}
///**
//* enables logging
//* \param[in] enabled control flag
//*/
//void FPEngine_setLogEnabled(CIFPEngine* fpEngine, const bool* enabled)
//{
//	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
//	mFPEngine->setLogEnabled(*enabled);
//}


bool FPEngine_setLogger(CIFPEngine* fpEngine, CILogger* logger, const unsigned int id)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    return mFPEngine->setLogStream(id, *(CLogger*)logger);
}


int FPEngine_getLogsCount(CIFPEngine* fpEngine)
{
    std::vector<std::string> logs;
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->getLogDescription(&logs);
    return logs.size();
}

int FPEngine_getLogDescription(CIFPEngine* fpEngine, char* log_descr, size_t size, unsigned int id)
{
    int result = -1;
    std::vector<std::string> logs;
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->getLogDescription(&logs);
    if (id < logs.size())
    {
        result = (logs[id].size() + 1 < size) ? logs[id].size() + 1 : size;
        strncpy(log_descr, logs[id].c_str(), result);
    }
    return result;
}




/**
* resets internal object state, disables logging
*/
void FPEngine_restart(CIFPEngine* fpEngine)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	mFPEngine->restart();
}

/**
* main method, process inertial sensors data and initiates processing pipeline
* \param[in] increment position increments in local frame
*/
void FPEngine_processIncrements(CIFPEngine* fpEngine, const CCoordinatesIncrement* increment)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;

	Fppe::CoordinatesIncrement cpp_increment;
	cpp_increment.timestamp = increment->timestamp;
	cpp_increment.d_x = increment->d_x;
	cpp_increment.d_y = increment->d_y;
	cpp_increment.d_floor = increment->d_floor;
	memcpy(cpp_increment.covariance_yx, increment->covariance_yx, sizeof(increment->covariance_yx));
	cpp_increment.d_floor_std = increment->d_floor_std;
	cpp_increment.is_motion = increment->is_motion;
	// cpp_increment.attitude = increment->attitude;
	Fppe::Attitude cpp_attitude;
	cpp_attitude.is_valid = increment->attitude.is_valid;
	memcpy(cpp_attitude.quaternion, increment->attitude.quaternion, sizeof(increment->attitude.quaternion));
	memcpy(cpp_attitude.covariance_quaternion, increment->attitude.covariance_quaternion, sizeof(increment->attitude.covariance_quaternion));
	cpp_increment.attitude = cpp_attitude;
	cpp_increment.is_transit = increment->is_transit;
	mFPEngine->processIncrements(cpp_increment);
}

void FPEngine_processTpnOutput(CIFPEngine* fpEngine, const CTpnOutput* tpn_output)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;

    TpnOutput cpp_tpn_output = TpnOutput();

    cpp_tpn_output.timestamp = tpn_output->timestamp;
    cpp_tpn_output.position.lattitude = tpn_output->position.lattitude;
    cpp_tpn_output.position.longitude = tpn_output->position.longitude;
    cpp_tpn_output.position.user_heading = tpn_output->position.user_heading;
    cpp_tpn_output.position.sigma_north = tpn_output->position.sigma_north;
    cpp_tpn_output.position.sigma_east = tpn_output->position.sigma_east;
    cpp_tpn_output.position.sigma_user_heading = tpn_output->position.sigma_user_heading;
    cpp_tpn_output.position.misalignment = tpn_output->position.misalignment;
    cpp_tpn_output.position.sigma_misalignment = tpn_output->position.sigma_misalignment;
    cpp_tpn_output.position.floor = tpn_output->position.floor;
    cpp_tpn_output.position.altitude = tpn_output->position.altitude;
    cpp_tpn_output.position.sigma_altitude = tpn_output->position.sigma_altitude;
    cpp_tpn_output.position.navigation_phase = tpn_output->position.navigation_phase;
    cpp_tpn_output.position.fidgeting_flag = tpn_output->position.fidgeting_flag;
    cpp_tpn_output.position.is_valid = tpn_output->position.is_valid;
    cpp_tpn_output.position.mode_of_transit = 1; // default is assumed

    cpp_tpn_output.attitude.orientation_id = tpn_output->attitude.orientation_id;
    cpp_tpn_output.attitude.roll = tpn_output->attitude.roll;
    cpp_tpn_output.attitude.pitch = tpn_output->attitude.pitch;
    cpp_tpn_output.attitude.heading = tpn_output->attitude.heading;
    cpp_tpn_output.attitude.sigma_roll = tpn_output->attitude.sigma_roll;
    cpp_tpn_output.attitude.sigma_pitch = tpn_output->attitude.sigma_pitch;
    cpp_tpn_output.attitude.sigma_heading = tpn_output->attitude.sigma_heading;
    cpp_tpn_output.attitude.is_valid = tpn_output->attitude.is_valid;

    cpp_tpn_output.pdr.stride_length = tpn_output->pdr.stride_length;
    cpp_tpn_output.pdr.is_valid = tpn_output->pdr.is_valid;
    cpp_tpn_output.pdr.misalignment = tpn_output->pdr.misalignment;
    cpp_tpn_output.pdr.misalignment_p1 = tpn_output->pdr.misalignment_p1;
    cpp_tpn_output.pdr.misalignment_p2 = tpn_output->pdr.misalignment_p2;
    cpp_tpn_output.pdr.use_case = tpn_output->pdr.use_case;

    cpp_tpn_output.mag_meas.mX = tpn_output->mag_meas.mX;
    cpp_tpn_output.mag_meas.mY = tpn_output->mag_meas.mY;
    cpp_tpn_output.mag_meas.mZ = tpn_output->mag_meas.mZ;
    cpp_tpn_output.mag_meas.sigma_mX = tpn_output->mag_meas.sigma_mX;
    cpp_tpn_output.mag_meas.sigma_mY = tpn_output->mag_meas.sigma_mY;
    cpp_tpn_output.mag_meas.sigma_mZ = tpn_output->mag_meas.sigma_mZ;
    cpp_tpn_output.mag_meas.level_of_calibration = tpn_output->mag_meas.level_of_calibration;
    cpp_tpn_output.mag_meas.is_valid = tpn_output->mag_meas.is_valid;
    cpp_tpn_output.mag_meas.covarianceMatrix[0][0] = tpn_output->mag_meas.covarianceMatrix[0][0];
    cpp_tpn_output.mag_meas.covarianceMatrix[0][1] = tpn_output->mag_meas.covarianceMatrix[0][1];
    cpp_tpn_output.mag_meas.covarianceMatrix[0][2] = tpn_output->mag_meas.covarianceMatrix[0][2];
    cpp_tpn_output.mag_meas.covarianceMatrix[1][0] = tpn_output->mag_meas.covarianceMatrix[1][0];
    cpp_tpn_output.mag_meas.covarianceMatrix[1][1] = tpn_output->mag_meas.covarianceMatrix[1][1];
    cpp_tpn_output.mag_meas.covarianceMatrix[1][2] = tpn_output->mag_meas.covarianceMatrix[1][2];
    cpp_tpn_output.mag_meas.covarianceMatrix[2][0] = tpn_output->mag_meas.covarianceMatrix[2][0];
    cpp_tpn_output.mag_meas.covarianceMatrix[2][1] = tpn_output->mag_meas.covarianceMatrix[2][1];
    cpp_tpn_output.mag_meas.covarianceMatrix[2][2] = tpn_output->mag_meas.covarianceMatrix[2][2];

    mFPEngine->processTpnOutput(cpp_tpn_output);
    //mFPEngine->processTpnOutput(*(reinterpret_cast<TpnOutput*>(const_cast<CTpnOutput*>(tpn_output))));
}

void FPEngine_processTpnOutputWithExtraInformation(CIFPEngine* fpEngine, const CTpnOutput* tpn_output, CTpnPositionExtra* tpn_pos_extra)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;

    TpnOutput cpp_tpn_output = TpnOutput();

    cpp_tpn_output.timestamp = tpn_output->timestamp;
    cpp_tpn_output.position.lattitude = tpn_output->position.lattitude;
    cpp_tpn_output.position.longitude = tpn_output->position.longitude;
    cpp_tpn_output.position.user_heading = tpn_output->position.user_heading;
    cpp_tpn_output.position.sigma_north = tpn_output->position.sigma_north;
    cpp_tpn_output.position.sigma_east = tpn_output->position.sigma_east;
    cpp_tpn_output.position.sigma_user_heading = tpn_output->position.sigma_user_heading;
    cpp_tpn_output.position.misalignment = tpn_output->position.misalignment;
    cpp_tpn_output.position.sigma_misalignment = tpn_output->position.sigma_misalignment;
    cpp_tpn_output.position.floor = tpn_output->position.floor;
    cpp_tpn_output.position.altitude = tpn_output->position.altitude;
    cpp_tpn_output.position.sigma_altitude = tpn_output->position.sigma_altitude;
    cpp_tpn_output.position.navigation_phase = tpn_output->position.navigation_phase;
    cpp_tpn_output.position.fidgeting_flag = tpn_output->position.fidgeting_flag;
    cpp_tpn_output.position.is_valid = tpn_output->position.is_valid;
    cpp_tpn_output.position.mode_of_transit = tpn_pos_extra->mode_of_transit;

    cpp_tpn_output.attitude.orientation_id = tpn_output->attitude.orientation_id;
    cpp_tpn_output.attitude.roll = tpn_output->attitude.roll;
    cpp_tpn_output.attitude.pitch = tpn_output->attitude.pitch;
    cpp_tpn_output.attitude.heading = tpn_output->attitude.heading;
    cpp_tpn_output.attitude.sigma_roll = tpn_output->attitude.sigma_roll;
    cpp_tpn_output.attitude.sigma_pitch = tpn_output->attitude.sigma_pitch;
    cpp_tpn_output.attitude.sigma_heading = tpn_output->attitude.sigma_heading;
    cpp_tpn_output.attitude.is_valid = tpn_output->attitude.is_valid;

    cpp_tpn_output.pdr.stride_length = tpn_output->pdr.stride_length;
    cpp_tpn_output.pdr.is_valid = tpn_output->pdr.is_valid;
    cpp_tpn_output.pdr.misalignment = tpn_output->pdr.misalignment;
    cpp_tpn_output.pdr.misalignment_p1 = tpn_output->pdr.misalignment_p1;
    cpp_tpn_output.pdr.misalignment_p2 = tpn_output->pdr.misalignment_p2;
    cpp_tpn_output.pdr.use_case = tpn_output->pdr.use_case;

    cpp_tpn_output.mag_meas.mX = tpn_output->mag_meas.mX;
    cpp_tpn_output.mag_meas.mY = tpn_output->mag_meas.mY;
    cpp_tpn_output.mag_meas.mZ = tpn_output->mag_meas.mZ;
    cpp_tpn_output.mag_meas.sigma_mX = tpn_output->mag_meas.sigma_mX;
    cpp_tpn_output.mag_meas.sigma_mY = tpn_output->mag_meas.sigma_mY;
    cpp_tpn_output.mag_meas.sigma_mZ = tpn_output->mag_meas.sigma_mZ;
    cpp_tpn_output.mag_meas.level_of_calibration = tpn_output->mag_meas.level_of_calibration;
    cpp_tpn_output.mag_meas.is_valid = tpn_output->mag_meas.is_valid;
    cpp_tpn_output.mag_meas.covarianceMatrix[0][0] = tpn_output->mag_meas.covarianceMatrix[0][0];
    cpp_tpn_output.mag_meas.covarianceMatrix[0][1] = tpn_output->mag_meas.covarianceMatrix[0][1];
    cpp_tpn_output.mag_meas.covarianceMatrix[0][2] = tpn_output->mag_meas.covarianceMatrix[0][2];
    cpp_tpn_output.mag_meas.covarianceMatrix[1][0] = tpn_output->mag_meas.covarianceMatrix[1][0];
    cpp_tpn_output.mag_meas.covarianceMatrix[1][1] = tpn_output->mag_meas.covarianceMatrix[1][1];
    cpp_tpn_output.mag_meas.covarianceMatrix[1][2] = tpn_output->mag_meas.covarianceMatrix[1][2];
    cpp_tpn_output.mag_meas.covarianceMatrix[2][0] = tpn_output->mag_meas.covarianceMatrix[2][0];
    cpp_tpn_output.mag_meas.covarianceMatrix[2][1] = tpn_output->mag_meas.covarianceMatrix[2][1];
    cpp_tpn_output.mag_meas.covarianceMatrix[2][2] = tpn_output->mag_meas.covarianceMatrix[2][2];

    mFPEngine->processTpnOutput(cpp_tpn_output);
}

/**
* pushes WiFi measurement into the processing pipeline
* \param[in] scan_wifi WiFi measurements vector with timestamp
*/
void FPEngine_processWiFi(CIFPEngine* fpEngine, const CWiFiScanResult* scan_wifi)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;

    Fppe::WiFiScanResult fppe_scan_wifi;
    fppe_scan_wifi.timestamp = scan_wifi->timestamp;
    for (uint16_t i = 0; i < scan_wifi->n_scans_wifi; ++i)
    {
        Fppe::WiFiMeasurement meas = {};
        meas.timestamp = scan_wifi->scanWiFi[i].timestamp;
        meas.mac = scan_wifi->scanWiFi[i].mac;
        meas.rssi = scan_wifi->scanWiFi[i].rssi;
        meas.frequency = scan_wifi->scanWiFi[i].frequency;

        fppe_scan_wifi.scanWiFi.push_back(meas);
    }

mFPEngine->processWiFi(fppe_scan_wifi);
}

/**
* pushes BLE measurement into the processing pipeline
* \param[in] scan_ble BLE measurements vector with timestamp
*/
void FPEngine_processBLE(CIFPEngine* fpEngine, const CBleScanResult* scan_ble)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;

    Fppe::BleScanResult fppe_scan_ble;
    fppe_scan_ble.timestamp = scan_ble->timestamp;
    for (uint16_t i = 0; i < scan_ble->n_scans_ble; i++)
    {
        CBleMeasurement Cscan = scan_ble->scanBle[i];
        Fppe::BleMeasurement scan;
        scan.timestamp = Cscan.timestamp;
        scan.mac = Cscan.mac;
        scan.rssi = Cscan.rssi;
        scan.frequency = Cscan.frequency;
        scan.major = Cscan.major;
        scan.minor = Cscan.minor;
        memcpy(scan.uuid, Cscan.uuid, sizeof(Cscan.uuid));
        scan.txPower = Cscan.txPower;
        scan.hasMAC = Cscan.hasMAC;

        scan.proximity = Cscan.proximity;
        scan.accuracy = Cscan.accuracy;

        fppe_scan_ble.scanBle.push_back(scan);
    }

    mFPEngine->processBLE(fppe_scan_ble);
}


/**
* pushes Magnetic measurement into the processing pipeline
* \param[in] mag_data magnetic vector with timestamp
*/
void FPEngine_processMFP(CIFPEngine* fpEngine, const CMagneticVector* mag_data, int64_t timestamp)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	mFPEngine->processMFP((*(MagneticVector*)mag_data), timestamp);
}

/**
* pushes external location measurement into the processing pipeline
* \param[in] position external position
*/
void FPEngine_processExternalPosition(CIFPEngine* fpEngine, const CPosition* position)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->processExternalPosition(*(Fppe::Position*)position);
}

/**
* pushes collaboration data into the processing pipeline
* \param[in] collaboration_position - an array of collaboration positions
* \param[in] size - size of array of collaboration positions
*/
void FPEngine_processInputCollaboration(CIFPEngine* fpEngine, CICollaborationData* collaboration_position, size_t size)
{
	std::vector<Fppe::CollaborationData> collaboration_data;
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;

	for (uint16_t i = 0; i < size; ++i)
	{
		Fppe::CollaborationData collaboration_data_element;

		collaboration_data_element.BSSID = collaboration_position[i].BSSID;
		collaboration_data_element.cov_ne[0][0] = collaboration_position[i].cov_ne[0][0];
		collaboration_data_element.cov_ne[0][1] = collaboration_position[i].cov_ne[0][1];
		collaboration_data_element.cov_ne[1][0] = collaboration_position[i].cov_ne[1][0];
		collaboration_data_element.cov_ne[1][1] = collaboration_position[i].cov_ne[1][1];
		collaboration_data_element.distance = collaboration_position[i].distance;
		collaboration_data_element.distance_uncertainty = collaboration_position[i].distance_uncertainty;
		collaboration_data_element.floor_number = collaboration_position[i].floor_number;
		collaboration_data_element.floor_std = collaboration_position[i].floor_std;
		collaboration_data_element.is_distance_valid = collaboration_position[i].is_distance_valid;
		collaboration_data_element.is_floor_valid = collaboration_position[i].is_floor_valid;
		collaboration_data_element.is_position_valid = collaboration_position[i].is_position_valid;
		collaboration_data_element.lattitude = collaboration_position[i].lattitude;
		collaboration_data_element.longitude = collaboration_position[i].longitude;
		collaboration_data_element.timestamp = collaboration_position[i].timestamp;
		
		collaboration_data.push_back(collaboration_data_element);
	}

	mFPEngine->processInputCollaboration(collaboration_data);
}

DLL_EXPORT int FPEngine_getProximityBeaconsNumber(CIFPEngine* fpEngine, const int16_t floor, const BleBeaconType beacon_type)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    return mFPEngine->getProximityBeaconsNumber(floor, beacon_type);
}

CIPositionCallbackObject* PositionCallbackObject_new(const CIPositionUpdate posCbk)
{
	CPositionUpdate* pCbkObj = new CPositionUpdate(posCbk);
	CIPositionCallbackObject *pCallbackObj = (CIPositionCallbackObject*)pCbkObj;
	return pCallbackObj;
}

void PositionCallbackObject_free(CIPositionCallbackObject *positionCallback)
{
	delete (CPositionUpdate *)positionCallback;
}


CIVenueDetectionCallbackObject* VenueDetectionCallbackObject_new(const CIVenueDetectionUpdate venueCbk)
{
    CVenueDetectionUpdate* pCbkObj = new CVenueDetectionUpdate(venueCbk);
    CIVenueDetectionCallbackObject *pCallbackObj = (CIVenueDetectionCallbackObject*)pCbkObj;
    return pCallbackObj;
}

void VenueDetectionCallbackObject_free(CIVenueDetectionCallbackObject *venueDetectionCallback)
{
    delete (CVenueDetectionUpdate *)venueDetectionCallback;
}

CIExtendedProximityCallbackObject* ExtendedProximityCallbackObject_new(const CIExtendedProximityUpdate Cbk)
{
    CExtendedProximityUpdate* pCbkObj = new CExtendedProximityUpdate(Cbk);
    CIExtendedProximityCallbackObject *pCallbackObj = (CIExtendedProximityCallbackObject*)pCbkObj;
    return pCallbackObj;
}

void ExtendedProximityCallbackObject_free(CIExtendedProximityCallbackObject *pCallback)
{
    delete (CExtendedProximityUpdate *)pCallback;
}

/**
* Sets main filter position callback
* param[in] pPosCbk pointer to the callback implementation
*/
void FPEngine_setPositionCallbackMixed(CIFPEngine* fpEngine, CIPositionCallbackObject *pPosCbkObj)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setPositionCallbackMixed(reinterpret_cast<Fppe::IPositionUpdate*>(pPosCbkObj));
}

/**
* Sets WiFi only position callback
* param[in] pPosCbk pointer to the callback implementation
*/
void FPEngine_setPositionCallbackWiFi(CIFPEngine* fpEngine, CIPositionCallbackObject *pPosCbkObj)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setPositionCallbackWiFi(reinterpret_cast<Fppe::IPositionUpdate*>(pPosCbkObj));
}

/**
* Sets BLE only position callback
* param[in] pPosCbk pointer to the callback implementation
*/
void FPEngine_setPositionCallbackBLE(CIFPEngine* fpEngine, CIPositionCallbackObject *pPosCbkObj)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setPositionCallbackBLE(reinterpret_cast<Fppe::IPositionUpdate*>(pPosCbkObj));
}

/**
* Sets BLE proximity only position callback
* param[in] pPosCbk pointer to the callback implementation
*/
void FPEngine_setPositionCallbackBLEProximity(CIFPEngine* fpEngine, CIPositionCallbackObject *pPosCbkObj)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setPositionCallbackBLEProximity(reinterpret_cast<Fppe::IPositionUpdate*>(pPosCbkObj));
}

/**
* Sets MFP only(mfp+pdr) position callback
* param[in] pPosCbk pointer to the callback implementation
*/
void FPEngine_setPositionCallbackMFP(CIFPEngine* fpEngine, CIPositionCallbackObject *pPosCbkObj)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setPositionCallbackMFP(reinterpret_cast<Fppe::IPositionUpdate*>(pPosCbkObj));
}

/***/
void FPEngine_setExtendedProximityCallback(CIFPEngine* fpEngine, CIExtendedProximityCallbackObject *pExtProxCbkObj)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setExtendedProximityCallback(reinterpret_cast<Fppe::IExtendedProximityUpdate*>(pExtProxCbkObj));
}

/**
* Sets WiFi venue detection callback
* param[in] pPosCbk pointer to the callback implementation
*/
void FPEngine_setVenueDetectionCallbackWiFi(CIFPEngine* fpEngine, CIVenueDetectionCallbackObject *pVenueWiFiCbkObj)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	mFPEngine->setVenueDetectionCallbackWiFi(reinterpret_cast<Fppe::IVenueDetectionUpdate*>(pVenueWiFiCbkObj));
}

/**
* initialize WiFi module and loads fingerprint from  the file
* \param[in] wifi_db_name fingerprint filename
* \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
* \return success status
*/
CReturnStatus FPEngine_initializeWiFI(CIFPEngine* fpEngine, const char* const pWiFiMap, const size_t wifiFileSizeInBytes, const double min_p)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
  CReturnStatus result = (CReturnStatus)mFPEngine->initializeWiFi(pWiFiMap, wifiFileSizeInBytes, min_p);
	return result;
}

DLL_EXPORT CReturnStatus FPEngine_initializeWiFI_Ex(CIFPEngine* fpEngine, const char* const pWiFiMap, const size_t wifiFileSizeInBytes)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    CReturnStatus result = (CReturnStatus)mFPEngine->initializeWiFi(pWiFiMap, wifiFileSizeInBytes);
    return result;
}

/**
* initialize BLE module and loads fingerprint from the file
* \param[in] ble_db_name fingerprint filename
* \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
* \return success status
*/
CReturnStatus FPEngine_initializeBLE(CIFPEngine* fpEngine, const char* const pBleMap, const size_t bleFileSizeInBytes, const double min_p)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	CReturnStatus result = (CReturnStatus)mFPEngine->initializeBLE(pBleMap, bleFileSizeInBytes, min_p);
	return result;
}

CReturnStatus FPEngine_initializeBLE_Ex(CIFPEngine* fpEngine, const char* const pBleMap, const size_t bleFileSizeInBytes)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    CReturnStatus result = (CReturnStatus)mFPEngine->initializeBLE(pBleMap, bleFileSizeInBytes);
    return result;
}

/**
* initialize BLE proximity module and loads fingerprint from the file
* \param[in] ble_db_name fingerprint filename
* \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
* \return success status
*/
CReturnStatus FPEngine_initializeBLEProximity(CIFPEngine* fpEngine, const char* const pBleMap, const size_t bleFileSizeInBytes)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	CReturnStatus result = (CReturnStatus)mFPEngine->initializeBLEProximity(pBleMap, bleFileSizeInBytes);
	return result;
}

/**
* initialize MFP module and loads fingerprint from the file
* \param[in] mfp_db_name fingerprint filename
* \param[in] max_X defines fingerprint size on X axis [m] [TODO  to be moved into FP file]
* \param[in] max_Y defines fingerprint size on Y axis [m] [TODO  to be moved into FP file]
* \param[in] cellSize defines internal map discrete [m] [TODO  to be moved into FP file]
* \param[in] minFloor define minimum floor number [TODO  to be moved into FP file]
* \param[in] maxFloor define maximum floor number [TODO  to be moved into FP file]
* \return success status
*/
CReturnStatus FPEngine_initializeMFP(CIFPEngine* fpEngine, const char* const pMFPMap, const size_t mfpFileSizeInBytes, const double max_X, const double max_Y, const double cellSize, const int minFloor, const int maxFloor)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	CReturnStatus result = (CReturnStatus)mFPEngine->initializeMFP(pMFPMap, mfpFileSizeInBytes, max_X, max_Y, cellSize, minFloor, maxFloor);
	return result;
}

/**
* initialize MFP module and loads fingerprint from a specified buffer
* mfp4 format is supported
* \param[in] fpEngine - CIFPEngine instance pointer
* \param[in] pMFPMap - MFP FP buffer pointer (includeing header)
* \param[in] mfpFileSizeInBytes - MFP FP buffer size (includeing header)
* \return success status
*/
CReturnStatus FPEngine_initializeMFP_Ex(CIFPEngine* fpEngine, const char* const pMFPMap, const size_t mfpFileSizeInBytes)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    CReturnStatus result = (CReturnStatus)mFPEngine->initializeMFP(pMFPMap, mfpFileSizeInBytes);
    return result;
}

/**
* initialize Map Matching module and loads map from  the specified memory buffer
* \param[in] pMap map buffer
* \param[in] mapFileSizeInBytes size of the buffer in bytes
* \return success status
*/
CReturnStatus FPEngine_initializeMapMatching(CIFPEngine* fpEngine, const uint8_t* const pMap, const size_t mapFileSizeInBytes)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    CReturnStatus result = (CReturnStatus)mFPEngine->initializeMapMatching(pMap, mapFileSizeInBytes);
    return result;
}

/**
* Defines venue local frame
* \param[in] venue  venue parameters with local frame origin
* \return success
*/
CReturnStatus FPEngine_setVenueParams(CIFPEngine* fpEngine, const Venue* venue)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    CReturnStatus result = (CReturnStatus)mFPEngine->setVenueParams(*venue);
    return result;
}

/**
* Set venue parameters (local frame, size)
*/
DLL_EXPORT CReturnStatus FPEngine_setVenueParamsEx(CIFPEngine* fpEngine, const BaseVenueType* venue)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    CReturnStatus result = (CReturnStatus)mFPEngine->setVenueParams(*venue);
    return result;
}

/**
* Get venue parameters
*/
DLL_EXPORT BaseVenueType FPEngine_getVenueParamsEx(CIFPEngine* fpEngine)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    BaseVenueType venue = mFPEngine->getVenueParams();
    return venue;
}

/**
* Get WiFi fingerprint info
*\param[in] fpEngine - CIFPEngine instance pointer
* \return structure FPHeaderBaseType with WiFi fingerprint information
*/
DLL_EXPORT FPHeaderBaseType FPEngine_getWfpInfo(CIFPEngine* fpEngine)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    FPHeaderBaseType fpHeader = mFPEngine->getWfpInfo();
    return fpHeader;
}

/**
* Get magnetic fingerprint info
*/
DLL_EXPORT FPHeaderBaseType FPEngine_getMfpInfo(CIFPEngine* fpEngine)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    FPHeaderBaseType fpHeader = mFPEngine->getMfpInfo();
    return fpHeader;
}

/**
* Get BLE fingerprint info
*/
DLL_EXPORT FPHeaderBaseType FPEngine_getBfpInfo(CIFPEngine* fpEngine)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    FPHeaderBaseType fpHeader = mFPEngine->getBfpInfo();
    return fpHeader;
}

/**
* Get BLE proximity BD info
*/
DLL_EXPORT FPHeaderBaseType FPEngine_getProximityDbInfo(CIFPEngine* fpEngine)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	FPHeaderBaseType fpHeader = mFPEngine->getProximityDbInfo();
    return fpHeader;
}


/** updater WiFi control
* \param[in] enable control flag
*/
void FPEngine_setUpdateWiFi(CIFPEngine* fpEngine, bool enable)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	mFPEngine->setUpdateWiFi(enable);
}

/** updater BLE control
* \param[in] enable control flag
*/
void FPEngine_setUpdateBLE(CIFPEngine* fpEngine, bool enable)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	mFPEngine->setUpdateBLE(enable);
}

/** updater BLE proximity control
* \param[in] enable control flag
*/
void FPEngine_setUpdateBLEProximity(CIFPEngine* fpEngine, bool enable)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	mFPEngine->setUpdateBLEProximity(enable);
}

/** updater MFP control
*\ param[in] enable control flag
*/
void FPEngine_setUpdateMFP(CIFPEngine* fpEngine, bool enable)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	mFPEngine->setUpdateMFP(enable);
}

/** updater Map Matching control
*\ param[in] enable control flag
*/
void FPEngine_setUpdateMapMatching(CIFPEngine* fpEngine, bool enable) /**< updater Map Matching control */
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setUpdateMapMatching(enable);
}

/** updater External Pos control
* \param[in] enable set Pos update enable/disable
*/
void FPEngine_setUpdateExternalPosition(CIFPEngine* fpEngine, bool enable)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setUpdateExternalPosition(enable);
}

/** updater Collabortion control
* \param[in] enable set Pos update enable/disable
*/
void FPEngine_setUpdateCollaboration(CIFPEngine* fpEngine, bool enable)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	mFPEngine->setUpdateCollaboration(enable);
}

/** Corrector fusion filter position control
* \param[in] enable set Pos update enable/disable
*/
void FPEngine_setCorrectFusionFilterPosition(CIFPEngine* fpEngine, bool enable)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setCorrectFusionFilterPosition(enable);
}

/** Enables floor increment usage, switches to another motion model
* \param[in] enable control flag, if enabled uses increments in motion model, otherwise use deterministic floor model
*/
void FPEngine_setFloorIncrements(CIFPEngine* fpEngine, bool enable)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	mFPEngine->setFloorIncrements(enable);
}

/** Gets mg calibration params
* \param[out] bias_cov esimated magnetic bias with covariance
*/
bool FPEngine_getMagneticBias(CIFPEngine* fpEngine, CMagneticCalibrationParam *bias_cov)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;

	Fppe::MagneticCalibrationParam bias_cov_cpp;
	bool result = mFPEngine->getMagneticBias(&bias_cov_cpp);

	bias_cov->timestamp = bias_cov_cpp.timestamp;
	bias_cov->mX = bias_cov_cpp.mX;
	bias_cov->mY = bias_cov_cpp.mY;
	bias_cov->mZ = bias_cov_cpp.mZ;
	memcpy(bias_cov->covarianceMatrix, bias_cov_cpp.covarianceMatrix, sizeof(bias_cov->covarianceMatrix));
    
    return result;
}

/** Gets mg calibration params
* \param[in] bias_cov initial magnetic bias with covariance
*/
void FPEngine_setMagneticBias(CIFPEngine* fpEngine, const CMagneticCalibrationParam* bias_cov)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;

	Fppe::MagneticCalibrationParam bias_cov_cpp;
	bias_cov_cpp.timestamp = bias_cov->timestamp;
	bias_cov_cpp.mX = bias_cov->mX;
	bias_cov_cpp.mY = bias_cov->mY;
	bias_cov_cpp.mZ = bias_cov->mZ;
	memcpy(bias_cov_cpp.covarianceMatrix, bias_cov->covarianceMatrix, sizeof(bias_cov->covarianceMatrix));

	mFPEngine->setMagneticBias(bias_cov_cpp);
}

/** Gets platform type
* \return platform type
*/
PlatformType FPEngine_getPlatformType(CIFPEngine* fpEngine)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    return mFPEngine->getPlatformType();
}

/** Sets platform type
* \param[in] platform type
*/
void FPEngine_setPlatformType(CIFPEngine* fpEngine, PlatformType platform_type)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setPlatformType(platform_type);
}

/** Sets BLP pulling type
* \param[in] pulling type
* \param[in] pulling distance
* \param[in] pulling sigma
*/
void FPEngine_setBlpPulling(CIFPEngine* fpEngine, ePullingType type, double pulling_distance)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setBlpPulling(type, pulling_distance);
}

void FPEngine_setBlpDetectionEnable(CIFPEngine* fpEngine, bool enable)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setBlpDetectionEnable(enable);
}

void FPEngine_setBlpPositioningPdFilterParams(CIFPEngine* fpEngine,
    int peak_detector_max_delay_ms_in_moving,
    double descending_factor_in_moving,
    int peak_detector_max_delay_ms_in_stop,
    double descending_factor_in_stop
)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setBlpPositioningPdFilterParams(
        peak_detector_max_delay_ms_in_moving,
        descending_factor_in_moving,
        peak_detector_max_delay_ms_in_stop,
        descending_factor_in_stop);
}

void FPEngine_setBlpDetectionPdFilterParams(CIFPEngine* fpEngine,
    int peak_detector_max_delay_ms_in_moving,
    double descending_factor_in_moving,
    int peak_detector_max_delay_ms_in_stop,
    double descending_factor_in_stop
)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setBlpDetectionPdFilterParams(
        peak_detector_max_delay_ms_in_moving,
        descending_factor_in_moving,
        peak_detector_max_delay_ms_in_stop,
        descending_factor_in_stop);
}

void FPEngine_setBlpPositioningLogicParams(CIFPEngine* fpEngine,
    int filter_length, int repeat_number, int cutoff)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setBlpPositioningLogicParams(filter_length, repeat_number, cutoff);
}

void FPEngine_setBlpDetectionLogicParams(CIFPEngine* fpEngine,
    int filter_length, int repeat_number, int cutoff)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setBlpDetectionLogicParams(filter_length, repeat_number, cutoff);
}

/** gets Wi-Fi bias
* \param[out] bias rssi bias
* returns ReturnStatus::STATUS_SUCCESS if success, otherwise  error code
*/
CReturnStatus FPEngine_getWiFiBias(CIFPEngine* fpEngine, double *bias)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	return (CReturnStatus)mFPEngine->getWiFiBias(bias);
}

/** sets WiFi bias
* param[in] bias initial rssi bias
* param[in] delta_t time since last saved bias
*/
void FPEngine_setWiFiBias(CIFPEngine* fpEngine, const double* bias, int64_t delta_t)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	mFPEngine->setWiFiBias(*bias, delta_t);
}

/** gets BLE bias
* \param[out] bias rssi bias
* returns ReturnStatus::STATUS_SUCCESS if success, otherwise  error code
*/
CReturnStatus FPEngine_getBLEBias(CIFPEngine* fpEngine, double *bias)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	return (CReturnStatus)mFPEngine->getBLEBias(bias);
}

/** sets BLE bias
* param[in] bias initial rssi bias
* param[in] delta_t time since last saved bias
*/
void FPEngine_setBLEBias(CIFPEngine* fpEngine, const double* bias, int64_t delta_t)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	mFPEngine->setBLEBias(*bias, delta_t);
}

/** gets BLE bias and BLE bias uncertainty
* \param[out] bias - RSSI bias
* \param[out] bias_uncertainty - RSSI bias uncertainty
* returns ReturnStatus::STATUS_SUCCESS if success, otherwise  error code
*/
CReturnStatus FPEngine_getBLEBiasWithUncertainty(CIFPEngine* fpEngine, double *bias, double *bias_uncertainty)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	return (CReturnStatus)mFPEngine->getBLEBiasWithUncertainty(bias, bias_uncertainty);
}

/** sets BLE bias and bias uncertainty
* param[in] bias - RSSI bias
* param[in] bias_uncertainty - RSSI bias uncertainty
* param[in] bias_age - time since last saved bias [ms]
*/
void FPEngine_setBLEBiasWithUncertainty(CIFPEngine* fpEngine, const double*  bias, const double *bias_uncertainty, int64_t bias_age)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	mFPEngine->setBLEBiasWithUncertainty(*bias, *bias_uncertainty, bias_age);
}

/**
* Sets fine known initial position
* \param[in] position initial position in global frame
*/
void FPEngine_setStartPosition(CIFPEngine* fpEngine, const CPosition* position)
{
	Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
	mFPEngine->setStartPosition(*(Fppe::Position*)position);
}

/**
* Sets random seeds for internal randomizers
*/
void FPEngine_setRandomSeeds(CIFPEngine* fpEngine)
{
  Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
  mFPEngine->setRandomSeeds();
}

/**
* set use barometr
* param[in] enable/disable
*/
void FPEngine_setUseBarometer(CIFPEngine* fpEngine, bool enable)
{

  Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
  mFPEngine->setUseBarometer(enable);
}

/**
* set OS type
* param[in] OS type
*/
void FPEngine_setOsType(CIFPEngine* fpEngine, OperationSystemType os_type)
{

  Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
  mFPEngine->setOsType(os_type);
}


/**
* Control Mag PF
* param[in] enable disables/enables Mag filter
*/
void FPEngine_setMagFilterEnabled(CIFPEngine* fpEngine, bool enable)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setMagFilterEnabled(enable);
}

/**
* Control Mixed PF
* param[in] enable disables/enables Mixed filter
*/
void FPEngine_setMixedFilterEnabled(CIFPEngine* fpEngine, bool enable)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    mFPEngine->setMixedFilterEnabled(enable);
}

/**
* return Mag filter state
*/
bool FPEngine_getMagFilterEnabled(CIFPEngine* fpEngine)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    return mFPEngine->getMagFilterEnabled();
}

/**
* return Mixed filter state
*/
bool FPEngine_getMixedFilterEnabled(CIFPEngine* fpEngine)
{
    Fppe::FPEngine* mFPEngine = (Fppe::FPEngine*)fpEngine;
    return mFPEngine->getMixedFilterEnabled();
}