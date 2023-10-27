/**
* \file MfpBuilder.hpp
* \brief MFP Fingerprint Builder
* \author Mikhail Frolov (mfrolov@invensense.com)
* \version 0.1
* \copyright InvenSense, all rights reserved
* \date February 18, 2016
*/

#ifndef MFP_BUILDER_HPP
#define MFP_BUILDER_HPP

#include "Fpbl.hpp"
#include <iterator>
#include <cassert>
#include <fstream>
#include <math.h>
#include "IStdEstimator.hpp"

/**
 * Library namespace
 */
namespace Fpbl
{
    /**
    * MfpBuilder process mfp-grid data and generates  fingerprint
    */
	class IMfpBuilder;
	class MfpBuilder
    {
        public:

            /** fingerprint cell description, GM model */
            struct DBRecord
            {
                /** gaussian mix model parameters */
                struct GaussMix
                {
                    float mu1;  /**< gaussian #1 mean */
                    float s1;   /**< gaussian #1 std */
                    float w1;   /**< gaussian #1 weight */
                    float mu2;  /**< gaussian #2 mean */
                    float s2;   /**< gaussian #2 std */
                    float w2;   /**< gaussian #2 weight */
                    float reserved[2]; /**< reserved fileds */
                };

                /** field componets */
                ///@{
                GaussMix x;
                GaussMix y;
                GaussMix z;
                ///@}
            };

            class LocalDB
            {
                public: ///< TODO  hide data storage
                    typedef std::vector<DBRecord> Records;
                    Records records;
                    Grid    grid;
                    int     min_data_count;        // minimal permited data number per cell
                    typedef Records::const_iterator const_iterator;
                    typedef Records::iterator iterator;
                public:
                    LocalDB( FPGrid fingerprint )
                    {
                        grid = fingerprint.grid;
                        min_data_count = fingerprint.min_data_count;

                        assert(grid.max.x > grid.min.x);
                        assert( grid.max.y > grid.min.y );
                        assert( grid.max.floor >= grid.min.floor );
                        assert( grid.size >= .1 );
                        assert( grid.type == CellType::CELL_SQUARE );

                        records.resize( getCellsCount() );

                    }
                    const_iterator cbegin() const
                    {
                        return records.cbegin();
                    }
                    const_iterator cend() const
                    {
                        return records.cend();
                    }
                    Grid getGrid() const
                    {
                        return grid;
                    }

					ReturnStatus getRecord(const CoordinatesInGrid &position, DBRecord *record)
					{
						auto it = findRecord(position);
						if (it == records.end())
							return ReturnStatus::STATUS_UNKNOWN_ERROR;
						else
						{
							DBRecord tmp_record = *it;
							*record = tmp_record;
							return ReturnStatus::STATUS_SUCCESS;
						}

					}

                    //ReturnStatus getRecord( const CoordinatesInGrid &position, DBRecord *record ) const;
                    iterator findRecord( const CoordinatesInGrid &position )
                    {
                        size_t i_x = static_cast<size_t>( floor( ( position.x - grid.min.x - grid.size / 2 ) / grid.size + 0.5 ) );
                        size_t i_y = static_cast<size_t>( floor( ( position.y - grid.min.y - grid.size / 2 ) / grid.size + 0.5 ) );

                        if ( i_x >= size_x() ||
                                i_y >= size_y() ||
                                position.floor < grid.min.floor ||
                                position.floor > grid.max.floor )
                        {
                            return records.end();
                        }


                        size_t offset = ( position.floor - grid.min.floor ) * size_x() * size_y() + i_y * size_x() + i_x;

                        return records.begin() + offset;
                    }

                    size_t size() const
                    {
                        return getCellsCount();
                    }
                    size_t size_x() const
                    {
                        return static_cast<size_t>( ceil( ( grid.max.x - grid.min.x ) / grid.size ) );
                    }
                    size_t size_y() const
                    {
                        return static_cast<size_t>( ceil( ( grid.max.y - grid.min.y ) / grid.size ) );
                    }
                    size_t size_floor() const
                    {
                        return static_cast<size_t>( grid.max.floor - grid.min.floor + 1 );
                    }
                    size_t getCellsCount() const
                    {
                        size_t result = size_x() * size_y() * size_floor();
                        return result;
                    }
                    size_t size_in_bytes() const
                    {
                        return getCellsCount() * sizeof(DBRecord);
                    }
            };

            MfpBuilder();
            ~MfpBuilder();
            void setStdEstimatorType(eStdEstimatorType stdEstimatorType);
            void setDefaultMagdata(double mag_x, double mag_y, double mag_z);

            ReturnStatus buildFingerprint( const MagneticGrid &mfpGrid, LocalDB *mfpFp );
            ReturnStatus updateFingerprint(const Fpbl::PortalsGrid &portalGrid, Fpbl::MfpBuilder::LocalDB *mfpFp);
			ReturnStatus updateFingerprintForCrowdsourced(const Fpbl::CSGrid &csGrid, Fpbl::MfpBuilder::LocalDB *mfpFp);

        private:
            MfpBuilder( const MfpBuilder & ); //disable copy constructor
            MfpBuilder &operator=( const MfpBuilder & ); //disable copy
            //friend class IMfpBuilder;
            IMfpBuilder * mMfpBuilder;
    };

}
#endif
