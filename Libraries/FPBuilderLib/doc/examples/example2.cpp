#include "Fpbl.hpp"
#include "MFPBuilder.hpp"

Fpbl::Grid venue_grid_mfp;
//define area for FP building
venue_grid_mfp.type = Fpbl::CellType::CELL_SQUARE; //now supported only CELL_SQUARE
venue_grid_mfp.size = 1.0; // cell size [m]
//define rectangle area [[0,0,0],[64,32,0]]
venue_grid_mfp.min.x = 0.0;
venue_grid_mfp.min.y = 0.0;
venue_grid_mfp.min.floor = 0;
venue_grid_mfp.max.x = 64.0;
venue_grid_mfp.max.y = 32.0;
venue_grid_mfp.max.floor = 0;

Fpbl::MfpBuilder mfp_builder;
Fpbl::MfpBuilder::LocalDB db( venue_grid_mfp ); //create empty fingerprint

//build fingerprint
Fpbl::ReturnStatus result = mfp_builder.buildFingerprint( magnetic_grid, &db );

std::string output_file_name = "fingerprint.mfp3";
//save magnetic fingerprint in "mfp3" format
Fpbl::ReturnStatus save_file_result = db.save2File( output_file_name );
