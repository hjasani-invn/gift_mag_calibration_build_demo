# Python script to modify the Makefile while preserving formatting

# Specify the path to the original Makefile
makefile_path = "C:\ProgramData\Jenkins\.jenkins\workspace\mag_calibration_1\Libraries\CalibrationLib\makefile_stat"

# Read the original Makefile
with open(makefile_path, "r") as original_makefile:
    makefile_content = original_makefile.read()

# Replace 'mkdir -p' with 'md .\\linux\\obj\\'
modified_makefile_content = makefile_content.replace("mkdir -p", "md .\\linux\\obj\\")

# Write the modified Makefile back to the original file
with open(makefile_path, "w") as original_makefile:
    original_makefile.write(modified_makefile_content)
