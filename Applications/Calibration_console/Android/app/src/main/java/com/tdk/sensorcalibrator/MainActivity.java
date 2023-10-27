package com.tdk.sensorcalibrator;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.util.Log;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import android.widget.Toast;
import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.FileDescriptor;
import java.lang.Double;
import java.nio.channels.Channel;

import com.tdk.sensorcalibrator.SensorCalibrator;

public class MainActivity extends AppCompatActivity {

    private SensorCalibrator  mSensorCalibrator;
    private SensorCalibrationParams mCalibrationParams;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        mSensorCalibrator = new SensorCalibrator();
        mCalibrationParams = new SensorCalibrationParams();
        mCalibrationParams.CM = new double[9];
        mCalibrationParams.covB = new double[9];
        Log.d("CalibrationLib","Native code library.\n");

       // 1421751154.266572, 137.399994, -253.199997, 649.200012, 32.625000, 20
        mSensorCalibrator.startCalibration();
        mSensorCalibrator.ClearSensorsData();

        int count = 20000;
        double[] time = new double[count];
        double[] time_m = new double[count];
        double[] xa = new double[count];
        double[] ya = new double[count];
        double[] za = new double[count];
        double[] xg = new double[count];
        double[] yg = new double[count];
        double[] zg = new double[count];
        double[] xm = new double[count];
        double[] ym = new double[count];
        double[] zm = new double[count];
        double[] temperature = new double[count];
        double[] temperature_m = new double[count];
        int Len = count;
        int Len_m = count;

        time[0] =       1421751154.266572;
        time_m[0] =       1421751154.266572;
        xa[0] =        137.399994;
        ya[0] =        -253.199997;
        za[0] = 649.200012;
        xg[0] =        137.399994;
        yg[0] =        -253.199997;
        zg[0] = 649.200012;
        xm[0] =        137.399994;
        ym[0] =        -253.199997;
        zm[0] = 649.200012;
        temperature[0] = 32.625000;
        temperature_m[0] = 32.625000;
        Len = 1;
        Len_m = 1;
/*
        mSensorCalibrator.AddAccelData(
        time,  xa,   ya,   za,    temperature,     Len     );

        mSensorCalibrator.AddGyroData(
                time,  xg,   yg,   zg,    temperature,     Len     );
        mSensorCalibrator.AddMagneticData(
                time_m,  xm,   ym,   zm,    temperature_m,     Len_m     );

        int status = mSensorCalibrator.EstimateAccelCalibrationParams(mCalibrationParams,
                mCalibrationParams.CM, mCalibrationParams.covB);

        status = mSensorCalibrator.EstimateMagneticCalibrationParams(mCalibrationParams,
                mCalibrationParams.CM, mCalibrationParams.covB);
*/
/*
         FileOutputStream fos = null;
        try {
        fos = openFileOutput("content.txt", MODE_PRIVATE);
        fos.write("ffffffffffffffffffffff".getBytes());
        Toast.makeText(this, "Файл сохранен", Toast.LENGTH_SHORT).show();
            fos.flush();
            fos.close();
        }
        catch(IOException ex) {

            Toast.makeText(this, ex.getMessage(), Toast.LENGTH_SHORT).show();
        }
*/

        try {
            InputStream inputStream = openFileInput("2021_05_04_13_07_26_imu.csv");
            if (inputStream != null) {
                InputStreamReader isr = new InputStreamReader(inputStream);
                BufferedReader reader = new BufferedReader(isr);
                String line;
                StringBuilder builder = new StringBuilder();
                int i = 0;
                while ((line = reader.readLine()) != null) {
                    String[] row = line.split(",");
                    time[i] = (Double.valueOf(row[0])).doubleValue();
                    xa[i] = (Double.valueOf(row[1])).doubleValue();
                    ya[i] = (Double.valueOf(row[2])).doubleValue();
                    za[i] = (Double.valueOf(row[3])).doubleValue();
                    xg[i] = (Double.valueOf(row[4])).doubleValue();
                    yg[i] = (Double.valueOf(row[5])).doubleValue();
                    zg[i] = (Double.valueOf(row[6])).doubleValue();
                    temperature[i] = (Double.valueOf(row[7])).doubleValue();
                    //builder.append(line + "\n");
                    i++;
                }
                Len = i;
                inputStream.close();
            }
        } catch (Throwable t) {
            Toast.makeText(getApplicationContext(),
                    "Exception: " + t.toString(), Toast.LENGTH_LONG).show();
        }

        try {
            InputStream inputStream = openFileInput("2021_05_04_13_07_26_mag.csv");
            if (inputStream != null) {
                InputStreamReader isr = new InputStreamReader(inputStream);
                BufferedReader reader = new BufferedReader(isr);
                String line;
                StringBuilder builder = new StringBuilder();
                int i = 0;
                while ((line = reader.readLine()) != null) {
                    String[] row = line.split(",");
                    time_m[i] = (Double.valueOf(row[0])).doubleValue();
                    xm[i] = (Double.valueOf(row[1])).doubleValue();
                    ym[i] = (Double.valueOf(row[2])).doubleValue();
                    zm[i] = (Double.valueOf(row[3])).doubleValue();
                    temperature_m[i] = (Double.valueOf(row[4])).doubleValue();
                    //builder.append(line + "\n");
                    i++;
                }
                Len_m = i;
                inputStream.close();
            }
        } catch (Throwable t) {
            Toast.makeText(getApplicationContext(),
                    "Exception: " + t.toString(), Toast.LENGTH_LONG).show();
        }

        mSensorCalibrator.AddAccelData(
                time,  xa,   ya,   za,    temperature,     Len     );

        mSensorCalibrator.AddGyroData(
                time,  xg,   yg,   zg,    temperature,     Len     );
        mSensorCalibrator.AddMagneticData(
                time_m,  xm,   ym,   zm,    temperature_m,     Len_m     );

        mSensorCalibrator.OpenPrintData("test.txt");
        mSensorCalibrator.PrintData();
        mSensorCalibrator.ClosePrintData();

        int status = mSensorCalibrator.EstimateAccelCalibrationParams(mCalibrationParams,
                mCalibrationParams.CM, mCalibrationParams.covB);


         FileOutputStream fos = null;
        try {
        fos = openFileOutput("2021_05_04_13_07_26_imu_RESULT.txt", MODE_PRIVATE);
            String time_s = Double.toString(mCalibrationParams.time); ///< calibration time stamp, [sec]
            String bx_s = Double.toString(mCalibrationParams.bx);  ///< bias X
            String by_s = Double.toString(mCalibrationParams.by);  ///< bias Y
            String bz_s = Double.toString(mCalibrationParams.bz);  ///< bias Z
            String CM0_s = Double.toString(mCalibrationParams.CM[0]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM1_s = Double.toString(mCalibrationParams.CM[1]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM2_s = Double.toString(mCalibrationParams.CM[2]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM3_s = Double.toString(mCalibrationParams.CM[3]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM4_s = Double.toString(mCalibrationParams.CM[4]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM5_s = Double.toString(mCalibrationParams.CM[5]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM6_s = Double.toString(mCalibrationParams.CM[6]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM7_s = Double.toString(mCalibrationParams.CM[7]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM8_s = Double.toString(mCalibrationParams.CM[8]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String temperature_s = Double.toString(mCalibrationParams.temperature);  ///< sensor temperature through calibration, [C]
            String calibrationLevel_s = Double.toString(mCalibrationParams.calibrationLevel); ///< calibration accuracy level
            String covB0_s = Double.toString(mCalibrationParams.covB[0]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB1_s = Double.toString(mCalibrationParams.covB[1]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB2_s = Double.toString(mCalibrationParams.covB[2]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB3_s = Double.toString(mCalibrationParams.covB[3]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB4_s = Double.toString(mCalibrationParams.covB[4]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB5_s = Double.toString(mCalibrationParams.covB[5]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB6_s = Double.toString(mCalibrationParams.covB[6]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB7_s = Double.toString(mCalibrationParams.covB[7]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB8_s = Double.toString(mCalibrationParams.covB[8]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String DOP_s = Double.toString(mCalibrationParams.DOP); /// estimated dilution of precision during calibration
            String status_s = Integer.toString(status);

            fos.write(("time = " + time_s + "\n").getBytes());
            fos.write(("status = " + status_s + "\n").getBytes());
            fos.write(("DOP = " + DOP_s + "\n").getBytes());
            fos.write(("bx = " + bx_s + "\n").getBytes());
            fos.write(("by = " + by_s + "\n").getBytes());
            fos.write(("bz = " + bz_s + "\n").getBytes());
            fos.write(("CM0 = " + CM0_s + "\n").getBytes());
            fos.write(("CM1 = " + CM1_s + "\n").getBytes());
            fos.write(("CM2 = " + CM2_s + "\n").getBytes());
            fos.write(("CM3 = " + CM3_s + "\n").getBytes());
            fos.write(("CM4 = " + CM4_s + "\n").getBytes());
            fos.write(("CM5 = " + CM5_s + "\n").getBytes());
            fos.write(("CM6 = " + CM6_s + "\n").getBytes());
            fos.write(("CM7 = " + CM7_s + "\n").getBytes());
            fos.write(("CM8 = " + CM8_s + "\n").getBytes());
            fos.write(("temperature = " + temperature_s + "\n").getBytes());
            fos.write(("calibrationLevel = " + calibrationLevel_s + "\n").getBytes());
            fos.write(("covB0 = " + covB0_s + "\n").getBytes());
            fos.write(("covB1 = " + covB1_s + "\n").getBytes());
            fos.write(("covB2 = " + covB2_s + "\n").getBytes());
            fos.write(("covB3 = " + covB3_s + "\n").getBytes());
            fos.write(("covB4 = " + covB4_s + "\n").getBytes());
            fos.write(("covB5 = " + covB5_s + "\n").getBytes());
            fos.write(("covB6 = " + covB6_s + "\n").getBytes());
            fos.write(("covB7 = " + covB7_s + "\n").getBytes());
            fos.write(("covB8 = " + covB8_s + "\n").getBytes());
                Toast.makeText(this, "Файл сохранен", Toast.LENGTH_SHORT).show();
            fos.flush();
            fos.close();
        }
        catch(IOException ex) {

            Toast.makeText(this, ex.getMessage(), Toast.LENGTH_SHORT).show();
        }



        status = mSensorCalibrator.EstimateMagneticCalibrationParams(mCalibrationParams,
                mCalibrationParams.CM, mCalibrationParams.covB);

        fos = null;
        try {
            fos = openFileOutput("2021_05_04_13_07_26_mag_RESULT.txt", MODE_PRIVATE);
            String time_s = Double.toString(mCalibrationParams.time); ///< calibration time stamp, [sec]
            String bx_s = Double.toString(mCalibrationParams.bx);  ///< bias X
            String by_s = Double.toString(mCalibrationParams.by);  ///< bias Y
            String bz_s = Double.toString(mCalibrationParams.bz);  ///< bias Z
            String CM0_s = Double.toString(mCalibrationParams.CM[0]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM1_s = Double.toString(mCalibrationParams.CM[1]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM2_s = Double.toString(mCalibrationParams.CM[2]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM3_s = Double.toString(mCalibrationParams.CM[3]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM4_s = Double.toString(mCalibrationParams.CM[4]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM5_s = Double.toString(mCalibrationParams.CM[5]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM6_s = Double.toString(mCalibrationParams.CM[6]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM7_s = Double.toString(mCalibrationParams.CM[7]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String CM8_s = Double.toString(mCalibrationParams.CM[8]); // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
            String temperature_s = Double.toString(mCalibrationParams.temperature);  ///< sensor temperature through calibration, [C]
            String calibrationLevel_s = Double.toString(mCalibrationParams.calibrationLevel); ///< calibration accuracy level
            String covB0_s = Double.toString(mCalibrationParams.covB[0]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB1_s = Double.toString(mCalibrationParams.covB[1]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB2_s = Double.toString(mCalibrationParams.covB[2]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB3_s = Double.toString(mCalibrationParams.covB[3]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB4_s = Double.toString(mCalibrationParams.covB[4]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB5_s = Double.toString(mCalibrationParams.covB[5]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB6_s = Double.toString(mCalibrationParams.covB[6]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB7_s = Double.toString(mCalibrationParams.covB[7]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String covB8_s = Double.toString(mCalibrationParams.covB[8]); // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
            String DOP_s = Double.toString(mCalibrationParams.DOP); /// estimated dilution of precision during calibration
            String status_s = Integer.toString(status);

            fos.write(("time = " + time_s + "\n").getBytes());
            fos.write(("status = " + status_s + "\n").getBytes());
            fos.write(("DOP = " + DOP_s + "\n").getBytes());
            fos.write(("bx = " + bx_s + "\n").getBytes());
            fos.write(("by = " + by_s + "\n").getBytes());
            fos.write(("bz = " + bz_s + "\n").getBytes());
            fos.write(("CM0 = " + CM0_s + "\n").getBytes());
            fos.write(("CM1 = " + CM1_s + "\n").getBytes());
            fos.write(("CM2 = " + CM2_s + "\n").getBytes());
            fos.write(("CM3 = " + CM3_s + "\n").getBytes());
            fos.write(("CM4 = " + CM4_s + "\n").getBytes());
            fos.write(("CM5 = " + CM5_s + "\n").getBytes());
            fos.write(("CM6 = " + CM6_s + "\n").getBytes());
            fos.write(("CM7 = " + CM7_s + "\n").getBytes());
            fos.write(("CM8 = " + CM8_s + "\n").getBytes());
            fos.write(("temperature = " + temperature_s + "\n").getBytes());
            fos.write(("calibrationLevel = " + calibrationLevel_s + "\n").getBytes());
            fos.write(("covB0 = " + covB0_s + "\n").getBytes());
            fos.write(("covB1 = " + covB1_s + "\n").getBytes());
            fos.write(("covB2 = " + covB2_s + "\n").getBytes());
            fos.write(("covB3 = " + covB3_s + "\n").getBytes());
            fos.write(("covB4 = " + covB4_s + "\n").getBytes());
            fos.write(("covB5 = " + covB5_s + "\n").getBytes());
            fos.write(("covB6 = " + covB6_s + "\n").getBytes());
            fos.write(("covB7 = " + covB7_s + "\n").getBytes());
            fos.write(("covB8 = " + covB8_s + "\n").getBytes());
            Toast.makeText(this, "Файл сохранен", Toast.LENGTH_SHORT).show();
            fos.flush();
            fos.close();
        }
        catch(IOException ex) {

            Toast.makeText(this, ex.getMessage(), Toast.LENGTH_SHORT).show();
        }


    }
}
