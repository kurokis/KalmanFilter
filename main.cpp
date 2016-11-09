#include <math.h>
#include <iostream> // cout
#include <fstream> // ifstream, ofstream
#include <sstream> // string to numbers

#include "kalman_filter.h"
#include "matrix.h" // debug

using namespace std;

void handle_io(string input_filename,string output_filename);
bool process_data(int32_t * input_int32_t, float * input_float, float * output);

int main(void)
{

  string input_filename = "simulator.csv";
  string output_filename = "output.csv";

  ResetKalman();
  handle_io(input_filename,output_filename);

  return 0;
}
void handle_io(string input_filename,string output_filename)
{
  // initialize file read
  ifstream ifs(input_filename);
  if (!ifs) {
    cout << "Error:Input data file not found" << endl;
  }

  // initialize file write
  ofstream ofs(output_filename);

  // process each row in signal.csv and write output to signal_a.csv
  string str, token;

  const int input_cols_max = 24;
  const int output_cols = 28;
  int32_t input_int32_t[input_cols_max];
  float input_float[input_cols_max];
  float output[output_cols];
  char * e;

  while (getline(ifs, str))
  {
    int cols = 0;

    // read row
    istringstream stream(str);
    while (getline(stream, token, ','))
    {
      input_int32_t[cols] = (int32_t) std::stol(token);
      input_float[cols] = std::stof(token);
      cols++;
    }

    bool advance_timestep = process_data(input_int32_t,input_float,output);
    if(advance_timestep)
    {
      //write row
      for(int i = 0; i < output_cols; i++)
      {
        ofs << output[i] << ",";
      }
      ofs << endl;
    }
  }
}
bool process_data(int32_t * input_int32_t, float * input_float, float * output)
{

  // options
  static bool usevision = true;
  static bool usegps = false;
  static bool calibrate = true;

  // static variables
  static bool gpsflag = false;
  static bool visionflag = false;
  static bool firstimu = true;
  static bool firstgps = true;
  static bool advance_timestep = false;

  static int32_t longitude,latitude,height_mean_sea_level;
  static uint32_t horizontal_accuracy, vertical_accuracy;
  static float vision[3]; // velocity in b-frame (m/s)
  static float accelerometer[3],gyro[3];
  static float accelerometer_bias[3],gyro_bias[3];

  switch((int) input_int32_t[0])
  {
    case 0: // IMU
      // Scale is 5/1024/ADC_N_SAMPLES g/LSB.
      accelerometer[0] = ((float)input_int32_t[2])*5/1024/8 * 9.8; // m/s^2
      accelerometer[1] = ((float)input_int32_t[3])*5/1024/8 * 9.8; // m/s^2
      accelerometer[2] = ((float)input_int32_t[4])*5/1024/8 * 9.8; // m/s^2

      // Scale is 5/6.144/ADC_N_SAMPLES deg/s/LSB.
      gyro[0] = ((float)input_int32_t[5])*5/6.144/8 * 0.01745; // rad/s
      gyro[1] = ((float)input_int32_t[6])*5/6.144/8 * 0.01745; // rad/s
      gyro[2] = ((float)input_int32_t[7])*5/6.144/8 * 0.01745; // rad/s

      if(firstimu && calibrate)
      {
        accelerometer_bias[0] = accelerometer[0];
        accelerometer_bias[1] = accelerometer[1];
        accelerometer_bias[2] = accelerometer[2] + 9.8;
        gyro_bias[0] = gyro[0];
        gyro_bias[1] = gyro[1];
        gyro_bias[2] = gyro[2];
        firstimu = false;
      }
      accelerometer[0] -= accelerometer_bias[0];
      accelerometer[1] -= accelerometer_bias[1];
      accelerometer[2] -= accelerometer_bias[2];
      gyro[0] -= gyro_bias[0];
      gyro[1] -= gyro_bias[1];
      gyro[2] -= gyro_bias[2];

      KalmanTimeUpdate(gyro, accelerometer);

      static int ctr = 2;
      cout << endl << endl<< "ROW " << ctr++ << endl;

      //cout << "x pred:\n";
      //cout << KalmanQuat()[0] << " " << KalmanQuat()[1] << " " << KalmanQuat()[2] << " " << KalmanQuat()[3] << " ";
      //cout << KalmanVelocity()[0] << " " << KalmanVelocity()[1] << " " << KalmanVelocity()[2] << " ";
      //cout << KalmanPosition()[0] << " " << KalmanPosition()[1] << " " << KalmanPosition()[2] << " " << endl;

      cout << "Accelerometer update\n";
      KalmanAccelerometerUpdate(accelerometer);

      if(gpsflag && usegps)
      {
        cout << "GPS update" <<endl;
        KalmanGPSPositionUpdate(longitude, latitude, height_mean_sea_level,
          horizontal_accuracy, vertical_accuracy);
        gpsflag = false;
      }
      if(visionflag && usevision)
      {
        cout << "Vision update\n";
        KalmanVisionUpdate(vision);
        visionflag = false;
      }

      KalmanPerformMeasurementUpdate();

      cout << "x est:\n";
      cout << KalmanQuat()[0] << " " << KalmanQuat()[1] << " " << KalmanQuat()[2] << " " << KalmanQuat()[3] << " ";
      cout << KalmanVelocity()[0] << " " << KalmanVelocity()[1] << " " << KalmanVelocity()[2] << " ";
      cout << KalmanPosition()[0] << " " << KalmanPosition()[1] << " " << KalmanPosition()[2] << " " << endl;

      advance_timestep = true;
      break;
    case 1: // GPS
      cout << "GPS found" <<endl;
      longitude = input_int32_t[3];
      latitude = input_int32_t[4];
      height_mean_sea_level = input_int32_t[5];
      horizontal_accuracy = (uint32_t) input_int32_t[6];
      vertical_accuracy = (uint32_t) input_int32_t[7];
      gpsflag = true;
      if(firstgps)
      {
        SetGpsHome(longitude,latitude,height_mean_sea_level,(80.0)*0.01745);
        firstgps = false;
      }

      advance_timestep = false;
      break;
    case 8: // Ricoh sensor
      vision[0] = input_float[5] * 0.001 * 30;
      vision[1] = input_float[6] * 0.001 * 30;
      vision[2] = input_float[7] * 0.001 * 30;
      visionflag = true;

      advance_timestep = false;
      break;
    default:
      break;
  }

  output[0] = accelerometer[0];
  output[1] = accelerometer[1];
  output[2] = accelerometer[2];
  output[3] = gyro[0];
  output[4] = gyro[1];
  output[5] = gyro[2];
  output[6] = vision[0];
  output[7] = vision[1];
  output[8] = vision[2];
  output[9] = KalmanQuat()[0];
  output[10] = KalmanQuat()[1];
  output[11] = KalmanQuat()[2];
  output[12] = KalmanQuat()[3];
  output[13] = KalmanVelocity()[0];
  output[14] = KalmanVelocity()[1];
  output[15] = KalmanVelocity()[2];
  output[16] = KalmanPosition()[0];
  output[17] = KalmanPosition()[1];
  output[18] = KalmanPosition()[2];
  output[19] = KalmanPAlpha()[0*3+0];
  output[20] = KalmanPAlpha()[1*3+1];
  output[21] = KalmanPAlpha()[2*3+2];
  output[22] = KalmanPVelocity()[0*3+0];
  output[23] = KalmanPVelocity()[1*3+1];
  output[24] = KalmanPVelocity()[2*3+2];
  output[25] = KalmanPVelocity()[0*3+0];
  output[26] = KalmanPVelocity()[1*3+1];
  output[27] = KalmanPVelocity()[2*3+2];
  return advance_timestep;
}
