#include <math.h>
#include <iostream> // cout
#include <fstream> // ifstream, ofstream
#include <sstream> // string to numbers

#include "kalman_filter.h"

using namespace std;

void handle_io(string input_filename,string output_filename);
void process_data(int32_t * input_int32_t, float * input_float, float * output);

int main(void)
{
  string input_filename = "a03_straight1_13-49-08.CSV";
  string output_filename = "signal_a.csv";

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
  const int output_cols = 3;
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

    process_data(input_int32_t,input_float,output);

		//write row
    for(int i = 0; i < output_cols; i++)
    {
  		ofs << str << "," << output[i];
    }
    ofs << endl;

	}
}
void process_data(int32_t * input_int32_t, float * input_float, float * output)
{
  static bool gpsflag_ = false;
  static bool visionflag_ = false;

  static int32_t longitude,latitude,height_mean_sea_level;
  static uint32_t horizontal_accuracy, vertical_accuracy;
  static float vision[3]; // velocity in b-frame (m/s)
  static float accelerometer[3],gyro[3];

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

      //cout << "a:" << accelerometer[0] << " " << accelerometer[1] << " " << accelerometer[2] << " ";
      //cout << "g:" << gyro[0] << " " << gyro[1] << " " << gyro[2] << " " << endl;

      KalmanTimeUpdate(gyro, accelerometer);
      //KalmanAccelerometerUpdate(accelerometer);
      //if(gpsflag_)
      //{
      //  //KalmanGPSPositionUpdate(longitude, latitude, height_mean_sea_level,
      //  //  horizontal_accuracy, vertical_accuracy);
      //  gpsflag_ = false;
      //}
      //if(visionflag_)
      //{
      //  KalmanVisionUpdate(vision);
      //  visionflag_ = false;
      //}

      //cout << KalmanPosition()[0] << " " << KalmanPosition()[1] << " " << KalmanPosition()[2] << " " << endl;
      //cout << KalmanVelocity()[0] << " " << KalmanVelocity()[1] << " " << KalmanVelocity()[2] << " " << endl;
      break;
    case 1: // GPS
      longitude = input_int32_t[3];
      latitude = input_int32_t[4];
      height_mean_sea_level = input_int32_t[5];
      horizontal_accuracy = (uint32_t) input_int32_t[6];
      vertical_accuracy = (uint32_t) input_int32_t[7];
      //cout << longitude << latitude << height_mean_sea_level<<endl;
      //cout << horizontal_accuracy << " " << vertical_accuracy << endl;
      gpsflag_ = true;
      break;
    case 8: // Ricoh sensor
      vision[0] = input_float[5] * 0.001 * 30;
      vision[1] = input_float[5] * 0.001 * 30;
      vision[2] = input_float[5] * 0.001 * 30;
      //cout << "v:" << vision[0] << " " <<vision[1] << " " <<vision[2] << " " <<endl;
      visionflag_ = true;
      break;
    default:
      break;
  }

}
