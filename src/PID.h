#ifndef PID_H
#define PID_H

#include <vector>
#include <numeric>
#include <chrono>
#include <iostream>
#include <vector>
#include <random>
#include <cmath>

using std::vector;

class PID {
 public:
  /**
   * Constructor
   */
  PID(int tuning_data_size = 150, int num_records_to_skip = 50,
      float max_steering_amgle=1.0, float twiddle_tolerance = 0.01,
      double steering_noise = 0.0, double distance_noise = 0.0);

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Updates the PID given cross track error, steering angle, and speed.
   * @param cte The current cross track error
   * @param steering_angle The current steering angle
   * @param speed The current speed
   */
  void Update(double cte, double steering_angle, double speed);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Enables parameter tuning
   */
  inline void EnableParameterTuning(){this->initiate_parameter_tuning = true;};

  /**
   * Disables parameter tuning
   */
  inline void DisableParameterTuning(){this->initiate_parameter_tuning = false;};

  /**
   * Returns whether parameter tuning is enabled
   */
  inline bool IsParameterTuningEnabled(){return (this->initiate_parameter_tuning == true);};

 private:

  /**
   * Tune the PID control gains Kp, Ki, Kd.
   * @output Tuned control gains
   */
  vector<double> TuneControlGains();

  /**
   * Simulates a car advance/step-movement for tuning Control Gains (params).
   * @param (ref)x The current x-position of the car
   * @param (ref)y The current y-position of the car
   * @param (ref)orientation The current orientation of the car
   * @param steering The steering angle to use
   * @param distance The distance to move
   */
  void SimulateMove(double& x, double&y, double& orientation,
                    const double steering, const double distance);

  /**
   * Simulates a car trajectory based on orientation, steering angle, and
   * distance traveled.
   * @param params The current control gains
   */
  double SimulateTrajectory(const vector<double>& params);

  /**
   * Cost function (Sum of Squared Error) for tuning Control Gains (params).
   */
  // double CalculateCost(const vector<double> params, const vector<double> errors);
  double CalculateCost(const vector<double> CTEs);

  /**
   * Helper function to print the accumulated training data
   */
  void PrintTrainingData();

  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * Enum for indices of training data features
   */
  enum {
    CTE_IDX = 0,
    STEERING_ANGLE_IDX,
    SPEED_IDX,
    ELAPSED_TIME_IDX
  };

//  /**
//   * Elapsed time between messages
//   */
  std::chrono::time_point<std::chrono::high_resolution_clock> t_start;
  std::chrono::time_point<std::chrono::high_resolution_clock> t_stop;


  /**
   * History of records, that we will use to tune the control gain params
   * Contains 4 rows (features) and kNumTuningDataRecords columns/entries
   * Features: cte, steering angle, speed, elapsed time in seconds
   */
  // number of records used for tuning the control gain params
  const unsigned int kNumTuningDataRecords;
  // number of rows/features of the tuning data
  const unsigned int kNumTuningDataFeatures;
  // tolerance for Twiddle algorithm
  const double kTwiddleTolerance;
  // number of initial training records to skip while training
  const double kNumRecordsToSkip;
  // max steering angle in both directions
  const double kMaxSteeringAngle;
  // steering noise
  double steering_noise;
  // distance noise
  double distance_noise;
  // training data for tuning control input params
  vector<vector<double>> control_gains_tuning_data;
  bool initiate_parameter_tuning;
  bool params_tuned;


};

#endif  // PID_H
