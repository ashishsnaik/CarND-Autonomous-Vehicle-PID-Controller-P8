#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID(int tuning_data_size, int num_records_to_skip, float max_steering_angle,
          float twiddle_tolerance, double steering_noise,
          double distance_noise) : kNumTuningDataRecords(tuning_data_size),
                                    kNumTuningDataFeatures(4),
                                    kTwiddleTolerance(twiddle_tolerance),
                                    kNumRecordsToSkip(num_records_to_skip),
                                    kMaxSteeringAngle(max_steering_angle),
                                    control_gains_tuning_data(kNumTuningDataFeatures) {
  this->steering_noise = steering_noise;
  this->distance_noise = distance_noise;
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  DisableParameterTuning();
  params_tuned = false;

  t_start = std::chrono::high_resolution_clock::now();
  t_stop = std::chrono::high_resolution_clock::now();
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

bool bPrintData = true;

void PID::PrintTrainingData(){
  std::cout << "******* Training Data - Start ******" << std::endl;
  std::cout << "CTE\tSteering_Angle\tSpeed\tElapsed_Time" << std::endl;
  for (unsigned int col = 0; col < kNumTuningDataRecords; ++col) {
    for (unsigned int row = 0; row < 4; ++row) {
      std::cout << control_gains_tuning_data[row][col] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << "******* Training Data - End ******" << std::endl;
}


//void PID::Update(double cte, double steering_angle,
//                 double speed, double t_elapsed_seconds) {
void PID::Update(double cte, double steering_angle, double speed) {

  if (IsParameterTuningEnabled() && params_tuned == false){

    // calculate the elapsed time in seconds since last update
    // this is the time the car has ran on the track since steering angle update
    t_stop = std::chrono::high_resolution_clock::now();
    auto t_elapsed_sec = std::chrono::duration_cast<std::chrono::microseconds>(t_stop - t_start)/10e6;
    t_start = t_stop;

    std::cout << "**** Elapsed Time: " << t_elapsed_sec.count() << " Seconds"<< std::endl;


    // record control gain tuning data if we haven't yet
    if (control_gains_tuning_data[0].size() < kNumTuningDataRecords) {

      // record the cte, steering angle, speed, elapsed time in seconds
//      vector<double> tuning_data {cte, steering_angle, speed, t_elapsed_seconds};
      vector<double> tuning_data {cte, steering_angle, speed, t_elapsed_sec.count()};
      for (unsigned int i = 0; i < control_gains_tuning_data.size(); ++i){
        control_gains_tuning_data[i].push_back(tuning_data[i]);
      }

      std::cout << "Num Records: " << control_gains_tuning_data[0].size() << std::endl;

    } else if (!params_tuned){

//      if (bPrintData) {
//
//        PrintTrainingData();
//
//        bPrintData = false;
//      }

      std::cout << "Tuning parameters..... " << std::endl;
      vector<double> params = TuneControlGains();
      Kp = params[0];
      Ki = params[1];
      Kd = params[2];
      params_tuned = true;
      DisableParameterTuning();
      // std::cout << "Tuned Control Gains: Kp(" << Kp << ") Ki(" << Ki << ") Kd(" << Kd << ")" << std::endl;
    }
  }




  UpdateError(cte);
}

double PID::CalculateCost(const vector<double> CTEs){
  return inner_product(CTEs.begin(), CTEs.end(), CTEs.begin(), 0.0);
}

void PID::SimulateMove(double& x, double&y, double& orientation,
                       double steering, double distance) {

  /**
   * steering = front wheel steering angle, limited by kMaxSteeringAngle
   * distance = total distance driven, most be non-negative
   *
   */

  // validate steering angle and distance
  if (steering > kMaxSteeringAngle) steering = kMaxSteeringAngle;
  if (steering < -kMaxSteeringAngle) steering = -kMaxSteeringAngle;
  if (distance < 0.0) distance = 0.0;

  // apply noise to the steering and distance
  std::default_random_engine generator;
  std::normal_distribution<double> steering_distribution(steering, steering_noise);
  std::normal_distribution<double> distance_distribution(distance, steering_noise);

  double steering2 = steering_distribution(generator);
  double distance2 = distance_distribution(generator);
//  double steering2 = steering;
//  double distance2 = distance;
//  std::cout << "Distance: " << distance << " Distance 2: " << distance2 << std::endl;

  // execute motion
//  double turn = tan(steering2) * distance2 / 20.0;
  double turn = tan(steering2) * distance2;
  double turn_tolerance = 0.001;

  if (abs(turn) < turn_tolerance) {
    // approximate by a straight line
    x += distance2 * cos(orientation);
    y += distance * sin(orientation);
    orientation = fmod((orientation + turn), (2.0 * M_PI));
  } else {
    // approximate bicycle model for motion
    double radius = distance2 / turn;
    double cx = x - (sin(orientation) * radius);
    double cy = y + (cos(orientation) * radius);

    orientation = fmod((orientation + turn), (2.0 * M_PI));
    x = cx + (sin(orientation) * radius);
    y = cy - (cos(orientation) * radius);
  }
}

double PID::SimulateTrajectory(const vector<double>& params) {

  /**
   * Here we will simulate a trajectory based on the distance traveled
   * at each step in the training data, but use different steering angles
   * as calculated based on the tuned params.
   */
  vector<double> CTEs = control_gains_tuning_data[PID::CTE_IDX];
  vector<double> steering_angles = control_gains_tuning_data[PID::STEERING_ANGLE_IDX];
  vector<double> speeds = control_gains_tuning_data[PID::SPEED_IDX];
  vector<double> elapsed_times_sec = control_gains_tuning_data[PID::ELAPSED_TIME_IDX];

  vector<double> x_trajectory;
  vector<double> y_trajectory;

  // we skip the first kNumRecordsToSkip training records and use the
  // next entry in the training data as the starting position of the car
  int start_idx = kNumRecordsToSkip-1;
  double x = 0.0;
  double y = CTEs[start_idx]; // cte is essentially the car's y-coordinate
  double orientation = steering_angles[start_idx]; // car's orientation at start

  double err = 0.0;
  double prev_cte = y;
  double int_cte = 0.0; // integral component

  x_trajectory.push_back(x);
  y_trajectory.push_back(y);

  // start from the second record
  for (unsigned int i = start_idx; i < kNumTuningDataRecords; ++i) {
    double cte = y; // proportional component
    double diff_cte = cte - prev_cte; // differential component
    int_cte += cte;
    prev_cte = cte;

    double steer = -params[0] * cte - params[1] * int_cte - params[2] * diff_cte;
    double distance = (speeds[i]) * elapsed_times_sec[i];
//    double distance = speeds[i] * 0.447 * elapsed_times_sec[i]; // speed mph to m/s multiplied by time in seconds

    SimulateMove(x, y, orientation, steer, distance);
    x_trajectory.push_back(x);
    y_trajectory.push_back(y);

    // accumulate squared error
    err += cte*cte;

  }

//  std::cout << "Trajectory Size: " << x_trajectory.size() << std::endl;

  return err;
}


// Implements the TWIDDLE Algorithm
vector<double> PID::TuneControlGains(){

//  // ensure that we have the required number of records
//  if (control_gains_tuning_data.size() != kNumTuningDataRecords){
//    std::cout << "ERROR: Insufficient Tuning Data" << std::endl;
//    return;
//  }

  // start the parameter optimization from the experimentally selected
  // control gains on which the initial best error is calculated
  vector<double> p {Kp, Ki, Kd};
//  vector<double> p {0.0, 0.0, 0.0};
  vector<double> dp {1.0, 0.0, 1.0};

  // this is the Sum of Squared Errors of the training data that we gathered
  // from the initial few seconds run
  double best_err = CalculateCost(control_gains_tuning_data[PID::CTE_IDX]);
  std::cout << "Original Error: " << best_err << std::endl;

  int it = 0;
  double err = 0.0;

  // TWIDDLE loop
  while (accumulate(dp.begin(), dp.end(), 0.0) > kTwiddleTolerance){
//  for (int j = 0; j < 1000; ++j) {

    for (unsigned int i = 0; i < p.size(); ++i) {
//      std::cout << "Tuning param: " << i+1 << " = " << p[i];
      p[i] += dp[i];
      err = SimulateTrajectory(p);
      // if better cost/err, update best error and try to optimize the param further
      if (err < best_err) {
        best_err = err;
        dp[i] *= 1.1;
      } else { // else try to optimize param and calculate cost
        p[i] -= 2 * dp[i];
        err = SimulateTrajectory(p);
        if (err < best_err) {
          best_err = err;
          dp[i] *= 1.1;
        } else {
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }

//      std::cout << " To: " << p[i] << " Err: " << err << " Best Err: " << best_err << std::endl;
    }  // for (unsigned int i = 0; i < p.size(); ++i)

    ++it;

  }  // TWIDDLE loop

  std::cout << "Best Error: " << best_err << std::endl;
  std::cout << "Twiddle Iterations: " << it << std::endl;
  std::cout << "Original Control Gains: " << Kp << ", " << Ki << ", " << Kd << std::endl;
  std::cout << "Tuned Control Gains: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;

  // return the tuned params
  return p;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  std::cout << "PARAMETERS: " << Kp << " " << Ki << " " << Kd << std::endl;
  double total_error = (-Kp * p_error - Ki * i_error - Kd * d_error);

  // ensure that the total error is between -1.0 and 1.0
  if (total_error < -1.0) {
    total_error = -1.0;
  }else if (total_error > 1.0){
    total_error = 1.0;
  }

  return total_error;
}

//double PID::CalculateCost(const vector<double> params, const vector<double> errors){
//  double cost = 0.0;
//  // calculate the cost
//  for (unsigned int i = 0; i < params.size(); ++i){
//    cost -= params[i]*errors[i];
//  }
//  return cost;
//}
