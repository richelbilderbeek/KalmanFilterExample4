///Kalman filter example
///Adapted from merge from www.adrianboeing.com and http://greg.czerniak.info/guides/kalman1
///now working with matrices

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>
#include "kalmanfilter.h"
#include "whitenoisesystem.h"

///Context:
///A constant DC voltage measured with a noisy multimeter
int main()
{
  using boost::numeric::ublas::matrix;
  using boost::numeric::ublas::vector;

  ///The state matrix
  const vector<double> x_real(1,1.25); //Volts

  ///Real measurement noise
  const vector<double> x_real_noise(1,0.25);

  ///Guess of the state matrix
  const vector<double> x_first_guess(1,10.0);

  ///Guess of the covariances
  const matrix<double> p_first_guess(1,1,1.0);

  //Estimated measurement noise
  const matrix<double> measurement_noise(1,1,0.1);

  const matrix<double> observation(1,1,1.0);

  //Estimated process noise
  const matrix<double> process_noise(1,1,0.0001);

  const matrix<double> state_transition(1,1,1.0);

  const WhiteNoiseSystem s(x_real,x_real_noise);

  KalmanFilter k(x_first_guess,p_first_guess,measurement_noise,observation,process_noise,state_transition);
\
  std::cout << "Real,measured,Kalman\n";
  for (int i=0;i!=100;++i)
  {
    //Perform a noisy measurement
    const vector<double> z_measured = s.Measure();
    //Pass this measurement to the filter
    k.SupplyMeasurement(z_measured);
    //Display what the filter predicts
    const vector<double> x_est_last = k.Predict();
    std::cout << x_real(0) << "," << z_measured(0) << "," << x_est_last(0) << '\n';
  }
}
