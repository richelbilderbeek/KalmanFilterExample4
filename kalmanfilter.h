#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix.hpp>

struct KalmanFilter
{
  ///Initialize the filter with a first measurent
  KalmanFilter(
    const boost::numeric::ublas::vector<double>& first_x,
    const boost::numeric::ublas::matrix<double>& first_p,
    const boost::numeric::ublas::matrix<double>& measurement_noise,
    const boost::numeric::ublas::matrix<double>& observation,
    const boost::numeric::ublas::matrix<double>& process_noise,
    const boost::numeric::ublas::matrix<double>& state_transition
    );

  ///Give the filter a real measurement, so it will update itself
  void SupplyMeasurement(const boost::numeric::ublas::vector<double>& x);

  ///Let the filter predict
  const boost::numeric::ublas::vector<double>& Predict() const { return m_x; }

  ///Let the filter predict
  const boost::numeric::ublas::matrix<double>& PredictCovariance() const { return m_p; }

  private:

  //R: Estimated measurement noise: How to estimate this?
  const boost::numeric::ublas::matrix<double> m_measurement_noise;

  //H
  const boost::numeric::ublas::matrix<double> m_observation;

  ///The (current prediction of the) covariance
  boost::numeric::ublas::matrix<double> m_p;

  //Q: Process noise: How to estimate this?
  const boost::numeric::ublas::matrix<double> m_process_noise;

  //F: state transition matrix
  const boost::numeric::ublas::matrix<double> m_state_transition;

  ///The (current prediction of the) measurement
  boost::numeric::ublas::vector<double> m_x;

};

#endif // KALMANFILTER_H
