#include "kalmanfilter.h"

KalmanFilter::KalmanFilter(
  const boost::numeric::ublas::vector<double>& first_x,
  const boost::numeric::ublas::matrix<double>& first_p,
  const boost::numeric::ublas::matrix<double>& measurement_noise,
  const boost::numeric::ublas::matrix<double>& observation,
  const boost::numeric::ublas::matrix<double>& process_noise,
  const boost::numeric::ublas::matrix<double>& state_transition)
  : m_measurement_noise(measurement_noise),
    m_observation(observation),
    m_p(first_p),
    m_process_noise(process_noise),
    m_state_transition(state_transition),
    m_x(first_x)
{
  assert(process_noise.size1() >= 1);
  assert(process_noise.size2() >= 1);
}

const boost::numeric::ublas::matrix<double> Inverse(
  const boost::numeric::ublas::matrix<double>& m)
{
  assert(m.size1() == m.size2() && "Can only calculate the inverse of square matrices");
  assert(m.size1() == 1 && m.size2() == 1 && "Only for 1x1 matrices");
  assert(m(0,0) != 0.0 && "Cannot take the inverse of matrix [0]");
  boost::numeric::ublas::matrix<double> n(1,1);
  n(0,0) =  1.0 / m(0,0);
  return n;
}

void KalmanFilter::SupplyMeasurement(const boost::numeric::ublas::vector<double>& x)
{
  using boost::numeric::ublas::identity_matrix;
  using boost::numeric::ublas::matrix;
  using boost::numeric::ublas::prod;
  using boost::numeric::ublas::trans;
  using boost::numeric::ublas::vector;
  ///Debug statements to keep code below clean
  /// 2/7) Covariance prediction
  assert(m_state_transition.size2() == m_p.size1());
  assert( prod(m_state_transition,m_p).size2()
    ==  trans(m_state_transition).size1() );
  assert(matrix<double>(prod(
      matrix<double>(prod(m_state_transition,m_p)),
      trans(m_state_transition))).size1()
    == m_process_noise.size1());
  assert(matrix<double>(prod(
      matrix<double>(prod(m_state_transition,m_p)),
      trans(m_state_transition))).size2()
    == m_process_noise.size2());

  /// 1/7) State prediction
  const vector<double> x_current = prod(m_state_transition,m_x);
  /// 2/7) Covariance prediction
  const matrix<double> p_current
    = matrix<double>(
      prod(
        matrix<double>(prod(m_state_transition,m_p)),
        trans(m_state_transition)
      )
    )
    + m_process_noise;
  /// 3/7) Innovation (y with a squiggle above it)
  const vector<double> z_measured = x; //x has noise in it
  const vector<double> innovation = z_measured - prod(m_observation,x_current);
  /// 4/7) Innovation covariance (S)
  const matrix<double> innovation_covariance
    = matrix<double>(prod(
          matrix<double>(prod(m_observation,p_current)),
          trans(m_observation)
        ))
    + m_measurement_noise;
  /// 5/7) Kalman gain (K)
  const matrix<double> kalman_gain
    = prod(
        matrix<double>(
        prod(
          p_current,
          trans(m_observation)
        )),
        Inverse(innovation_covariance)
      );
  /// 6/7) Update state prediction
  m_x = x_current + prod(kalman_gain,innovation);
  /// 7/7) Update covariance prediction
  const identity_matrix<double> my_identity_matrix(kalman_gain.size1());
  m_p = prod(
    my_identity_matrix
      - matrix<double>(prod(kalman_gain,m_observation)),
    p_current
  );
}

