#include "whitenoisesystem.h"

#include <boost/random/normal_distribution.hpp>
#include <boost/random/lagged_fibonacci.hpp>

WhiteNoiseSystem::WhiteNoiseSystem(
  const boost::numeric::ublas::vector<double>& real_value,
  const boost::numeric::ublas::vector<double>& stddev)
  : m_mean(real_value),
    m_stddev(stddev)
{
  assert(m_mean.size() == m_stddev.size() && "Every value must have one measurement noise");
}

double WhiteNoiseSystem::GetRandomNormal(const double mean, const double sigma)
{
  boost::normal_distribution<double> norm_dist(mean, sigma);
  static boost::lagged_fibonacci19937 engine;
  const double value = norm_dist.operator () <boost::lagged_fibonacci19937>((engine));
  return value;
}

const boost::numeric::ublas::vector<double> WhiteNoiseSystem::Measure() const
{
  const std::size_t sz = m_mean.size();
  boost::numeric::ublas::vector<double> measured(sz);
  for (std::size_t i = 0; i!=sz; ++i)
  {
    measured(i) = GetRandomNormal(m_mean(i),m_stddev(i));
  }
  return measured;
}
