#ifndef WHITENOISESYSTEM_H
#define WHITENOISESYSTEM_H

#include <boost/numeric/ublas/matrix.hpp>

struct WhiteNoiseSystem
{
  WhiteNoiseSystem(
    const boost::numeric::ublas::vector<double>& real_value,
    const boost::numeric::ublas::vector<double>& stddev);

  ///Measure a value from this system with normally distributed noise
  const boost::numeric::ublas::vector<double> Measure() const;

  ///Peek what the real value is
  const boost::numeric::ublas::vector<double>& PeekAtRealValue() const { return m_mean; }

  private:
  ///The real value of the system
  const boost::numeric::ublas::vector<double> m_mean;

  ///The amount of noise in the system
  ///A noise of zero indicates a system that can be measured accurately to infinite precision
  const boost::numeric::ublas::vector<double> m_stddev;

  ///Obtain a random number from a normal distribution
  ///From http://www.richelbilderbeek.nl/CppGetRandomNormal.htm
  static double GetRandomNormal(const double mean = 0.0, const double sigma = 1.0);

};

#endif // WHITENOISESYSTEM_H
