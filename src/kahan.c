#include <numeric>

#include "robot_driver/kahan.h"

inline const KahanAccumulation KahanSum::performSingleSum(KahanAccumulation accumulation, double value) const
{
  KahanAccumulation result;
  double y = value - accumulation.correction;
  double t = accumulation.sum + y;
  result.correction = (t - accumulation.sum) - y;
  result.sum = t;
  return result;
}

inline const KahanAccumulation KahanSum::performSum(const std::vector<double> &nums) const
{
  KahanAccumulation init = {0};
  KahanAccumulation result = std::accumulate(nums.begin(), nums.end(), init, KahanSum::performSingleSum);
  return result;
}
