#include <vector>

struct KahanAccumulation
{
  double sum, correction;
};

class KahanSum
{
public:
  inline const KahanAccumulation performSingleSum(KahanAccumulation accumulation, double value) const;
  inline const KahanAccumulation performSum(const std::vector<double> &nums) const;
};
