#include <eigen_extensions/random.h>

using namespace std;
using namespace Eigen;

namespace eigen_extensions
{

  UniformSampler::UniformSampler(uint64_t seed) :
    Sampler(),
    mersenne_(seed)
  {
  }

  double UniformSampler::sample()
  {
    return mersenne_();
  }

  GaussianSampler::GaussianSampler(double mean, double stdev, uint64_t seed) :
    Sampler(),
    mersenne_(seed),
    normal_(mean, stdev),
    vg_(mersenne_, normal_)
  {
  }

  double GaussianSampler::sample()
  {
    return vg_();
  }

  void sampleSparseGaussianVector(int rows, int nnz, SparseVector<double>* vec)
  {
    assert(rows >= nnz);

    vector<int> indices(rows);
    for(size_t i = 0; i < indices.size(); ++i)
      indices[i] = i;
    random_shuffle(indices.begin(), indices.end());

    GaussianSampler gs;
    *vec = SparseVector<double>(rows);
    vec->reserve(nnz);
    for(int i = 0; i < nnz; ++i)
      vec->coeffRef(indices[i]) = gs.sample();
  }

} // namespace

