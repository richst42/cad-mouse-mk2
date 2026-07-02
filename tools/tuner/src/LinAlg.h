#pragma once

#include <array>
#include <cstddef>
#include <vector>

// Small fixed-size linear algebra helpers for the calibration solver.
// Everything is double precision and allocation-free except where noted.
namespace linalg {

using Vec9 = std::array<double, 9>;

double dot(const Vec9& a, const Vec9& b);
double norm(const Vec9& a);
Vec9 add(const Vec9& a, const Vec9& b);
Vec9 sub(const Vec9& a, const Vec9& b);
Vec9 scale(const Vec9& a, double s);
Vec9 normalize(const Vec9& a);  // returns zero vector if norm is ~0

// Mean of a sample set.
Vec9 mean(const std::vector<Vec9>& samples);

// Symmetric eigendecomposition via cyclic Jacobi rotations.
// a: n x n symmetric matrix (destroyed), n <= 9.
// eigval: descending eigenvalues; eigvec: columns are the matching vectors.
void jacobiEigen(double a[9][9], int n, double eigval[9], double eigvec[9][9]);

// Covariance (about the mean) of a sample set, into cov[9][9].
void covariance(const std::vector<Vec9>& samples, double cov[9][9]);

// Top eigenpairs of the covariance of `samples`.
// Returns eigenvalues (descending); directions go into dirs.
void pca(const std::vector<Vec9>& samples, int count, Vec9* dirs,
         double* eigenvalues);

// Gauss-Jordan inverse of a 6x6 matrix. Returns false if singular.
bool invert6(const double in[6][6], double out[6][6]);

// Moore-Penrose pseudo-inverse of a 9x6 matrix with independent columns:
// M = (J^T J)^-1 J^T. Returns false if J^T J is singular.
bool pinv9x6(const double J[9][6], double M[6][9]);

}  // namespace linalg
