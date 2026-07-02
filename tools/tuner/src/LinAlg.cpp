#include "LinAlg.h"

#include <cmath>
#include <cstring>

namespace linalg {

double dot(const Vec9& a, const Vec9& b) {
  double s = 0.0;
  for (int i = 0; i < 9; i++) s += a[i] * b[i];
  return s;
}

double norm(const Vec9& a) { return std::sqrt(dot(a, a)); }

Vec9 add(const Vec9& a, const Vec9& b) {
  Vec9 r;
  for (int i = 0; i < 9; i++) r[i] = a[i] + b[i];
  return r;
}

Vec9 sub(const Vec9& a, const Vec9& b) {
  Vec9 r;
  for (int i = 0; i < 9; i++) r[i] = a[i] - b[i];
  return r;
}

Vec9 scale(const Vec9& a, double s) {
  Vec9 r;
  for (int i = 0; i < 9; i++) r[i] = a[i] * s;
  return r;
}

Vec9 normalize(const Vec9& a) {
  const double n = norm(a);
  if (n < 1e-12) {
    Vec9 zero{};
    return zero;
  }
  return scale(a, 1.0 / n);
}

Vec9 mean(const std::vector<Vec9>& samples) {
  Vec9 m{};
  if (samples.empty()) return m;
  for (const Vec9& s : samples) m = add(m, s);
  return scale(m, 1.0 / static_cast<double>(samples.size()));
}

void jacobiEigen(double a[9][9], int n, double eigval[9], double eigvec[9][9]) {
  // Identity for the accumulated rotations.
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      eigvec[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }

  const int kMaxSweeps = 64;
  for (int sweep = 0; sweep < kMaxSweeps; sweep++) {
    double off = 0.0;
    for (int p = 0; p < n; p++) {
      for (int q = p + 1; q < n; q++) {
        off += a[p][q] * a[p][q];
      }
    }
    if (off < 1e-24) break;

    for (int p = 0; p < n; p++) {
      for (int q = p + 1; q < n; q++) {
        if (std::fabs(a[p][q]) < 1e-30) continue;

        const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        const double t = (theta >= 0 ? 1.0 : -1.0) /
                         (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
        const double c = 1.0 / std::sqrt(t * t + 1.0);
        const double s = t * c;

        for (int k = 0; k < n; k++) {
          const double akp = a[k][p];
          const double akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < n; k++) {
          const double apk = a[p][k];
          const double aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
        for (int k = 0; k < n; k++) {
          const double vkp = eigvec[k][p];
          const double vkq = eigvec[k][q];
          eigvec[k][p] = c * vkp - s * vkq;
          eigvec[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }

  for (int i = 0; i < n; i++) eigval[i] = a[i][i];

  // Sort descending by eigenvalue (selection sort, tiny n).
  for (int i = 0; i < n - 1; i++) {
    int best = i;
    for (int j = i + 1; j < n; j++) {
      if (eigval[j] > eigval[best]) best = j;
    }
    if (best != i) {
      std::swap(eigval[i], eigval[best]);
      for (int k = 0; k < n; k++) std::swap(eigvec[k][i], eigvec[k][best]);
    }
  }
}

void covariance(const std::vector<Vec9>& samples, double cov[9][9]) {
  std::memset(cov, 0, sizeof(double) * 81);
  if (samples.size() < 2) return;

  const Vec9 m = mean(samples);
  for (const Vec9& s : samples) {
    const Vec9 d = sub(s, m);
    for (int i = 0; i < 9; i++) {
      for (int j = 0; j < 9; j++) {
        cov[i][j] += d[i] * d[j];
      }
    }
  }
  const double inv = 1.0 / static_cast<double>(samples.size() - 1);
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9; j++) {
      cov[i][j] *= inv;
    }
  }
}

void pca(const std::vector<Vec9>& samples, int count, Vec9* dirs,
         double* eigenvalues) {
  double cov[9][9];
  covariance(samples, cov);

  double eigval[9];
  double eigvec[9][9];
  jacobiEigen(cov, 9, eigval, eigvec);

  for (int c = 0; c < count; c++) {
    eigenvalues[c] = eigval[c];
    for (int i = 0; i < 9; i++) {
      dirs[c][i] = eigvec[i][c];
    }
  }
}

bool invert6(const double in[6][6], double out[6][6]) {
  // Gauss-Jordan with partial pivoting on an augmented [A | I] system.
  double aug[6][12];
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      aug[i][j] = in[i][j];
      aug[i][j + 6] = (i == j) ? 1.0 : 0.0;
    }
  }

  for (int col = 0; col < 6; col++) {
    int pivot = col;
    for (int r = col + 1; r < 6; r++) {
      if (std::fabs(aug[r][col]) > std::fabs(aug[pivot][col])) pivot = r;
    }
    if (std::fabs(aug[pivot][col]) < 1e-12) return false;
    if (pivot != col) {
      for (int j = 0; j < 12; j++) std::swap(aug[col][j], aug[pivot][j]);
    }

    const double inv = 1.0 / aug[col][col];
    for (int j = 0; j < 12; j++) aug[col][j] *= inv;

    for (int r = 0; r < 6; r++) {
      if (r == col) continue;
      const double f = aug[r][col];
      if (f == 0.0) continue;
      for (int j = 0; j < 12; j++) aug[r][j] -= f * aug[col][j];
    }
  }

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      out[i][j] = aug[i][j + 6];
    }
  }
  return true;
}

bool pinv9x6(const double J[9][6], double M[6][9]) {
  double jtj[6][6];
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      double s = 0.0;
      for (int k = 0; k < 9; k++) s += J[k][i] * J[k][j];
      jtj[i][j] = s;
    }
  }

  double inv[6][6];
  if (!invert6(jtj, inv)) return false;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 9; j++) {
      double s = 0.0;
      for (int k = 0; k < 6; k++) s += inv[i][k] * J[j][k];
      M[i][j] = s;
    }
  }
  return true;
}

}  // namespace linalg
