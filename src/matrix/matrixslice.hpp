#ifndef MATRIXSLICE_HPP
#define MATRIXSLICE_HPP

#include "matrix.hpp"

namespace Sol {

enum class Axis {
  row,
  col,
};

template <class T> class MatrixSlice {
public:
  // Member types
  using size_type = typename Matrix<T>::size_type;
  using iterator = typename Matrix<T>::iterator;
  using const_iterator = typename Matrix<T>::const_iterator;
  using reference = typename Matrix<T>::reference;
  using const_reference = typename Matrix<T>::const_reference;

  MatrixSlice() = delete;
  MatrixSlice(Matrix<T> &, size_type start_row, size_type start_col);
  MatrixSlice(const MatrixSlice &) = default;
  MatrixSlice(MatrixSlice &&) = default;
  MatrixSlice &operator=(const MatrixSlice &) = default;
  MatrixSlice &operator=(MatrixSlice &&) = default;

  constexpr size_type num_rows() const noexcept { return num_rows_; }

  constexpr size_type num_cols() const noexcept { return num_cols_; }

  reference operator()(size_type i, size_type j);
  MatrixSlice operator()(size_type i, Axis axis);
  reference at(size_type i, size_type j);

  friend std::ostream &operator<<(std::ostream &out, const MatrixSlice &slice) {
    for (auto i{0}; i < 5; ++i) {
      std::cout << slice.data_[i] << " ";
    }
    return out;
  }

private:
  /* const */ iterator data_;
  /* const */ size_type num_rows_;
  /* const */ size_type num_cols_;
  /* const */ size_type offset_row_;
  /* const */ size_type offset_col_;
  void row_range_check(size_type i) const;
  void col_range_check(size_type j) const;
};

template <class T>
MatrixSlice<T>::MatrixSlice(Matrix<T> &mat, size_type start_row,
                            size_type start_col)
    : data_{mat.begin()}, num_rows_{mat.num_rows() - start_row},
      num_cols_{mat.num_cols() - start_col}, offset_row_{mat.num_rows()},
      offset_col_{mat.num_cols()} {
  mat.at(start_row, start_col);
  data_ += start_col + start_row * mat.num_cols();
}

template <class T> void MatrixSlice<T>::row_range_check(size_type i) const {
  if (i >= num_rows_) {
    std::stringstream warning{};
    warning << "row_index i (which is " << i
            << ") greater equal to this->num_rows_ (which is " << num_rows_
            << ").";
    throw std::out_of_range(warning.str());
  }
}

template <class T> void MatrixSlice<T>::col_range_check(size_type j) const {
  if (j >= num_cols_) {
    std::stringstream warning{};
    warning << "col_index j (which is " << j
            << ") greater equal to this->num_cols_ (which is " << num_cols_
            << ").";
    throw std::out_of_range(warning.str());
  }
}

template <class T>
typename MatrixSlice<T>::reference MatrixSlice<T>::operator()(size_type i,
                                                              size_type j) {
  return data_[i * offset_col_ + j];
}

template <class T>
MatrixSlice<T> MatrixSlice<T>::operator()(size_type i, Axis axis) {
  return MatrixSlice<T>(num_cols(), num_rows());
}

template <class T>
typename MatrixSlice<T>::reference MatrixSlice<T>::at(size_type i,
                                                      size_type j) {
  row_range_check(i);
  col_range_check(j);
  return operator()(i, j);
}

} // namespace Sol

#endif // MATRIXSLICE_HPP
