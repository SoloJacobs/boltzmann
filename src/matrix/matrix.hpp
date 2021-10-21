#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace Sol {

template <class T> class Matrix;

template <class T>
constexpr bool operator==(const Matrix<T> &mat_1, const Matrix<T> &mat_2);

template <class T>
std::ostream &operator<<(std::ostream &, const Matrix<T> &mat);

// Row-Major order is the default
template <class T> class Matrix {
public:
  // Member types
  using size_type = typename std::vector<T>::size_type;
  using reference = typename std::vector<T>::reference;
  using const_reference = typename std::vector<T>::const_reference;
  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  // Constructors
  Matrix() = delete;
  constexpr Matrix(size_type i, size_type j);
  constexpr Matrix(size_type num_rows, size_type num_cols, const T &value);

  //
  reference operator()(size_type i, size_type j);
  const_reference operator()(size_type i, size_type j) const;
  reference at(size_type i, size_type j);
  const_reference at(size_type i, size_type j) const;

  friend constexpr bool operator==<T>(const Matrix &mat_1, const Matrix &mat_2);
  friend std::ostream &operator<<<T>(std::ostream &, const Matrix &mat);

  constexpr iterator begin() noexcept;
  constexpr const_iterator begin() const noexcept;
  constexpr const_iterator cbegin() const noexcept;

  constexpr iterator end() noexcept;
  constexpr const_iterator end() const noexcept;
  constexpr const_iterator cend() const noexcept;

  constexpr size_type num_rows() const noexcept;
  constexpr size_type num_cols() const noexcept;

private:
  // Members
  std::vector<T> data_;
  size_type num_rows_;
  size_type num_cols_;

  void row_range_check(size_type i) const;
  void col_range_check(size_type j) const;
};

// Begin of template<T> section

template <class T>
constexpr Matrix<T>::Matrix(size_type num_rows, size_type num_cols)
    : num_rows_{num_rows}, num_cols_{num_cols}, data_(num_cols * num_rows) {}

template <class T>
constexpr Matrix<T>::Matrix(size_type num_rows, size_type num_cols,
                            const T &value)
    : num_rows_{num_rows}, num_cols_{num_cols},
      data_(num_cols * num_rows, value) {}

// Access operator using Row-Major order
template <class T>
typename Matrix<T>::reference Matrix<T>::operator()(size_type i, size_type j) {
  return data_[i * num_cols_ + j];
}

template <class T>
typename Matrix<T>::const_reference Matrix<T>::operator()(size_type i,
                                                          size_type j) const {
  return data_[i * num_cols_ + j];
}

//// Access operator using Column-Major order
// template<class T>
// T & Matrix<T>::operator()(size_type i , size_type j)
//{
//	return data[i + j*num_rows];
//}

template <class T>
typename Matrix<T>::reference Matrix<T>::at(size_type i, size_type j) {
  row_range_check(i);
  col_range_check(j);
  return operator()(i, j);
}

template <class T>
typename Matrix<T>::const_reference Matrix<T>::at(size_type i,
                                                  size_type j) const {
  row_range_check(i);
  col_range_check(j);
  return operator()(i, j);
}

template <class T> void Matrix<T>::row_range_check(size_type i) const {
  if (i >= num_rows_) {
    std::stringstream warning{};
    warning << "row_index i (which is " << i
            << ") greater equal to this->num_rows_ (which is " << num_rows_
            << ").";
    throw std::out_of_range(warning.str());
  }
}

template <class T> void Matrix<T>::col_range_check(size_type j) const {
  if (j >= num_cols_) {
    std::stringstream warning{};
    warning << "col_index j (which is " << j
            << ") greater equal to this->num_cols_ (which is " << num_cols_
            << ").";
    throw std::out_of_range(warning.str());
  }
}

template <class T>
constexpr bool operator==(const Matrix<T> &mat_1, const Matrix<T> &mat_2) {
  return (mat_1.num_cols_ == mat_2.num_cols_) &&
         (mat_1.num_rows_ == mat_2.num_rows_) && (mat_1.data_ == mat_2.data_);
}

template <class T>
constexpr typename Matrix<T>::iterator Matrix<T>::begin() noexcept {
  return data_.begin();
}

template <class T>
constexpr typename Matrix<T>::const_iterator Matrix<T>::begin() const noexcept {
  return data_.begin();
}

template <class T>
constexpr typename Matrix<T>::const_iterator
Matrix<T>::cbegin() const noexcept {
  return data_.cbegin();
}

template <class T>
constexpr typename Matrix<T>::iterator Matrix<T>::end() noexcept {
  return data_.end();
}

template <class T>
constexpr typename Matrix<T>::const_iterator Matrix<T>::end() const noexcept {
  return data_.end();
}

template <class T>
constexpr typename Matrix<T>::const_iterator Matrix<T>::cend() const noexcept {
  return data_.cend();
}

template <class T>
constexpr typename Matrix<T>::size_type Matrix<T>::num_rows() const noexcept {
  return num_rows_;
}

template <class T>
constexpr typename Matrix<T>::size_type Matrix<T>::num_cols() const noexcept {
  return num_cols_;
}

template <class T>
std::ostream &operator<<(std::ostream &out, const Matrix<T> &mat) {
  for (typename Matrix<T>::size_type i{0}; i < mat.num_rows_; ++i) {
    for (typename Matrix<T>::size_type j{0}; j < mat.num_cols_; ++j) {
      out << mat(i, j) << " ";
    }
    out << "\n";
  }
  out << std::flush;
  return out;
}

// End of template<T> section
//

} // namespace Sol

#endif // MATRIX_HPP
