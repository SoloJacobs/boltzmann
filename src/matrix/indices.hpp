#ifndef INDICES_HPP
#define INDICES_HPP

#include "matrix.hpp"

class Indices : public std::input_iterator_tag {
private:
public:
  using size_type = std::size_t;
  class iterator {
  public:
    using iterator_category = std::input_iterator_tag;
    using value_type = std::pair<size_type, size_type>;
    using difference_type = std::ptrdiff_t;
    using pointer = const value_type *;
    using reference = value_type;

    explicit iterator(size_type num_cols)
        : j_max_{num_cols - 1}, i_{0}, j_{0} {}
    explicit iterator(size_type num_rows, size_type num_cols)
        : j_max_{num_cols - 1}, i_{num_rows}, j_{0} {}

    iterator &operator++() {
      if (j_ == j_max_) {
        j_ = 0;
        ++i_;
      } else {
        ++j_;
      }
      return *this;
    }

    iterator operator++(int) {
      iterator retval = *this;
      ++(*this);
      return retval;
    }

    bool operator==(iterator other) const {
      return (i_ == other.i_) && (j_ == other.j_);
    }

    bool operator!=(iterator other) const { return !(*this == other); }

    reference operator*() const { return std::pair(i_, j_); }

  private:
    /* const */ size_type j_max_;
    size_type i_;
    size_type j_;
  };

  iterator begin() const { return iterator(num_cols); }

  iterator end() const { return iterator(num_rows, num_cols); }

  explicit Indices(size_type num_rows, size_type num_cols)
      : num_rows{num_rows}, num_cols{num_cols} {}

	template<class T>
  explicit Indices(const T & mat)
      : num_rows{mat.num_rows()}, num_cols{mat.num_cols()} {}

private:
  size_type num_rows;
  size_type num_cols;
};

#endif // INDICES_HPP
