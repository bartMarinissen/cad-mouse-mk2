#pragma once

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Arduino.h"

#include "ElementStorage.h"

namespace BLA
{

// Vendored and patched: see TODO/eigen-to-bla-migration.md. Upstream
// MatrixBase inherits Arduino's Printable (a `virtual size_t printTo(Print&)
// const = 0` interface) purely so `Serial.print(myMatrix)` works. Nothing in
// this codebase uses that. But the moment a class has any virtual function
// its instances carry a hidden vtable pointer -- 4 bytes per BLA::Matrix,
// which on the 4,641-entry bicubic table alone was +18.5KB of RAM (measured:
// the table went from 8 to 12 bytes/entry). Dropped the Printable base and
// the `final` on printTo() below so nothing here is virtual; printTo() still
// works as a plain member function, just not through a Printable&/Print's
// polymorphic interface.
template <typename DerivedType, int rows, int cols, typename d_type>
struct MatrixBase
{
   public:
    constexpr static int Rows = rows;
    constexpr static int Cols = cols;
    using DType = d_type;

    constexpr DType &operator()(int i, int j = 0) { return static_cast<DerivedType *>(this)->operator()(i, j); }

    constexpr DType operator()(int i, int j = 0) const { return static_cast<const DerivedType *>(this)->operator()(i, j); }

    // Vendored addition (see TODO/eigen-to-bla-migration.md): upstream 5.1 has no named
    // accessors at all, only operator()(i,j). The first BLA port translated every Eigen
    // .x()/.y()/.z() call site to (0)/(1)/(2) by hand; a later port brought in enough more
    // Eigen-idiom source in one go that hand-translating every new site was worse than
    // closing the gap here once. Column-vector-shaped only (Cols==1), each requiring enough
    // rows to make sense -- static_assert fires only when a call actually instantiates the
    // method, so e.g. .z() on a Vec2 is a compile error, not silently wrong. Mirrors
    // operator()'s const/non-const pair so `v.x() = f;` works exactly like `v(0) = f;` does.
    // constexpr for the same reason operator() above is: usable on a constexpr Vec2/Vec3
    // (e.g. BicubicField's constexpr constructor reading its Vec2 origin/far members)
    // whenever the concrete derived type's own operator() is constexpr too -- this only
    // enables that, it doesn't force it for derived types that aren't.
    constexpr DType &x() { static_assert(cols == 1 && rows >= 1, "x() needs a column vector with at least 1 row"); return (*this)(0); }
    constexpr DType x() const { static_assert(cols == 1 && rows >= 1, "x() needs a column vector with at least 1 row"); return (*this)(0); }
    constexpr DType &y() { static_assert(cols == 1 && rows >= 2, "y() needs a column vector with at least 2 rows"); return (*this)(1); }
    constexpr DType y() const { static_assert(cols == 1 && rows >= 2, "y() needs a column vector with at least 2 rows"); return (*this)(1); }
    constexpr DType &z() { static_assert(cols == 1 && rows >= 3, "z() needs a column vector with at least 3 rows"); return (*this)(2); }
    constexpr DType z() const { static_assert(cols == 1 && rows >= 3, "z() needs a column vector with at least 3 rows"); return (*this)(2); }

    // Vendored addition: a derived Matrix's own constexpr constructor
    // (ElementStorage.h) default-constructs this empty base subobject, and
    // an explicitly-defaulted special member function is implicitly
    // constexpr already if the implicit definition would qualify (this one
    // does -- no members, nothing to initialize) -- confirmed by testing
    // without this keyword, which still compiles. Kept explicit anyway as
    // a guarantee rather than relying on that implicit rule holding across
    // compilers -- see TODO/eigen-to-bla-migration.md.
    constexpr MatrixBase() = default;

    template <typename MatType>
    MatrixBase(const MatrixBase<MatType, Rows, Cols, DType> &mat)
    {
        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                static_cast<DerivedType &>(*this)(i, j) = mat(i, j);
            }
        }
    }

    MatrixBase &operator=(const MatrixBase &mat)
    {
        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                static_cast<DerivedType &>(*this)(i, j) = mat(i, j);
            }
        }

        return static_cast<DerivedType &>(*this);
    }

    template <typename MatType>
    MatrixBase &operator=(const MatrixBase<MatType, Rows, Cols, DType> &mat)
    {
        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                static_cast<DerivedType &>(*this)(i, j) = mat(i, j);
            }
        }

        return static_cast<DerivedType &>(*this);
    }

    DerivedType &operator=(DType elem)
    {
        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                static_cast<DerivedType &>(*this)(i, j) = elem;
            }
        }

        return static_cast<DerivedType &>(*this);
    }

    void Fill(const DType &val) { *this = val; }

    template <typename DestType>
    Matrix<Rows, Cols, DestType> Cast()
    {
        Matrix<Rows, Cols, DestType> ret;

        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                ret(i, j) = (DestType)(*this)(i, j);
            }
        }

        return ret;
    }

    template <int SubRows, int SubCols>
    RefMatrix<DerivedType, SubRows, SubCols> Submatrix(int row_start, int col_start)
    {
        return RefMatrix<DerivedType, SubRows, SubCols>(static_cast<DerivedType &>(*this), row_start, col_start);
    }

    template <int SubRows, int SubCols>
    RefMatrix<const DerivedType, SubRows, SubCols> Submatrix(int row_start, int col_start) const
    {
        return RefMatrix<const DerivedType, SubRows, SubCols>(static_cast<const DerivedType &>(*this), row_start,
                                                              col_start);
    }

    RefMatrix<DerivedType, 1, Cols> Row(int row_start)
    {
        return RefMatrix<DerivedType, 1, Cols>(static_cast<DerivedType &>(*this), row_start, 0);
    }

    RefMatrix<const DerivedType, 1, Cols> Row(int row_start) const
    {
        return RefMatrix<const DerivedType, 1, Cols>(static_cast<const DerivedType &>(*this), row_start, 0);
    }

    RefMatrix<DerivedType, Rows, 1> Column(int col_start)
    {
        return RefMatrix<DerivedType, Rows, 1>(static_cast<DerivedType &>(*this), 0, col_start);
    }

    RefMatrix<const DerivedType, Rows, 1> Column(int col_start) const
    {
        return RefMatrix<const DerivedType, Rows, 1>(static_cast<const DerivedType &>(*this), 0, col_start);
    }

    MatrixTranspose<DerivedType> operator~() { return MatrixTranspose<DerivedType>(static_cast<DerivedType &>(*this)); }
    MatrixTranspose<DerivedType> transpose() { return MatrixTranspose<DerivedType>(static_cast<DerivedType &>(*this)); }

    MatrixTranspose<const DerivedType> operator~() const
    {
        return MatrixTranspose<const DerivedType>(static_cast<const DerivedType &>(*this));
    }
    MatrixTranspose<const DerivedType> transpose() const
    {
        return MatrixTranspose<const DerivedType>(static_cast<const DerivedType &>(*this));
    }

    Matrix<Rows, Cols, DType> operator-() const
    {
        Matrix<Rows, Cols, DType> ret;

        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                ret(i, j) = -(*this)(i, j);
            }
        }

        return ret;
    }

    // Not `virtual`/`final` -- see the note above MatrixBase's declaration.
    size_t printTo(Print& p) const
    {
        size_t n;
        n = p.print('[');

        for (int i = 0; i < Rows; i++)
        {
            n += p.print('[');

            for (int j = 0; j < Cols; j++)
            {
                n += p.print(static_cast<const DerivedType *>(this)->operator()(i, j));
                n += p.print((j == Cols - 1) ? ']' : ',');
            }

            n += p.print((i == Rows - 1) ? ']' : ',');
        }
        return n;
    }
};

template <typename DerivedType>
using DownCast = MatrixBase<DerivedType, DerivedType::Rows, DerivedType::Cols, typename DerivedType::DType>;

}  // namespace BLA

#include "impl/Types.h"
#include "impl/BasicLinearAlgebra.h"
#include "impl/NotSoBasicLinearAlgebra.h"
