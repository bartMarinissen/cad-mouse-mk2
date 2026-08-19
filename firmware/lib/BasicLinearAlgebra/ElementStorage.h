#pragma once

namespace BLA
{

template <typename DerivedType, int rows, int cols, typename DType>
struct MatrixBase;

template <int Rows, int Cols = 1, typename DType = float>
class Matrix : public MatrixBase<Matrix<Rows, Cols, DType>, Rows, Cols, DType>
{
   public:
    DType storage[Rows * Cols];

    constexpr DType &operator()(int i, int j = 0) { return storage[i * Cols + j]; }
    constexpr DType operator()(int i, int j = 0) const { return storage[i * Cols + j]; }

    Matrix() = default;

    // Vendored addition (see TODO/eigen-to-bla-migration.md): this is the
    // constructor a literal fixed matrix like BicubicField.cpp's Catmull-Rom
    // basis goes through. Made constexpr so such matrices can be declared
    // `static constexpr` -- a real language guarantee of compile-time
    // construction, and a precondition for use in contexts that require a
    // constant expression (static_assert, template arguments, array
    // bounds). Measured to make no difference to generated code when the
    // matrix was already all-literal, since -O3 already constant-folded it
    // regardless (TODO/Performance.md's constexpr pass) -- the win here is
    // a language guarantee, not a measured performance one. A constexpr
    // constructor must initialize every member through the mem-initializer
    // list, not just assign to it in the body (the array default
    // constructor above stays a plain, possibly-uninitialized `= default`
    // -- deliberately not touched, since a matrix
    // that's about to be overwritten shouldn't pay for a zero-fill), so
    // this fills `storage` via the mem-initializer's aggregate-list syntax
    // instead of the row-major loop the old FillRowMajor used. Args are
    // already in row-major order (matching `storage`'s own layout), and any
    // cells left unlisted are zero-initialized by ordinary aggregate-init
    // rules -- the same behavior FillRowMajor's zero-fill base case used to
    // implement by hand.
    template <typename... TAIL>
    constexpr Matrix(DType head, TAIL... args) : storage{head, args...}
    {
        static_assert(1 + sizeof...(TAIL) <= Rows * Cols, "Too many arguments passed to Matrix");
    }

    template <typename DerivedType>
    Matrix(const MatrixBase<DerivedType, Rows, Cols, DType> &mat)
    {
        static_cast<MatrixBase<Matrix<Rows, Cols, DType>, Rows, Cols, DType> &>(*this) = mat;
    }

    Matrix &operator=(const Matrix &mat)
    {
        static_cast<MatrixBase<Matrix<Rows, Cols, DType>, Rows, Cols, DType> &>(*this) = mat;
        return *this;
    }
};

template <int Rows, int Cols = 1, typename DType = float>
class Zeros : public MatrixBase<Zeros<Rows, Cols, DType>, Rows, Cols, DType>
{
   public:
    DType operator()(int i, int j = 0) const { return 0.0; }

    Zeros() = default;
};

template <int Rows, int Cols = 1, typename DType = float>
class Ones : public MatrixBase<Ones<Rows, Cols, DType>, Rows, Cols, DType>
{
   public:
    DType operator()(int i, int j = 0) const { return 1.0; }

    Ones() = default;
};

template <int Rows, int Cols = 1, typename DType = float>
class Eye : public MatrixBase<Eye<Rows, Cols, DType>, Rows, Cols, DType>
{
   public:
    DType operator()(int i, int j = 0) const { return i == j; }

    Eye() = default;
};

template <typename RefType, int Rows, int Cols>
class RefMatrix : public MatrixBase<RefMatrix<RefType, Rows, Cols>, Rows, Cols, typename RefType::DType>
{
    RefType &parent_;
    const int row_offset_;
    const int col_offset_;

   public:
    explicit RefMatrix(RefType &parent, int row_offset = 0, int col_offset = 0)
        : parent_(parent), row_offset_(row_offset), col_offset_(col_offset)
    {
    }

    typename RefType::DType &operator()(int i, int j) { return parent_(i + row_offset_, j + col_offset_); }
    typename RefType::DType operator()(int i, int j) const { return parent_(i + row_offset_, j + col_offset_); }

    template <typename MatType>
    RefMatrix &operator=(const MatType &mat)
    {
        static_cast<MatrixBase<RefMatrix<RefType, Rows, Cols>, Rows, Cols, typename RefType::DType> &>(*this) = mat;
        return *this;
    }
};

template <typename RefType>
class MatrixTranspose
    : public MatrixBase<MatrixTranspose<RefType>, RefType::Cols, RefType::Rows, typename RefType::DType>
{
    RefType &parent_;

   public:
    explicit MatrixTranspose(RefType &parent) : parent_(parent) {}

    typename RefType::DType &operator()(int i, int j) { return parent_(j, i); }
    typename RefType::DType operator()(int i, int j) const { return parent_(j, i); }

    template <typename MatType>
    MatrixTranspose &operator=(const MatType &mat)
    {
        return static_cast<
                   MatrixBase<MatrixTranspose<RefType>, RefType::Cols, RefType::Rows, typename RefType::DType> &>(
                   *this) = mat;
    }
};

template <typename LeftType, typename RightType>
struct HorizontalConcat : public MatrixBase<HorizontalConcat<LeftType, RightType>, LeftType::Rows,
                                            LeftType::Cols + RightType::Cols, typename LeftType::DType>
{
    const LeftType &left;
    const RightType &right;

    HorizontalConcat(const LeftType &l, const RightType &r) : left(l), right(r) {}

    typename LeftType::DType operator()(int row, int col) const
    {
        return col < LeftType::Cols ? left(row, col) : right(row, col - LeftType::Cols);
    }
};

template <typename TopType, typename BottomType>
struct VerticalConcat : public MatrixBase<VerticalConcat<TopType, BottomType>, TopType::Rows + BottomType::Rows,
                                          TopType::Cols, typename TopType::DType>
{
    const TopType &top;
    const BottomType &bottom;

    VerticalConcat(const TopType &t, const BottomType &b) : top(t), bottom(b) {}

    typename TopType::DType operator()(int row, int col) const
    {
        return row < TopType::Rows ? top(row, col) : bottom(row - TopType::Rows, col);
    }
};

template <int Rows, int Cols, typename DType, int TableSize>
struct SparseMatrix : public MatrixBase<SparseMatrix<Rows, Cols, DType, TableSize>, Rows, Cols, DType>
{
    DType end;

    static constexpr int Size = TableSize;

    struct Element
    {
        int row, col;
        DType val;

        Element() { row = col = -1; }

    } table[TableSize];

    DType &operator()(int row, int col)
    {
        int hash = (row * Cols + col) % TableSize;

        for (int i = 0; i < TableSize; i++)
        {
            Element &item = table[(hash + i) % TableSize];

            if (item.row == -1 || item.val == 0)
            {
                item.row = row;
                item.col = col;
                item.val = 0;
            }

            if (item.row == row && item.col == col)
            {
                return item.val;
            }
        }

        return end;
    }

    DType operator()(int row, int col) const
    {
        int hash = (row * Cols + col) % TableSize;

        for (int i = 0; i < TableSize; i++)
        {
            const Element &item = table[(hash + i) % TableSize];

            if (item.row == row && item.col == col)
            {
                return item.val;
            }
        }

        return 0;
    }
};

template <int Dim, class DType>
struct PermutationMatrix : public MatrixBase<PermutationMatrix<Dim, DType>, Dim, Dim, DType>
{
    int idx[Dim];

    DType operator()(int row, int col) const { return idx[col] == row; }
};

template <class ParentType>
struct LowerUnitriangularMatrix : public MatrixBase<LowerUnitriangularMatrix<ParentType>, ParentType::Rows,
                                                    ParentType::Cols, typename ParentType::DType>
{
    const ParentType &parent;

    LowerUnitriangularMatrix(const ParentType &obj) : parent(obj) {}

    typename ParentType::DType operator()(int row, int col) const
    {
        if (row > col)
        {
            return parent(row, col);
        }
        else if (row == col)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
};

template <class ParentType>
struct LowerTriangularMatrix : public MatrixBase<LowerTriangularMatrix<ParentType>, ParentType::Rows, ParentType::Cols,
                                                 typename ParentType::DType>
{
    const ParentType &parent;

    LowerTriangularMatrix(const ParentType &obj) : parent(obj) {}

    typename ParentType::DType operator()(int row, int col) const
    {
        if (row >= col)
        {
            return parent(row, col);
        }
        else
        {
            return 0;
        }
    }
};

template <class ParentType>
struct UpperTriangularMatrix : public MatrixBase<UpperTriangularMatrix<ParentType>, ParentType::Rows, ParentType::Cols,
                                                 typename ParentType::DType>
{
    const ParentType &parent;

    UpperTriangularMatrix(const ParentType &obj) : parent(obj) {}

    typename ParentType::DType operator()(int row, int col) const
    {
        if (row <= col)
        {
            return parent(row, col);
        }
        else
        {
            return 0;
        }
    }
};

}  // namespace BLA
