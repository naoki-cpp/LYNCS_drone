/*=========================================================
	===		Matrix struct
			2018 Naoki Yano
			Usage::
				construct::
				////
				Matrix<double,3,3> matrix {{{1,0,1},{0,1,0},[0,0,1]}}
				////
===========================================================*/
#pragma once
namespace lyncs
{

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
struct Matrix
{
	T matrix_[COLUMN][ROW];

  public:
	~Matrix();

	const T GetElement(const unsigned int i, const unsigned int j) const;
	const Matrix<T, 1, ROW> GetColumn(const unsigned int i) const;
	const Matrix<T, COLUMN, 1> GetRow(const unsigned int j) const;

	const void WriteElement(unsigned int i, unsigned int j, T element);
	//operators
	const Matrix<T, COLUMN, ROW> operator+(const Matrix<T, COLUMN, ROW> &matrix) const;
	template <const unsigned int ROW2>
	const Matrix<T, COLUMN, ROW2> operator*(const Matrix<T, ROW, ROW2> &matrix) const;
	Matrix<T, COLUMN, ROW> &operator=(Matrix<T, COLUMN, ROW> matrix);
};

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
Matrix<T, COLUMN, ROW>::~Matrix()
{
}
template <typename T, const unsigned int COLUMN, const unsigned int ROW>
const T Matrix<T, COLUMN, ROW>::GetElement(const unsigned int i, const unsigned int j) const //TODO:(naoki-cpp) i,jが無効の場合の処理をうまいことやる.
{
	if (i < COLUMN && j < ROW)
	{
		return matrix_[i][j];
	}
	return 0;
}

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
const Matrix<T, 1, ROW> Matrix<T, COLUMN, ROW>::GetColumn(const unsigned int i) const //TODO:(naoki-cpp) iが無効の場合の処理をうまいことやる.
{
	Matrix<T, 1, ROW> column;
	if (i < COLUMN)
	{
		for (int j = 0; j < ROW; j++)
		{
			column.WriteElement(1, j, this->GetElement(i, j));
		}
	}
	return column;
}

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
const Matrix<T, COLUMN, 1> Matrix<T, COLUMN, ROW>::GetRow(const unsigned int j) const //TODO:(naoki-cpp) iが無効の場合の処理をうまいことやる.
{
	Matrix<T, COLUMN, 1> row;
	if (j < ROW)
	{
		for (int i = 0; i < COLUMN; i++)
		{
			row.WriteElement(i, 1, this->GetElement(i, j));
		}
	}
	return row;
}

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
const void Matrix<T, COLUMN, ROW>::WriteElement(unsigned int i, unsigned int j, T element)
{
	if (i < COLUMN && j < ROW)
	{
		matrix_[i][j] = element;
	}
}

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
Matrix<T, COLUMN, ROW> &Matrix<T, COLUMN, ROW>::operator=(Matrix<T, COLUMN, ROW> matrix)
{
	for (int i = 0; i < COLUMN; i++)
	{
		for (int j = 0; i < ROW; j++)
		{
			matrix_[i][j] = matrix.GetElement(i, j);
		}
	}
	return *this;
}

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
const Matrix<T, COLUMN, ROW> Matrix<T, COLUMN, ROW>::operator+(const Matrix<T, COLUMN, ROW> &matrix) const
{
	Matrix<T, COLUMN, ROW> sum;
	for (int i = 0; i < COLUMN; i++)
	{
		for (int j = 0; j < ROW; j++)
		{
			sum.WriteElement(i, j, matrix_[i][j] + matrix.GetElement(i, j));
		}
	}
	return sum;
}
template <typename T, const unsigned int COLUMN, const unsigned int ROW>
template <unsigned int ROW2>
const Matrix<T, COLUMN, ROW2> Matrix<T, COLUMN, ROW>::operator*(const Matrix<T, ROW, ROW2> &matrix) const
{
	Matrix<T, COLUMN, ROW> product;
	for (int i = 0; i < COLUMN; i++)
	{
		for (int j = 0; j < ROW2; j++)
		{
			T sum(0);
			for (int u = 0; u < ROW; u++)
			{
				sum += matrix_[i][u] * matrix.GetElement(u, j);
			}
			product.WriteElement(i, j, sum);
		}
	}
	return product;
}

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
const Matrix<T, ROW, COLUMN> TransposeMatrix(const Matrix<T, COLUMN, ROW> matrix)
{
	Matrix<T, ROW, COLUMN> transposed;
	for (int i = 0; i < COLUMN; i++)
	{
		for (int j = 0; j < ROW; j++)
		{
			transposed.WriteElement(j, i, matrix.GetElement(i, j));
		}
	}
	return transposed;
}

} // namespace lyncs