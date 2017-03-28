#pragma once
#ifndef __MMATRIX
#define __MMATRIX
#include <typeinfo>
#include <cmath>
#include <utility>
namespace MMatrix {
	//◊›œÚ≈≈¡–
	template <typename T>
	class CMatrix {
	protected:
		T* data;
		unsigned int rowCount;
		unsigned int columnCount;
		unsigned int size;
	public:

		CMatrix():rowCount(0), columnCount(0), size(0){
			data = new T[0];
		}

		CMatrix(const unsigned int rowCount, const unsigned int columnCount)
			:rowCount(rowCount),columnCount(columnCount),size(rowCount*columnCount) {
			data = new T[size];
		}

		CMatrix(const unsigned int rowCount, const unsigned int columnCount, T values...)
			:rowCount(rowCount), columnCount(columnCount), size(rowCount*columnCount) {
			data = new T[size];
			T* p = data;
			va_list dataPtr;
			__crt_va_start(dataPtr,values);
			*p = values;
			p++;
			for (unsigned int i = 1; i < size; i++,p++) {
				if (typeid(T).name() == typeid(float).name() || typeid(T).name() == typeid(double).name()) {
					*p = __crt_va_arg(dataPtr, double);
				}
				else if (
					typeid(T).name() == typeid(int).name()
					|| typeid(T).name() == typeid(short).name()
					|| typeid(T).name() == typeid(char).name()
					|| typeid(T).name() == typeid(unsigned int).name()
					|| typeid(T).name() == typeid(unsigned short).name()
					|| typeid(T).name() == typeid(unsigned char).name()
					|| typeid(T).name() == typeid(bool).name()) {
					*p = __crt_va_arg(dataPtr, int);
				}else{
					*p == __crt_va_arg(dataPtr, T);
				}
			}
			__crt_va_end(dataPtr);
		}

		CMatrix(std::initializer_list<T> values) {
			new (this)CMatrix<T>(values.size(), 1, values);
		}

		CMatrix(std::initializer_list<std::initializer_list<T>> values) {
			columnCount = values.size();
			unsigned int max = 0;
			for (std::initializer_list<T> var : values) {
				if (var.size() > max) {
					max = var.size();
				}
			}
			rowCount = max;
			size = columnCount*rowCount;
			data = new T[size];
			T* p = data;
			for (std::initializer_list<T> var : values) {
				auto q= var.begin();
				for (unsigned int i = 0; i < rowCount; i++, q++, p++) {
					if (i < var.size()) {
						*p = *q;
					}
					else {
						*p = 0;
					}
				}
			}
		}

		CMatrix(const unsigned int rowCount, const unsigned int columnCount, std::initializer_list<T> values)
			:rowCount(rowCount), columnCount(columnCount), size(rowCount*columnCount) {
			data = new T[size];
			unsigned int i = 0;
			T* p = data;
			for each (T var in values)
			{
				if (i < size) {
					*p = var;
					p++;
					i++;
				}
				else {
					break;
				}
			}
		}

		CMatrix(const unsigned int rowCount, const unsigned int columnCount,T * data)
			:rowCount(rowCount), columnCount(columnCount), size(rowCount*columnCount),data(data) {
		}

		template<typename K>
		CMatrix(const CMatrix<K>& m) {
			T* ndata = new T[m.Size()];
			K* p = m.Data();
			T* q = ndata;
			for (unsigned int i = 0; i < m.Size(); i++, p++, q++) {
				*q = *p;
			}
			data = ndata;
			rowCount = m.RowCount();
			columnCount = m.ColumnCount();
			size = m.Size();
		}

		CMatrix(const CMatrix<T>& m) {
			T* ndata = new T[m.Size()];
			T* p = m.Data();
			T* q = ndata;
			for (unsigned int i = 0; i < m.Size(); i++, p++, q++) {
				*q = *p;
			}
			data = ndata;
			rowCount = m.RowCount();
			columnCount = m.ColumnCount();
			size = m.Size();
		}

		static CMatrix<T> Identity(const unsigned int n) {
			CMatrix<T> ret(n, n);
			for (unsigned int i = 0; i < n; i++) {
				for (unsigned int j = 0; j < n; j++) {
					ret(i, j) = (i == j) ? 1 : 0;
				}
			}
			return ret;
		}

		static CMatrix<T> Zero(const unsigned int r, const unsigned int c) {
			CMatrix<T> ret(r, c);
			for (int i = 0; i < ret.Size(); i++) {
				ret[i] = 0;
			}
			return ret;
		}

		~CMatrix(){
			delete[] data;
		}

		template <typename K>
		void Assign(std::pair<unsigned int ,unsigned int> rowColumn,const CMatrix<K>& m) {
			if (m.RowCount() + rowColumn.first > rowCount || m.ColumnCount() + rowColumn.second > columnCount) {
				throw;
			}
			for (unsigned int r = 0; r < m.RowCount(); r++) {
				for (unsigned int c = 0; c < m.ColumnCount(); c++) {
					(*this)(r + rowColumn.first, c + rowColumn.second) = m(r, c);
				}
			}
		}

		CMatrix Copy() {
			T * ndata = new T[size];
			T * q = ndata;
			T * p = data;
			for (unsigned int i = 0; i < size; i++, p++, q++) {
				*q = *p;
			}
			return CMatrix(rowCount, columnCount, ndata);
		}

		CMatrix Copy(std::pair<unsigned int, unsigned int> startRowColumn, std::pair<unsigned int, unsigned int> endRowColumn) {
			if (endRowColumn.first >= this->rowCount || endRowColumn.second >= this->columnCount) {
				throw;
			}
			if (startRowColumn.first > endRowColumn.first || startRowColumn.second > endRowColumn.second) {
				throw;
			}
			unsigned int rowCount = endRowColumn.first - startRowColumn.first + 1;
			unsigned int columnCount = endRowColumn.second - startRowColumn.second + 1;
			T * ndata = new T[rowCount*columnCount];
			T * q = ndata;
			for (unsigned int c = startRowColumn.second; c <= endRowColumn.second; c++) {
				for (unsigned int r = startRowColumn.first; r <= endRowColumn.first; r++, q++) {
					*q = (*this)(r, c);
				}
			}
			return CMatrix(rowCount, columnCount, ndata);
		}

		CMatrix Transpose() {
			T* ndata = new T[size];
			T* p = ndata;
			for (unsigned int r = 0; r < rowCount; r++) {
				for (unsigned int c = 0; c < columnCount; c++, p++) {
					*p = (*this)(r, c);
				}
			}
			return CMatrix(columnCount, rowCount, ndata);
		}

		CMatrix Inverse() {

		}

		CMatrix Row(unsigned int row) {
			if (row >= rowCount) {
				throw;
			}
			return Copy({ row,0 }, { row,columnCount - 1 });
		}

		CMatrix Column(unsigned int col) {
			if (col >= columnCount) {
				throw;
			}
			return Copy({ 0,col }, { rowCount - 1,col });
		}

		void Resize(const unsigned int rowCount, const unsigned int columnCount) {
			//unsigned int size = rowCount*columnCount;
			//T* newData = new T[size];
			//for (T* p = newData, unsigned int i = 0; i < size && i < this->size; i++, data++, p++) {
			//	*p = *data;
			//}
			//delete[] data;
			//data = newData;
			this->rowCount = rowCount;
			this->columnCount = columnCount;
			this->size = rowCount*columnCount;
		}

		unsigned int RowCount() const{
			return rowCount;
		}

		unsigned int ColumnCount() const {
			return columnCount;
		}

		unsigned int Size() const {
			return size;
		}

		T* Data() const{
			return data;
		}

		T& operator ()(const unsigned int& rowIndex,const unsigned int& columnIndex) const{
			return *(data + rowIndex + columnIndex*rowCount);
		}

		T& operator [](const unsigned int& index) const{
			return *(data + index);
		}
		
		operator T(){
			T temp = 0;
			T* p = data;
			for (unsigned int i = 0; i < size; i++, p++) {
				temp += (*p)*(*p);

			}
			return sqrt(temp);
		}

		template<typename K>
		CMatrix &operator =(const CMatrix<K>& m) {
			T* ndata = new T[m.Size()];
			K* p = m.Data();
			T* q = ndata;
			for (unsigned int i = 0; i < m.Size(); i++, p++, q++) {
				*q = *p;
			}
			delete[] data;
			data = ndata;
			rowCount = m.RowCount();
			columnCount = m.ColumnCount();
			size = m.Size();
			return *this;
		}

		CMatrix &operator =(const CMatrix<T>& m) {
			T* ndata = new T[m.Size()];
			T* p = m.Data();
			T* q = ndata;
			for (unsigned int i = 0; i < m.Size(); i++, p++, q++) {
				*q = *p;
			}
			delete[] data;
			data = ndata;
			rowCount = m.RowCount();
			columnCount = m.ColumnCount();
			size = m.Size();
			return *this;
		}

		CMatrix operator -() {
			return operator *(-1);
		}

		template<typename K>
		CMatrix &operator +=(const K& value) {
			T* p = data;
			for (unsigned int i = 0; i < size; i++, p++) {
				(*p) += value;
			}
			return *this;
		}

		template<typename K>
		CMatrix operator +(const K&value) {
			T* ndata = new T[size];
			T* p = data;
			T* q = ndata;
			for (unsigned int i = 0; i < size; i++, p++, q++) {
				*q = (*p) + value;
			}
			return CMatrix(rowCount, columnCount, ndata);
		}

		template<typename K>
		CMatrix &operator -=(const K& value) {
			T* p = data;
			for (unsigned int i = 0; i < size; i++, p++) {
				(*p) -= value;
			}
			return *this;
		}

		template<typename K>
		CMatrix operator -(const K&value) {
			T* ndata = new T[size];
			T* p = data;
			T* q = ndata;
			for (unsigned int i = 0; i < size; i++, p++, q++) {
				*q = (*p) - value;
			}
			return CMatrix(rowCount, columnCount, ndata);
		}

		template<typename K>
		CMatrix &operator *=(const K& value) {
			T* p = data;
			for (unsigned int i = 0; i < size; i++, p++) {
				(*p) *= value;
			}
			return *this;
		}

		template<typename K>
		CMatrix operator *(const K&value) {
			T* ndata = new T[size];
			T* p = data;
			T* q = ndata;
			for (unsigned int i = 0; i < size; i++, p++, q++) {
				*q = (*p)*value;
			}
			return CMatrix(rowCount, columnCount, ndata);
		}

		template<typename K>
		CMatrix &operator /=(const K&value) {
			return operator*=(1 / value);
		}

		template<typename K>
		CMatrix operator /(const K&value) {
			return operator*(1 / value);
		}

		template<typename K>
		CMatrix& operator +=(const CMatrix<K>& m) {
			if (columnCount != m.ColumnCount()
				|| rowCount != m.RowCount()) {
				throw;
			}
			T * p = data;
			K *q = m.Data();
			for (unsigned int i = 0; i < size; i++, p++, q++) {
				(*p) += *q;
			}
			return *this;
		}

		template<typename K>
		CMatrix operator +(const CMatrix<K>& m) {
			if (columnCount != m.ColumnCount()
				|| rowCount != m.RowCount()) {
				throw;
			}
			T * p = data;
			K *q = m.Data();
			T* ndata = new T[size];
			T *r = ndata;
			for (unsigned int i = 0; i < size; i++, p++, q++, r++) {
				*r = (*p) + (*q);
			}
			return CMatrix(rowCount, columnCount, ndata);
		}

		template<typename K>
		CMatrix& operator -=(const CMatrix<K>& m) {
			if (columnCount != m.ColumnCount()
				|| rowCount != m.RowCount()) {
				throw;
			}
			T * p = data;
			K *q = m.Data();
			for (unsigned int i = 0; i < size; i++, p++, q++) {
				(*p) -= *q;
			}
			return *this;
		}

		template<typename K>
		CMatrix operator -(const CMatrix<K>& m) {
			if (columnCount != m.ColumnCount()
				|| rowCount != m.RowCount()) {
				throw;
			}
			T * p = data;
			K *q = m.Data();
			T* ndata = new T[size];
			T *r = ndata;
			for (unsigned int i = 0; i < size; i++, p++, q++, r++) {
				*r = (*p) - (*q);
			}
			return CMatrix(rowCount, columnCount, ndata);
		}

		template<typename K>
		CMatrix &operator *=(const CMatrix<K>& m) {
			if (columnCount != m.RowCount()) {
				throw;
			}
			T* ndata = new T[rowCount*m.ColumnCount()];
			T* p = ndata;
			for (unsigned int r = 0; r < m.ColumnCount(); r++) {
				for (unsigned int c = 0; c < rowCount; c++, p++) {
					T temp = 0;
					for (unsigned int i = 0; i < columnCount; i++) {
						temp += (*this)(c, i)*m(i, r);
					}
					*p = temp;
				}
			}
			delete[] data;
			data = ndata;
			rowCount = rowCount;
			columnCount = m.ColumnCount();
			size = rowCount*columnCount;
			return *this;
		}

		template<typename K>
		CMatrix operator *(const CMatrix<K>& m) {
			if (columnCount != m.RowCount()) {
				throw;
			}
			T* ndata = new T[rowCount*m.ColumnCount()];
			T* p = ndata;
			for (unsigned int r = 0; r < m.ColumnCount(); r++) {
				for (unsigned int c = 0; c < rowCount; c++, p++) {
					T temp = 0;
					for (unsigned int i = 0; i < columnCount; i++) {
						temp += (*this)(c, i)*m(i, r);
					}
					*p = temp;
				}
			}
			return CMatrix(rowCount, m.ColumnCount(), ndata);
		}

	};

	template<typename T, typename K>
	CMatrix<T> KroneckerProduct(const CMatrix<T>& A, const CMatrix<K>& B) {
		CMatrix<T> ret(A.RowCount()*B.RowCount(), A.ColumnCount()*B.RowCount());
		for (unsigned int m = 0; m < A.RowCount(); m++) {
			for (unsigned int n = 0; n < A.ColumnCount(); n++) {
				T tempA = A(m, n);
				for (unsigned int p = 0; p < B.RowCount(); p++) {
					for (unsigned int q = 0; q < B.ColumnCount(); q++) {
						ret(m*B.RowCount() + p, n*B.ColumnCount() + q) = tempA*B(p, q);
					}
				}
			}
		}
		return ret;
	}

	template<typename T, typename K>
	CMatrix<T> HadamardProduct(const CMatrix<T>& A, const CMatrix<K>& B) {
		if (A.RowCount() != B.RowCount() || A.ColumnCount() != B.ColumnCount()) {
			throw;
		}
		T * ndata = new T[A.Size()];
		T * n = ndata;
		T * a = A.Data();
		T * b = B.Data();
		for (unsigned int i = 0; i < A.Size(); i++, n++, a++, b++) {
			*n = (*a)*(*b);
		}
		return CMatrix<T>(A.RowCount(), A.ColumnCount(), ndata);
	}

	template<typename T>
	CMatrix<T> powM(const CMatrix<T>& A, unsigned int x) {
		if (A.RowCount() != A.ColumnCount())
		{
			throw;
		}
		if (x == 0) {
			return CMatrix<T>::Identity(A.RowCount());
		}
		CMatrix<T> ret = A;
		for (unsigned int i = 1; i < x; i++) {
			ret *= A;
		}
		return ret;
	}

	template<typename T, typename K>
	CMatrix<T> StrassenSquareMMultiply(const CMatrix<T>& A, const CMatrix<K>& B
		, std::pair<std::pair<unsigned int, unsigned int>, std::pair<unsigned int, unsigned int>> rangeA
	, std::pair<std::pair<unsigned int, unsigned int>, std::pair<unsigned int, unsigned int>> rangeB) {
		//if (A.RowCount() != A.ColumnCount()) {
		//	throw;
		//}
		//if (A.RowCount() != B.RowCount() || A.ColumnCount() != B.ColumnCount()) {
		//	throw;
		//}
		//float nn = log2(A.RowCount());
		//if (nn - (int)nn != 0) {
		//	throw;
		//}
		unsigned int ALeft = rangeA.first.second;
		unsigned int ARight = rangeA.second.second;
		unsigned int ATop = rangeA.first.first;
		unsigned int BLeft = rangeB.first.second;
		unsigned int BTop = rangeB.first.first;
		if (ARight - ALeft == 1) {
			//T temp = A(ATop, ALeft)*B(BTop, BLeft);
			return CMatrix<T>(1, 1, A(ATop, ALeft)*B(BTop, BLeft));
		}
		unsigned int ABottom = rangeA.second.first;
		unsigned int AMidX = (ARight + ALeft) / 2;
		unsigned int AMidY = (ABottom + ATop) / 2;
		unsigned int BBottom = rangeB.second.first;
		unsigned int BRight = rangeB.second.second;
		unsigned int BMidX = (BRight + BLeft) / 2;
		unsigned int BMidY = (BBottom + BTop) / 2;
		std::pair<std::pair<unsigned int, unsigned int>, std::pair<unsigned int, unsigned int>> A11{ { ATop,ALeft },{ AMidY,AMidX } };
		std::pair<std::pair<unsigned int, unsigned int>, std::pair<unsigned int, unsigned int>> A12{ { ATop,AMidX },{ AMidY, ARight } };
		std::pair<std::pair<unsigned int, unsigned int>, std::pair<unsigned int, unsigned int>> A21{ {AMidY,ALeft},{ABottom,AMidX} };
		std::pair<std::pair<unsigned int, unsigned int>, std::pair<unsigned int, unsigned int>> A22{ {AMidY,AMidX},{ABottom,ARight} };
		std::pair<std::pair<unsigned int, unsigned int>, std::pair<unsigned int, unsigned int>> B11{ { BTop,BLeft },{ BMidY,BMidX } };
		std::pair<std::pair<unsigned int, unsigned int>, std::pair<unsigned int, unsigned int>> B12{ { BTop,BMidX },{ BMidY, BRight } };
		std::pair<std::pair<unsigned int, unsigned int>, std::pair<unsigned int, unsigned int>> B21{ { BMidY,BLeft },{ BBottom,BMidX } };
		std::pair<std::pair<unsigned int, unsigned int>, std::pair<unsigned int, unsigned int>> B22{ { BMidY,BMidX },{ BBottom,BRight } };
		CMatrix<T> C11 = StrassenSquareMMultiply(A, B, A11, B11) + StrassenSquareMMultiply(A, B, A12, B21);
		CMatrix<T> C12 = StrassenSquareMMultiply(A, B, A11, B12) + StrassenSquareMMultiply(A, B, A12, B22);
		CMatrix<T> C21 = StrassenSquareMMultiply(A, B, A21, B11) + StrassenSquareMMultiply(A, B, A22, B21);
		CMatrix<T> C22 = StrassenSquareMMultiply(A, B, A21, B12) + StrassenSquareMMultiply(A, B, A22, B22);
		unsigned int size = (ARight - ALeft)*(ABottom - ATop);
		T* data = new T[size];
		T* d = data;
		for (unsigned int c11 = 0; c11 < AMidX - ALeft; c11++) {
			for (unsigned int r11 = 0; r11 < AMidY - ATop; r11++, d++) {
				*d = C11(r11, c11);
			}
			for (unsigned int r21 = 0; r21 < ABottom - AMidY; r21++, d++) {
				*d = C21(r21, c11);
			}
		}
		for (unsigned int c12 = 0; c12 < AMidX - ALeft; c12++) {
			for (unsigned int r11 = 0; r11 < AMidY - ATop; r11++, d++) {
				*d = C12(r11, c12);
			}
			for (unsigned int r21 = 0; r21 < ABottom - AMidY; r21++, d++) {
				*d = C22(r21, c12);
			}
		}
		return CMatrix<T>(ABottom - ATop, ARight - ALeft, data);
	}
	
	template<typename T, typename K>
	CMatrix<T> StrassenSquareMMultiply(const CMatrix<T>& A, const CMatrix<K>& B) {
		return StrassenSquareMMultiply(A, B, { { 0,0 },{ A.RowCount(),A.ColumnCount() } }, { { 0,0 },{ B.RowCount(),B.ColumnCount() } });
	}

	template<typename T>
	void printf(CMatrix<T> m) {

		for (unsigned int r = 0; r < m.RowCount(); r++) {
			for (unsigned int c = 0; c < m.ColumnCount(); c++) {
				std::cout << m(r, c) << "\t";
			}
			std::cout << std::endl;
		}
	}
}
#endif // !__MMATRIX