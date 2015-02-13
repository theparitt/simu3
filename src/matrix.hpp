/*--------------------------------------------------------------
 __    __                                         __    __
/\ \__/\ \                                     __/\ \__/\ \__
\ \ ,_\ \ \___      __  _____     __     _ __ /\_\ \ ,_\ \ ,_\
 \ \ \/\ \  _ `\  /'__`\\ '__`\ /'__`\  /\`'__\/\ \ \ \/\ \ \/
  \ \ \_\ \ \ \ \/\  __/ \ \L\ \\ \L\.\_\ \ \/ \ \ \ \ \_\ \ \_
   \ \__\\ \_\ \_\ \____\ \ ,__/ \__/.\_\\ \_\  \ \_\ \__\\ \__\
    \/__/ \/_/\/_/\/____/\ \ \/ \/__/\/_/ \/_/   \/_/\/__/ \/__/
                          \ \_\
                           \/_/
----------------------------------------------------------------*/

/*
 * matrix.hpp
 * Copyright (C) 2014 [theparitt]
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by theparit peerasathien <theparitt@gmail.com>, December 2014
 */


#ifndef __MATRIX_H__
#define __MATRIX_H__


#include <vector>
#include <cassert>
#include <cstring>
#include <stdexcept>


#ifndef __SMALL_VALUE
	const double SMALL_VALUE = 0.00001; //��Ҥ��abs���¡��Ҥ�Ңͧ��ǹ������ͧ��0
#define __SMALL_VALUE
#endif




//inline function ����������� x �դ�����Ѻ 0 ��������
inline bool is_closeto_zero(double x)
{
	//��constant 'small_value' �繪�ǧ㹡�÷��ͺ���x
	return ( (-SMALL_VALUE<(double)(x))&&(double(x)<SMALL_VALUE) );
};

inline bool is_closeto(int x, int y)
{
	return x==y;
};

inline bool is_not_closeto(int x, int y)
{
	return x!=y;
};

inline bool is_closeto(float x, float y)
{
	return ((y-SMALL_VALUE<x)&&(x<y+SMALL_VALUE));
};

inline bool is_not_closeto(float x, float y)
{
	return !((y-SMALL_VALUE<x)&&(x<y+SMALL_VALUE));
};

inline bool is_closeto(double x, double y)
{
	return ((y-SMALL_VALUE<x)&&(x<y+SMALL_VALUE));
};

inline bool is_not_closeto(double x, double y)
{
	return !((y-SMALL_VALUE<x)&&(x<y+SMALL_VALUE));
};






class matrix_container
{



protected:


	void create(int nrow, int ncol)
	{
		assert( data == NULL );
		data = new double[ nrow*ncol ];
	};


	void create(const matrix_container& A)
	{
		assert( data == NULL );
		this->nrow = A.nrow;
		this->ncol = A.ncol;

		data = new double[ nrow*ncol ];
		memcpy(this->data, A.data, sizeof(double)*A.nrow*A.ncol );
	}


	double* data;

public:
	size_t nrow;
	size_t ncol;



	matrix_container()
		:data( NULL ), nrow(0), ncol(0)
	{

	}


	matrix_container(size_t nrow, size_t ncol)
		:data( NULL ), nrow(nrow), ncol(ncol)
	{
		create(nrow, ncol);
	};



	matrix_container(const matrix_container& A)
		:data( NULL ), nrow(0), ncol(0)
	{
		create( A );
	};


	double operator()(size_t i, size_t j) const
	{
		return data[ i*ncol + j ];
	}


	double& operator()(size_t i, size_t j)
	{
		return data[ i*ncol + j ];
	}


	double at(size_t i, size_t j) const
	{
		return data[ i*ncol + j ];
	}


	double& at(size_t i, size_t j)
	{
		return data[ i*ncol + j ];
	}


	virtual ~matrix_container()
	{
		nrow = 0;
		ncol = 0;
		if( data != NULL )
		{
			delete data;
			data = NULL;
		}
	};


	virtual matrix_container& operator=(const matrix_container& rvalue)
	{
		if (&rvalue == this)
		{
			return *this;
		}

		assert( nrow == rvalue.nrow && ncol == rvalue.ncol );
		memcpy( data, rvalue.data, nrow*ncol*sizeof(double) );
		return *this;
	};



	void set_all(double val)
	{
		memset( data, val, nrow*ncol*sizeof(double) );
	}

};






class matrix_core: public matrix_container
{

protected:



	//------------------------------------------------------------------------------
public:
	matrix_core()
		:matrix_container()
	{
	};


	matrix_core(const matrix_core& A)
		:matrix_container( A )
	{
	}


	matrix_core(size_t nrow, size_t ncol)
		:matrix_container(nrow, ncol)
	{

	};


	virtual ~matrix_core()
	{
	};


	void debug(const char* s="", const char* e="") const
	{
		printf("%s\n", s);
		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				printf("%f ", at(i,j) );
			}
			printf("\n");
		}
		printf("%s\n", e);
	}


	void set_identity()
	{
		assert( nrow == ncol );
		set_all(0.0);
		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				at(i,i) = 1.0;
			}
		}
	};


	double& operator()(size_t i,size_t j)
	{
		assert ( (i >= 0) && (i<nrow) && (j >= 0) && (j<ncol) );
		return at(i,j);
	};


	double operator()(size_t i,size_t j) const
	{
		assert ( (i >= 0) && (i<nrow) && (j >= 0) && (j<ncol) );
		return at(i,j);
	};


	void set_all_to(const double& value)
	{
		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				at(i,j) = value;
			}
		}
	};





	void swap_row(size_t i, size_t m)
	{
		assert( (i>=0) && (m>=0) && (i<nrow) && m<nrow);

		double temp;
		for(size_t j=0; j<ncol; j++)
		{
			temp = at(i,j);
			at(i,j) = at(m,j);
			at(m,j) = temp;
		}

	}



	void swap_col(size_t j, size_t n)
	{
		assert( j>=0 || n>=0 || j<ncol || n<ncol );

		double temp;
		for(size_t i=0; i<ncol; i++)
		{
			temp = at(i,j);
			at(i,j) = at(i,n);
			at(i,n) = temp;
		}
	}

	//unary operator
	virtual matrix_core& operator+=(const matrix_core& rvalue)
	{
		assert( (rvalue.nrow == nrow) && (rvalue.ncol == ncol) );

		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				at(i,j) += rvalue(i,j);
			}
		}
		return *this;
	};


	virtual matrix_core& operator+=(const double& rvalue)
	{
		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				at(i,j) += rvalue;
			}
		}
		return *this;
	};


	virtual matrix_core& operator-=(const matrix_core& rvalue)
	{
		assert ( (rvalue.ncol == ncol) && (rvalue.nrow == nrow) );

		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				at(i,j) -= rvalue(i,j);
			}
		}
		return *this;
	};


	virtual matrix_core& operator-=(const double& rvalue)
	{
		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				at(i,j) -= (double)rvalue;
			}
		}
		return *this;
	};


	virtual matrix_core& operator*=(const double& rvalue)
	{
		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				at(i,j) *= (double)rvalue;
			}
		}
		return *this;
	}


	matrix_core& operator/=(const double& rvalue)
	{
		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				at(i,j)/= (double)rvalue;
			}
		}
		return *this;
	}


	matrix_core& operator<(const double& rvalue)
	{
		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				if( at(i,j) >= rvalue)
				{
					at(i,j) = static_cast<double>(0.0);
				}
			}
		}
		return *this;
	}

	matrix_core& operator<=(const double& rvalue)
	{
		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				if (at(i,j) > rvalue)
				{
					at(i,j) = static_cast<double>(0.0);
				}
			}
		}
		return *this;
	}

	matrix_core& operator>(const double& rvalue)
	{
		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				if (at(i,j) <= rvalue)
				{
					at(i,j) = static_cast<double>(0.0);
				}
			}
		}

		return *this;
	}

	matrix_core& operator>=(const double& rvalue)
	{
		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				if( at(i,j) < rvalue)
				{
					at(i,j) = 0.0;
				}
			}
		}

		return *this;
	}

	matrix_core& operator==(const double& rvalue)
	{
		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				if (at(i,j) != rvalue)
				{
					at(i,j) = 0.0;
				}
			}
		}

		return *this;
	}

	matrix_core& operator!=(const double& rvalue)
	{
		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				if (at(i,j) == rvalue)
				{
					at(i,j) = 0.0;
				}
			}
		}

		return *this;
	}


	friend matrix_core operator+(const matrix_core& lvalue, const matrix_core& rvalue)
	{
		assert ( (lvalue.ncol == rvalue.ncol)&&(lvalue.nrow == rvalue.nrow) );

		matrix_core result(lvalue);
		for(size_t i=0; i<lvalue.nrow; i++)
		{
			for(size_t j=0; j<lvalue.ncol; j++)
			{
				result(i,j) += rvalue(i,j);
			}
		}

		return result;
	};



	friend matrix_core operator+(const matrix_core& lvalue, const double& rvalue)
	{
		matrix_core result(lvalue);
		for(size_t i=0; i<lvalue.nrow; i++)
		{
			for(size_t j=0; j<lvalue.ncol; j++)
			{
				result(i,j) += rvalue;
			}
		}

		return result;
	};



	friend matrix_core operator+(const double& lvalue, const matrix_core& rvalue)
	{
		matrix_core result(rvalue);
		for(size_t i=0; i<rvalue.nrow; i++)
		{
			for(size_t j=0; j<rvalue.ncol; j++)
			{
				result(i,j) += lvalue;
			}
		}

		return result;
	};



	friend matrix_core operator-(const matrix_core& lvalue, const matrix_core& rvalue)
	{
		assert ( (lvalue.ncol == rvalue.ncol) && (lvalue.nrow == rvalue.nrow) );

		matrix_core result(lvalue);
		for(size_t i=0; i<lvalue.nrow; i++)
		{
			for(size_t j=0; j<lvalue.ncol; j++)
			{
				result(i,j) -= rvalue(i,j);
			}
		}

		return result;
	}


	friend matrix_core operator-(const matrix_core& lvalue, const double& rvalue)
	{
		matrix_core result(lvalue);
		for(size_t i=0; i<lvalue.nrow; i++)
		{
			for(size_t j=0; j<lvalue.ncol; j++)
			{
				result(i,j) -= rvalue;
			}
		}
		return result;
	};


	friend matrix_core operator-(const double& lvalue, const matrix_core& rvalue)
	{
		matrix_core result(rvalue);
		for(size_t i=0; i<rvalue.nrow; i++)
		{
			for(size_t j=0; j<rvalue.ncol; j++)
			{
				result(i,j) = lvalue-result(i,j);
			}
		}
		return result;
	};


	friend matrix_core operator*(const matrix_core& lvalue, const matrix_core& rvalue)
	{
		assert ( (lvalue.nrow == rvalue.ncol) );

		matrix_core result(lvalue.ncol,rvalue.nrow);

		for(size_t i=0; i<lvalue.nrow; i++)
		{
			for(size_t k=0; k<rvalue.ncol; k++)
			{
				double sum = static_cast<double>(0.0);
				for(size_t j=0; j<lvalue.nrow; j++)
					sum += (lvalue(i,j) * rvalue(j,k) );
				result(i,k) = sum;
			}
		}

		return result;
	};


	friend matrix_core operator*(const matrix_core& lvalue, const double& rvalue)
	{
		matrix_core result(lvalue);
		for(size_t i=0; i<lvalue.nrow; i++)
		{
			for(size_t j=0; j<lvalue.ncol; j++)
			{
				result(i,j) *= rvalue;
			}
		}
		return result;
	};


	friend matrix_core operator*(const double& lvalue, const matrix_core& rvalue)
	{
		matrix_core result(rvalue);
		for(size_t i=0; i<rvalue.nrow; i++)
		{
			for(size_t j=0; j<rvalue.ncol; j++)
			{
				result(i,j) *= lvalue;
			}
		}
		return result;
	};


	friend matrix_core operator/(const matrix_core& lvalue, const double& rvalue)
	{
		matrix_core result(lvalue);
		for(size_t i=0; i<lvalue.nrow; i++)
		{
			for(size_t j=0; j<lvalue.ncol; j++)
			{
				result(i,j) /= rvalue;
			}
		}
		return result;
	};


	// sum triagonal member of matrix
	friend double tr(const matrix_core& mat)
	{
		double sum = 0.0;
		for(size_t i=0; i<mat.nrow; i++)
		{
			sum += mat(i,i);
		}

		return sum;
	}


	//operator, ����������Ѻ���matrix 2�����Ҵ��¡ѹ (augment)
	friend matrix_core operator,(const matrix_core& lvalue, const matrix_core& rvalue)
	{
		assert(lvalue.nrow == rvalue.nrow);

		matrix_core result(lvalue.nrow, lvalue.ncol + rvalue.ncol);
		for(size_t i=0; i<result.nrow; i++)
		{
			for(size_t j=0; j<result.ncol; j++)
			{
				if(j<lvalue.ncol)
				{
					result(i,j) = lvalue(i,j);
				}
				else{
					result(i,j) = rvalue(i,j-lvalue.ncol);
				}
			}
		}
		return result;
	};


	//matrix transpose
	inline matrix_core tp() const
	{	//���ҧ�ҡ��Ҵ m*n �����transpose ���Ǩ��� n*m
		matrix_core result(this->ncol, this->nrow);
		for(size_t i=0; i<nrow; i++)
		{
			for(size_t j=0; j<ncol; j++)
			{
				result(i,j) = at(j,i);
			}
		}
		return result;
	};

	//Matrix inverse
	virtual matrix_core inv() const
	{
		assert(ncol == nrow);

		matrix_core iden(nrow,ncol);
		iden.set_identity();
		//���¡gauss-jordan ������X ����� inverse �ҡ AX = B
		return gauss_jordan_elimination( *this, iden );
	};


	friend inline bool is_square_matrix(const matrix_core& matrix)
	{
		if (matrix.nrow == matrix.ncol)
			return true;
		return false;
	}

	friend inline bool is_not_square_matrix(const matrix_core& matrix)
	{
		if (matrix.ncol == matrix.nrow)
			return false;
		return true;
	}


	//Matrix Gauss-Jordan elimination AX=B  ⴹ���X��ͼ��Ѿ����return
	friend matrix_core gauss_jordan_elimination(const matrix_core& matrix_a, const matrix_core& matrix_b)
	{	//�� �� double �������������ա�ûѴ�ء����
		matrix_core temp = ( matrix_a, matrix_b );


		double pivot; //pivot �����ҪԤ��� i=j
		double c;
		for(size_t i=0; i<matrix_a.ncol; i++) //��� i  �繵����ҧ�ԧ
		{	//��Ǩ�ͺ��ҵ�ͧ��Ѻ�ǡѹ�������
			//��Ѻ��(row interchange) �����������ѭ�� / ���� 0
			for(size_t k=i+1; k<matrix_a.ncol; k++) //partial pivoting
			{   //���º��ºpivot�Ѩ�غѹ�Ѻ�����
				if( abs(temp(k,i)) > abs(temp(i,i)) )
					temp.swap_row(k,i);//��Ѻ����Ƿ����pivot�ç��ѡ���ʹ� �٧�ش
			}
			pivot = (double)(temp(i,i));//�纤��pivot
			if(pivot == 0) //�ѡ pivot �������� 0 �������������
				throw std::invalid_argument("pivot cannot be zero");
			//��ҷء��ǵ������ô��� pivot
			for(size_t j=0; j<temp.ncol; j++)
			{
				temp(i,j) /= pivot;  //�� pivot ��÷ء���� row ���
			}
			for(size_t m=0; m<matrix_a.nrow; m++)
			{	//�ӷء�Ƿ��������Ǣͧ�����ҧ�ԧ
				if(i != m)
				{
					c = temp(m,i);
					for(size_t n=0; n<temp.ncol;n++)
						temp(m,n) -= c*temp(i,n);
				}
			}
		}
		//��Ҥ��੾����ǹ������ѧ�ͧtemp �͡��
		int a_ncol = matrix_a.nrow;
		int b_nrow = matrix_b.ncol;
		int b_ncol = matrix_b.nrow;
		matrix_core result(b_nrow, b_ncol);
		for(int i=0; i<b_nrow; i++)
		{
			for(int j=0; j<b_ncol; j++)
			{
				result(i,j) = temp(i,j+a_ncol);
			}
		}
		return result;
	};

	//�����matrix U �� upper triangular matrix ��������
	friend inline bool is_lower_triangular(const matrix_core& U)
	{
		for(size_t i=0; i<U.nrow; i++)
		{
			for(size_t j=0; j<U.ncol; j++)
			{
				if(i < j)//��Ǩ�ͺ��ǹ upper
				{
					double element = (double)U(i,j);
					if( !is_closeto_zero(element) )
						return false;
				}
			}
		}
		return true;
	};

	//�����matrix L �� upper triangular matrix ��������
	friend inline bool is_upper_triangular(const matrix_core& L)
	{
		for(size_t i=0; i<L.nrow; i++)
		{
			for(size_t j=0; j<L.ncol; j++)
			{
				if(i > j)//��Ǩ�ͺ��ǹ lower
				{
					double m = (double)L(i,j);
					if( !is_closeto_zero(m) )
						return false;
				}
			}
		}
		return true;
	};


	//A Lower-Triangular matrix   ��������¹��������� AX = B
	//��X �ҡGaussian Elimination ����  forward substitution
	friend matrix_core forward_substitution(const matrix_core& A, const matrix_core& B)
	{
		if( !is_lower_triangular(A) )
		{
			throw std::invalid_argument("matrix A is not lower-triangular matrix");
		}
		if ( is_not_square_matrix(A) )
		{
			throw std::invalid_argument("matrix A must be square");
		}
		matrix_core X(B.nrow, B.ncol);
		double sum;
		for(size_t i=0; i<B.nrow; i++) //i ��row index �ͧX
		{   //��ǧ�ͺ��͹��ҵ��diagonal��0�������
			if( A(i,i) == 0.0)
			{
				throw std::invalid_argument("diagonal element is zero");
			}
			for(size_t j=0; j<B.ncol; j++)//j ��col index �ͧX)
			{   //�Ҥ�� x ��͹��Ѻ
				if( i==0 ) //�繵���á�ͧX ���� X[m-1] = B[m-1]/A[m-1,m-1]
				{
					X(i,j) = (double)(B(i,j))/(double)(A(i,i));
				}
				else
				{   //sum ��Ƿ�������͹˹��Xi �����Ҩ���
					sum = 0.0;
					for(size_t n=0; n<i; n++)
						sum += A(i,n)*X(n,j);
					X(i,j) = (double)((B(i,j) - sum))/(double)(A(i,i) );
				}
			}
		}

		return X;
	};

	//A ���Upper-Triangular matrix   ��������¹��������� AX = B
	//��X �ҡGaussian Elimination ����  back substitution
	friend matrix_core back_substitution(const matrix_core& A, const matrix_core& B)
	{
		if( !is_upper_triangular(A) )
		{
			throw std::invalid_argument("Matrix A is not upper-triangular matrix");
		}
		if ( is_not_square_matrix(A) )
		{
			throw std::invalid_argument("Matrix A must be square");
		}
		matrix_core X(B.nrow, B.ncol);
		double sum;
		for(size_t i=B.nrow-1; i>=0; i--) //i ��row pivot index �ͧX
		{   //��ǧ�ͺ��͹��ҵ��diagonal��0�������
			if((double)(A(i,i)) == 0.0)
			{
				throw std::invalid_argument("Diagonal element is zero");
			}
			for(size_t j=0; j<B.ncol; j++)//j ��col index �ͧX)
			{   //�Ҥ�� x ��͹��Ѻ
				if( i==B.ncol-1 ) //�繵���á�ͧX ���� X[m-1] = B[m-1]/A[m-1,m-1]
				{
					X(i,j) = (double)(B(i,j))/(double)(A(i,i));
				}
				else
				{   //sum ��Ƿ�������͹˹��Xi �����Ҩ���
					sum = 0.0;
					for(size_t n=A.nrow-1; n>i; n--)
						sum += A(i,n)*X(n,j);
					X(i,j) = (double)((B(i,j) - sum))/(double)(A(i,i));
				}
			}
		}

		return X;
	};

	//�¡Matrix A ����� LU �·��L = lower matrix U = upper matrix
	friend void lu_decomposition(const matrix_core& A, matrix_core& L, matrix_core &U,
		matrix_core& P)
	{
		assert( A.nrow == A.ncol );

		//��駤������������Ѻ L U
		L.create(A.nrow, A.ncol); //���ҧL����բ�Ҵ��ҡѺA
		L.set_all_to(0.0); //���L �� identity
		P.create(A.nrow, A.ncol); //���ҧPermutation matrix����բ�Ҵ��ҡѺA
		P.set_identity();
		matrix_core dA = (matrix_core)(A); //��A����繪�Դ double
		double m;
		for(size_t p=0; p<A.nrow-1; p++)//p ��� index �ͧ pivot equation
		{
			for(size_t k=p+1;k<A.nrow; k++) //��partial pivoting
			{   //���º��ºpivot�Ѩ�غѹ�Ѻ�����
				if( abs(dA(k,p)) > abs(dA(p,p)) )
				{
					dA.swap_row(k,p);//��Ѻ����Ƿ����pivot�ç��ѡ���ʹ� �٧�ش
					L.swap_row(k,p);
					P.swap_row(k,p);//�������permutation matrix
				}
			}
			if(dA(p,p) == 0.0)
				throw std::invalid_argument("pivot cannot be zero");
			L(p,p) = 1.0; //��������� diagonal element ����ѧ
			for(size_t i=p+1; i<A.nrow; i++)//i ���index �ͧrow����eliminate
			{
				m = dA(i,p)/dA(p,p);
				L(i,p) = m;
				for(size_t j=0; j<A.ncol; j++)
					dA(i,j) -= m*dA(p,j);
				U = dA;
			}
		}
		L(A.nrow-1,A.ncol-1) = 1.0; //���diagonal element ����ش����
	};


	//��determinant ����LU-decomposition
	friend double det(const matrix_core& A)
	{
		assert(A.nrow == A.ncol);

		int sign = 1; //'+' = true / '-' = FALSE
		matrix_core dA = (matrix_core)(A); //��A����繪�Դ double
		double m;
		for(size_t p=0; p<A.ncol-1; p++)//p ��� index �ͧ pivot equation
		{
			for(size_t k=p+1;k<A.ncol; k++) //��partial pivoting
			{   //���º��ºpivot�Ѩ�غѹ�Ѻ�����
				if( abs(dA(k,p)) > abs(dA(p,p)) )
				{
					dA.swap_row(k,p);//��Ѻ����Ƿ����pivot�ç��ѡ���ʹ� �٧�ش
					sign *= -1; //����ա����Ѻ�ǡѹ ����ͧ���¨�����¹
				}
			}
			if(dA(p,p) == 0.0) //�ѹ error 㹡�óշ����pivot ��0
				throw std::invalid_argument("pivot cannot be zero");
			for(size_t i=p+1; i<A.ncol; i++)//i ���index �ͧrow����eliminate
			{
				m = dA(i,p)/dA(p,p);
				for(size_t j=0; j<A.nrow; j++)
					dA(i,j) -= m*dA(p,j);
			}
		}
		//���determinat ��ͤ�ҼŤٳ�ͧdiagonal�ͧtriangular matrix
		double result = 1;
		for(size_t i=0; i<A.ncol; i++)
			result *= dA(i,i);

		return sign * result;
	};


};//class matrix_T














#endif //__MATRIX_H__





/* ������ҧ����

matrix_t<double> mat(3,3);
matrix_t<double> mat2(3,3);

mat(0,0) = 5; mat(0,1) = 7; mat(0,2) = 5;
mat(1,0) = 7; mat(1,1) = 3; mat(1,2) = 2;
mat(2,0) = 5; mat(2,1) = 5; mat(2,2) = 6;

mat2 = mat.inv();
mat2.debug();

std::cout << "det: " << det(mat2);
getchar();

*/
