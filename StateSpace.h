#pragma once

#include <string>
#include <vector>
#include "Block.h"

namespace DiscreteTime
{
	/*
	A must be an N-by-N matrix, where n is the number of states. -> State transition matrix 
	B must be an N-by-M matrix, where m is the number of inputs.
	C must be an R-by-N matrix, where r is the number of outputs.
	D must be an R-by-M matrix, where r is the number of outputs
	*/

	template< std::uint32_t N, std::uint32_t M, std::uint32_t R>
	class StateSpace final : public Block
	{
	public:
		
		using AMatrix = std::double_t[N][N];
		using BMatrix = std::double_t[N][M];
		using CMatrix = std::double_t[R][N];
		using DMatrix = std::double_t[R][M];
		
		StateSpace() {};

		~StateSpace() {};

		void setParameters(const AMatrix& A, const BMatrix& B, const CMatrix& C, const DMatrix& D, const std::string& name)
		{
			if (false == _isParamsSet)
			{
				memcpy(_A, A, sizeof(double_t) *N*N);
				memcpy(_B, B, sizeof(double_t) *N*M);
				memcpy(_C, C, sizeof(double_t) *R*N);
				memcpy(_D, D, sizeof(double_t) *R*M);
				for (size_t i = 0; i < N; i++)
				{
					_X[i] = 0.00;
					_oldX[i] = 0.00;
				}
				for (size_t i = 0; i < M; i++)
				{
					_Y[i] = 0.00;
				}
				setName(name);
				_isParamsSet = true;
			}
		}

		void reset()
		{
			if (_isParamsSet)
			{
				for (size_t i = 0; i < N; i++)
				{
					_X[i] = 0.00;
				}
				for (size_t i = 0; i < M; i++)
				{
					_Y[i] = 0.00;
				}
			}
		}
				
		void process(const std::vector<std::double_t>& u)
		{
			//_X =  A * _oldX + B * u
			//_Y = C * _X + D * u;
			//_oldX = _X;
			std::double_t temp1[N];
			std::double_t res = 0.00;
			for (std::uint32_t n = 0U; n < N; n++)
			{
				res = 0.00;
				for (std::uint32_t j = 0U; j < N; j++)
				{
					res += _A[n][j] * _oldX[j];
				}
				temp1[n] = res;
			}

			res = 0.00;
			for (std::uint32_t n = 0U; n < N; n++)
			{
				res = 0.00;
				for (std::uint32_t j = 0U; j < M; j++)
				{
					res += _B[n][j] * u[j];
				}
				_X[n] = res + temp1[n];
				_oldX[n] = _X[n];
			}

			res = 0.00;
			std::double_t temp2[R];
			for (std::uint32_t r = 0U; r < R; r++)
			{
				res = 0.00;
				for (std::uint32_t n = 0U; n < N; n++)
				{
					res += _C[r][n] * _X[n];
				}
				temp2[r] = res;
			}

			res = 0.00;
			for (std::uint32_t r = 0U; r < R; r++)
			{
				res = 0.00;
				for (std::uint32_t m = 0U; m < M; m++)
				{
					res += _D[r][m] * u[m];
				}
				_Y[r] = res + temp2[r];
			}
		}
		
		const std::double_t getY( std::uint32_t i) const
		{
			if (i < N)
			{
				return _Y[i];
			}
			return 0.00;
		}

	private:

		std::double_t _A[N][N];
		std::double_t _B[N][M];
		std::double_t _C[R][N];
		std::double_t _D[R][M];
				
		std::double_t _X[N];
		std::double_t _oldX[N];
		std::double_t _Y[M];

		bool _isParamsSet = false;
	};
}



