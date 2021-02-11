// Copyright 2019 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Author: lucas catabriga
 * Creation: 2019-09-12
 *
 * Utility class: 2x2 Matrix
 */

#ifndef MATRIX22_HPP
#define MATRIX22_HPP

#include "vector2d.hpp"

class Matrix22
{
public:
    Matrix22()
    {
        matrix[0][0] = 0;
        matrix[0][1] = 0;
        matrix[1][0] = 0;
        matrix[1][1] = 0;
    }

    Matrix22(double a, double b, double c, double d)
    {
        matrix[0][0] = a;
        matrix[0][1] = b;
        matrix[1][0] = c;
        matrix[1][1] = d;
    }

    Matrix22(Vector2D vec1, Vector2D vec2)
    {
        matrix[0][0] = vec1.x;
        matrix[0][1] = vec2.x;
        matrix[1][0] = vec1.y;
        matrix[1][1] = vec2.y;
    }

    Matrix22& operator *=(double t)
    {
        matrix[0][0] *= t;
        matrix[0][1] *= t;
        matrix[1][0] *= t;
        matrix[1][1] *= t;
        return (*this);
    }

    Matrix22 operator *(double t)
    {
        Matrix22 m;
        m.matrix[0][0] = matrix[0][0] * t;
        m.matrix[0][1] = matrix[0][1] * t;
        m.matrix[1][0] = matrix[1][0] * t;
        m.matrix[1][1] = matrix[1][1] * t;
        return m;
    }

    Matrix22 operator *(const Matrix22& m)
    {
        Matrix22 newM;

        for(int i=0; i<2; i++)
        {
            for(int j=0; j<2; j++)
            {
                newM.matrix[i][j] = 0;

                for(int k=0; k<2; k++)
                {
                    newM.matrix[i][j] += matrix[i][k] * m.matrix[k][j];
                }
            }
        }

        return newM;
    }

    Vector2D operator *(const Vector2D& m)
    {
        double x = matrix[0][0]*m.x + matrix[0][1]*m.y;
        double y = matrix[1][0]*m.x + matrix[1][1]*m.y;

        return Vector2D(x,y);
    }

    Vector2D pre_multiply(const Vector2D& m)
    {
        double x = m.x*matrix[0][0] + m.y*matrix[1][0];
        double y = m.x*matrix[0][1] + m.y*matrix[1][1];

        return Vector2D(x,y);
    }

    Matrix22 operator +(const Matrix22& m)
    {
        Matrix22 newM;

        for(int i=0; i<2; i++)
        {
            for(int j=0; j<2; j++)
            {
                newM.matrix[i][j] = matrix[i][j] + m.matrix[i][j];
            }
        }

        return newM;
    }

    double getDet() const
    {
        return matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0];
    }

    Matrix22 getInverse() const
    {
        double det = getDet();
        Matrix22 inverse(matrix[1][1], -matrix[0][1], -matrix[1][0], matrix[0][0]);
        inverse *= (1.0/det);
        return inverse;
    }

    Vector2D getEigenValues() const
    {
        double T = matrix[0][0] + matrix[1][1];
        double det = getDet();

        double D = sqrt(T*T/4.0 - det);

        double L1 = T/2.0 + D;
        double L2 = T/2.0 - D;

        return Vector2D(L1, L2);
    }

    // Returns each eingen vector on a column in the matrix
    Matrix22 getEigenVectors() const
    {
    	Vector2D eingenValues = getEigenValues();

    	return getEigenVectors(eingenValues);
    }

    Matrix22 getEigenVectors(Vector2D eingenValues) const
    {
    	// check http://www.math.harvard.edu/archive/21b_fall_04/exhibits/2dmatrices/ for algorithm

    	double L1 = eingenValues[0];
    	double L2 = eingenValues[1];

    	double a = matrix[0][0];
    	double b = matrix[0][1];
    	double c = matrix[1][0];
    	double d = matrix[1][1];

    	Vector2D eingen_vector_1;
    	Vector2D eingen_vector_2;

    	// Avoids numerical instability close to zero
    	if(std::abs(b) > std::abs(c))
    	{	
    		eingen_vector_1 = Vector2D(b, L1-a);
    		eingen_vector_2 = Vector2D(b, L2-a);
    	}
    	else
    	{
    		eingen_vector_1 = Vector2D(L1-d, c);
    		eingen_vector_2 = Vector2D(L2-d, c);
    	}

    	return Matrix22(eingen_vector_1, eingen_vector_2);
    }

public:
    double matrix[2][2];
};

#endif