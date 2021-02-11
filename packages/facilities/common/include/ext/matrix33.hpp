// Copyright 2020 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Author: lucas catabriga
 * Creation: 2020-02-08
 *
 * Utility class: 3x3 Matrix
 */

#ifndef MATRIX33_HPP
#define MATRIX33_HPP

#include "vector3d.hpp"
#include "matrix22.hpp"

#include "opencv2/opencv.hpp"

class Matrix33
{
public:
    Matrix33()
    {
        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
            {
                matrix[i][j] = 0;
            }
        }
    }

    Matrix33(Vector3D vec1, Vector3D vec2, Vector3D vec3)
    {
        matrix[0][0] = vec1.x;
        matrix[0][1] = vec2.x;
        matrix[0][2] = vec3.x;
        matrix[1][0] = vec1.y;
        matrix[1][1] = vec2.y;
        matrix[1][2] = vec3.y;
        matrix[2][0] = vec1.z;
        matrix[2][1] = vec2.z;
        matrix[2][2] = vec3.z;
    }

    static Matrix33 I()
    {
        Matrix33 ident;
        ident.matrix[0][0] = 1.0;
        ident.matrix[1][1] = 1.0;
        ident.matrix[2][2] = 1.0;
        return ident;
    }

    Matrix33& operator *=(double t)
    {
        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
            {
                matrix[i][j] *= t;
            }
        }

        return (*this);
    }

    Matrix33 operator *(double t)
    {
        Matrix33 m;

        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
            {
                m.matrix[i][j] = matrix[i][j] * t;
            }
        }

        return m;
    }

    Matrix33 operator *(const Matrix33& m)
    {
        Matrix33 newM;

        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
            {
                newM.matrix[i][j] = 0;

                for(int k=0; k<3; k++)
                {
                    newM.matrix[i][j] += matrix[i][k] * m.matrix[k][j];
                }
            }
        }

        return newM;
    }

    Vector3D operator *(const Vector3D& m)
    {
        double x = matrix[0][0]*m.x + matrix[0][1]*m.y + matrix[0][2]*m.z;
        double y = matrix[1][0]*m.x + matrix[1][1]*m.y + matrix[1][2]*m.z;
        double z = matrix[2][0]*m.x + matrix[2][1]*m.y + matrix[2][2]*m.z;

        return Vector3D(x,y,z);
    }

    Vector3D pre_multiply(const Vector3D& m)
    {
        double x = m.x*matrix[0][0] + m.y*matrix[1][0] + matrix[2][0]*m.z;
        double y = m.x*matrix[0][1] + m.y*matrix[1][1] + matrix[2][1]*m.z;
        double z = m.x*matrix[0][2] + m.y*matrix[1][2] + matrix[2][2]*m.z;

        return Vector3D(x,y,z);
    }

    Matrix33 operator +(const Matrix33& m) const
    {
        Matrix33 newM;

        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
            {
                newM.matrix[i][j] = matrix[i][j] + m.matrix[i][j];
            }
        }

        return newM;
    }

    Matrix33 operator -(const Matrix33& m) const
    {
        Matrix33 newM;

        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
            {
                newM.matrix[i][j] = matrix[i][j] - m.matrix[i][j];
            }
        }

        return newM;
    }

    double getDet() const
    {
        double a = matrix[0][0]*(matrix[1][1]*matrix[2][2] - matrix[1][2]*matrix[2][1]);
        double b = matrix[0][1]*(matrix[1][0]*matrix[2][2] - matrix[1][2]*matrix[2][0]);
        double c = matrix[0][2]*(matrix[1][0]*matrix[2][1] - matrix[1][1]*matrix[2][0]);

        return (a - b + c);
    }

    Matrix33 getInverse() const
    {
        double a = matrix[0][0];
        double b = matrix[0][1];
        double c = matrix[0][2];
        double d = matrix[1][0];
        double e = matrix[1][1];
        double f = matrix[1][2];
        double g = matrix[2][0];
        double h = matrix[2][1];
        double i = matrix[2][2];

        Matrix33 inv;

        inv.matrix[0][0] = e*i - f*h;
        inv.matrix[0][1] = c*h - b*i;
        inv.matrix[0][2] = b*f - c*e;
        inv.matrix[1][0] = f*g - d*i;
        inv.matrix[1][1] = a*i - c*g;
        inv.matrix[1][2] = c*d - a*f;
        inv.matrix[2][0] = d*h - e*g;
        inv.matrix[2][1] = b*g - a*h;
        inv.matrix[2][2] = a*e - b*d;

        double det = getDet();

        inv = inv * (1.0/det);

        return inv;
    }

    Matrix33 getTranspose() const
    {
        Matrix33 T;

        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
            {
                T.matrix[i][j] = matrix[j][i];
            }
        }

        return T;
    }

    Vector3D getEigenValues() const
    {        
        cv::Mat M(3,3, CV_64FC1, (double*)matrix);
        cv::Mat vals;
        cv::Mat vecs;
        cv::eigen(M, vals, vecs);

        Vector3D eigenVals;

        for(int i=0; i<3; i++)
        {
            eigenVals[i] = vals.at<double>(i);
        }
     
        return eigenVals;
    }

    Matrix33 getEigenVectors() const
    {
        cv::Mat M(3,3, CV_64FC1, (double*)matrix);
        cv::Mat vals;
        cv::Mat vecs;
        cv::eigen(M, vals, vecs);

        Matrix33 eigenVectors;

        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
            {
                eigenVectors.matrix[i][j] = vecs.at<double>(j,i);
            }            
        }
     
     
        return eigenVectors;
    }

public:
    double matrix[3][3];


};

#endif