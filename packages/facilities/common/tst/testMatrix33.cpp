// Copyright 2020-2021 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Author: Lucas Catabriga
 * Creation: 2020-02-09
 *
 * Unit test for Matrix33 class
 */

#include "ext/matrix33.hpp"

#include <gtest/gtest.h>
#include <math.h>


TEST(TestMatrix33, testSum)
{
    Vector3D v1(1, 4, 7);
    Vector3D v2(2, 5, 8);
    Vector3D v3(3, 6, 9);
    Matrix33 mat1(v1, v2, v3);

    Matrix33 mat2 = mat1 + mat1;

    EXPECT_DOUBLE_EQ( 2, mat2.matrix[0][0]);
    EXPECT_DOUBLE_EQ( 4, mat2.matrix[0][1]);
    EXPECT_DOUBLE_EQ( 6, mat2.matrix[0][2]);
    EXPECT_DOUBLE_EQ( 8, mat2.matrix[1][0]);
    EXPECT_DOUBLE_EQ(10, mat2.matrix[1][1]);
    EXPECT_DOUBLE_EQ(12, mat2.matrix[1][2]);
    EXPECT_DOUBLE_EQ(14, mat2.matrix[2][0]);
    EXPECT_DOUBLE_EQ(16, mat2.matrix[2][1]);
    EXPECT_DOUBLE_EQ(18, mat2.matrix[2][2]);
}

TEST(TestMatrix33, testMatrixMult)
{
    Vector3D v1(1, 4, 7);
    Vector3D v2(2, 5, 8);
    Vector3D v3(3, 6, 9);
    Matrix33 mat1(v1, v2, v3);

    Matrix33 mat2 = mat1 * mat1;

    EXPECT_DOUBLE_EQ( 30, mat2.matrix[0][0]);
    EXPECT_DOUBLE_EQ( 36, mat2.matrix[0][1]);
    EXPECT_DOUBLE_EQ( 42, mat2.matrix[0][2]);
    EXPECT_DOUBLE_EQ( 66, mat2.matrix[1][0]);
    EXPECT_DOUBLE_EQ( 81, mat2.matrix[1][1]);
    EXPECT_DOUBLE_EQ( 96, mat2.matrix[1][2]);
    EXPECT_DOUBLE_EQ(102, mat2.matrix[2][0]);
    EXPECT_DOUBLE_EQ(126, mat2.matrix[2][1]);
    EXPECT_DOUBLE_EQ(150, mat2.matrix[2][2]);
}

TEST(TestMatrix33, testMatrixMultI)
{
    Vector3D v1(1, 4, 7);
    Vector3D v2(2, 5, 8);
    Vector3D v3(3, 6, 9);
    Matrix33 mat1(v1, v2, v3);

    Matrix33 mat2 = mat1 * Matrix33::I();

    EXPECT_DOUBLE_EQ( 1, mat2.matrix[0][0]);
    EXPECT_DOUBLE_EQ( 2, mat2.matrix[0][1]);
    EXPECT_DOUBLE_EQ( 3, mat2.matrix[0][2]);
    EXPECT_DOUBLE_EQ( 4, mat2.matrix[1][0]);
    EXPECT_DOUBLE_EQ( 5, mat2.matrix[1][1]);
    EXPECT_DOUBLE_EQ( 6, mat2.matrix[1][2]);
    EXPECT_DOUBLE_EQ( 7, mat2.matrix[2][0]);
    EXPECT_DOUBLE_EQ( 8, mat2.matrix[2][1]);
    EXPECT_DOUBLE_EQ( 9, mat2.matrix[2][2]);
}

TEST(TestMatrix33, testMatrixVectorMult)
{
    Vector3D v1(1, 4, 7);
    Vector3D v2(2, 5, 8);
    Vector3D v3(3, 6, 9);
    Matrix33 mat1(v1, v2, v3);

    Vector3D v4 = mat1 * Vector3D(-2, 0, 13);

    EXPECT_DOUBLE_EQ( 37, v4[0]);
    EXPECT_DOUBLE_EQ( 70, v4[1]);
    EXPECT_DOUBLE_EQ(103, v4[2]);
}

TEST(TestMatrix33, testVectorMatrixMult)
{
    Vector3D v1(1, 4, 7);
    Vector3D v2(2, 5, 8);
    Vector3D v3(3, 6, 9);
    Matrix33 mat1(v1, v2, v3);

    Vector3D v4 = mat1.pre_multiply(Vector3D(-2, 0, 13));

    EXPECT_DOUBLE_EQ( 89, v4[0]);
    EXPECT_DOUBLE_EQ(100, v4[1]);
    EXPECT_DOUBLE_EQ(111, v4[2]);
}

TEST(TestMatrix33, testDetZero)
{
    Vector3D v1(1, 4, 7);
    Vector3D v2(2, 5, 8);
    Vector3D v3(3, 6, 9);
    Matrix33 mat1(v1, v2, v3);
    
    double det = mat1.getDet();

    EXPECT_DOUBLE_EQ( 0, det);
}

TEST(TestMatrix33, testDetI)
{
    double det = Matrix33::I().getDet();

    EXPECT_DOUBLE_EQ( 1, det);
}

TEST(TestMatrix33, testDetNonZero)
{
    Vector3D v1(1, 1, 3);
    Vector3D v2(0, -2, 3);
    Vector3D v3(2, -4, 7);
    Matrix33 mat1(v1, v2, v3);
    double det = mat1.getDet();

    EXPECT_DOUBLE_EQ( 16, det);
}

TEST(TestMatrix33, testInv)
{
    Vector3D v1(1, 1, 3);
    Vector3D v2(0, -2, 3);
    Vector3D v3(2, -4, 7);
    Matrix33 mat1(v1, v2, v3);

    Matrix33 inv = mat1.getInverse();

    EXPECT_DOUBLE_EQ(-0.125, inv.matrix[0][0]);
    EXPECT_DOUBLE_EQ(0.375, inv.matrix[0][1]);
    EXPECT_DOUBLE_EQ(0.25, inv.matrix[0][2]);
    EXPECT_DOUBLE_EQ(-1.1875, inv.matrix[1][0]);
    EXPECT_DOUBLE_EQ(0.0625, inv.matrix[1][1]);
    EXPECT_DOUBLE_EQ(0.375, inv.matrix[1][2]);
    EXPECT_DOUBLE_EQ(0.5625, inv.matrix[2][0]);
    EXPECT_DOUBLE_EQ(-0.1875, inv.matrix[2][1]);
    EXPECT_DOUBLE_EQ(-0.125, inv.matrix[2][2]);
}

TEST(TestMatrix33, testInvMul)
{
    Vector3D v1(-1, 1, 3);
    Vector3D v2(3, -2, 5);
    Vector3D v3(8, -3, 7);
    Matrix33 mat1(v1, v2, v3);

    Matrix33 mul = mat1.getInverse() * mat1;

    EXPECT_NEAR(1, mul.matrix[0][0], 1e-12);
    EXPECT_NEAR(0, mul.matrix[0][1], 1e-12);
    EXPECT_NEAR(0, mul.matrix[0][2], 1e-12);
    EXPECT_NEAR(0, mul.matrix[1][0], 1e-12);
    EXPECT_NEAR(1, mul.matrix[1][1], 1e-12);
    EXPECT_NEAR(0, mul.matrix[1][2], 1e-12);
    EXPECT_NEAR(0, mul.matrix[2][0], 1e-12);
    EXPECT_NEAR(0, mul.matrix[2][1], 1e-12);
    EXPECT_NEAR(1, mul.matrix[2][2], 1e-12);
}

static bool isEigenValue(double eig, Vector3D eigenValues, double tolerance=1e-6)
{
    double valueFound = false;
    for(int i=0; i<3; i++)
    {
        if(abs(eig - eigenValues[i]) < tolerance)
        {
            valueFound = true;
        }
    }

    return valueFound;
}

TEST(TestMatrix33, testEingenValues)
{
    // Matrix must be symmetric
    Vector3D v1(1, 4, 7);
    Vector3D v2(4, 5, 8);
    Vector3D v3(7, 8, 9);
    Matrix33 mat1(v1, v2, v3);

    Vector3D eig = mat1.getEigenValues();

    EXPECT_TRUE(isEigenValue( 18.8302358, eig));
    EXPECT_TRUE(isEigenValue(-3.15746784, eig));
    EXPECT_TRUE(isEigenValue(-0.67276795, eig));
}

TEST(TestMatrix33, testEingenValuesI)
{
    // Matrix must be symmetric
    Vector3D v1(1, 0, 0);
    Vector3D v2(0, 1, 0);
    Vector3D v3(0, 0, 1);
    Matrix33 mat1(v1, v2, v3);

    Vector3D eig = mat1.getEigenValues();

    EXPECT_NEAR( 1, eig[0], 1e-6);
    EXPECT_NEAR( 1, eig[1], 1e-6);
    EXPECT_NEAR( 1, eig[2], 1e-6);
}

TEST(TestMatrix33, testEingenVectors)
{
    // Matrix must be symmetric
    Vector3D v1(1, 4, 7);
    Vector3D v2(4, 5, 8);
    Vector3D v3(7, 8, 9);
    Matrix33 mat1(v1, v2, v3);

    Vector3D eigvals = mat1.getEigenValues();
    Matrix33 eigvecs = mat1.getEigenVectors();

    for(int i=0; i<3; i++)
    {
        Vector3D eigvec(eigvecs.matrix[0][i], eigvecs.matrix[1][i], eigvecs.matrix[2][i]);

        Vector3D matrixmulvec = mat1 * eigvec;
        Vector3D eigvalmulvec = eigvec * eigvals[i];

        EXPECT_NEAR( matrixmulvec[0], eigvalmulvec[0], 1e-4);
        EXPECT_NEAR( matrixmulvec[1], eigvalmulvec[1], 1e-4);
        EXPECT_NEAR( matrixmulvec[2], eigvalmulvec[2], 1e-4);
    }
}

TEST(TestMatrix33, testEingenVectors2)
{
    // Matrix must be symmetric
    Vector3D v1(1, 0, 0);
    Vector3D v2(0, -2, 0);
    Vector3D v3(0, 0, 2);
    Matrix33 mat1(v1, v2, v3);

    Vector3D eigvals = mat1.getEigenValues();
    Matrix33 eigvecs = mat1.getEigenVectors();

    for(int i=0; i<3; i++)
    {
        Vector3D eigvec(eigvecs.matrix[0][i], eigvecs.matrix[1][i], eigvecs.matrix[2][i]);

        Vector3D matrixmulvec = mat1 * eigvec;
        Vector3D eigvalmulvec = eigvec * eigvals[i];

        EXPECT_NEAR( matrixmulvec[0], eigvalmulvec[0], 1e-4);
        EXPECT_NEAR( matrixmulvec[1], eigvalmulvec[1], 1e-4);
        EXPECT_NEAR( matrixmulvec[2], eigvalmulvec[2], 1e-4);
    }
}

TEST(TestMatrix33, testEingenVectors3)
{
    // Matrix must be symmetric
    Vector3D v1(1, 0, 0);
    Vector3D v2(0, 0, -2);
    Vector3D v3(0, -2, 0);
    Matrix33 mat1(v1, v2, v3);

    Vector3D eigvals = mat1.getEigenValues();
    Matrix33 eigvecs = mat1.getEigenVectors();

    for(int i=0; i<3; i++)
    {
        Vector3D eigvec(eigvecs.matrix[0][i], eigvecs.matrix[1][i], eigvecs.matrix[2][i]);

        Vector3D matrixmulvec = mat1 * eigvec;
        Vector3D eigvalmulvec = eigvec * eigvals[i];

        EXPECT_NEAR( matrixmulvec[0], eigvalmulvec[0], 1e-4);
        EXPECT_NEAR( matrixmulvec[1], eigvalmulvec[1], 1e-4);
        EXPECT_NEAR( matrixmulvec[2], eigvalmulvec[2], 1e-4);
    }
}

TEST(TestMatrix33, testEingenVectors4)
{
    // Matrix must be symmetric
    Vector3D v1(0, 1, 0);
    Vector3D v2(1, 0, 0);
    Vector3D v3(0, 0, 2);
    Matrix33 mat1(v1, v2, v3);

    Vector3D eigvals = mat1.getEigenValues();
    Matrix33 eigvecs = mat1.getEigenVectors();

    for(int i=0; i<3; i++)
    {
        Vector3D eigvec(eigvecs.matrix[0][i], eigvecs.matrix[1][i], eigvecs.matrix[2][i]);

        Vector3D matrixmulvec = mat1 * eigvec;
        Vector3D eigvalmulvec = eigvec * eigvals[i];

        EXPECT_NEAR( matrixmulvec[0], eigvalmulvec[0], 1e-4);
        EXPECT_NEAR( matrixmulvec[1], eigvalmulvec[1], 1e-4);
        EXPECT_NEAR( matrixmulvec[2], eigvalmulvec[2], 1e-4);
    }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
