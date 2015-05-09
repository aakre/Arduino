//
//  Matrix.h
//  KalmanFilter
//
//  Created by Øyvind Løberg Aakre on 19.02.15.
//  Based on MatrixMath from the Arduino forum
//

#ifndef __KalmanFilter__Matrix__
#define __KalmanFilter__Matrix__

#include "Arduino.h"

class Matrix {
    int rows;
    int cols;
    float *mat;
public:
    Matrix() {
        rows = 1;
        cols = 1;
        mat = new float[rows*cols];
        Zeros();
    };
    Matrix(float *m, int r, int c) {
        rows = r;
        cols = c;
        mat  = new float[rows*cols];
        Copy(m);
    };
    Matrix(int r, int c) {
        rows = r;
        cols = c;
        mat  = new float[rows*cols];
        Zeros();
    }
    void Zeros();
    int  Invert();
    void Copy(const Matrix m);
    void Copy(float *m);
    void Print(String label);
    void Eye();
    void Diag(float *d);
    Matrix Transpose();
    
    Matrix& operator+=(const Matrix &rhs);
    Matrix& operator-=(const Matrix &rhs);
    friend Matrix operator+(Matrix lhs, const Matrix &rhs);
    friend Matrix operator-(const Matrix lhs, const Matrix &rhs);
    friend Matrix operator*(const Matrix lhs, const Matrix rhs);
    friend Matrix operator/(const Matrix lhs, const Matrix rhs);
    friend Matrix operator/=(const Matrix lhs, const Matrix rhs);
};

#endif /* defined(__KalmanFilter__Matrix__) */
