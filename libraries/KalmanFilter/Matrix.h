//
//  Matrix.h
//  KalmanFilter
//
//  Created by Øyvind Løberg Aakre on 19.02.15.
//  Based on MatrixMath from the Arduino forum
//

#ifndef __Matrix__Matrix__
#define __Matrix__Matrix__

#include "Arduino.h"

class matrix {
    int rows;
    int cols;
    float *mat;
public:
    matrix() {
        rows = 1;
        cols = 1;
        mat = new float[rows*cols];
        Zeros();
    };
    matrix(float *m, int r, int c) {
        rows = r;
        cols = c;
        mat  = new float[rows*cols];
        Copy(m);
    };
    matrix(int r, int c) {
        rows = r;
        cols = c;
        mat  = new float[rows*cols];
        Zeros();
    }
    void Zeros();
    int  Invert();
    void Copy(const matrix m);
    void Copy(float *m);
    void Print(String label);
    void Eye();
    void Diag(float *d);
    matrix Transpose();
    
    matrix& operator+=(const matrix &rhs);
    matrix& operator-=(const matrix &rhs);
    friend matrix operator+(matrix lhs, const matrix &rhs);
    friend matrix operator-(const matrix lhs, const matrix &rhs);
    friend matrix operator*(const matrix lhs, const matrix rhs);
    friend matrix operator/(const matrix lhs, const matrix rhs);
    friend matrix operator/=(const matrix lhs, const matrix rhs);
};

#endif /* defined(__matrix__matrix__) */
