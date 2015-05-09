//
//  matrix.cpp
//  KalmanFilter
//
//  Created by Øyvind Løberg Aakre on 19.02.15.
//  Based on MatrixMath from the Arduino forum
//

#include "matrix.h"
#include <math.h>


void matrix::Zeros() {
    int i,j;
    for (i=0; i<rows; i++) {
        for (j=0; j<cols;j++) {
            mat[cols*i+j] = 0;
        }
    }
}

void matrix::Copy(float *m) {
    int i,j;
    for (i=0; i<rows; i++) {
        for (j=0; j<cols;j++) {
            mat[cols*i+j] = m[cols*i+j];
        }
    }
}

void matrix::Copy(const matrix m){
    /* Something buggy here... */
    rows = m.rows;
    cols = m.cols;
    this->Copy((float*)m.mat);
};

void matrix::Eye() {
    Zeros();
    for (int i=0; i<rows; i++) {
        mat[rows*i+i] = 1;
    }
};

void matrix::Diag(float *d) {
    for (int i=0; i<cols; i++) {
        mat[cols*i+i] = d[i];
    }
};

void matrix::Print(String label) {
    int i,j;
    Serial.println();
    Serial.println(label);
    for (i=0; i<rows; i++) {
        for (j=0; j<cols; j++) {
            Serial.print(mat[cols*i+j], 5); // 5 digits
            Serial.print("\t");
        }
         Serial.println();
    }
    Serial.println();
};

matrix matrix::Transpose() {
    matrix res(cols, rows);
    int i, j;
    for (i=0;i<rows;i++)
        for(j=0;j<cols;j++)
            res.mat[rows*j+i]=mat[cols*i+j];
    return res;
}

int matrix::Invert(){ // Inverts in-place!
    if (rows!=cols) {
        return 0;
    }
    int pivrow = 0;		// keeps track of current pivot row
    int k,i,j;		// k: overall index along diagonal; i: row index; j: col index
    int pivrows[rows]; // keeps track of rows swaps to undo at end
    float tmp;		// used for finding max value and making column swaps
    
    for (k = 0; k < rows; k++)
    {
        // find pivot row, the row with biggest entry in current column
        tmp = 0;
        for (i = k; i < rows; i++)
        {
            if (abs(mat[i*rows+k]) >= tmp)	// 'Avoid using other functions inside abs()?'
            {
                tmp = abs(mat[i*rows+k]);
                pivrow = i;
            }
        }
        
        // check for singular matrix
        if (mat[pivrow*rows+k] == 0.0f)
        {
            Serial.println("Inversion failed due to singular matrix");
            return 0;
            }
            
            // Execute pivot (row swap) if needed
            if (pivrow != k)
            {
                // swap row k with pivrow
                for (j = 0; j < rows; j++)
                {
                    tmp = mat[k*rows+j];
                    mat[k*rows+j] = mat[pivrow*rows+j];
                    mat[pivrow*rows+j] = tmp;
                }
            }
            pivrows[k] = pivrow;	// record row swap (even if no swap happened)
            
            tmp = 1.0f/mat[k*rows+k];	// invert pivot element
            mat[k*rows+k] = 1.0f;		// This element of input matrix becomes result matrix
            
            // Perform row reduction (divide every element by pivot)
            for (j = 0; j < rows; j++)
            {
                mat[k*rows+j] = mat[k*rows+j]*tmp;
            }
            
            // Now eliminate all other entries in this column
            for (i = 0; i < rows; i++)
            {
                if (i != k)
                {
                    tmp = mat[i*rows+k];
                    mat[i*rows+k] = 0.0f;  // The other place where in matrix becomes result mat
                    for (j = 0; j < rows; j++)
                    {
                        mat[i*rows+j] = mat[i*rows+j] - mat[k*rows+j]*tmp;
                    }
                }
            }
            }
            
            // Done, now need to undo pivot row swaps by doing column swaps in reverse order
            for (k = rows-1; k >= 0; k--)
            {
                if (pivrows[k] != k)
                {
                    for (i = 0; i < rows; i++)
                    {
                        tmp = mat[i*rows+k];
                        mat[i*rows+k] = mat[i*rows+pivrows[k]];
                        mat[i*rows+pivrows[k]] = tmp;
                    }
                }
            }
            return 1;
    
};


// Operator overloading
matrix& matrix::operator+=(const matrix &rhs) {
    int i,j;
    for (i=0; i<rows; i++) {
        for (j=0; j<cols;j++) {
            mat[cols*i+j] += rhs.mat[cols*i+j];
        }
    }
    return *this;
};

matrix operator+(matrix lhs, const matrix &rhs) {
    matrix res(lhs.rows, lhs.cols);
    int i,j;
    for (i=0; i<lhs.rows; i++) {
        for (j=0; j<lhs.cols;j++) {
            res.mat[lhs.cols*i+j] = lhs.mat[lhs.cols*i+j] + rhs.mat[lhs.cols*i+j];
        }
    }
    return res;
};

matrix& matrix::operator-=(const matrix &rhs) {
    int i,j;
    for (i=0; i<rows; i++) {
        for (j=0; j<cols;j++) {
            mat[cols*i+j] -= rhs.mat[cols*i+j];
        }
    }
    return *this;
};

matrix operator-(matrix lhs, const matrix &rhs) {
    matrix res(lhs.rows, lhs.cols);
    int i,j;
    for (i=0; i<lhs.rows; i++) {
        for (j=0; j<lhs.cols;j++) {
            res.mat[lhs.cols*i+j] = lhs.mat[lhs.cols*i+j] - rhs.mat[lhs.cols*i+j];
        }
    }
    return res;
};

matrix operator*(const matrix lhs, const matrix rhs) {
    // lhs = matrix (m x p)
    // rhs = matrix (p x n)
    // m = number of rows in lhs
    // p = number of columns in lhs = number of rows in rhs
    // n = number of columns in rhs
    // res = output matrix = lhs*rhs (m x n)
    int i, j, k;
    matrix res(lhs.rows, rhs.cols);
    for (i=0;i<res.rows;i++) {
        for(j=0;j<res.cols;j++) {
            res.mat[res.cols*i+j]=0;
            for (k=0;k<lhs.cols;k++) {
                res.mat[res.cols*i+j] += lhs.mat[lhs.cols*i+k] * rhs.mat[rhs.cols*k+j];
            }
        }
    }
    return res;
}

matrix operator/(const matrix lhs, const matrix rhs) {
    // Matlab equivalent: lhs * inv(rhs) or lhs / rhs
    matrix tmp(rhs.rows, rhs.cols);
    tmp.Copy(rhs.mat);
    int r = tmp.Invert();
    if (r==0) {
        return lhs;
    } else {
        matrix res(lhs.rows, lhs.cols);
        return lhs*tmp;
    }
};

matrix operator/=(const matrix lhs, const matrix rhs) {
    // Matlab equivalent: inv(lhs) * rhs or lhs \ rhs;
    matrix tmp(lhs.rows, lhs.cols);
    tmp.Copy((float*)lhs.mat);
    int r = tmp.Invert();
    if (r==0) {
        return rhs;
    } else {
        matrix res(rhs.rows, rhs.cols);
        return tmp*rhs;
    }
};


