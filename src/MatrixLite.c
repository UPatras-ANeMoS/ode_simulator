//
//  MatrixLite.c
//  CoreTypes
//
//  Created by Kostis Giannousakis on 06/02/16.
//  Copyright © 2016 Kostis Giannousakis. All rights reserved.
//

#include "MatrixLite.h"


CTResult mxSystemSolve(CTFloat *A, CTFloat *b, const CTUInteger n)
{
    CTInteger i, j, k;
    CTFloat tmp;
    for (k = 0; k < n; k++)
    {
        // Find the row where A[i][k] != 0
        for (i = k; i < n && !A[i*n + k]; i++);
        
        if (i == n)
            return CTError;
        else //if (j > i)
        { // Swap i with j
            for (j = 0; j < n; j++) {
                tmp = A[k*n + j];
                A[k*n + j] = A[i*n + j];
                A[i*n + j] = tmp;
            }
            
            tmp  = b[k];
            b[k] = b[i];
            b[i] = tmp;
        }
        
        // Normalize A[k][k]
        b[k] /= A[k*n + k];
        for (j = n-1; j >= k; j--)
            A[k*n + j] /= A[k*n + k];
        
        // Zero k column of below rows
        for (i = k+1; i < n; i++) {
            //            if (A[j*n + i]) {
            b[i] -= A[i*n + k] * b[k];
            for (j = n-1; j >= k; j--)
                A[i*n + j] -= A[i*n + k] * A[k*n + j];
            //            }
        }
    }
    
    for (k = n-1; k > 0; k--) {
        for (i = 0; i < k; i++) {
            if (A[i*n + k]) {
                b[i] -= A[i*n + k] * b[k];
                A[i*n + k] = 0; // A[i*n + k] -= A[i*n + k] * A[k*n + k]
            }
        }
    }
    
    return CTSuccess;
    
}