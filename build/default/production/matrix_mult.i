# 1 "matrix_mult.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "C:/Program Files/Microchip/MPLABX/v5.50/packs/Microchip/PIC18F-K_DFP/1.4.87/xc8\\pic\\include\\language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "matrix_mult.c" 2
# 1 "./matrix_mult.h" 1
# 41 "./matrix_mult.h"
typedef struct MatrixStruct {
    unsigned char rows;
    unsigned char columns;
    float elements[4];
} Matrix;



void mat_init(unsigned char rows, unsigned char columns, Matrix * matrix);
void mat_set(unsigned char row, unsigned char column, float element, Matrix * matrix);
float mat_get(unsigned char row, unsigned char column, Matrix *);
void mat_fill_zero(Matrix *);
void mat_multiply(Matrix *matrixA, Matrix *matrixB, Matrix *matrixC);
void mat_transpose(Matrix* matrix, Matrix* result);
# 1 "matrix_mult.c" 2


void mat_init(unsigned char rows, unsigned char columns, Matrix* matrix) {

    matrix->rows = rows;
    matrix->columns = columns;
    mat_fill_zero(matrix);

    return;
}

void mat_set(unsigned char row, unsigned char column, float element, Matrix* matrix) {

    matrix->elements[(row - 1) * matrix->columns + (column - 1)] = element;

    return;
}

float mat_get(unsigned char row, unsigned char column, Matrix* matrix) {
    return matrix->elements[(row - 1) * matrix->columns + (column - 1)];
}

void mat_fill_zero(Matrix *matrix) {

    for (int i = 0; i < 4; i++) {
        matrix->elements[i] = 0;
    }

    return;
}

void mat_multiply(Matrix* matrixA, Matrix* matrixB, Matrix* matrixC) {





    mat_init(matrixA->rows, matrixB->columns, matrixC);
    mat_fill_zero(matrixC);

    for (int i = 1; i <= matrixC->rows; i++) {
        for (int j = 1; j <= matrixC->columns; j++) {
            for (int k = 1; k <= matrixA->columns; k++) {
                mat_set(i, j, mat_get(i, j, matrixC) + mat_get(i, k, matrixA) * mat_get(k, j, matrixB), matrixC);
            }
        }
    }

}



void mat_transpose(Matrix* matrix, Matrix* result) {




    mat_init(2, 2, result);

    float matrix_1_2 = mat_get(1, 2, matrix);
    mat_set(2, 1, matrix_1_2, result);

    float matrix_1_1 = mat_get(1, 1, matrix);
    mat_set(1, 1, matrix_1_1, result);

    float matrix_2_1 = mat_get(2, 1, matrix);
    mat_set(1, 2, matrix_2_1, result);

    float matrix_2_2 = mat_get(2, 2, matrix);
    mat_set(2, 2, matrix_2_2, result);
}
