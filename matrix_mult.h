#ifndef MATRIX_MULT
#define MATRIX_MULT  


// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation


/*
 * Partially taken from: https://github.com/webDva/matrixmul/blob/master/matrixmul.h
 */


// Constant(s)

#define MAX_ELEMENTS 4

// Type(s)

typedef struct MatrixStruct {
    unsigned char rows;
    unsigned char columns;
    float elements[MAX_ELEMENTS];
} Matrix;

// Function declarations

void mat_init(unsigned char rows, unsigned char columns, Matrix * matrix);
void mat_set(unsigned char row, unsigned char column, float element, Matrix * matrix);
float mat_get(unsigned char row, unsigned char column, Matrix *);
void mat_fill_zero(Matrix *);
void mat_multiply(Matrix *matrixA, Matrix *matrixB, Matrix *matrixC);
void mat_transpose(Matrix* matrix, Matrix* result); // Function to transpose a 2x2 matrix
                                                //                         ^^^

#endif 

