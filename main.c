/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC18F26K83
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"

/*
                         Main application
 */
/*
void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();
    
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts
    // Use the following macros to:

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();
            
    ADCC_DischargeSampleCapacitor();

    while (1)
    {
        // Add your application code

        printf("Analog 1: %d \n\r", ADCC_GetSingleConversion(channel_ANB1));
        printf("Analog 2: %d \n\r\n\r", ADCC_GetSingleConversion(channel_ANB2));
        
        __delay_ms(1000);
    }
}
*/
/**
 End of File
*/


// This program estimates a battery system's SOC (state of charge) with respect to time
// based off simulated values for the circuit's V (voltage) and I (current).
// 
// Created by:
//  - Ayush Saha, 3/20/2022 (Rev 1)
//  - Melody Liu (Rev 2)

#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include<stdbool.h>

#include "matrix_mult.h"
#include "frame.h"

#define RX_BUFFER_LENGTH 128

//#pragma warning (disable:4996)


// Function declarations
float VOC(float SOC, float Voc0);
float hk(float SOC_val, float I, float Voc0, float R0);
void fk(Matrix* xhatk_1, float I, float dt, float Cbat, float Cc, float Rc, Matrix* fk_);
void EKF(Matrix* xhatk_1, Matrix* Pk_1, float I, float Ik_1, float V, float Voc0,
	float Rk, Matrix* Aprime, Matrix* Cprime, Matrix* Eprime, Matrix* Fprime,
	Matrix* fk_, float dt, float Cbat, float Cc, float Rc, Matrix* Qk1,
	Matrix* Aprime_transpose, Matrix* Eprime_transpose,
	Matrix* sub_mat_1, Matrix* sub_mat_2, Matrix* sub_mat_3, Matrix* xhat,
	Matrix* P, Matrix* Cprime_transpose, Matrix* Lk, float R0, 
	Matrix* xhatCorrected, Matrix* PCorrected);
 

//------------------------------- SMALLER CALCULATION FUNCTIONS --------------------------------

// Functions we need to calculate
float VOC(float SOC, float Voc0) {
	return 0.007 * SOC + Voc0;
}

float hk(float SOC_val, float I, float Voc0, float R0) {
	return VOC(SOC_val, Voc0) - (R0 * I);		// hk_ is calculated voltage?
}

void fk(Matrix* xhatk_1, float I, float dt, float Cbat, float Cc, float Rc, Matrix* fk_) {
    float array_mat[2][1] = { {(-1) * I / Cbat}, {(I / Cc) - (mat_get(2, 1, xhatk_1) / (Cc * Rc))} };
   
    array_mat[0][0] *= dt;
    mat_set(1, 1, mat_get(1, 1, xhatk_1) + array_mat[0][0], fk_);
    array_mat[1][0] *= dt;
    mat_set(2, 1, mat_get(2, 1, xhatk_1) + array_mat[1][0], fk_);
    
    
	// xhat = previous xhat + change in xhat in time (dt)
	// xhat is a derivative
}


//------------------------------------ EKF CALCULATOR FUNCTION -------------------------------------

// EKF Calculator function (to compute values of xhatCorrected and PCorrected continuously)
void EKF(Matrix* xhatk_1, Matrix* Pk_1, float I, float Ik_1, float V, float Voc0,
	float Rk, Matrix* Aprime, Matrix* Cprime, Matrix* Eprime, Matrix* Fprime,
	Matrix* fk_, float dt, float Cbat, float Cc, float Rc, Matrix* Qk1,
	Matrix* Aprime_transpose, Matrix* Eprime_transpose,
	Matrix* sub_mat_1, Matrix* sub_mat_2, Matrix* sub_mat_3, Matrix* xhat,
	Matrix* P, Matrix* Cprime_transpose, Matrix* Lk, float R0, 
	Matrix* xhatCorrected, Matrix* PCorrected) {

	uint8_t row, col, index; // used for loops

	//----------------- CALCULATIONS FOR xhat -----------------
	fk(xhatk_1, I, dt, Cbat, Cc, Rc, fk_);
    for (row = 1; row <= 2; row++) {
        mat_set(row, 1, mat_get(row, 1, fk_), xhat);
    }

	//----------------- CALCULATIONS FOR P -----------------
	// Multiplying Aprime * Pk_1 * Aprime_transpose
	mat_multiply(Aprime, Pk_1, sub_mat_1);
	mat_multiply(sub_mat_1, Aprime_transpose, sub_mat_2);

	// Multiplying Eprime * Qk1 * Eprime_transpose
	mat_multiply(Eprime, Qk1, sub_mat_1);
	mat_multiply(sub_mat_1, Eprime_transpose, sub_mat_3);

	// P = (Aprime * Pk_1 * Aprime_transpose) + (Eprime * Qk1 * Eprime_transpose)
    for (row = 1; row <= 2; row++) {
		for (col = 1; col <= 2; col++) {
            mat_set(row, col, mat_get(row, col, sub_mat_2) + mat_get(row, col, sub_mat_3), P);
		}
	}

	//----------------- CALCULATIONS FOR Lk -----------------
    // error_estimate = (Cprime * P * Cprime_tranpose)
	mat_multiply(P, Cprime_transpose, sub_mat_1);
	float error_estimate = 0.0, combined_error = 0.0;
    for (row = 1; row <= 2; row++) {
        error_estimate += mat_get(row, 1, sub_mat_1) * mat_get(row, 1, Cprime);
	}
	// combined_error = [(Cprime * P * Cprime_tranpose) + Rk]^(-1)
	combined_error = error_estimate + Rk;
	combined_error = pow(combined_error, (-1));

	// Lk = combined_error * (P * Cprime_transpose)
    for (row = 1; row <= 2; row++) {
		mat_set(row, 1, mat_get(row, 1, sub_mat_1) * combined_error, Lk);
	}

	//--------------- CALCULATIONS FOR xhatCorrected ---------------
	// temp_val = (SOC + Vc) - hk(100*SOC, I, Voc0, R0)
	float temp_val = 0.0;
    temp_val = (V + mat_get(2, 1, xhat)) - hk(100 * mat_get(1, 1, xhat), I, Voc0, R0);
	
	// xhatCorrected = (Lk * temp_val) * xhat
	for (row = 1; row <= 2; row++) {
		mat_set(row, 1, mat_get(row, 1, Lk) * temp_val, sub_mat_1);
		mat_set(row, 1, mat_get(row, 1, xhat) + mat_get(row, 1, sub_mat_1), xhatCorrected);
	}  

	//----------------- CALCULATIONS FOR PCorrected -----------------
	// FINDING: Lk * Cprime
    for (row = 1; row <= 2; row++) {
		for (index = 1; index <= 2; index++) {
			mat_set(row, index, mat_get(row, 1, Lk) * mat_get(index, 1, Cprime), sub_mat_1); // sussy, check later
		}
	}
	// FINDING: (Lk * Cprime) * P
	mat_multiply(sub_mat_1, P, sub_mat_2);

	// PCorrected = P - [(Lk * Cprime) * P]
    for (row = 1; row <= 2; row++) {
		for (col = 1; col <= 2; col++) {
			mat_set(row, col, mat_get(row, col, P) - mat_get(row, col, sub_mat_2), PCorrected);
		}
	}

}


//--------------------------------------- DRIVER CODE ----------------------------------------

// Constants
#define dt 0.01		// Sampling Period
#define R0 0.01
#define Rc 0.015
#define Cc 2400
#define Cbat 18000
#define Voc0 3.435

void main(void) {
    
    SYSTEM_Initialize();
    
    ADCC_DischargeSampleCapacitor();

	// Initializing probability matrices
    Matrix Aprime;
    mat_init(2, 2, &Aprime);
    mat_set(1, 1, 1.0, &Aprime);
    mat_set(2, 2, exp(-dt / (Cc * Rc)), &Aprime);
    Matrix Aprime_transpose;
    mat_transpose(&Aprime, &Aprime_transpose);
    
    Matrix Eprime;
    mat_init(2, 2, &Eprime);
    mat_set(1, 1, 1.0, &Eprime);
    mat_set(2, 2, 1.0, &Eprime);
    Matrix Eprime_transpose;
    mat_transpose(&Eprime, &Eprime_transpose);   
    
    Matrix Fprime;
    mat_init(2, 2, &Fprime);
    mat_set(1, 1, 1.0, &Fprime);
    mat_set(2, 2, 1.0, &Fprime);    
    
    Matrix Cprime; // VOC(SOC)
    mat_init(2, 1, &Cprime);
    mat_set(1, 1, 0.007, &Cprime);
    mat_set(2, 1, -1.0, &Cprime);
    Matrix Cprime_transpose;
    mat_init(2, 1, &Cprime_transpose);
    mat_set(1, 1, mat_get(1, 1, &Cprime), &Cprime_transpose);
    mat_set(2, 1, mat_get(1, 2, &Cprime), &Cprime_transpose);


	// Coefficients for probability
	float Rk = pow(10, -4);

	// --------------------------------------------------------------------------
	// At time k
	// --------------------------------------------------------------------------

	// Variables Qualitatively
	// (Constant) VOC : Open circuit voltage
	// (Time variant) Vc : Voltage across capacitor in battery circuit model
	// (Time variant) V : Voltage we measure(output from battery circuit model)
	// (Constant)Cc : Capacitor's capacitance
	// (Constant)Rc : Resistor in parallel with capacitor
	// (Constant)R0 : Ohmic resistance
	// (Constant)Cbat : Capacity of battery in AmpHours ?

    
	//// Initializing the matrixes we'll need to use
    Matrix xhat;	//xhat is a 2-by-1 matrix, top value is: SOC_estimated, bottom is Vc_estimated
    mat_init(2, 1, &xhat);
	Matrix Qk1;
    mat_init(2, 2, &Qk1);
    mat_set(1, 1, 2.5 * pow(10, -7), &Qk1);
    Matrix xhatk_1;
    mat_init(2, 1, &xhatk_1);
    Matrix fk_;
    mat_init(2, 1, &fk_);
    Matrix xhatCorrected;
    mat_init(2, 1, &xhatCorrected);
    Matrix PCorrected;
    mat_init(2, 2, &PCorrected);
    Matrix Lk;
    mat_init(2, 1, &Lk);
    Matrix P;
    mat_init(2, 2, &P);
    mat_set(1, 1, Rk, &P);
    mat_set(2, 2, Rk, &P);
    Matrix Pk_1;
    mat_init(2, 2, &Pk_1);
    mat_set(1, 1, Rk, &Pk_1);
    mat_set(2, 2, Rk, &Pk_1);
    Matrix sub_mat_1;
    mat_init(2, 2, &sub_mat_1);
    Matrix sub_mat_2;
    mat_init(2, 2, &sub_mat_2);
    Matrix sub_mat_3;
    mat_init(2, 2, &sub_mat_3);
    
    
	// Variables subject to constant change
	uint8_t i, j;
    float actualSOC = 1.0;
    float Vc = 0.0;
	float I = 0.0;
	float Ik_1 = 0.0;
	float V = 0.0;		// current V
    mat_set(1, 1, actualSOC, &xhatk_1);
	mat_set(2, 1, Vc, &xhatk_1);

    uint8_t cumulative_time = 0;
     
    static uint8_t rx_buf[RX_BUFFER_LENGTH] = {0};
    static Frame_t rx_frame; 
    Frame_Init(&rx_frame, rx_buf, RX_BUFFER_LENGTH);
    
    while (1) { 
        while(UART1_is_rx_ready())
        {
            FrameResult_t res = Frame_Update(rx_frame, UART1_Read());
            if(res == FRAME_COMPLETE){
                int16_t vc_int = rx_frame->buf[0] + (rx_frame->buf[1] << 8); 
                Vc = (float)vc_int;
                int16_t v_int = rx_frame->buf[2] + (rx_frame->buf[3] << 8); 
                V = (float)v_int;
                int16_t i_int = rx_frame->buf[4] + (rx_frame->buf[5] << 8); 
                I = (float)i_int;
                break;
            }
        }
            

//        printf("[INPUT] ActualSOC: %.6lf, Vc: %.6lf, V: %.6lf, I: %.6lf\n\r", actualSOC, Vc, V, I);

        // FUNCTION TO CALCULATE xhatCorrected & PCorrected
        EKF(&xhatk_1, &Pk_1, I, Ik_1, V, Voc0, Rk, &Aprime, &Cprime, &Eprime,
            &Fprime, &fk_, dt, Cbat, Cc, Rc, &Qk1,
            &Aprime_transpose, &Eprime_transpose, &sub_mat_1, &sub_mat_2,
            &sub_mat_3, &xhat, &P, &Cprime_transpose, &Lk, R0, &xhatCorrected,
            &PCorrected);


        // Setting the xhatk_1 and Pk_1 values
        for (i = 1; i <= 2; i++) {
            // set values of xhatk_1 to xhatCorrected
            mat_set(i, 1, mat_get(i, 1, &xhatCorrected), &xhatk_1);

            // For each column of second matrix
            for (j = 1; j <= 2; j++) {
                // set values of Pk_1 to PCorrected
                mat_set(i, j, mat_get(i, j, &PCorrected), &Pk_1);
            }
        }
        
        uint8_t SOC = (uint8_t)(mat_get(1, 1, &xhatCorrected) * 100);
        while(!UART1_is_tx_ready());
        UART1_Write(SOC); // send estimated SOC
 
    }

}


 