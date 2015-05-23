//
//  main.c
//  Handin 1 - Embedded Systems
//
//  Created by Björn Nilsson on 18/04/15.
//  Copyright (c) 2015 Björn Nilsson. All rights reserved.
//
#include<stdio.h>
#include<math.h>

// Simulation variables
double r[1000];
double u[1000];
double y[1000];

double p[1000];
double i[1000];
double d[1000];
int t; //time index

// Constants for process
#define h 1 // 0.1
#define T 10 //3
#define K 2
#define a_process exp(-h/T)
#define b_process ((1 - exp(-h/T))*K)
#define c_process 1

// Variables for process
double x_process;

// Constants for controller
#define Kd 0 //2
#define Kp 0 //4
#define Ki 1 //0.5
#define Tf 4

//#define a_c (4*Kp*Tf - 2*Kp*h -Ki*Tf*h + Ki*h*h)
//#define b_c (-8*Kp*Tf + Ki*h*h - 2*Ki*h*Tf+ Ki*h*h + Ki*h-2*Kd)
//#define c_c (2*Kp*h + 4*Kp*h + Ki*h*h + 2*Ki*h*Tf + 2*Kd)
//#define d_c (4*Tf -2*h)
//#define e_c (8*Tf)
//#define f_c (4*Tf +2*h)

//#define Ts h
//#define a_c (-Ts*Kp*2 + Tf*Kp*4+Kd*4)
//#define b_c (-Tf*Kp*8+Ts*Ts*Ki-Tf*Ts*Ki*2-Kd*8)
//#define c_c (Ts*Kp*2+Tf*Kp*4 + Ts*Ts*Ki + Tf*Ts*Ki*2+Kd*4)eclip
//#define d_c (-Ts*2 + Tf*4)
//#define e_c (Tf*8)
//#define f_c (Ts*2+Tf*4)

//#define a_c (4*Kp*Tf + 4*Kd - 2*Ts*Kp - 2*Ts*Ki*Tf + Ki*Ts*Ts)
//#define b_c (-8*Kp*Tf -8*Kd +2*Ki*Ts*Ts)
//#define c_c (4*Kp*Tf + 4*Kd + 2*Ts*Kp + 2*Ts*Ki*Tf + Ki*Ts*Ts)
//#define d_c (4*Tf - 2*Ts)
//#define e_c (-8*Tf)
//#define f_c (4*Tf + 2*Ts)


//function declarations
void controller();
void process();

int main(){
    //Setup
    t = 2; //time index start at 2 because we're using 2 previous values of r and u
    x_process = 0;
    
    FILE *input_file = fopen("setpointvalues.txt", "r");
    
    if (!input_file){
        return 1;
    }
    
    //simulate controller and process using input from file
    while (fscanf(input_file,"%lf", &r[t]) > 0){
        printf("%f \n", r[t]);
        
        //send reference to Controller
        controller();
        //send control signal to process
        process();
        ++t;
    }
    fclose(input_file);
    
    FILE *output_file = fopen("output.txt","w"); // create output.txt
    FILE *ControllerOutput_file = fopen("ControllerOutput.txt","w"); // create ControllerOutput.txt
    
    int j;
    for(j = 2; j < t; j++){
        //TODO print to file
        printf("%f \n", y[j]);
        fprintf(output_file,"%f\n", y[j]);
        fprintf(ControllerOutput_file, "%f\n", u[j]);
    }
    fclose(output_file);
    fclose(ControllerOutput_file);
    return 0;
}

void controller(){
    //implementation of difference equation for F(z)
    //u[t] = (a_c*(r[t-2]-y[t-2])+b_c*(r[t-1]-y[t-1])+c_c*(r[t]-y[t]) -d_c*u[t-2]-e_c*u[t-1])/f_c;
    double e_0 = r[t]-y[t];    // e [n]
    double e_1 = r[t-1]-y[t-1];// e [n-1]
    
    p[t]= Kp*e_0;
    i[t]= i[t-1] + Ki*h*(e_0 + e_1)/2;
    d[t]=(2*Kd*(e_0 - e_1) - d[t-1]*(h-2*Tf))/(h+2*Tf);
    
    u[t] = p[t] + i[t] + d[t];
}

void process(){
    y[t] = c_process*x_process;
    x_process = a_process*x_process + b_process*u[t];
    
}

