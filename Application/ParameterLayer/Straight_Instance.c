#include "Straight_Instance.h"
#include "pid.h"

#define K_MATRIX_FIT_COEFFICIENT                                         \
    {                                                                    \
        {{-7.063155f, -200.683088f, 522.852712f, -561.525307f},          \
         {0.226674f, -19.123111f, 7.701899f, -9.058826f},                \
         {-0.973013f, -29.587510f, 93.131638f, -103.299980f},            \
         {-1.530980f, -30.041075f, 88.873828f, -98.548485f},             \
         {47.724642f, -236.822484f, 535.159449f, -466.012198f},          \
         {5.579767f, -26.696687f, 62.489382f, -57.229439f}},             \
        {                                                                \
            {81.190126f, -270.089588f, 426.334871f, -214.655415f},       \
                {4.548271f, 16.209619f, -70.827687f, 90.182188f},        \
                {21.553309f, -107.551921f, 241.596715f, -208.640445f},   \
                {25.500409f, -134.072616f, 325.260593f, -303.409732f},   \
                {22.115288f, 1072.856666f, -3384.129833f, 3762.486752f}, \
                {-0.954173f, 114.418923f, -354.561640f, 390.764174f}     \
        }                                                                \
    }

#define K_MATRIX_COEFFICIENT {                                                \
    {-19.875336f, -1.708253f, -2.279373f, -3.180075f, 25.255751f, 3.422449f}, \
    {58.485414f, 7.131971f, 10.797665f, 14.494327f, 68.992251f, 5.496438f}}

K_Matrix_t K_Matrix[Leg_Num] = {
    [R_Leg].K_coefficient = K_MATRIX_COEFFICIENT,
    [L_Leg].K_coefficient = K_MATRIX_COEFFICIENT,
    [R_Leg].K_coefficient_fit = K_MATRIX_FIT_COEFFICIENT,
    [L_Leg].K_coefficient_fit = K_MATRIX_FIT_COEFFICIENT,
};

X_Matrix_t X_Matrix[Leg_Num];
State_info_t Straight_Leg_info[Leg_Num];
Ex_leg_data_t Ex_leg_data[Leg_Num];
u_t u[Leg_Num];

Straight_Leg_t Straight_Leg[Leg_Num] = {
    [R_Leg] = {
        .info = &Straight_Leg_info[R_Leg],
        .K_info = &K_Matrix[R_Leg],
        .X_info = &X_Matrix[R_Leg],
        .Ex_leg_data = &Ex_leg_data[R_Leg],
        .u = &u[R_Leg],
        .init = Straight_Leg_Init,
    },
    [L_Leg] = {
        .info = &Straight_Leg_info[L_Leg],
        .K_info = &K_Matrix[L_Leg],
        .X_info = &X_Matrix[L_Leg],
        .Ex_leg_data = &Ex_leg_data[L_Leg],
        .u = &u[L_Leg],
        .init = Straight_Leg_Init,
    },
};
