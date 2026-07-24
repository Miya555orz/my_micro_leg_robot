/**

  ******************************************************************************

  * File Name          : Straight_Leg_Calc.c

  * Description        : Code for Straight_Leg_Calc applications

  ******************************************************************************

  * @attention

  * Copyright (c) 2026 SZU RobotPilots.

  * @author 

  * Liang 741427745@qq.com

  ==============================================================================

                      ##### How To Use #####

  ==============================================================================

  (#) 调用初始化函数

	  Straight_Leg_Init

	  之后的步骤可以使用函数指针，此处以原函数名讲解

  

  (#) 调用获取外部数据API，《请确保输入的数据方向正确》，

		Straight_Leg_Target_State_update、

		Straight_Leg_External_data_update


  （#）Chassis_State_Var_Update函数直接对状态量操作,不额外调用API获取函数

		

  (#) 在使用《LQR模式》时先调用K_Matrix_Fitting_Update，后调用Straight_Leg_Torque_Cal，

		即可自动计算控制量u=[Tw Tp]^T

		

  (#)调用以下函数获取控制量的值

		Get_LQR_Tw

		Get_LQR_Tp

  

  */

#include "Straight_Leg_Calc.h"

static void Straight_Leg_Mat_Init(Straight_Leg_t* My_Model);

static void Straight_Leg_Target_State_update(Straight_Leg_t* My_Model,float tar_thetal,float tar_thetald1,

											float tar_s,float tar_sd1,

											float tar_thetab,float tar_thetabd1);//目标值API

static void Straight_Leg_External_data_update(Straight_Leg_t* My_Model,float l0);

static void Straight_Leg_Lmat_update(Straight_Leg_t* My_Model);

static void K_Matrix_Fitting_Update(Straight_Leg_t* My_Model);

static void Straight_Leg_Torque_Cal(Straight_Leg_t* My_Model);

static void Straight_Leg_Err_State_update(Straight_Leg_t* My_Model);//方便debug

static void Straight_Leg_Xmat_update(Straight_Leg_t* My_Model);

static float Get_LQR_Tw(Straight_Leg_t* My_Model);

static float Get_LQR_Tp(Straight_Leg_t* My_Model);

											

/**

  * @brief  直腿模型软件层初始化

  * @param  Straight_Leg_t* model 直腿结构体

  * @retval None

  */

void Straight_Leg_Init(Straight_Leg_t* My_Model)

{

	Straight_Leg_Mat_Init(My_Model);//矩阵初始化，用于后面计算

	

	My_Model->LQR_cal = Straight_Leg_Torque_Cal;

	My_Model->K_fitting = K_Matrix_Fitting_Update; 

	My_Model->target_state_update=Straight_Leg_Target_State_update;

	My_Model->ex_data_update=Straight_Leg_External_data_update;

	My_Model->get_Tw=Get_LQR_Tw;

	My_Model->get_Tp=Get_LQR_Tp;

	

}



/**

  * @brief  计算直腿模型驱动轮，关节力矩

  * @param  Straight_Leg_t* model 直腿结构体

  * @param  State_Var_t* var      状态变量

  * @retval None

  */

void Straight_Leg_Torque_Cal(Straight_Leg_t* My_Model)

{

	arm_status status;

	

	Straight_Leg_Xmat_update(My_Model);//更新状态矩阵

	

	status |=Matrix_Subtract(&My_Model->X_info->X_target_mat, &My_Model->X_info->X_state_mat, &My_Model->X_info->X_err_mat);//X_err=X_target-X_state

	//计算矩阵误差

	Straight_Leg_Err_State_update(My_Model);//目标-测量

	//处理矩阵误差

	status |=Matrix_Multiply(&My_Model->K_info->K_mat,&My_Model->X_info->X_err_mat,&My_Model->u->u_mat);

	//解出控制量

}



/**

  * @brief  根据实时腿长拟合K矩阵，在腿长更新后调用

  * @param  Straight_Leg_t* My_Model

  * @retval None

  * @note   p00 + p10*ll + p01*lr + p20*ll^2 + p11*ll*lr + p02*lr^2

  */



static void K_Matrix_Fitting_Update(Straight_Leg_t* My_Model)

{

	arm_status status;

	/*腿长拟合K矩阵*/

	float l0=My_Model->Ex_leg_data->l0;

	float l0_coefficient_vector[4];

	mat l0_Coefficient_Vector;

	l0_coefficient_vector[0]=1;

	l0_coefficient_vector[1]=l0;

	l0_coefficient_vector[2]=l0*l0;

	l0_coefficient_vector[3]=l0*l0*l0;

	

	Matrix_Init(&l0_Coefficient_Vector,4,1,l0_coefficient_vector);

	

	float K_Tw_coe[6];

	mat K_Tw_Vector;//仅用于运算

	Matrix_Init(&K_Tw_Vector,6,1,K_Tw_coe);

	

	mat K_Tw;//仅用于存储到K_coefficient

	Matrix_Init(&K_Tw,1,6,&My_Model->K_info->K_coefficient[0][0]);

	

	status |=Matrix_Multiply(&My_Model->K_info->K_fit_Tw_mat,&l0_Coefficient_Vector,&K_Tw_Vector);

	status |=Matrix_Transpose(&K_Tw_Vector,&K_Tw);

	

	

	float K_Tp_coe[6];

	mat K_Tp_Vector;//仅用于运算

	Matrix_Init(&K_Tp_Vector,6,1,K_Tp_coe);

	

	mat K_Tp;//仅用于存储到K_coefficient

	Matrix_Init(&K_Tp,1,6,&My_Model->K_info->K_coefficient[1][0]);

	

	status |=Matrix_Multiply(&My_Model->K_info->K_fit_Tp_mat,&l0_Coefficient_Vector,&K_Tp_Vector);

	status |=Matrix_Transpose(&K_Tp_Vector,&K_Tp);

	

}



/**

  * @brief  状态结构体目标更新API

  * @param  Straight_Leg_t* model 直腿结构体

  * @retval None

  */

static void Straight_Leg_Target_State_update(Straight_Leg_t* My_Model,float tar_thetal,float tar_thetald1,

											float tar_s,float tar_sd1,

											float tar_thetab,float tar_thetabd1)

{

	My_Model->info->target_thetal  =  tar_thetal;

	My_Model->info->target_thetald1=  tar_thetald1  ;

	

	My_Model->info->target_s  		= tar_s ;

	My_Model->info->target_sd1		= tar_sd1  ;

	

	My_Model->info->target_thetab  =  tar_thetab  ;

	My_Model->info->target_thetabd1=  tar_thetabd1  ;

	

}



/**

  * @brief  外部数据更新API

  * @param  Straight_Leg_t* model 直腿结构体

  * @retval None

  */

static void Straight_Leg_External_data_update(Straight_Leg_t* My_Model,float l0)

{

	My_Model->Ex_leg_data->l0=l0;

}



/**

  * @brief  直腿状态矩阵和目标矩阵更新

  * @param  Straight_Leg_t* model 直腿结构体

  * @retval None

  */

static void Straight_Leg_Xmat_update(Straight_Leg_t* My_Model)

{

	/*状态矩阵更新*/

	My_Model->X_info->X_state_mat_storage[X_s] = My_Model->info->s;

	My_Model->X_info->X_state_mat_storage[X_sd1] = My_Model->info->sd1;

	

	My_Model->X_info->X_state_mat_storage[X_thetal] = My_Model->info->thetal;

	My_Model->X_info->X_state_mat_storage[X_thetald1] = My_Model->info->thetald1;

	

	My_Model->X_info->X_state_mat_storage[X_thetab] = My_Model->info->thetab;

	My_Model->X_info->X_state_mat_storage[X_thetabd1] = My_Model->info->thetabd1;

	/*目标矩阵更新*/

	My_Model->X_info->X_target_mat_storage[X_s] = My_Model->info->target_s;

	My_Model->X_info->X_target_mat_storage[X_sd1] = My_Model->info->target_sd1;

	

	My_Model->X_info->X_target_mat_storage[X_thetal] = My_Model->info->target_thetal;

	My_Model->X_info->X_target_mat_storage[X_thetald1] = My_Model->info->target_thetald1;

	

	My_Model->X_info->X_target_mat_storage[X_thetab] = My_Model->info->target_thetab;

	My_Model->X_info->X_target_mat_storage[X_thetabd1] = My_Model->info->target_thetabd1;

}



/**

  * @brief  状态结构体误差更新,方便debug

  * @param  Straight_Leg_t* model 直腿结构体

  * @retval None

  */

static void Straight_Leg_Err_State_update(Straight_Leg_t* My_Model)

{

	My_Model->info->s_err  		=My_Model->X_info->X_err_mat_storage[X_s] ;

	My_Model->info->sd1_err		=My_Model->X_info->X_err_mat_storage[X_sd1]  ;

	

	My_Model->info->thetal_err  =My_Model->X_info->X_err_mat_storage[X_thetal]  ;

	My_Model->info->thetald1_err=My_Model->X_info->X_err_mat_storage[X_thetald1]  ;

	

	My_Model->info->thetab_err  =My_Model->X_info->X_err_mat_storage[X_thetab]  ;

	My_Model->info->thetabd1_err=My_Model->X_info->X_err_mat_storage[X_thetabd1]  ;

	

}







/**

  * @brief  获取结构体里的驱动轮力矩Tw，计算需要外部自己调用

  * @param  Straight_Leg_t* model 直腿结构体

  * @retval None

  */

static float Get_LQR_Tw(Straight_Leg_t* My_Model)

{

	return My_Model->u->u_mat_storage[Tw];

}



/**

  * @brief  获取结构体里的关节力矩Tp，计算需要外部自己调用

  * @param  Straight_Leg_t* model 直腿结构体

  * @retval None

  */

static float Get_LQR_Tp(Straight_Leg_t* My_Model)

{

	return My_Model->u->u_mat_storage[Tp];

}



/**

  * @brief  直腿相关矩阵初始化

  * @param  Straight_Leg_t* model 直腿结构体

  * @retval None

  */

static void Straight_Leg_Mat_Init(Straight_Leg_t* My_Model)

{

	/*初始化X部分的矩阵*/

	Matrix_Init(&My_Model->X_info->X_state_mat, 6, 1, My_Model->X_info->X_state_mat_storage);

	memset(My_Model->X_info->X_state_mat_storage, 0, 6*sizeof_float);

	Matrix_Init(&My_Model->X_info->X_target_mat, 6, 1, My_Model->X_info->X_target_mat_storage);

	memset(My_Model->X_info->X_target_mat_storage, 0, 6*sizeof_float);

	Matrix_Init(&My_Model->X_info->X_err_mat, 6, 1, My_Model->X_info->X_err_mat_storage);

	memset(My_Model->X_info->X_err_mat_storage, 0, 6*sizeof_float);



	/*初始化K部分的矩阵*/

	Matrix_Init(&My_Model->K_info->K_mat, 2, 6, &My_Model->K_info->K_coefficient[0][0]);



	Matrix_Init(&My_Model->K_info->K_fit_Tw_mat, 6, 4, &My_Model->K_info->K_coefficient_fit[0][0][0]);

	

	Matrix_Init(&My_Model->K_info->K_fit_Tp_mat, 6, 4, &My_Model->K_info->K_coefficient_fit[1][0][0]);

	

	/*初始化u控制量矩阵*/

	Matrix_Init(&My_Model->u->u_mat,2,1,My_Model->u->u_mat_storage);

	memset(&My_Model->u->u_mat_storage, 0, 2*sizeof_float);

}







