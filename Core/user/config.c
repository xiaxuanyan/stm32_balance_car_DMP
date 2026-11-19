#include "config.h"

float MDB = 65.0f;

float target_speed, target_turn;

float DKP = 170.0f;
float DKD = 5.1f;
float VKP = 0.7f;                       //0.9有些大，但是能平衡的停下，不要震荡
float VKI = 0.0035f;                     //0.0045同上
float PKP = 0.001f;
float TKP = -70.0f;
float V_integral_max = 8.0f;
float V_output_max = 30.0f;
float P_output_max = 25.0f;





