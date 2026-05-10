#include "KF.h"
/**************************************************************************
Function: 鍗″皵鏇兼护娉�
Input   : 瑙掗�熷害锛屽姞閫熷害
Output  : 鏃�
**************************************************************************/
float KF_X(float acce_Y, float acce_Z, float gyro_X) // 杈撳叆閲忥細Y杞村姞閫熷害锛孼杞村姞閫熷害锛孹杞磋�掗�熷害銆�
{
    static float x_hat[2][1] = {0}; // 鍚庨獙浼拌��
    static float x_hat_minus[2][1] = {0}; // 鍏堥獙浼拌��
    static float p_hat[2][2] = {{1, 0}, {0, 1}}; // 鍚庨獙璇�宸�鍗忔柟宸�鐭╅樀
    static float p_hat_minus[2][2] = {0}; // 鍏堥獙璇�宸�鍗忔柟宸�鐭╅樀
    static float K[2][1] = {0}; // 鍗″皵鏇煎�炵泭
    const float Ts = 0.005; // 閲囨牱闂撮殧(5ms)
    const float I[2][2] = {{1, 0}, {0, 1}};
    float u[1][1] = {{gyro_X}};
    float A[2][2] = {{1, -Ts}, {0, 1}}; // A鐭╅樀
    float B[2][1] = {{Ts}, {0}}; // B鐭╅樀
    float C[1][2] = {{1, 0}};// C鐭╅樀
    float Q[2][2] = {{1e-10, 0}, {0, 1e-10}}; // 杩囩▼鍣�澹�
    float R[1][1] = {{1e-4}}; // 娴嬮噺鍣�澹�
    float A_T[2][2] = {{1, 0}, {-Ts, 1}}; // A鐭╅樀鐨勮浆缃�
    float C_T[2][1] = {{1}, {0}}; // C鐭╅樀鐨勮浆缃�
    float temp_1[2][1] = {0}; // 鐢ㄤ互瀛樺偍涓�闂磋�＄畻缁撴灉
    float temp_2[2][1] = {0}; // 鐢ㄤ互瀛樺偍涓�闂磋�＄畻缁撴灉
    float temp_3[2][2] = {0}; // 鐢ㄤ互瀛樺偍涓�闂磋�＄畻缁撴灉
    float temp_4[2][2] = {0}; // 鐢ㄤ互瀛樺偍涓�闂磋�＄畻缁撴灉
    float temp_5[1][2] = {0}; // 鐢ㄤ互瀛樺偍涓�闂磋�＄畻缁撴灉
    float temp_6[1][1] = {0}; // 鐢ㄤ互瀛樺偍涓�闂磋�＄畻缁撴灉
    float y = atan2(-acce_Y, acce_Z); // 鍒╃敤鍔犻�熷害璁＄畻瑙掑害
    // 棰勬祴閮ㄥ垎
    // 鍏堥獙浼拌�″叕寮�
    mul(2, 2, 2, 1, A, x_hat, temp_1);
    mul(2, 1, 1, 1, B, u, temp_2);
    x_hat_minus[0][0] = temp_1[0][0] + temp_2[0][0];
    x_hat_minus[1][0] = temp_1[1][0] + temp_2[1][0];
    // 鍏堥獙璇�宸�鍗忔柟宸�鍏�寮�
    mul(2, 2, 2, 2, A, p_hat, temp_3);
    mul(2, 2, 2, 2, temp_3, A_T, temp_4);
    p_hat_minus[0][0] = temp_4[0][0] + Q[0][0];
    p_hat_minus[0][1] = temp_4[0][1] + Q[0][1];
    p_hat_minus[1][0] = temp_4[1][0] + Q[1][0];
    p_hat_minus[1][1] = temp_4[1][1] + Q[1][1];
    // 鏍℃�ｉ儴鍒�
    // 鍗″皵鏇煎�炵泭鍏�寮�
    mul(1, 2, 2, 2, C, p_hat_minus, temp_5);
    mul(1, 2, 2, 1, temp_5, C_T, temp_6);
    temp_6[0][0] = 1.0f / (temp_6[0][0] + R[0][0]);
    mul(2, 2, 2, 1, p_hat_minus, C_T, temp_1);
    mul(2, 1, 1, 1, temp_1, temp_6, K);
    // 鍚庨獙浼拌�″叕寮�
    mul(1, 2, 2, 1, C, x_hat_minus, temp_6);
    temp_6[0][0] = y - temp_6[0][0];
    mul(2, 1, 1, 1, K, temp_6, temp_1);
    x_hat[0][0] = x_hat_minus[0][0] + temp_1[0][0];
    x_hat[1][0] = x_hat_minus[1][0] + temp_1[1][0];
    // 鏇存柊璇�宸�鍗忔柟宸�鍏�寮�
    mul(2, 1, 1, 2, K, C, temp_3);
    temp_3[0][0] = I[0][0] - temp_3[0][0];
    temp_3[0][1] = I[0][1] - temp_3[0][1];
    temp_3[1][0] = I[1][0] - temp_3[1][0];
    temp_3[1][1] = I[1][1] - temp_3[1][1];
    mul(2, 2, 2, 2, temp_3, p_hat_minus, p_hat);
    // 杩斿洖鍊�
    return x_hat[0][0];
}

/**************************************************************************
Function: 鍗″皵鏇兼护娉�
Input   : 瑙掗�熷害锛屽姞閫熷害
Output  : 鏃�
**************************************************************************/
float KF_Y(float acce_X, float acce_Z, float gyro_Y) // 杈撳叆閲忥細X杞村姞閫熷害锛孼杞村姞閫熷害锛孻杞磋�掗�熷害銆�
{
    static float x_hat[2][1] = {0}; // 鍚庨獙浼拌��
    static float x_hat_minus[2][1] = {0}; // 鍏堥獙浼拌��
    static float p_hat[2][2] = {{1, 0}, {0, 1}}; // 鍚庨獙璇�宸�鍗忔柟宸�鐭╅樀
    static float p_hat_minus[2][2] = {0}; // 鍏堥獙璇�宸�鍗忔柟宸�鐭╅樀
    static float K[2][1] = {0}; // 鍗″皵鏇煎�炵泭
    const float Ts = 0.005; // 閲囨牱闂撮殧(5ms)
    const float I[2][2] = {{1, 0}, {0, 1}};
    float u[1][1] = {{gyro_Y}};
    float A[2][2] = {{1, -Ts}, {0, 1}}; // A鐭╅樀
    float B[2][1] = {{Ts}, {0}}; // B鐭╅樀
    float C[1][2] = {{1, 0}};// C鐭╅樀
    float Q[2][2] = {{1e-10, 0}, {0, 1e-10}}; // 杩囩▼鍣�澹�
    float R[1][1] = {{1e-4}}; // 娴嬮噺鍣�澹�
    float A_T[2][2] = {{1, 0}, {-Ts, 1}}; // A鐭╅樀鐨勮浆缃�
    float C_T[2][1] = {{1}, {0}}; // C鐭╅樀鐨勮浆缃�
    float temp_1[2][1] = {0}; // 鐢ㄤ互瀛樺偍涓�闂磋�＄畻缁撴灉
    float temp_2[2][1] = {0}; // 鐢ㄤ互瀛樺偍涓�闂磋�＄畻缁撴灉
    float temp_3[2][2] = {0}; // 鐢ㄤ互瀛樺偍涓�闂磋�＄畻缁撴灉
    float temp_4[2][2] = {0}; // 鐢ㄤ互瀛樺偍涓�闂磋�＄畻缁撴灉
    float temp_5[1][2] = {0}; // 鐢ㄤ互瀛樺偍涓�闂磋�＄畻缁撴灉
    float temp_6[1][1] = {0}; // 鐢ㄤ互瀛樺偍涓�闂磋�＄畻缁撴灉
    float y = atan2(-acce_X, acce_Z); // 鍒╃敤鍔犻�熷害璁＄畻瑙掑害
    // 棰勬祴閮ㄥ垎
    // 鍏堥獙浼拌�″叕寮�
    mul(2, 2, 2, 1, A, x_hat, temp_1);
    mul(2, 1, 1, 1, B, u, temp_2);
    x_hat_minus[0][0] = temp_1[0][0] + temp_2[0][0];
    x_hat_minus[1][0] = temp_1[1][0] + temp_2[1][0];
    // 鍏堥獙璇�宸�鍗忔柟宸�鍏�寮�
    mul(2, 2, 2, 2, A, p_hat, temp_3);
    mul(2, 2, 2, 2, temp_3, A_T, temp_4);
    p_hat_minus[0][0] = temp_4[0][0] + Q[0][0];
    p_hat_minus[0][1] = temp_4[0][1] + Q[0][1];
    p_hat_minus[1][0] = temp_4[1][0] + Q[1][0];
    p_hat_minus[1][1] = temp_4[1][1] + Q[1][1];
    // 鏍℃�ｉ儴鍒�
    // 鍗″皵鏇煎�炵泭鍏�寮�
    mul(1, 2, 2, 2, C, p_hat_minus, temp_5);
    mul(1, 2, 2, 1, temp_5, C_T, temp_6);
    temp_6[0][0] = 1.0f / (temp_6[0][0] + R[0][0]);
    mul(2, 2, 2, 1, p_hat_minus, C_T, temp_1);
    mul(2, 1, 1, 1, temp_1, temp_6, K);
    // 鍚庨獙浼拌�″叕寮�
    mul(1, 2, 2, 1, C, x_hat_minus, temp_6);
    temp_6[0][0] = y - temp_6[0][0];
    mul(2, 1, 1, 1, K, temp_6, temp_1);
    x_hat[0][0] = x_hat_minus[0][0] + temp_1[0][0];
    x_hat[1][0] = x_hat_minus[1][0] + temp_1[1][0];
    // 鏇存柊璇�宸�鍗忔柟宸�鍏�寮�
    mul(2, 1, 1, 2, K, C, temp_3);
    temp_3[0][0] = I[0][0] - temp_3[0][0];
    temp_3[0][1] = I[0][1] - temp_3[0][1];
    temp_3[1][0] = I[1][0] - temp_3[1][0];
    temp_3[1][1] = I[1][1] - temp_3[1][1];
    mul(2, 2, 2, 2, temp_3, p_hat_minus, p_hat);
    // 杩斿洖鍊�
    return x_hat[0][0];
}

/**************************************************************************
Function: 鐭╅樀涔樻硶
Input   : 闇�瑕佺浉涔樼殑涓や釜鐭╅樀浠ュ強瀹冧滑鐨勫昂瀵�
Output  : 鐩镐箻鍚庣殑鐭╅樀
**************************************************************************/
void mul(int A_row, int A_col, int B_row, int B_col, float A[][A_col], float B[][B_col], float C[][B_col])
{
    if (A_col == B_row)
    {
        for (int i = 0; i < A_row; i++)
        {
            for (int j = 0; j < B_col; j++)
            {
                C[i][j] = 0; // 鍒濆�嬪寲
                for (int k = 0; k < A_col; k++)
                {
                    C[i][j] += A[i][k]*B[k][j];
                }
            }
        }
    }
    else
    {
        printf("閿欒��锛氱煩闃电殑灏哄�镐笉瀵癸紒");
    }
}

















