#include "GimbalTask.hpp"

static PID YAW_Angle_PID;
static PID YAW_Speed_PID;
static PID PITCH_Angle_PID;
static PID PITCH_Speed_PID;
static float YAW_target_angle = 0.0f;
static bool startflag = false;

void Gimbal_StartMotors(void)
{
    YAW_Motor.StartMotor(MotorModel::GM6020, YAW_CAN);
    PITCH_Motor.StartMotor(MotorModel::GM6020, PITCH_CAN);
}
void Gimbal_Init(void)
{
    YAW_Angle_PID.SetParams(20.0f, 0.0f, 0.0f, 17000.0f, 17000.0f);
    YAW_Speed_PID.SetParams(70.0f, 0.02f, 0.0f, 17000.0f, 17000.0f);
    PITCH_Angle_PID.SetParams(20.0f, 0.0f, 0.0f, 17000.0f, 17000.0f);
    PITCH_Speed_PID.SetParams(70.0f, 0.01f, 0.0f, 17000.0f, 17000.0f);
}

void Gimbal_UpDate(void)
{
    if (startflag == true)
    {
        YAW_target_angle = INS.YawTotalAngle;
        startflag = false;
    }
    if (dr16_remote.ch0 > 1024U + 30U || dr16_remote.ch0 < 1024U - 30U)
    {
        YAW_target_angle -= ((float)(dr16_remote.ch0) - 1024) / 1320.0f * 0.5f;
    }

    YAW_Angle_PID.AngleCalc(YAW_Speed_PID,
                            YAW_target_angle,
                            INS.YawTotalAngle,
                            YAW_Motor.speed);
    YAW_Motor.SetCurrent(MotorModel::GM6020, YAW_CAN, YAW_ID, (int16_t)YAW_Speed_PID.GetOutput());

    PITCH_Angle_PID.GravityAngleCalc(PITCH_Speed_PID,
                                     60.0f,
                                     (-30.0f) * PI / 180.0f,
                                     ((dr16_remote.ch1 - 364U) / 1320.0f * 40.0f - 20.0f),
                                     INS.Pitch,
                                     PITCH_Motor.speed);
    PITCH_Motor.SetCurrent(MotorModel::GM6020, PITCH_CAN, PITCH_ID, (int16_t)PITCH_Speed_PID.GetOutput());

    Gimbal_StartMotors();
}

void Gimbal_Standby(void)
{
    YAW_Motor.SetCurrent(MotorModel::GM6020, YAW_CAN, YAW_ID, 0);
    PITCH_Motor.SetCurrent(MotorModel::GM6020, PITCH_CAN, PITCH_ID, 0);

    Gimbal_StartMotors();

    startflag = true;
}