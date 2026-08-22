#include "GimbalTask.hpp"

static PID YAW_Angle_PID;
static PID YAW_Speed_PID;
static PID PITCH_Angle_PID;
static PID PITCH_Speed_PID;
static float YAW_target_angle = 0.0f;

void Gimbal_Init(void)
{
    YAW_Angle_PID.SetParams(20.0f, 0.0f, 0.0f, 17000.0f, 17000.0f);
    YAW_Speed_PID.SetParams(60.0f, 0.01f, 0.0f, 17000.0f, 17000.0f);
    PITCH_Angle_PID.SetParams(15.0f, 0.0f, 0.0f, 17000.0f, 17000.0f);
    PITCH_Speed_PID.SetParams(40.0f, 0.01f, 0.0f, 17000.0f, 17000.0f);
}

void Gimbal_Update(void)
{
    if (dr16_remote.s1 == 1)
    {
        if (dr16_remote.ch0 > 1024U + 15U || dr16_remote.ch0 < 1024U - 15U)
        {
            YAW_target_angle += ((float)(dr16_remote.ch0) - 1024) / 1320.0f * 0.5f;
        }

        YAW_Angle_PID.AngleCalc(YAW_Speed_PID,
                                YAW_target_angle,
                                INS.YawTotalAngle,
                                YAW_Motor.speed);
        YAW_Motor.SetCurrent(MotorModel::GM6020, YAW_ID, (int16_t)YAW_Speed_PID.GetOutput());

        PITCH_Angle_PID.GravityAngleCalc(PITCH_Speed_PID,
                                         60.0f,
                                         -30.0f,
                                         ((dr16_remote.ch1 - 364U) / 1320.0f * 40.0f - 20.0f),
                                         -INS.Pitch,
                                         PITCH_Motor.speed);
        PITCH_Motor.SetCurrent(MotorModel::GM6020, PITCH_ID, (int16_t)PITCH_Speed_PID.GetOutput());
    }
    else
    {
        YAW_Motor.SetCurrent(MotorModel::GM6020, YAW_ID, 0);
        PITCH_Motor.SetCurrent(MotorModel::GM6020, PITCH_ID, 0);
    }

    YAW_Motor.StartMotor(MotorModel::GM6020, YAW_CAN);
    PITCH_Motor.StartMotor(MotorModel::GM6020, PITCH_CAN);
}