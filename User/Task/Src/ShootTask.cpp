#include "ShootTask.hpp"

static PID LEFT_FRICTION_PID;
static PID RIGHT_FRICTION_PID;
static uint8_t shoot_flag = 0;
static float feeder_speed = 0.0f;

void Shoot_Init(void)
{
    LEFT_FRICTION_PID.SetParams(40.0f, 0.0f, 0.0f, 17000.0f, 17000.0f);
    RIGHT_FRICTION_PID.SetParams(40.0f, 0.0f, 0.0f, 17000.0f, 17000.0f);
    while (!FEEDER_Motor.state)
    {
        FEEDER_Motor.Enable(FEEDER_CAN, FEEDER_TX_ID);
    }
}

void Shoot_Update(void)
{
    if (dr16_remote.s1 == 1)
    {
        LEFT_FRICTION_PID.SpeedCalc(600.0f, LEFT_FRICTION_Motor.speed);
        RIGHT_FRICTION_PID.SpeedCalc(-600.0f, RIGHT_FRICTION_Motor.speed);

        LEFT_FRICTION_Motor.SetCurrent(MotorModel::M3508, LEFT_FRICTION_ID, (int16_t)LEFT_FRICTION_PID.GetOutput());
        LEFT_FRICTION_Motor.SetCurrent(MotorModel::M3508, RIGHT_FRICTION_ID, (int16_t)RIGHT_FRICTION_PID.GetOutput());

        if (dr16_remote.wheel > 1530U && (!shoot_flag))
        {
            feeder_speed = 30.0f;
            shoot_flag = 1;
        }

        if (LEFT_FRICTION_Motor.speed < 550 && RIGHT_FRICTION_Motor.speed > -550 && shoot_flag)
        {
            feeder_speed = 0.0f;
            shoot_flag = 0;
        }

        FEEDER_Motor.MITCmd(FEEDER_CAN, FEEDER_TX_ID, 0, feeder_speed, 0, 1, 0);
    }
    else
    {
        LEFT_FRICTION_Motor.SetCurrent(MotorModel::M3508, LEFT_FRICTION_ID, 0);
        RIGHT_FRICTION_Motor.SetCurrent(MotorModel::M3508, RIGHT_FRICTION_ID, 0);
        FEEDER_Motor.MITCmd(FEEDER_CAN, FEEDER_TX_ID, 0, 0, 0, 1, 0);
    }

    LEFT_FRICTION_Motor.StartMotor(MotorModel::M3508, LEFT_FRICTION_CAN);
}