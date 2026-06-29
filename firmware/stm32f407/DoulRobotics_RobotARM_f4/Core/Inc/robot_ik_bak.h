#ifndef ROBOT_IK_H
#define ROBOT_IK_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ROBOT_IK_OK = 0,
    ROBOT_IK_MAX_ITER = 1,
    ROBOT_IK_UNREACHABLE = 2,
    ROBOT_IK_BAD_ARG = 3
} RobotIKStatus;

typedef struct
{
    float slider_m;  // URDF unit: meter
    float l1_rad;    // URDF unit: rad
    float l2_rad;    // URDF unit: rad
} RobotIKJoint;

typedef struct
{
    float x_mm;
    float y_mm;
    float z_mm;
} RobotIKPoint;

typedef struct
{
    RobotIKJoint q;
    RobotIKPoint tcp_mm;
    float error_mm;
    int iterations;
} RobotIKResult;

void RobotIK_GetZeroPose(RobotIKJoint *q);
void RobotIK_ClampJoint(RobotIKJoint *q);
void RobotIK_FK(const RobotIKJoint *q, RobotIKPoint *tcp_mm);

RobotIKStatus RobotIK_SolveXYZ(
    float target_x_mm,
    float target_y_mm,
    float target_z_mm,
    const RobotIKJoint *seed_q,
    RobotIKResult *out
);

float RobotIK_RadToDeg(float rad);
float RobotIK_DegToRad(float deg);

#ifdef __cplusplus
}
#endif

#endif
