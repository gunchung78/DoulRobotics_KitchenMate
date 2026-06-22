#include "robot_ik.h"
#include <math.h>
#include <string.h>

#ifndef ROBOT_IK_PI
#define ROBOT_IK_PI 3.14159265358979323846f
#endif

#define IK_MAX_ITER             80
#define IK_ACCEPT_ERROR_MM      1.0f
#define IK_FINITE_DIFF_M        0.0005f   // 0.5 mm for slider derivative
#define IK_FINITE_DIFF_RAD      0.0005f   // rad for revolute derivative
#define IK_DAMPING              0.020f
#define IK_MAX_STEP_SLIDER_M    0.020f    // one iteration max 20 mm
#define IK_MAX_STEP_RAD         0.0872665f // one iteration max 5 deg

// ============================================================
// Values extracted from doulrobot.urdf
// Chain: x_linear_motor -> slider -> joint1_rev05 -> l1
//        -> joint2_rev06 -> l2 -> joint3_rev04 -> tcp_frame -> tcp
// q units are same as URDF: slider meter, l1/l2 rad.
// ============================================================

#define SLIDER_MIN_M   0.0f
#define SLIDER_MAX_M   0.5f
#define L1_MIN_RAD    -1.8326f
#define L1_MAX_RAD     1.8326f
#define L2_MIN_RAD    -3.14159f
#define L2_MAX_RAD     3.14159f

typedef struct
{
    float x, y, z;
} Vec3;

typedef struct
{
    float m[4][4];
} Mat4;

static float clampf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float RobotIK_RadToDeg(float rad)
{
    return rad * (180.0f / ROBOT_IK_PI);
}

float RobotIK_DegToRad(float deg)
{
    return deg * (ROBOT_IK_PI / 180.0f);
}

void RobotIK_GetZeroPose(RobotIKJoint *q)
{
    if (q == 0) return;
    q->slider_m = 0.0f;
    q->l1_rad = 0.0f;
    q->l2_rad = 0.0f;
}

void RobotIK_ClampJoint(RobotIKJoint *q)
{
    if (q == 0) return;
    q->slider_m = clampf_local(q->slider_m, SLIDER_MIN_M, SLIDER_MAX_M);
    q->l1_rad   = clampf_local(q->l1_rad,   L1_MIN_RAD,     L1_MAX_RAD);
    q->l2_rad   = clampf_local(q->l2_rad,   L2_MIN_RAD,     L2_MAX_RAD);
}

static void mat4_identity(Mat4 *T)
{
    int r, c;
    for (r = 0; r < 4; r++)
    {
        for (c = 0; c < 4; c++)
        {
            T->m[r][c] = (r == c) ? 1.0f : 0.0f;
        }
    }
}

static void mat4_mul(const Mat4 *A, const Mat4 *B, Mat4 *C)
{
    int r, c, k;
    Mat4 tmp;

    for (r = 0; r < 4; r++)
    {
        for (c = 0; c < 4; c++)
        {
            float s = 0.0f;
            for (k = 0; k < 4; k++)
            {
                s += A->m[r][k] * B->m[k][c];
            }
            tmp.m[r][c] = s;
        }
    }

    *C = tmp;
}

static void mat4_origin_xyz_rpy(float x, float y, float z,
                                float roll, float pitch, float yaw,
                                Mat4 *T)
{
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    mat4_identity(T);

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    T->m[0][0] = cy * cp;
    T->m[0][1] = cy * sp * sr - sy * cr;
    T->m[0][2] = cy * sp * cr + sy * sr;

    T->m[1][0] = sy * cp;
    T->m[1][1] = sy * sp * sr + cy * cr;
    T->m[1][2] = sy * sp * cr - cy * sr;

    T->m[2][0] = -sp;
    T->m[2][1] = cp * sr;
    T->m[2][2] = cp * cr;

    T->m[0][3] = x;
    T->m[1][3] = y;
    T->m[2][3] = z;
}

static void mat4_revolute_axis_angle(float ax, float ay, float az, float q, Mat4 *T)
{
    float n = sqrtf(ax * ax + ay * ay + az * az);
    float c = cosf(q);
    float s = sinf(q);
    float v = 1.0f - c;

    mat4_identity(T);

    if (n < 1.0e-9f)
    {
        return;
    }

    ax /= n;
    ay /= n;
    az /= n;

    T->m[0][0] = ax * ax * v + c;
    T->m[0][1] = ax * ay * v - az * s;
    T->m[0][2] = ax * az * v + ay * s;

    T->m[1][0] = ay * ax * v + az * s;
    T->m[1][1] = ay * ay * v + c;
    T->m[1][2] = ay * az * v - ax * s;

    T->m[2][0] = az * ax * v - ay * s;
    T->m[2][1] = az * ay * v + ax * s;
    T->m[2][2] = az * az * v + c;
}

static void mat4_prismatic_axis(float ax, float ay, float az, float q, Mat4 *T)
{
    float n = sqrtf(ax * ax + ay * ay + az * az);

    mat4_identity(T);

    if (n < 1.0e-9f)
    {
        return;
    }

    ax /= n;
    ay /= n;
    az /= n;

    T->m[0][3] = ax * q;
    T->m[1][3] = ay * q;
    T->m[2][3] = az * q;
}

static void apply_origin(Mat4 *T,
                         float x, float y, float z,
                         float roll, float pitch, float yaw)
{
    Mat4 A;
    mat4_origin_xyz_rpy(x, y, z, roll, pitch, yaw, &A);
    mat4_mul(T, &A, T);
}

static void apply_prismatic(Mat4 *T, float ax, float ay, float az, float q)
{
    Mat4 A;
    mat4_prismatic_axis(ax, ay, az, q, &A);
    mat4_mul(T, &A, T);
}

static void apply_revolute(Mat4 *T, float ax, float ay, float az, float q)
{
    Mat4 A;
    mat4_revolute_axis_angle(ax, ay, az, q, &A);
    mat4_mul(T, &A, T);
}

void RobotIK_FK(const RobotIKJoint *q_in, RobotIKPoint *tcp_mm)
{
    RobotIKJoint q;
    Mat4 T;

    if (q_in == 0 || tcp_mm == 0)
    {
        return;
    }

    q = *q_in;
    RobotIK_ClampJoint(&q);

    mat4_identity(&T);

    // joint slider: parent x_linear_motor -> child joint1_rev05
    apply_origin(&T,
                 -0.645739f, -0.0108398f, -0.0319808f,
                 -1.5708f, 1.0e-12f, -1.5708f);
    apply_prismatic(&T, 0.0f, 0.0f, 1.0f, q.slider_m);

    // joint l1: parent joint1_rev05 -> child joint2_rev06
    apply_origin(&T,
                 -0.0086f, -0.093f, 0.1242f,
                 0.0f, 0.0f, 0.0f);
    apply_revolute(&T, 0.0f, 0.0f, 1.0f, q.l1_rad);

    // joint l2: parent joint2_rev06 -> child joint3_rev04
    apply_origin(&T,
                 0.0f, -0.2987f, -0.1656f,
                 0.0f, 0.0f, 3.14159f);
    apply_revolute(&T, 0.0f, 0.0f, 1.0f, q.l2_rad);

    // fixed joint tcp_frame: parent joint3_rev04 -> child tcp
    apply_origin(&T,
                 -0.0216302f, 0.308299f, -0.0395f,
                 -1.83528f, 1.5708f, 0.0f);

    tcp_mm->x_mm = T.m[0][3] * 1000.0f;
    tcp_mm->y_mm = T.m[1][3] * 1000.0f;
    tcp_mm->z_mm = T.m[2][3] * 1000.0f;
}

static float point_error_mm(const RobotIKPoint *a, const RobotIKPoint *b)
{
    float dx = a->x_mm - b->x_mm;
    float dy = a->y_mm - b->y_mm;
    float dz = a->z_mm - b->z_mm;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static int solve_3x3(float A[3][3], float b[3], float x[3])
{
    int i, j, k, pivot;
    float M[3][4];

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            M[i][j] = A[i][j];
        }
        M[i][3] = b[i];
    }

    for (i = 0; i < 3; i++)
    {
        float max_abs;
        pivot = i;
        max_abs = fabsf(M[i][i]);

        for (j = i + 1; j < 3; j++)
        {
            float v = fabsf(M[j][i]);
            if (v > max_abs)
            {
                max_abs = v;
                pivot = j;
            }
        }

        if (max_abs < 1.0e-9f)
        {
            return 0;
        }

        if (pivot != i)
        {
            for (k = i; k < 4; k++)
            {
                float tmp = M[i][k];
                M[i][k] = M[pivot][k];
                M[pivot][k] = tmp;
            }
        }

        {
            float div = M[i][i];
            for (k = i; k < 4; k++)
            {
                M[i][k] /= div;
            }
        }

        for (j = 0; j < 3; j++)
        {
            if (j == i) continue;
            float factor = M[j][i];
            for (k = i; k < 4; k++)
            {
                M[j][k] -= factor * M[i][k];
            }
        }
    }

    x[0] = M[0][3];
    x[1] = M[1][3];
    x[2] = M[2][3];
    return 1;
}


static RobotIKStatus solve_from_seed(
    const RobotIKPoint *target,
    const RobotIKJoint *seed_q,
    RobotIKResult *out)
{
    RobotIKJoint q;
    int iter;

    q = *seed_q;
    RobotIK_ClampJoint(&q);

    for (iter = 0; iter < IK_MAX_ITER; iter++)
    {
        RobotIKPoint p0;
        float e[3];
        float err;
        float J[3][3];
        float JTJ[3][3];
        float JTe[3];
        float dq[3];
        int c, r, k;

        RobotIK_FK(&q, &p0);

        e[0] = target->x_mm - p0.x_mm;
        e[1] = target->y_mm - p0.y_mm;
        e[2] = target->z_mm - p0.z_mm;
        err = sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);

        if (err <= IK_ACCEPT_ERROR_MM)
        {
            out->q = q;
            out->tcp_mm = p0;
            out->error_mm = err;
            out->iterations = iter;
            return ROBOT_IK_OK;
        }

        // Numerical Jacobian. Unit: mm / q_unit.
        for (c = 0; c < 3; c++)
        {
            RobotIKJoint qp = q;
            RobotIKJoint qm = q;
            RobotIKPoint pp, pm;
            float h = (c == 0) ? IK_FINITE_DIFF_M : IK_FINITE_DIFF_RAD;

            if (c == 0)
            {
                qp.slider_m += h;
                qm.slider_m -= h;
            }
            else if (c == 1)
            {
                qp.l1_rad += h;
                qm.l1_rad -= h;
            }
            else
            {
                qp.l2_rad += h;
                qm.l2_rad -= h;
            }

            RobotIK_ClampJoint(&qp);
            RobotIK_ClampJoint(&qm);
            RobotIK_FK(&qp, &pp);
            RobotIK_FK(&qm, &pm);

            J[0][c] = (pp.x_mm - pm.x_mm) / (2.0f * h);
            J[1][c] = (pp.y_mm - pm.y_mm) / (2.0f * h);
            J[2][c] = (pp.z_mm - pm.z_mm) / (2.0f * h);
        }

        // Damped least squares: (J^T J + lambda^2 I) dq = J^T e
        for (r = 0; r < 3; r++)
        {
            JTe[r] = 0.0f;
            for (k = 0; k < 3; k++)
            {
                JTe[r] += J[k][r] * e[k];
            }

            for (c = 0; c < 3; c++)
            {
                JTJ[r][c] = 0.0f;
                for (k = 0; k < 3; k++)
                {
                    JTJ[r][c] += J[k][r] * J[k][c];
                }
            }
        }

        JTJ[0][0] += IK_DAMPING * IK_DAMPING;
        JTJ[1][1] += IK_DAMPING * IK_DAMPING;
        JTJ[2][2] += IK_DAMPING * IK_DAMPING;

        if (!solve_3x3(JTJ, JTe, dq))
        {
            break;
        }

        dq[0] = clampf_local(dq[0], -IK_MAX_STEP_SLIDER_M, IK_MAX_STEP_SLIDER_M);
        dq[1] = clampf_local(dq[1], -IK_MAX_STEP_RAD,      IK_MAX_STEP_RAD);
        dq[2] = clampf_local(dq[2], -IK_MAX_STEP_RAD,      IK_MAX_STEP_RAD);

        q.slider_m += dq[0];
        q.l1_rad   += dq[1];
        q.l2_rad   += dq[2];
        RobotIK_ClampJoint(&q);
    }

    out->q = q;
    RobotIK_FK(&q, &out->tcp_mm);
    out->error_mm = point_error_mm(&out->tcp_mm, target);
    out->iterations = iter;

    if (out->error_mm <= IK_ACCEPT_ERROR_MM)
    {
        return ROBOT_IK_OK;
    }

    return ROBOT_IK_UNREACHABLE;
}

static float estimate_slider_from_x(float target_x_mm)
{
    // At zero pose, TCP x is about -726.637 mm and slider +0.1 m moves TCP x by +100 mm.
    return clampf_local((target_x_mm + 726.637f) / 1000.0f, SLIDER_MIN_M, SLIDER_MAX_M);
}

RobotIKStatus RobotIK_SolveXYZ(
    float target_x_mm,
    float target_y_mm,
    float target_z_mm,
    const RobotIKJoint *seed_q,
    RobotIKResult *out)
{
    RobotIKPoint target;
    RobotIKJoint seeds[13];
    RobotIKResult best;
    RobotIKStatus best_status = ROBOT_IK_UNREACHABLE;
    int seed_count = 0;
    int i;
    float sx;

    if (out == 0)
    {
        return ROBOT_IK_BAD_ARG;
    }

    target.x_mm = target_x_mm;
    target.y_mm = target_y_mm;
    target.z_mm = target_z_mm;

    sx = estimate_slider_from_x(target_x_mm);

    if (seed_q != 0)
    {
        seeds[seed_count] = *seed_q;
        RobotIK_ClampJoint(&seeds[seed_count]);
        seed_count++;
    }

    // Multi-start seeds. This makes single-point commands more reliable when the robot is far from target.
    seeds[seed_count++] = (RobotIKJoint){ sx,  0.0f,                  0.0f };
    seeds[seed_count++] = (RobotIKJoint){ sx, -1.5707963f,           -1.5707963f };
    seeds[seed_count++] = (RobotIKJoint){ sx, -1.5707963f,            1.5707963f };
    seeds[seed_count++] = (RobotIKJoint){ sx,  1.5707963f,           -1.5707963f };
    seeds[seed_count++] = (RobotIKJoint){ sx,  1.5707963f,            1.5707963f };
    seeds[seed_count++] = (RobotIKJoint){ sx, -1.8326f,              -1.5707963f };
    seeds[seed_count++] = (RobotIKJoint){ sx, -1.8326f,               1.5707963f };
    seeds[seed_count++] = (RobotIKJoint){ sx,  1.8326f,              -1.5707963f };
    seeds[seed_count++] = (RobotIKJoint){ sx,  1.8326f,               1.5707963f };
    seeds[seed_count++] = (RobotIKJoint){ sx, -1.5707963f,           -3.14159f };
    seeds[seed_count++] = (RobotIKJoint){ sx,  1.5707963f,            3.14159f };
    seeds[seed_count++] = (RobotIKJoint){ sx,  0.0f,                  3.14159f };

    memset(&best, 0, sizeof(best));
    best.error_mm = 1.0e30f;

    for (i = 0; i < seed_count; i++)
    {
        RobotIKResult cand;
        RobotIKStatus st;

        RobotIK_ClampJoint(&seeds[i]);
        st = solve_from_seed(&target, &seeds[i], &cand);

        if (cand.error_mm < best.error_mm)
        {
            best = cand;
            best_status = st;
        }

        if (st == ROBOT_IK_OK)
        {
            // Use the first valid solution. Since seed_q is tried first, normal motion stays continuous.
            *out = cand;
            return ROBOT_IK_OK;
        }
    }

    *out = best;
    return best_status;
}
