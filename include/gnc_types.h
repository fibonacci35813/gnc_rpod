/**
 * @file    gnc_types.h
 * @brief   Fundamental GNC types, constants, and status codes.
 *
 * All physical quantities are SI units unless noted:
 *   length   : metres (m)
 *   velocity : metres per second (m/s)
 *   time     : seconds (s)
 *   mass     : kilograms (kg)
 *   force    : newtons (N)
 *   angle    : radians (rad)
 */

#ifndef GNC_TYPES_H
#define GNC_TYPES_H

#include <stdint.h>

/* -----------------------------------------------------------------------
 * Compile-time constants (Rule 8: simple object-like macros only)
 * ----------------------------------------------------------------------- */
#define GNC_MAX_SIM_STEPS   10000U   /* hard sim loop ceiling             */
#define GNC_MAX_WAYPOINTS       8U   /* guidance waypoint table size       */
#define GNC_STATE_DIM           6U   /* nav filter state: [x y z vx vy vz] */
#define GNC_MEAS_DIM            3U   /* measurement: [range az el]         */
#define GNC_THRUSTER_AXES       6U   /* ±x, ±y, ±z pairs                  */
#define GNC_MAX_RW              3U   /* three reaction wheels (orthogonal) */

/* Physical / mission constants */
#define GNC_MU_EARTH    3.986004418e14  /* Earth GM (m^3/s^2)             */
#define GNC_ISS_ALTITUDE    4.0e5       /* representative orbit alt (m)    */
#define GNC_EARTH_RADIUS    6.371e6     /* Earth mean radius (m)           */
#define GNC_CHASER_MASS     500.0       /* chaser wet mass (kg)            */
#define GNC_MAX_THRUST        50.0      /* max per-axis thrust (N)         */
#define GNC_MIN_IMPULSE_BIT   0.1       /* minimum thruster impulse (N·s)  */
#define GNC_DT_SEC            1.0       /* simulation / control step (s)   */

/* Docking success thresholds */
#define GNC_DOCK_POS_TOL    0.05        /* 5 cm position tolerance (m)     */
#define GNC_DOCK_VEL_TOL    0.01        /* 1 cm/s velocity tolerance (m/s) */

/* -----------------------------------------------------------------------
 * Status / error codes
 * ----------------------------------------------------------------------- */
typedef enum {
    GNC_OK              =  0,
    ERR_NULL_PTR        = -1,
    ERR_BAD_PARAM       = -2,
    ERR_SINGULAR        = -3,
    ERR_BOUNDS          = -4,
    ERR_NOT_CONVERGED   = -5,
    ERR_DOCKED          =  1    /* positive: informational, not failure    */
} GncStatus;

/* -----------------------------------------------------------------------
 * Vector and matrix types (fixed-size, value semantics)
 * No double-pointer dereference anywhere (Rule 9).
 * ----------------------------------------------------------------------- */
typedef struct { double v[3]; } Vec3;

/* 3×3 matrix stored row-major: m[row][col] */
typedef struct { double m[3][3]; } Mat3x3;

/* 6×6 matrix for covariance / state transition */
typedef struct { double m[6][6]; } Mat6x6;

/* -----------------------------------------------------------------------
 * Navigation state: 6-DOF relative LVLH
 * x = radial, y = along-track, z = cross-track  (target-centred LVLH)
 * ----------------------------------------------------------------------- */
typedef struct {
    Vec3   pos;          /* relative position (m)          */
    Vec3   vel;          /* relative velocity (m/s)        */
    Mat6x6 cov;          /* estimation error covariance    */
    double time_s;       /* current mission elapsed time   */
    uint8_t valid;       /* 1 = state is valid             */
} NavState;

/* -----------------------------------------------------------------------
 * Guidance waypoint and active reference
 * ----------------------------------------------------------------------- */
typedef struct {
    Vec3   pos_ref;      /* reference position (m)         */
    Vec3   vel_ref;      /* reference velocity (m/s)       */
    double corridor_m;   /* approach corridor half-width   */
    double hold_time_s;  /* time to hold before advancing  */
} Waypoint;

typedef struct {
    Waypoint table[GNC_MAX_WAYPOINTS]; /* waypoint list                   */
    uint32_t count;                    /* number of valid waypoints       */
    uint32_t active;                   /* index of active waypoint        */
    double   phase_elapsed_s;          /* seconds spent in active phase   */
} GuidancePlan;

/* -----------------------------------------------------------------------
 * Control command and fuel accounting
 * ----------------------------------------------------------------------- */
typedef struct {
    Vec3   force_N;      /* commanded force vector (N) in LVLH            */
    double delta_v_mps;  /* ΔV magnitude this step (m/s)                  */
    double prop_step_kg; /* propellant consumed this step (kg)             */
} ControlCmd;

typedef struct {
    double total_dv_mps; /* cumulative ΔV expended (m/s)                  */
    double prop_kg;      /* propellant mass consumed (kg) (Tsiolkovsky)   */
    uint32_t fire_count; /* total thruster firing events                  */
} FuelState;

/* -----------------------------------------------------------------------
 * Attitude state and command
 * q[4] = [q0, q1, q2, q3] where q0 is the scalar part (w, x, y, z convention)
 * omega[3] = angular velocity (rad/s) in body frame
 * ----------------------------------------------------------------------- */
typedef struct {
    double q[4];         /* unit quaternion [w, x, y, z]        */
    double omega[3];     /* body angular velocity (rad/s)        */
} AttState;

typedef struct {
    double torque[3];    /* commanded torque vector (N·m)        */
} AttCmd;

/* -----------------------------------------------------------------------
 * Environment perturbation model (Phase 7)
 * True dynamics use J2 + drag; onboard GNC stays with plain CW.
 * ----------------------------------------------------------------------- */
typedef struct {
    uint8_t use_j2;       /* 1 = include J2 oblateness perturbation    */
    uint8_t use_drag;     /* 1 = include atmospheric drag              */
    double  Cd;           /* drag coefficient (dimensionless, typ 2.2) */
    double  area_m2;      /* cross-sectional area (m^2)                */
} EnvModel;

/* -----------------------------------------------------------------------
 * Simulation telemetry record (one per step)
 * ----------------------------------------------------------------------- */
typedef struct {
    uint32_t step;
    double   time_s;
    Vec3     true_pos;   /* true (simulated) relative position             */
    Vec3     true_vel;   /* true (simulated) relative velocity             */
    Vec3     est_pos;    /* estimated position from nav filter             */
    Vec3     est_vel;    /* estimated velocity from nav filter             */
    Vec3     cmd_force;  /* control force command (N)                      */
    double   range_m;    /* scalar range to docking port                   */
    double   delta_v;    /* ΔV this step (m/s)                             */
} TelemetryRecord;

#endif /* GNC_TYPES_H */
