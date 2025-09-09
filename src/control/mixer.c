#include "math.h"       // Math functions (e.g., sqrtf, roundf, powf)
#include "FreeRTOS.h"   // FreeRTOS core definitions (needed for task handling and timing)
#include "task.h"       // FreeRTOS task functions (e.g., vTaskDelay)
#include "supervisor.h" // Functions to check flight status (e.g., supervisorIsArmed)
#include "commander.h"  // Access to commanded setpoints (e.g., commanderGetSetpoint)
#include "estimator.h"  // Estimation framework for sensor fusion
#include "motors.h"     // Low-level motor control interface (e.g., motorsSetRatio)
#include "debug.h"      // Debug printing functions (e.g., DEBUG_PRINT)
#include "log.h"        // Logging utilities to send data to the CFClient

// Physical constants
static const float pi = 3.1416f; // Mathematical constant
static const float g = 9.81f;    // Gravitational acceleration [m/s^2]
static const float dt = 0.005f;  // Loop time step [s] (5 ms -> 200 Hz)

// Quadcopter parameters
static const float l = 35.0e-3f;   // Distance from motor to quadcopter center of mass [m]
static const float m = 38.6e-3f;   // Mass [kg]
static const float Ixx = 20.0e-6f; // Moment of inertia around x-axis [kg.m^2]
static const float Iyy = 20.0e-6f; // Moment of inertia around y-axis [kg.m^2]
static const float Izz = 40.0e-6f; // Moment of inertia around z-axis [kg.m^2]

// Actuators
float pwm1, pwm2, pwm3, pwm4; // Motors PWM

// System inputs
float ft;                     // Thrust force [N]
float tx, ty, tz;             // Roll, pitch and yaw torques [N.m]

// Get reference setpoints from commander module
void reference()
{
    // Declare variables that store the most recent setpoint and state from commander
    static setpoint_t setpoint;
    static state_t state;

    // Retrieve the current commanded setpoints and state from commander module
    commanderGetSetpoint(&setpoint, &state);

    // Extract position references from the received setpoint
    ft =  roundf((setpoint.position.z) * 2.0f) / 100.0f;    // Thrust force command [N] (maps 0.5m -> 0.01N)
    tx = -roundf((setpoint.position.y) * 2.0f) / 1000.0f;   // Roll torque command [N.m] (maps 0.5m -> 0.001N.m)
    ty =  roundf((setpoint.position.x) * 2.0f) / 1000.0f;   // Pitch torque command [N.m] (maps 0.5m -> 0.001N.m)
    tz = 0.0f;                                              // Yaw torque command [N.m]

    // Print debug info for the control efforts
    DEBUG_PRINT("Ft (N): %.2f | Tx (N.m): %.3f | Ty (N.m): %.3f  | Tz (N.m): %.3f \n", (double)ft, (double)tx, (double)ty, (double)tz);
}

// Compute motor commands
void mixer()
{
    // Quadcopter parameters
    static const float a2 = 5.98e-8f; // Quadratic motor model gain [s^2/rad^2]
    static const float a1 = 2.22e-4f; // Linear motor model gain [s/rad]
    static const float kl = 2.99e-8f; // Lift constant [N.s^2]
    static const float kd = 1.35e-10f; // Drag constant [N.m.s^2]

    // Compute required motor angular velocities squared (omega^2)
    float omega1 = (ft/(4.0f*l))-(tx/(4.0f*kl*l))-(ty/(4.0f*kl*l))-(tz/(4.0f*kd));
    float omega2 = (ft/(4.0f*kl))-(tx/(4.0f*kl*l))+(ty/(4.0f*kl*l))+(tz/(4.0f*kd));
    float omega3 = (ft/(4.0f*kl))+(tx/(4.0f*kl*l))+(ty/(4.0f*kl*l))-(tz/(4.0f*kd));
    float omega4 = (ft/(4.0f*kl))+(tx/(4.0f*kl*l))-(ty/(4.0f*kl*l))+(tz/(4.0f*kd));

    // Clamp to non-negative and take square root (omega)
    omega1 = sqrtf(abs(omega1));
    omega2 = sqrtf(abs(omega2));
    omega3 = sqrtf(abs(omega3));
    omega4 = sqrtf(abs(omega4));

    // Compute motor PWM using motor model
    pwm1 = a2*omega1*omega1+a1*omega1;
    pwm2 = a2*omega2*omega2+a1*omega2;
    pwm3 = a2*omega3*omega3+a1*omega3;
    pwm4 = a2*omega4*omega4+a1*omega4;
}