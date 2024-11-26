#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/pwm.h"
#include "inc/hw_ints.h"
#include "driverlib/qei.h"
#include <math.h>

// Define parameters (system constants)
#define M 1.0      // mass of the cart (kg)
#define m 0.1      // mass of the pendulum (kg)
#define g 9.81     // gravity (m/s^2)
#define l 0.5      // length of the pendulum (m)
#define c 0.1      // damping coefficient (N·s/m)
#define b 0.1      // damping coefficient for pendulum (N·s·rad)
#define I 0.01     // moment of inertia (kg·m^2)
#define kt 0.1     // motor constant (Nm/A)
#define kb 0.1     // back EMF constant (V·s/rad)
#define Rm 1.0     // motor resistance (ohms)
#define r 0.05     // radius of motor shaft (m)
#define MAX_DUTY_CYCLE 1000  // Max PWM duty cycle value (arbitrary)

volatile int position_cart = 0;
volatile int position_pendulum = 0;
volatile int velocity_cart = 0;
volatile int velocity_pendulum = 0;
volatile float u0 = 0;   // Control input (force)

float X_des[4] = {0.0, M_PI, 0.0, 0.0};  // Desired state: [cart_pos, pendulum_angle, cart_vel, pendulum_ang_vel]
float KK[4] = {5.0, 5.0, 1.0, 1.0};  // LQR gains (example values)
// Encoder variables
signed int read_encoder(void);
int32_t position;
int32_t velocity;
int32_t rpm;
int32_t xMax = 14808;          // Maximum desired position
int32_t xDesired = 14808;      // Desired position
int32_t xOld = 0;              // Previous position
float errorOld = 0;            // Previous error for derivative
float errorSum = 0;            // Integral error term

// Motor control variables
int32_t widthAdjust;
int32_t newDuty;
signed int direction = 0;

// System control parameters
float theta, theta_dot;        // Pendulum angle and angular velocity
float x_dot;                   // Cart velocity
float theta_accel;             // Pendulum acceleration

// LQR Controller Parameters (Assumed values)
float A[4][4] = {
    {0, 0, 1, 0},
    {0, 0, 0, 1},
    {0, 0, -0.05, -0.01},
    {0, 0.10, -0.15, -0.05}
};
float B[4] = {0, 0, 0.03, 0.01};
float K[4] = {-30, -50, -2, -1};  // Example LQR gains  from simulation

// Function Prototypes
void Timer_Init(void);
void Hardware_Init(void);
void PWM_Init(void);
void QEI_Init(void);
void update_motor_control(void);
void Timer0IntHandler(void);

// Main function
int main(void)
{
    Hardware_Init();  // Hardware configuration
    PWM_Init();       // PWM initialization
    QEI_Init();       // QEI initialization
    Timer_Init();     // Timer initialization for interrupts

    while(1) {
        // Main loop for continuous control
    }
}

// Configure System Clock and peripherals
void Hardware_Init(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  // GPIO F enable for PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  // Pin for motor direction
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);   // PWM1
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);  // Motor direction
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  // QEI on Port D
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);   // QEI Module 0
}

// Timer interrupt initialization
void Timer_Init(void)
{
    uint32_t ui32Period;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ui32Period = (SysCtlClockGet() * 0.001);  // Set timer period (1 ms)
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();
    TimerEnable(TIMER0_BASE, TIMER_A);
}

// Initialize PWM for motor control
void PWM_Init(void)
{
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);  // PWM pins
    GPIOPinConfigure(GPIO_PF2_M1PWM6);  // PWM on pin PF2
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);  // Down counting mode for PWM
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);  // Set PWM clock division factor
    uint32_t ui32Load = (SysCtlPWMClockGet() / PWM_FREQUENCY) - 1;
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Load);
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);  // Enable PWM generator
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2, 2);  // Motor direction initialization
}

// Initialize QEI for encoder reading
void QEI_Init(void)
{
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);  // Configure QEI pins for cart encoder
    GPIOPinConfigure(GPIO_PD6_PHA0);  // QEI Channel A (PHA)
    GPIOPinConfigure(GPIO_PD7_PHB0);  // QEI Channel B (PHB)
    
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);  // Configure QEI pins for pendulum encoder
    GPIOPinConfigure(GPIO_PD4_PHA1);  // QEI Channel A (PHA)
    GPIOPinConfigure(GPIO_PD5_PHB1);  // QEI Channel B (PHB)

    QEIDisable(QEI0_BASE);  // Disable QEI module before configuration
    QEIDisable(QEI1_BASE);  // Disable QEI module before configuration

    QEIIntDisable(QEI0_BASE, QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    QEIIntDisable(QEI1_BASE, QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), MAX_POSITION);  // Cart QEI
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), MAX_POSITION);  // Pendulum QEI

    QEIEnable(QEI0_BASE);  // Enable QEI module for cart
    QEIEnable(QEI1_BASE);  // Enable QEI module for pendulum

    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, 32000);  // Configure cart velocity capture
    QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, 32000);  // Configure pendulum velocity capture
    QEIVelocityEnable(QEI0_BASE);  // Enable cart velocity capture
    QEIVelocityEnable(QEI1_BASE);  // Enable pendulum velocity capture
}

// Encoder reading function
// map to get position and velocity readings
// signed int read_encoder(void)
// {
//     position = QEIPositionGet(QEI0_BASE);  // Get position from encoder
//     // map to get x and x_Dot
//     if (position > MAX_POSITION / 2) {
//         return -(MAX_POSITION - position);
//     } else {
//         return position;
//     }
// }

// Saturation function
float sat(float x, float x_max, float x_min) {
    return (x > x_max) ? x_max : ((x < x_min) ? x_min : x);
}

// signed int read_encoder_pendulum(void)
// {
//     position_pendulum = QEIPositionGet(QEI1_BASE);  // Get pendulum position (theta) from encoder
//     velocity_pendulum = QEIVelocityGet(QEI1_BASE);  // Get pendulum velocity (theta_dot) from encoder
//     return position_pendulum;
// }

// Main control loop (ISR-driven)
void Timer0IntHandler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);  // Clear interrupt flag

    // Read encoder values
    position_cart = QEIPositionGet(QEI0_BASE);       // Get cart position
    // map to x
    position_pendulum = QEIPositionGet(QEI1_BASE);   // Get pendulum position
    velocity_cart = QEIVelocityGet(QEI0_BASE);       // Get cart velocity
    //map to v
    velocity_pendulum = QEIVelocityGet(QEI1_BASE);   // Get pendulum velocity

    // Convert encoder readings to physical values
    float theta = (position_pendulum * 2 * M_PI) / MAX_POSITION;  // Pendulum angle in radians
    float theta_dot = (velocity_pendulum * 2 * M_PI) / MAX_POSITION;  // Pendulum angular velocity
    float x_dot = velocity_cart;  // Cart velocity

    // Compute new state with RK4 (if needed) or directly use values
    float X[2] = {position_cart, theta};
    float X_dot[2] = {x_dot, theta_dot};
    
    // Use the Inverted_Pendulum2ode function to calculate accelerations (dxdt)
    // float dxdt[2];
    // Inverted_Pendulum2ode(X, X_dot, u0, M, m, g, l, c, b, I, dxdt);

    // // Extract accelerations (x_ddot, theta_ddot)
    // float x_ddot = dxdt[0];
    // float theta_ddot = dxdt[1];

    // Energy-based swing-up control
    float E = m * g * l * (1 - cos(theta)) + (0.5) * (I + m * l * l) * (theta_dot * theta_dot);
    float Er = 2*m*g*l; // Reference energy, could be tuned based on the system
    float accel = 2 * (E - Er) * sign(theta_dot * cos(theta));
    accel = k_swing * g * sat(accel, n * g, -n * g);  // Swing-up control

    // Feedback Linearization control for pendulum stabilization
    float u_swing = (M + m) * accel + 0 * x_dot - m * l * ((theta_dot * theta_dot) * sin(theta)) - m * l * cos(theta) * ((b * theta_dot + m * l * accel * cos(theta) + m * g * l * sin(theta)) / (I + m * l * l));

    // LQR control law
    float u_lqr = -KK[0] * (position_cart - X_des[0]) - KK[1] * (theta - X_des[1]) - KK[2] * (x_dot - X_des[2]) - KK[3] * (theta_dot - X_des[3]);
    u_lqr = sat(u_lqr, 12, -12);  // Saturation for voltage limits

    // Convert LQR control voltage to force using motor constants
    float u_volt = volt2force(u_lqr, x_dot, kt, kb, Rm, r);

    // Control switching logic based on pendulum position
    if (abs(X_des[1] - theta) * (180 / M_PI) <= 30) {  // If pendulum is close to vertical
        u0 = u_lqr;  // Use LQR control
    } else {
        u0 = u_swing;  // Use swing-up control
    }

    // Apply control force to motor (set PWM duty cycle)
    update_motor_control();
}

// Function to apply control to the motor
void update_motor_control(void) {
    // Convert control force to PWM duty cycle (scaling by motor characteristics)
    int newDuty = (int32_t)(u0 * MAX_DUTY_CYCLE / 12);  // Scale based on voltage limits (12V max)
    newDuty = sat(newDuty, MAX_DUTY_CYCLE, 0);  // Saturate to [0, MAX_DUTY_CYCLE]

    // Set the PWM duty cycle (write value to PWM module)
    PWMDutySet(PWM1_BASE, PWM_OUT_6, newDuty);  // Apply PWM control to motor
    int direction = (newDuty >= 0) ? 1 : -1;  // Determine motor direction
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2, direction);  // Set motor direction
}

// // Function to compute accelerations using the inverted pendulum equations of motion
// void Inverted_Pendulum2ode(float* X, float* X_dot, float u, float M, float m, float g, float l, float c, float b, float I, float* dxdt) {
//     float theta = X[1];
//     float x_dot = X_dot[0];
//     float theta_dot = X_dot[1];

//     // Compute forces and accelerations based on the system equations
//     float F = u;
//     float alpha_a = (m * m * l * l * sin(theta) * sin(theta) + M * m * l * l + (M + m) * I);
    
//     float x_ddot = (b * m * l * theta_dot * cos(theta) + m * m * l * l * g * sin(theta) * cos(theta) + (I + m * l * l) * (F - c * x_dot + m * l * sin(theta) * theta_dot * theta_dot)) / alpha_a;
//     float theta_ddot = -(F * m * l * cos(theta) - c * m * l * x_dot * cos(theta) + m * m * l * l * theta_dot * theta_dot * sin(theta) * cos(theta) + (M + m) * (b * theta_dot + m * g * l * sin(theta))) / alpha_a;
    
//     dxdt[0] = x_ddot;
//     dxdt[1] = theta_ddot;
// }

// Function to convert voltage to force (for motor control)
float volt2force(float v, float x_dot, float kt, float kb, float Rm, float r) {
    return (kt * v * r - kt * kb * x_dot) / (Rm * r * r);
}
