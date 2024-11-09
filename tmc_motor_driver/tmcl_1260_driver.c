//------------------------------------------------------------------------------
// Project  : Ventilator Control System
// File     : tmcl_1260_driver.tmcl
// Brief    : TMCL driver for Trinamic stepper motor control
// Author   : Nicholas Antoniades
// Date     : 2020
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Global Variable Definitions
//------------------------------------------------------------------------------
// System State Variables
systemState = 0           // Current system state
cycleStage = 8           // Current stage of breathing cycle (0=exp, 1=insp)

// Motion Control Variables
velocityInsp = 1         // Inspiration velocity
velocityExp = 2          // Expiration velocity
accelMax = 3            // Maximum acceleration
currentMax = 4          // Maximum current (0-256)
decelMax = 5            // Maximum deceleration

// Position Variables
positionInsp = 6        // Inspiration target position
positionExp = 7         // Expiration target position

// System Constants
POSITION_FLATPLATE = 45000
POSITION_DOME = 38000

//------------------------------------------------------------------------------
// Global Parameter Configuration
//------------------------------------------------------------------------------
// Initialize system state variables in Bank 2 (User Variables)
SGP systemState, 2, 0
SGP velocityInsp, 2, 20000
SGP velocityExp, 2, 25000
SGP accelMax, 2, 7629278
SGP currentMax, 2, 200
SGP decelMax, 2, 7629278
SGP positionInsp, 2, 0
SGP positionExp, 2, -POSITION_DOME
SGP cycleStage, 2, 0

// Store parameters in EEPROM for persistence across power cycles
STGP systemState, 2
STGP velocityInsp, 2
STGP velocityExp, 2
STGP accelMax, 2
STGP currentMax, 2
STGP decelMax, 2
STGP positionInsp, 2
STGP positionExp, 2
STGP cycleStage, 2

//------------------------------------------------------------------------------
// Motor Configuration
//------------------------------------------------------------------------------
// Update axis parameters from global variables
GGP velocityExp, 2       // Load expiration velocity
AAP 4, 0                // Set maximum velocity
GGP accelMax, 2         // Load maximum acceleration
AAP 5, 0                // Set acceleration
GGP currentMax, 2       // Load maximum current
AAP 6, 0                // Set maximum current
GGP decelMax, 2         // Load maximum deceleration
AAP 17, 0               // Set deceleration

// Set coordinate positions
GGP positionInsp, 2     // Load inspiration position
ACO 1, 0                // Set inspiration coordinate
GGP positionExp, 2      // Load expiration position
ACO 2, 0                // Set expiration coordinate

//------------------------------------------------------------------------------
// Initial Axis Configuration
//------------------------------------------------------------------------------
SAP 1, 0, 0             // Set actual position to zero
SAP 16, 0, 0            // Disable acceleration/deceleration phase for position control
SGP 77, 0, 1            // Enable automatic program execution on startup

//------------------------------------------------------------------------------
// StallGuard Configuration
//------------------------------------------------------------------------------
stallGuardValue = 2      // StallGuard threshold (-64=highest sensitivity, 63=lowest)

SAP 173, 0, 1           // Disable StallGuard filter
SAP 174, 0, stallGuardValue  // Set StallGuard threshold
SAP 181, 0, 40000       // Set minimum speed for StallGuard stop

// Configure StallGuard interrupt
VECT 15, returnToZero   // Set interrupt vector for StallGuard
EI 15                   // Enable StallGuard interrupt
EI 255                  // Enable global interrupt processing

//------------------------------------------------------------------------------
// State Machine Implementation
//------------------------------------------------------------------------------

// State 0: Homing Sequence
state0:
    // Disable limit switches during homing
    SAP 12, 0, 1        // Disable right limit switch
    SAP 13, 0, 1        // Disable left limit switch

    // Configure and execute reference search
    SAP 193, 0, 65      // Set reference search mode (right switch)
    SAP 194, 0, 16000   // Set reference search speed
    SAP 195, 0, 14000   // Set reference switch speed
    RFS START, 0        // Begin reference search
    WAIT RFS, 0, 0      // Wait for completion
    SGP cycleStage, 2, 0 // Set to expiration stage

    // Monitor for state change
    loop0:
        GGP systemState, 2   // Get current state
        COMP 1              // Compare with state 1
        JC EQ, state1       // Jump if equal
        JA loop0            // Otherwise continue monitoring

    JA state0               // Loop back to start of state 0

// State 1: Normal Operation
state1:
    // Check for return to homing state
    GGP systemState, 2
    COMP 0
    JC EQ, state0

    // Update target positions from memory
    GGP positionInsp, 2
    ACO 1, 0
    GGP positionExp, 2
    ACO 2, 0

    // Inspiration phase
    SGP cycleStage, 2, 1    // Set inspiration stage
    GGP velocityInsp, 2     // Load inspiration velocity
    AAP 4, 0                // Set velocity
    SIO 0, 2, 1             // Signal inspiration phase
    MVP COORD, 0, 1         // Move to inspiration position
    WAIT POS, 0, 0          // Wait for completion

    // Check for state change
    GGP systemState, 2
    COMP 0
    JC EQ, state0

    // Expiration phase
    SGP cycleStage, 2, 0    // Set expiration stage
    GGP velocityExp, 2      // Load expiration velocity
    AAP 4, 0                // Set velocity
    SIO 0, 2, 0             // Signal expiration phase
    MVP COORD, 0, 2         // Move to expiration position
    WAIT POS, 0, 0          // Wait for completion

    JA state1               // Loop back to start of state 1

//------------------------------------------------------------------------------
// Interrupt Handlers
//------------------------------------------------------------------------------

// StallGuard interrupt handler
returnToZero:
    GGP cycleStage, 2       // Get current cycle stage
    COMP 0                  // Compare with expiration stage
    JC EQ, state0          // Return to homing if in expiration
    RETI                    // Return from interrupt

//------------------------------------------------------------------------------