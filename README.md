## 1 Magnetic Levitation Systems

To design and experiment with the TEC-1 SBC for investigating the application of the Lorentz force in magnetic levitation systems, here’s an approach:

### Objective:
Explore the relationship between the magnetic field, current, and resulting forces to optimize stability and efficiency in magnetic levitation systems, such as maglev trains or levitating objects.

---

### Experiment Setup:
1. **TEC-1 SBC Setup**:
   - Use the TEC-1 system to calculate and control the Lorentz force parameters.
   - Utilize the I/O ports and programmable logic to handle sensor inputs (current and magnetic field) and outputs (PWM signals to electromagnets).

2. **Magnetic Field Generation**:
   - Connect an electromagnet to a power amplifier controlled by the TEC-1. Use the TEC-1's GPIO to modulate current via PWM.

3. **Force Measurement**:
   - Place a Hall effect sensor and a load cell near the electromagnet. The Hall sensor measures the magnetic field strength, and the load cell measures the resulting force on a levitating object.

4. **Feedback Loop**:
   - Use a PID control algorithm implemented on the TEC-1 in MINT. The control loop adjusts the current in real-time to stabilize the levitating object by keeping it at a set distance from the magnet.

5. **Data Analysis**:
   - Use MINT to:
     - Calculate Lorentz force using \( F = qvB \) for charged particles or \( F = I \cdot L \cdot B \) for current-carrying conductors.
     - Log and analyze data (magnetic field strength, current, force) to optimize levitation stability.

---

### Sudo Code Implementation:
#### Feedback Control:
``` 
Procedure Init
    Set i to 0                           // Initialize control variables
    Set target_distance to 100           // Desired levitation distance in mm
    Set kp to 10, ki to 1, kd to 1       // PID gains
    Set system_on to True                // Enable system
End Procedure

Procedure PID_Loop
    While system_on is True              // Run while system is on
        Set error to (sensor_distance - target_distance)  // Calculate error
        Set proportional to (error * kp)                  // Proportional term
        Update error_sum                                  // Accumulate error for integral
        Set integral to (error_sum * ki)                  // Integral term
        Calculate error_diff                              // Change in error for derivative
        Set derivative to (error_diff * kd)              // Derivative term
        Set pwm_out to (proportional + integral + derivative)  // Compute PWM output
        Write pwm_out to PWM                             // Update PWM
    End While
End Procedure

Procedure Stop
    Set system_on to False                // Disable system
End Procedure

Procedure Main
    Call Init                             // Initialize system
    Call PID_Loop                         // Start feedback loop
    Call Stop                             // Stop system on termination
End Procedure

```

---

### Experimental Procedure:
1. **Setup the Circuit**:
   - Use the TEC-1 to control current through the electromagnet.
   - Connect the Hall sensor and load cell to the TEC-1’s input ports for real-time data acquisition.

2. **Calibrate the Sensors**:
   - Map sensor readings to actual magnetic field strength (Tesla) and force (Newton).

3. **Run the Experiment**:
   - Place a small conductive or magnetic object within the magnetic field.
   - Observe how varying current and field strength affect levitation stability.

4. **Analyze Results**:
   - Log data in MINT arrays and calculate the relationship between  
   - Optimize the PID gains for stability and efficiency.

---

### Extensions:
- Incorporate wireless monitoring with a LoRa module for remote data logging.
- Use the TEC-1’s interrupt capabilities to handle rapid changes in magnetic field or current.

This setup allows you to explore magnetic levitation concepts while leveraging the TEC-1’s unique capabilities in control systems.


## mint code

```
// Magnetic Levitation Control Program
// Variables:
// t - target distance (0-255)
// d - current distance reading
// e - error value
// p - proportional term
// i - integral term
// v - derivative term
// o - output PWM value
// l - last error (for derivative)
// s - system state (0=off, 1=on)

:I              // Initialize
0 e! 0 i! 0 l!  // Clear error, integral, last error
100 t!          // Set target distance to 100
5 k!            // Set proportional gain
1 s!            // Enable system
;

:R              // Read sensor (mock for testing)
#00FF &         // Read and mask to 8 bits
d!              // Store in d
;

:C              // Calculate PID terms
d t - e!        // Calculate error
e k * p!        // Proportional term
i e + i!        // Update integral
e l - v!        // Calculate derivative
l e!            // Store error for next iteration
;

:O              // Output control signal
p i + v + o!    // Sum PID terms
o #FF & #80 /O  // Output to port 80, masked to 8 bits
;

:M              // Main control loop
s /T = (        // While system enabled
R C O           // Read, Calculate, Output
100()           // Delay
)
;

:H              // Halt system
0 s!            // Disable system
0 o! o #80 /O   // Zero output
;
```

1. **Code Structure and Variables**:
- Single-letter variables are used per MINT requirements (a-z lowercase)
- Main control variables:
  - t: Target distance (setpoint)
  - d: Current distance reading
  - e: Error value
  - p: Proportional term
  - i: Integral term
  - v: Derivative term
  - o: Output value
  - s: System state

2. **Function Breakdown**:

`:I` (Initialize):
- Clears error (e), integral (i), and last error (l)
- Sets target distance (t) to 100
- Sets proportional gain (k) to 5
- Enables system by setting s to 1

`:R` (Read Sensor):
- Reads sensor input and masks to 8 bits (#00FF &)
- Stores value in d variable

`:C` (Calculate):
- Calculates error (e) as difference between current and target
- Computes proportional term (p = error × gain)
- Updates integral term (i)
- Calculates derivative (v) from error difference
- Stores current error for next iteration

`:O` (Output):
- Sums PID terms into output value
- Masks output to 8 bits and sends to port 80

`:M` (Main Loop):
- Runs while system is enabled (s = true)
- Executes Read-Calculate-Output sequence
- Includes delay for stability

`:H` (Halt):
- Disables system
- Zeros output signal

3. **Improvements from Original**:
- Simplified variable usage for MINT's limitations
- Added proper error handling with masking
- Included system state control
- Added delay for stability
- Proper port output handling

4. **Practical Usage**:
To use this code:
1. Initialize the system: `I`
2. Start the control loop: `M`
3. To stop: `H`


## diagram of magnetic levitation experiment setup

These diagrams show:

1. **System Architecture**  
- Shows the logical connections between components
- Illustrates data flow through the system
- Highlights the four main subsystems: TEC-1, Sensors, Magnetic System, and Interface

2. **Physical Setup**  
- Shows the spatial arrangement of components
- Illustrates the magnetic field interaction
- Details sensor placement around the levitating object
- Shows physical connections to the TEC-1

Key Components Shown:
1. TEC-1 Computer with display and keypad
2. Electromagnet positioned above
3. Levitating object in the middle
4. Three sensors:
   - Hall effect sensor (magnetic field measurement)
   - Distance sensor (position feedback)
   - Load cell (force measurement)
5. Magnetic field lines showing the interaction
6. Control connections between TEC-1 and components



![image](https://github.com/user-attachments/assets/f8a2553d-100f-4d7d-93a7-2b1b0a054b5e)

![image](https://github.com/user-attachments/assets/680d0fd3-f761-4394-8666-d2a412014fb8)



## 2 Electromagnetic Braking


This implementation could provide insights into optimizing electromagnetic braking systems for energy-efficient, high-performance applications.

### Objective:
Design an electromagnetic braking system using the TEC-1 SBC to investigate how the Lorentz force affects braking performance in high-speed trains or electric vehicles. The focus is on improving braking force and energy efficiency by analyzing the magnetic field strength and current.

---

### Approach:
1. **Investigate Magnetic Field Dynamics**:
   - Measure and calculate the Lorentz force generated during braking.
   - Analyze the relationship between current, magnetic field strength, and braking force.

2. **Feedback Control**:
   - Implement a real-time feedback loop to optimize braking force.
   - Use sensors to monitor wheel speed and magnetic field strength.

3. **Energy Recovery**:
   - Investigate energy efficiency by analyzing induced currents in braking coils.

---

### Pseudocode:
Here’s a structured pseudocode for the TEC-1 SBC to simulate and control an electromagnetic braking system.

#### 1. **Initialization**
```plaintext
Initialize system variables:
    - Magnetic field strength (B)
    - Current (I)
    - Velocity of the object (v)
    - Braking force (F)

Set target deceleration rate (D_target)
Set PID control parameters (Kp, Ki, Kd)

Initialize sensors:
    - Speed sensor
    - Magnetic field sensor
    - Current sensor

Initialize output control:
    - PWM for electromagnets
```

#### 2. **Real-Time Data Acquisition**
```plaintext
Read sensors:
    - Velocity (v)
    - Magnetic field strength (B_actual)
    - Current (I_actual)
```

#### 3. **Calculate Lorentz Force**
```plaintext
Calculate braking force:
    F = I_actual * L * B_actual  // Lorentz force equation
```

#### 4. **Feedback Control**
```plaintext
Calculate deceleration error:
    Error = D_target - (F / Mass)

Compute PID terms:
    Proportional = Kp * Error
    Integral += Ki * Error
    Derivative = Kd * (Error - Previous_Error)

Update PWM signal:
    PWM = Proportional + Integral + Derivative
    Output PWM to electromagnet control
```

#### 5. **Energy Recovery Analysis**
```plaintext
Measure induced current in braking coils
Log recovered energy for analysis
```

#### 6. **Data Logging and Display**
```plaintext
Log sensor data:
    - Velocity, Current, Magnetic field, Braking force
Display critical values on TEC-1 display:
    - Current velocity
    - Braking force
    - Energy recovered
```

#### 7. **Shutdown**
```plaintext
If velocity = 0:
    Disable electromagnets
    Stop data acquisition
```

---

### MINT Sudo Code Example


```mint
:Init 
    0 v! 0 b! 0 i!           // Initialize velocity, magnetic field, and current
    5 target_d!              // Set target deceleration
    10 kp! 1 ki! 1 kd!       // PID gains
    /T braking!              // Enable braking system
;

:Calculate_Force
    b i * 1. f!              // F = I * L * B (assuming L=1)
;

:PID_Control
    target_d v - error!      // Calculate error
    error kp * proportional! // Proportional term
    error_sum ki * integral! // Integral term
    error_diff kd * derivative!
    proportional integral + derivative + pwm_out!
    pwm_out pwm_write!       // Adjust PWM
;

:Data_Log
    v . f .                  // Log velocity and braking force
;

:Main
    Init                     // Initialize system
    /U (                     // Unlimited loop for real-time control
        sensor_velocity_read! sensor_field_read! sensor_current_read!
        Calculate_Force
        PID_Control
        Data_Log
        v 0 = ( /F braking! ) // Stop if velocity reaches 0
    )
;
```

---

### Experiment Plan
1. **Setup the Circuit**:
   - Use the TEC-1 SBC to control and monitor electromagnets for braking.
   - Connect speed sensors, Hall sensors (for magnetic field), and current sensors to the TEC-1's I/O.

2. **Calibrate the System**:
   - Map sensor readings to real-world values (e.g., magnetic field in Tesla, current in Amperes).

3. **Conduct Experiments**:
   - Test braking performance under varying current and field strengths.
   - Measure deceleration rates and energy recovery.

4. **Analyze Data**:
   - Evaluate braking efficiency, force response, and energy recovery metrics.


## optimized MINT code 

MINT code for the electromagnetic braking system:

```
// Variables used:
// v - velocity reading
// b - magnetic field reading
// i - current reading
// f - calculated force
// e - error value
// p - proportional term
// d - derivative term
// n - integral term
// t - target deceleration
// k - proportional gain
// j - integral gain
// m - derivative gain
// o - output PWM value
// s - system state (0=off, 1=on)
// l - last error (for derivative)
// w - PWM output port (#80)

:I              // Initialize system
#80 w!          // Set PWM output port
0 v! 0 b! 0 i!  // Clear readings
0 e! 0 n! 0 l!  // Clear PID values
50 t!           // Target deceleration
5 k! 1 j! 2 m!  // Set PID gains
1 s!            // Enable system
0 o! o w /O     // Zero output
;

:R              // Read sensors
#81 /I v!       // Read velocity from port 81
#82 /I b!       // Read mag field from port 82
#83 /I i!       // Read current from port 83
;

:F              // Calculate force
b i * 100 / f!  // F = B*I (scaled by 100)
;

:P              // PID calculation
t v - e!        // Error = target - velocity
e k * p!        // Proportional term
n e + n!        // Update integral
n j * d!        // Integral term
e l - m * m!    // Derivative term
l e!            // Store error for next cycle
p d + m + o!    // Sum PID terms
o #FF & o!      // Limit to 8 bits
o w /O          // Output to PWM port
;

:D              // Display status
`V:` v .        // Show velocity
`F:` f .        // Show force
`E:` e .        // Show error
;

:M              // Main control loop
s /T = (        // While system enabled
R F P           // Read-Force-PID sequence
v . f .         // Log v and f
100()           // Delay for stability
v 0 = (         // Check if stopped
H               // If yes, halt
)
)
;

:H              // Halt system
0 s!            // Disable system
0 o! o w /O     // Zero output
`HALT`          // Display halt
;

:T              // Test sequence
I               // Initialize
M               // Run main loop
;

```



1. **Key Improvements**:
- Proper variable usage (single letters only)
- Correct MINT syntax for port I/O
- Added scaling for calculations
- Proper error handling
- System state control
- Added display functionality
- Included test sequence

2. **Function Breakdown**:

`:I` (Initialize):
- Sets up PWM port (#80)
- Clears all variables
- Sets PID parameters
- Enables system
- Ensures safe initial state

`:R` (Read Sensors):
- Reads from three input ports:
  - Port 81: Velocity
  - Port 82: Magnetic field
  - Port 83: Current
- Uses proper MINT I/O commands

`:F` (Force Calculation):
- Calculates B*I relationship
- Includes scaling to prevent overflow
- Stores in f variable

`:P` (PID Control):
- Calculates error
- Implements all three PID terms
- Properly scales and limits output
- Sends to PWM port

`:D` (Display):
- Shows key variables
- Uses proper MINT string format

`:M` (Main Loop):
- Implements control sequence
- Includes safety check
- Has stability delay
- Logs key values

`:H` (Halt):
- Safe shutdown
- Clears outputs
- Displays status

`:T` (Test):
- Complete test sequence
- Safe initialization

3. **Key Corrections from Original**:
- Proper port I/O syntax
- Correct variable scope
- Proper scaling for 16-bit math
- Added safety checks
- Improved display feedback
- Better error handling

4. **Usage**:
To use the system:
1. Run test sequence: `T`
2. For manual control:
   - Initialize: `I`
   - Start control: `M`
   - Stop system: `H`

## diagrams 
showing both the system architecture and physical setup of the electromagnetic braking system.

![image](https://github.com/user-attachments/assets/b3db59b6-e532-4b33-b6f2-9be960e074ec)

![image](https://github.com/user-attachments/assets/98570d0b-aacd-43fe-bd13-2f61e4c397f8)


These diagrams show:

1. **System Architecture**  
- Shows the logical connections between components
- Illustrates data flow through the system
- Highlights four main subsystems:
  - TEC-1 Computer
  - Sensor Inputs
  - Braking System
  - User Interface

2. **Physical Setup**  
- Shows spatial arrangement of components:
  - Rotating wheel with brake assembly
  - Three sensor positions
  - TEC-1 computer with display
- Illustrates:
  - Magnetic field lines from brake
  - Eddy current representation
  - Control connections
  - Sensor placement

Key Components:
1. **TEC-1 System**:
   - Z80 CPU processing
   - 7-segment display
   - Keypad input
   - PWM output control

2. **Sensors**:
   - Velocity sensor (Port 81)
   - Hall effect sensor (Port 82)
   - Current sensor (Port 83)

3. **Braking Mechanism**:
   - Electromagnetic brake coil
   - Rotating wheel
   - Eddy current generation
   - Magnetic field interaction

4. **Control Lines**:
   - Sensor inputs to TEC-1
   - PWM control to brake
   - Display feedback
   - User input

