## Magnetic Levitation Systems

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

### Code Implementation:
#### MINT Program for Feedback Control:
```mint
:Init 
    0 i!                  // Initialize control variables
    100 target_distance!  // Desired levitation distance in mm
    10 kp! 1 ki! 1 kd!    // PID gains
    /T system_on!         // Enable system
;

:PID_Loop
    system_on? /T = (    // Run while system is on
        sensor_distance? target_distance - error!  // Calculate error
        error kp * proportional!                  // Proportional term
        error_sum ki * integral!                  // Integral term
        error_diff kd * derivative!               // Derivative term
        proportional integral + derivative + pwm_out!
        pwm_out pwm_write!                        // Update PWM
    )
;

:Stop
    /F system_on!  // Disable system
;

:Main
    Init           // Initialize system
    /U PID_Loop    // Start feedback loop
    Stop           // Stop system on termination
;
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
   - Log data in MINT arrays and calculate the relationship between \( I \), \( B \), and \( F \).
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

1. **System Architecture** (Mermaid Diagram):
- Shows the logical connections between components
- Illustrates data flow through the system
- Highlights the four main subsystems: TEC-1, Sensors, Magnetic System, and Interface

2. **Physical Setup** (SVG Diagram):
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


