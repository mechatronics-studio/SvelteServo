# The USB Motion Platform

## Introduction

I started the USB Motion Platform because I feel that making hobbyist motion systems is far more expensive and complicated than it needs to be. As a mechatronics engineer and specialist in servo based robotics, I hope that this platform can help teach and empower the robotics community. 

Stepper motors are the typical go-to choice. They do provide a solution but are open loop and their dynamic performance is poor compared conventional servos.

Today's most accessible servo solutions are really not that accessible:
* Clearpath provides an industrial class all-in-one integrated servo motor but is expensive ($250 for the lowest cost motor) and requires additional low level hardware to interface to and power. 
* ODrive provides a small form factor controller but remains extremely expensive ($230 for the controller alone) and utilizes proprietary firmware. Additional low level hardware is still required to interface to the controller.

Both of these vendors provide very capable solutions, especially for industrial or 'introductory industrial', but none of them fit into the vision I have for hobbyist robotics.

The USB Motion Platform has a simple goal; to make motion control as simple as plugging in power and a USB port. Open Source hardware, firmware, and software form the backbone of the platform and the MIT license permits for wide use and application with little restriction.

The USB Motion Platform is made up of USB Motion Devices, which consists of a force/torque generating decice (like a motor) and a motion processor. Multiple USB Motion Devices can be connected to a host device that issues commands and coordinates complex multi-axis motion. 

At this time, the USB Motion Platform exclusively supports brushed DC Motors with two channel incremental quadrature encoders. Support for future technology (brushless DC motors, absolute encoders, etc.) may come at a future time.

## External USB Motion Device Connections

Power is provided by a simple barrel connector (5.5 OD x 2.6 ID) while data is provided by a USB 1.1 (Full Speed) serviced by a USB C connector.

Using a barrel connector for power provides a high amount of flexability with regards to the power source for the motor. Cheap DC supplies, variable power supplies, and even batteries are simple to wire.

USB-C connectors are reversable and the ubiquitous standard for modern data transfer. 

## Computational Hardware

Due to its speed, peripheral selection, affordability, availability, and open SDK, the RP2040 processor from the Rasberry PI Foundation is a natural choice. The following hardware features of the RP2040 make it well suited for this application:

1. 133 Mhz, dual core for task distribution
2. Programmable IO for quadrature decoding
3. USB Full Speed Built-In
4. 256k ram, large for a processor in this market position
5. A powerful and open c build platform
6. Lighting fast integer mathematics implementation

Core0 of the RP2040 is assigned to executive functions, namely USB Communication and trajectory planning. 

Core1 of the RP2040 is assigned to updating the main servo control loop, solving a fixed time differential equation of a simulated servo motor, and passing servo control diagnostics data back to Core0.

The RP2040 does not possess a floating point arithmatic units (FPU), therfore, all real-time mathematics is done in scientific notation integer format.

## Scientific Notation Integer Mathematics

Scientific notation integer mathematics is quite simple and utilizes fixed point arithmatic. If you have ever done a calculation in a manner like $4820133023*2 = 482*10^7*2=964*10^7$, you will have an appreciation for this kind of arithmetic.

Numbers are stored as two parts, a coefficient $C$ and an exponent $E$:

$$ 0.0004921392 = \underbrace{4921}_C * 10^{\overbrace{-7}^E}$$
$$ 4921392 = \underbrace{4921}_C * 10^{\overbrace{3}^E}$$

During multiplication, the coefficients are multiplied together and the exponents are added:

$$892837 * 0.0928182 = 8928*10^2 * 9281*10^{-5} = 82860768*10^{-3} =  8286*10^{1}$$

A floating point calculation would yield $82871.5232334$ yielding an error of 0.8% certainly reasonable if we intellegently choose our variables.

For perspective, the Raspberry Pi Foundation has documented that floating point multiplication takes 61.5us, which corresponds to 8,180 cpu cycles at 133 Mhz. In contrast, RP2040s integer addition/subtration take just a single clock cycle while multiplication and division take less than ten. All things considered, even if we utilized 100 cpu cycles for all the integer operations, we would still be over 80x faster than using a floating point operation. For this reason, at the expense of readability, scientific notation integer arthimetic is riddled throughout the firmware. 

The number of significant digits must be carefully chosen to support the accuracy requirements and the datatype.

### **Scientific Integer Notation Multiplication**

Given two numbers represented by, $N_1 = C_1*10^{E_1}$, and $N_2 = C_2*10^{E_2}$, their multiplication $N_3 = N_1 * N_2$ is given by the following algorithm.

Multiply their coefficients:

$$A = C_1 * C_2$$

This multiplication will result in a number with single digit scientific notation exponent equal to $2X-2$ or $2X-1$ where $X$ is the number of digits in the coefficient. For example, 4 coefficents would yield a minimum of $1,000,000$ and a maximum of $99,980,001$). We next check whether $A$ needs to be divided by $10^{X-1}$ or $10^X$ to put it's coefficient in alignment. We also start the exponent:

IF $A > 10^{2X-1}$:
  $$C_3 = \frac{A}{10^{X}},\ B = X$$
ELSE:
  $$C_3 = \frac{A}{10^{X-1}},\ B = X-1$$

Then, we add the exponents of both $N_1$ and $N_2$ to $E_3$:

$E_3 = B + E_1 + E_2$

The process of floating point multiplication has been aproximated with one integer multiplication operation, one if/else statement, one integer division operation, and up to three integer addition/subtraction operations. I estimate this takes less than 50 cpu cycles, for a 163x speedup vs floating point arithmetic.

*Usage Notes:* Greater than four digits in the coefficient can potentially overload a 32-bit signed integer while greater than nine significant digits can potentially overload a 64-bit signed integer.

### **Scientific Notation Integer Division**

Given two numbers represented by, $N_1 = C_1*10^{E_1}$, and $N_2 = C_2*10^{E_2}$, their division $N_3 = \frac{N_1}{N_2}$ is given by the following algorithm.

Multiply the coeffficent of $N_1$ by $10^X$ where $X$ is the number of digits in the coefficient and subtract $X$ from the exponent $E_1$:

$$A = C_1*10^X,\ B = E_1 - X$$

Perform integer division between $A$ and $C_2$

$$D = \frac{A}{C_2}$$

The result is guaranteed to to have a single digit scientific notation exponential equal to the source number's coefficients unless $C_2$ is equal to $10^{X-1}$ (1 with all leading zeros) where $X$ is the number of digits in the source number's coefficients.

IF $C_2$ > $10^{X-1}$
$$C_3 = D,\ F = B + E_1 + E_2$$
ELSE
$$C_3 = \frac{D}{10},\ F = B + 1 + E_2 + E_2$$

The process of floating point division has been approximated with one integer multiplication operation, up to two integer division operation, one if/else statement, and up to four integer addition/subtraction operations. The Raspberry Pi foundation has published a floating point division operation time of 74.7us, or 9935 cycles. I estimate that this takes less than 60 cpu cycles for a 165x speedup vs floating point arithmetic.

### **Scientific Notation Integer Addition**

Given two numbers represented by, $N_1 = C_1*10^{E_1}$, and $N_2 = C_2*10^{E_2}$, their addition $N_3 = N_1 + N_2$ is given by the following algorithm.

Check if one of the numbers is significantly smaller than the other number such that adding the two numbers together would not change the result in scientific notation. This would occur if one number's exponent is different from the other number greater than or equal to the number of digits in the coefficient $X$. If either of the below statements are true, one of the inputs is chosen and the addition is complete:

$$ A = E_1 - E_2 $$

IF $A \ge X$

$C_3 = C_1, E_3 = E_1$

ELSE IF $A \le - X$

$C_3 = C_2, E_3 = E_2$

If both of these conditions fail, then the addition is meaningful. The coefficients must be leveled to perform addition:

IF $A > 0$

$C_1 = C_1*10^A$

$D = C_1 + C_2$

$C_3 = \frac{D}{10^A}$

$E_3 = E_1$

ELSE IF $A < 0$

$C_2 = C_2*10^{-A}$

$D = C_1 + C_2$

$C_3 = \frac{D}{10^{-A}}$

$E_3 = E_2$

The process of floating point addition has been approximated with up to two else/if statements, one integer multiplication, one integer addition, and one integer division. The Raspberry Pi foundation has published a floating point division operation time of 72.4us, or 9642 cycles. I estimate that this takes less than 50 cpu cycles for a 160x speedup vs floating point arithmetic.

*USAGE NOTES:* The addition problem must be well suited to the scientific notation integer addition operation. Attempting to add small numbers to large numbers will never yield anything different than the large number. Any numbers smaller than the significant digits will be lost during the operation.

### **Scientific Notation Integer Subtraction**

Given two numbers represented by, $N_1 = C_1*10^{E_1}$, and $N_2 = C_2*10^{E_2}$, their subtraction $N_3 = N_1 - N_2$ is given by the following algorithm.

While addition of an arbitarily small number to a large number will only yield a different answer if the small number is large enough to "roll up" the least significant digit of the large number, subracting an near arbitrarily small scientifically notated number from a large scientifically notated number will result in a "roll down" of the large number's least significant digit. This behavior is artificial, undesirable, and controlled by checking for the same significance condition applied for addition.

If one number's exponent is different from the other number by an amount greater than or equal to the number of digits in the coefficient $X$, the larger magnitude input is chosen, the sign is compensated for, and the subtraction is complete:

$$ A = E_1 - E_2 $$

IF $A \ge X$

$C_3 = C_1, E_3 = E_1$

ELSE IF $A \le - X$

$C_3 = -C_2, E_3 = E_2$

If both of these conditions fail, then the subtraction is meaningful. The coefficients must be leveled to perform addition:

IF $A > 0$

$C_1 = C_1*10^A$

$D = C_1 - C_2$

$C_3 = \frac{D}{10^A}$

$E_3 = E_1$

ELSE IF $A < 0$

$C_2 = C_2*10^{-A}$

$D = C_1 - C_2$

$C_3 = \frac{D}{10^{-A}}$

$E_3 = E_2$

The process of floating point subtraction has been approximated with up to two else/if statements, one integer multiplication, one integer subtraction, and one integer division. The Raspberry Pi foundation has published a floating point division operation time of 72.4us, or 9642 cycles. I estimate that this takes less than 50 cpu cycles for a 160x speedup vs floating point arithmetic.

*USAGE NOTES:* The subtraction problem must be well suited to the scientific notation integer subtraction operation. Attempting to subtract small numbers from large numbers will never yield anything different than the large number. Subtracting large numbers from small numbers will effectively multiply the large numbers by $-1$. Any numbers smaller than the significant digits will be lost during the operation.

## High Level Programmatic Structures

A number of names are used in this document which refer to specific programmatic structures within the firmware. These are described in this section.

### **Motion Queue**

The motion queue is an instance of a thread data transfer queue built in the RP2040 C SDK. The motion queue is used by core0 to transmit the recieved desired position/velocity data to core1.

Each item in the motion queue is a single 32-bit value. The number of items in the motion queue is hardcoded in the firmware and is set in the default trunk to 1000.

### **RAM Variables**

RAM Variables are initialized from hard coded values and correspond to USB Motion Device parameters that change the device operation during runtime.

All parameters are owned by core0 and are freely accessible and modifiable by host commands.

Changing most of these values will temporarily halt motion control. While some paramters ouright disable control or modify the control method, other parameters are utilized by core1 in high speed loops and their arbitrary modification by core0 could result in memory access races. When core0 recieves a host command to modify a paramter used by core1, the core0 copy is modified and core0 makes a request to core1 to modify its internal copy. At this point, core1 stops executing its high speed loops, copies the modified parameter to its internal copy, and resumes its high speed loops.

*servo_update_interval_us*- The number of microseconds between updates of the main servo loop. (default $1000$)

*motion_update_interval_scalar*- Describes the number of main servo loops that occur before updating the desired position/velocity from the motion queue. (default $1$)

*control_mode*- An enumerated value that describes which control mode to use. (default $1$)

*servo_enable*- Describes whether to enable the servo loop. $1$ for true, $0$ for false. (default $0$)

NEED TO CREATE COMMANDS FOR BELOW

*proportional_gain*- Up to 3 significant digits

*derivative_gain*

*integral_gain*

simulated_servo_update_interval_dms

simulated_source_voltage

simulated_servo_torque_constant_mNm_per_ampere

simulated_servo_winding_resistance_ohms

simulated_servo_counts_per_revolution

simulated_servo_inertia_kg_m^2



### **Flash Variables**

At the moment, the only flash variable is the device name which is a string up to 25 characters (including spaces). The device name is stored at the back end of the flash chip.

As it is written to flash, this variable persists during a reset. This variable is independent of firmware updates. If the flash is empty, the firmware will default the device name to "GENERIC USB MOTION DEVICE". The name can be used by the host to identify the USB Motion Device and assign different behaviors. For example a device named "ROBOT LEFT LEG" likely has a different sequence of commanded positions than "ROBOT RIGHT ARM".

Flash Variables are loaded into RAM at startup. Flash variables can be modified by an appropriate call from the host. Flash variables should not be changed frequently as flash has a limited number of write cycles (typically a few tens of thousands) before degredation.

## Startup

Upon startup, the current position is assumed to be $0$ and the hard coded RAM Variables are loaded. 

Typically, the host will send a short startup command sequence to configure the RAM variables for the application.

## Control Modes

Different control modes permit for various methods of motion control. An RAM variable called *control_mode* enumerates which control mode is active from the below list:

1. Absolute Position Mode, Counts
1. Absolute Velocity Mode, Counts


[//]: # "* Absolute Position Mode, Output Units"
[//]: # "* Absolute Velocity Mode, Output Units"
[//]: # "* Step/Direction Mode"

A flash variable named *default_control_mode* sets a default control_mode when the USB Motion Device is restarted.

### **(1)  Absolute Position Mode, Counts**

In the counts absolute position mode, the host sends a list of positions in counts to the USB Motion Device. Each of these positions is piled into the motion queue.

The desired and actual counts is stored in a 32 bit signed integer. The maximum storable actual and desired counts are $\pm 2,147,483,647$. At a motor speed of 5000RPM with a 4000 counts per revolution quadrature encoder, this corresponds to 107 minutes of continuous revolution before rolling over. If higher durations of continuous motion are needed, the absolute velocity mode is likely a more appropriate candidate for the application.

At the position update frequency, core1 removes positions from the motion queue and updates the desired positions for the main control loop.

The position update period is scaled from the servo update period.

$$T_p = motion\_update\_interval\_scalar*servo\_update\_interval\_us $$
 
### **(2) Absolute Velocity Mode, Counts**

In the counts absolute velocity mode, the host sends a list of velocities in counts/second to the USB Motion Device. Each of these velocites is piled into the motion queue.

At the velocity update frequency, core1 removes velocities from the motion queue and updates the desired velocities for the main control loop.

In velocity control mode, the hardware quadrature counter is reset to $0$ every $60$ seconds.

The velocity update period is scaled from the servo update period.

$$T_v = motion\_update\_interval\_scalar*servo\_update\_interval\_us $$

## Closed Loop Control Algorithm

USB Motion Devices are equipped with a Proportional Integral Derivative (PID) controller with input ceiling.

INSERT PICTURE.

## Communication Protocol

Commands are sent asynchronously into buffers via an unencrypted USB serial port.

* Commands are structured in two parts, an executive call followed by supporting inputs. 

* All executive calls are integers and interpreted in a switch/case jump table.

* All supporting inputs are numeric, typically integers unless otherwise indicated.

* The executive call interpreter expects the exact correct number of supporting inputs. To ensure maximum interpretation rate, there is no error checking or validation on commands within USB Motion Devices. All error checking or validation must be done on the host device.

* Each executive call possesses a debug flag which is set to an integer. Debug flag values other than $0$ cause a particular output to be returned from the core0 processor via USB when that executive call is sent to the device. Each executive call has it's own behavior for certain flag values and some executive calls might have multiple debug flags available. Any debug flag value which is not defined is equivalent to $0$. It is possible to set many debug flags (though only one per executive call), which will have the effect of clogging up the console with a lot of data. For clarity, only the debug flags that are needed should be enabled.

### **Executive Call 0: Add Single Integers to the Motion Buffer**

This executive call has one supporting input, an integer value to be added to the motion buffer.

    0 3910

If the motion buffer is full, the integer will not be added to the motion buffer and

    E0
will be returned.

If the debug flag for this executive call is set to $1$

    S0 X \n
    
will be returned indicating that the integer X was added to the motion buffer.


### **Executive Call 1: Add Multiple Integers to the Motion Buffer**

This executive call's first supporting input specifies the number of integers to be added to the motion buffer. The next supporting inputs are the integers to be added to the motion buffer.

Example 1:

    1 1 3928

Example 2:

    1 3 4929 2904 103049

If the motion buffer is too full to hold all of the integers to be added none of the integers will be added to the motion buffer and

    E1
will be returned.

If the debug flag for this executive call is set to 1.

    S0 Y X1 X2 X3 ... \n
    
will be returned indicating that $Y$ integers were added to the motion buffer and each integer $X1$, $X2$, etc. was added to the motion buffer.

### **Executive Call 3: NEED TO POPULATE**

### **Executive Call 3: Set Servo Enable**

This call modifies the RAM value *servo_enable*, which enables or disables the main servo loop. Changing this value halts all servo execution, modifies the core0 copy of *servo_enable*, and then if servo_enable is $1$, begins it again.

This executive call accepts a single suporting argument which is whether the servo is enabled or not. The below example enables the servo loop.

    3 1

If the supporting argument is $-1$, then the *servo_enable* is queried instead of being set and the following is returned:

    SERVO_ENABLE X \n

where $X$ is the value of *servo_enable*.

If the debug flag for this executive call is set to 1

    SERVO ENABLE MODIFIED TO X \n

will be returned indicating that the servo has been enabled or disabled.

### **Executive Call 4: NEED TO POPULATE**

### **Executive Call 5: Change Servo Loop Interval**

This call modifies the RAM variable value *servo_update_interval_us*, which sets the period of the main servo loop in microseconds, $dt$. Changing this value halts all servo execution, clears the motion buffer, modifies the core0 and core 1 copy of *servo_update_interval_us*, and then if servo_enable is $1$, begins it again.

This executive call accepts a single supporting argument which is the desired servo loop interval in decimilliseconds(tenths of a millisecond). The below example sets the servo loop interval $dt$ to 10 decimilliseconds, which is one millisecond.

    5 10

The frequency of the servo control loop is given by:

$$f_s = \frac{1\ \  servo loop}{dt \ \ decimilliseconds}(\frac{10000\ \  decimilliseconds}{1\ \  second}) = \frac{10000}{dt} \frac{servoloops}{second}$$

From the example command issued above, this would yield

$$f_s = \frac{10000}{10} = 1000 \frac{servoloops}{second} = 1\ kHz$$

It can be discovered than that the minimum Servo Loop Interval is 1 decimilliseconds, which would correspond to a servo update frequency of:

$$f_s = \frac{10000}{1} = 10000 \frac{servoloops}{second} = 10\ kHz$$

If the supporting argument is $-1$, then the *servo_update_interval_us* is queried instead of being set and the following is returned:

    SERVO_UPDATE_INTERVAL_US X \n

where $X$ is the value of *servo_update_interval_us*.

If the debug flag for this executive call is set to 1

    SERVO LOOP INTERVAL MODIFIED TO X MICROSECONDS \n

will be returned indicating that the loop interval has been modified to $X$ microseconds.

### **Executive Call 6: Change Motion Update Interval Scalar**

This call modifies the RAM variable value *motion_update_interval_scalar*, which sets the number of main servo loops that are called before the next integer is removed from the motion buffer. Changing this value halts all servo execution, clears the motion buffer, modifies the core0 and core 1 copy of *motion_update_interval_scalar*, and then if servo_enable is $1$, begins it again.

This executive call accepts a single supporting argument which is the desired number of main servo loops to be called before the next integer is removed from the motion buffer. The below example sets the motion_update_interval_scalar 1000, which if the main servo loop interval was 1 millisecond, would correspond to a 1 second update interval of the position.

    6 1000

If the supporting argument is $-1$, then the *motion_update_interval_scalar* is queried instead of being set and the following is returned:

    MOTION_UPDATE_INTERVAL_SCALAR X \n

where $X$ is the value of *motion_update_interval_scalar*.

If the debug flag for this executive call is set to 1

    MOTION UPDATE INTERVAL SCALAR MODIFIED TO X \n

will be returned indicating that the motion update interval scalar has been modified to $X$.

### **Executive Call 7: Change Motion Control Mode**

This call modifies the RAM variable value *control_mode*, which determines the state variable to be controlled. Changing this value halts all servo execution, clears the motion buffer, changes the controller function pointer to the appropriate controller, modifies the core0 copy of *control_mode*, and then, if servo_enable is $1$, begins it again.

This executive call accepts a single supporting argument which is the desired control mode. The below example sets the *control_mode* to 0:

    7 0


If the supporting argument is $-1$, then the *control_mode* is queried instead of being set and the following is returned:

    CONTROL_MODE X \n

where $X$ is the value of *control_mode*.

If the debug flag for this executive call is set to 1

    Control Mode MODIFIED TO X \n

will be returned indicating that the control mode has been modified to $X$.



### **Executive Call 8**

This executive call has two supporting input, an integer referencing an executive call to enable debugging output on and the flag to trigger for the executive call.

    6 0 1

## Time Synchronization

USB Motion Devices can be synchronized by zeroing the internal clock. This permits for 

Alternatively, an external clock line can be used that triggers the servo update loop. This requires another connection to the device but ensures that all linked USB Motion Devices act simultaneously.

## FAQ

*Why not use USB Power Delivery instead of a seperate power source?*

    Even though USB Power Delivery can deliver the voltages and current to power a motor, it is a complex protocol that requires devices to negotiate with power sources (wall bricks, laptops, etc.). Few computational devices are equipped to power a motor while power bricks are incapable of providing data to the controller. At this time, it just makes more sense to provide flexability and simplicity with two seperate cables than to combine them into one.

*Why use the MIT License?*

    Because.

*Is this platform secure? Is it safe?*

    The platform is safe and secure in the sense that once you undersand it's weaknesses it can be used safely and securly. The RP2040 device runs a firmware pogram stored on an off board flash chip which can be easily read and sabotaged. USB Communication occurs over an unencrypted communication line. The host is most typically a regular computer which can be subject to reboots and viruses. Power is provided over a simple voltage bus that can be unplugged or tampered with.
    
    You should assume that any physical or digital access by malicious individuals compromises the integrity of whatever machine was built using USB Motion Devices. The only method of ensuring security is to construct a physically isolated system that has no possible routes of communication to the outside world.
    
    As they are not safety rated, USB Motion Devices have many single point failure modes and should NEVER be used in safety critical applications. 

    This section is not meant to scare you away, only inform. You must evaluate the suitability of USB Motion Devices for your own application. Be realistic about how attractive your creation will be to malicous actors. Some examples of poor applications include a company who's products are accessible and subject to tampering or an industrial company that needs safety critical eqiupment.
    
    For many, USB Motion Devices will be a wonderfully simple addition to their toolkit and even for applications in which USB Motion Devices would be inapprpriate in the end product, they can serve as a powerful prototyping tool.
    
