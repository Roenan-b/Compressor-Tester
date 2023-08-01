## Compressor-Tester
-The "Compressor Tester" is a project that I inherited during my internship as an engineer with Air Lift.
-Created a system capable of testing the compressors the warranty team were receiving back as returns.**

## Project Requirements
Measurements for pressure are to be within ±2 psi (See testing data)
Written in Arduino
Visual indicators for each step/mode of testing


## Original System Overview
The system I was orignally given was meant to run/activate various air compressors for 3-4 seconds, filling a tank with a small amount of pressure via pneumatic connections
After 3-4 seconds, the system measures the pressure levels across the pneumatic circuit (Tank and air lines)
The measurements would be entered into some calculations that would determine if their was an air leak
If a leak was detected then the rate at which the air was leaking would be displayed on an LCD screen on top of the system (See pictures for more detail)
Exhausted all pressure from pneaumatic circuit


## New system Overview
The system will run various compressors for 60 seconds, filling a tank with as much pressure as it can in 60 seconds via pneumatic connections
After 60 seconds, a 10 second settle period is automatically entered
Once the pressure in penumatic circuit has settled for 10 seconds, system measures and displays the pressure in the circuit on the systems LCD
Exhausts all pressure from the circuit
Measurements for psi were ±.161 during the settled/on and ±.321 when system is idle
