# Navigation Loop
* calculate desired bearing
* get current heading

* PID 
    - inputs: 
        + desired bearing
        + current heading
        + max/min omega
    - output 
        + desired angular velocity of rover about its center (rad/s)
* Thoughts
    - probably want narrow range on omega output of PID

# Guidance Loop
* "inputs" 
    - desired angular velocity (rad/s)
    - MAX_WHEEL_SPEED constant
    - NOMINAL_FORWARD_SPEED constant
* solve for vl and vr using NOMINAL_FORWARD_SPEED and desired angular vel
* translate vl and vr (which are m/s of each side) to a desired cps for each l_cps and r_cps
* check for each wheel that the desired cps < MAX_WHEEL_SPEED
    - if not scale both by the scaling factor needed to get larger < MAX
* "outputs" 
    - desired cps for left wheel
    - desired cps for right wheel
    
# Control Loop (you have already)
* "inputs"
    - desired cps for left wheel
    - desired cps for right wheel
* low-level PID for each wheel
    - compare actual cps to desired 
    - adjusts PWM duty cycle via PID