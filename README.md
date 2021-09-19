# IndividuallyAddressedSteppersGobzzztbzzt--bzztbzzt.
It took all night, but I did it! An individually addressable stepper motor controller, and a library to boot (haha, boot, like a computer), all built in one night. This project makes it easier to get into the cooler sides of robotics. Robotics is one of those fields in which you need to know and do everything -- sourcing, electronics, manufacture, code and controls -- to make a cool robot. It takes so long to cross the threshold where students can make one on their own and even then it takes so much effort. So, I made infrastructure for makers to build their own projects off of.  The titular individually addressable motor controllers allow for smoother, synchronous control of motors. That means more exact motion following and generation, enabled especially by the library made for them by yours truly. Being packaged motor-controller circuits with communication and control protocols built-in makes them perfect for beginners and veterans looking to offload work.


This is how the project works:

  You flash an ID onto each stepper motor controller, which can be done using another booting device and pre-given code. (because of time constraints, the DI specific projet could not be made, but the header file for it, DIManager.h, was)
  With this code, you are giving each motor controller it's own ID.
  
  There are 5 connections to be made in between each successive motor controller: 
    VCC->VCC
    V12->V12
    GND->GND
    DI->DI (data in)
    DO -> DO (data out)
    
    This means you have one long run of five wires, which cleans up your workspace a lot and makes wiring easier. On your main processor, your "master", you connect the same, except the V12, which requires an external power source.
    You can connect a capacitor accross a 12V DC battery supply. 
    
    
    
    Thats all the wiring there is! No matter how many devices you want to chain, the wiring stays the same.  
   The code that you flashed will take care of the peripherals, but you need to modify the processor code. 
   
   
   The EditMe.h file will contain all the parts to edit, look at the name! If you have the function for motor velocities given the change in state (most probably your state will be the x, y, z co-ordinate system, so the change in state will be the velocities in each direction)
   you can put that into localIK. If you have the function for motor position, you can edit the other IK function. 
   Then, if you know the FK, or the state (probably x,y,z or x,y), given actuator values you can input them here. 

     I can work on an automatic state update function that doesn't need the IK using linearized matrices. This algorithm requires only the FK, making robot control even easier, but it is process intensive and takes a while to code.   
     A thorough dive into the state, or kinematics here for some learners that were curious would be a nice idea but I disn't have time
