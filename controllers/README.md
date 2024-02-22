This project is intended to be broken out to be used in several scenarios:

- controllers as an embedded library to be used on the teensy platform and compiled using platformio
- controllers as a simulink block in matlab for easy use in design of the controllers

The main idea of this project is that our controller implementation in matlab will be the exact same as on the car.

The interface between matlab and the controller will be kept as minimal as possible and are representative as possible as the actual implementation on the car.

TODOs:

- [ ] first proportional controller implemented and integrated into matlab
- [ ] workflow for getting the controllers onto the teensy worked out
    - i looked into using cmake under platformio and it doesnt look like its gonna be a good option for us
    - i think the best approach will be the following
        - cmake library creation for the simulink harness
        - cmake AND platformio lib for the actual controllers
            - when building the controllers for the teensy, platformio will knowing nothing of the cmake lib creation
            - each dependency will also have to have a platformio library created for them

examples of C/C++ libs getting used in matlab:

- https://github.com/osqp/osqp-matlab

    - uses mex stuff
    - this is prob the way we wanna go

docs for C/C++ in matlab:
https://www.mathworks.com/help/matlab/call-cpp-library-functions.html
