#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include "pros/adi.h" //allow for pneumatics to be used


//tutorial   https://www.youtube.com/watch?v=4DwnphVdJZY&ab_channel=SteamLabs

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-5, 4, -3},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({6, -9, 7}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(10);



//define a motors
//Motors
//motors take:
//port
//can be fliped by adding a - 
pros::Motor MyMotor1(5);


//pneumatics
//set up the pneumatics
//takes in the 3wire port
//if it wants to start extended or retracted
pros::adi::Pneumatics MyPiston('A', false);


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width distance between the 2 side whees
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

//pid setting for linear motion
// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);


//pid setting for angular motion like turning
// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);


//setting up odometry
// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);


//controler settings
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);
//controler settings
// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);


//set up chassis object so we can refrence the whole thing and be able to control the whole thing at once
// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen

            //graphic for where the robot thinks it is in x and y and its headding
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

//THIS IS WHERE WE SET UP THE PATH FROM THE FILE

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(pathlearning_jerryio_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    //lim example from the limlib tutorial
/*
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
*/

    //basic example
    /*
    //set up the motors
    pros::Motor left_wheels (LEFT_WHEELS_PORT);
    pros::Motor right_wheels (RIGHT_WHEELS_PORT, true); // This reverses the motor

    //move stright
    right_wheels.move_relative(1000, MOTOR_MAX_SPEED);
    left_wheels.move_relative(1000, MOTOR_MAX_SPEED);

    //and you can add in delays and stops and stuff

    
    pros::delay(1000);//delay for 1 second
    
    //would hopefully stop the wheels but i think you can have like set drive chains and shit to be able to like
    right_wheels.stop();
    left_wheels.stop();
    
    */

//youtube video example with descriptions of the functions
    //moveToPoint
    //takes in x cord in inches, 
    //y cord in inches, 
    //time out how long the robot has to be able to try to move there in millseconds,
    //extra prams {} like move backwards in this example use . to find what they are
    //if it should do this all async go to next point or wait so like if it reachs early flalse means it will always wait the time true means it will go as soon as it reaches the point
    chassis.moveToPoint(10, 7, 1000,{.forwards = false, .maxSpeed = 80},false );

    //moveToPose
    //like moveToPoint but also takes a headding theata
    //this is the angle the robot will end up in after it has reached is destination
    chassis.moveToPose(10, 7, 60, 1000);

    //turning based commands

    //turnToHeading
    //takes in headding theata
    //robot will turn to the target heading on the spot
    //the timeout
    //extra prams
    chassis.turnToHeading(60, 1000);

    //swingToHeading
    //like turn to heading but rather then doing it in place it will do it by only turning one side of the drive
    //takes in the heading theata
    //takes in a lemlib drive side for the locked side not allowed to move the left side
    //time out
    //extra prams
    chassis.swingToHeading(60, lemlib::DriveSide::LEFT, 1000);



    //pure pursuit
    //pid needs to be tuned correctly
    //for if you want your robot to follow a path of dots and such
    //realy good for long paths that are fluid and dynamic
    //to generate the path go to https://path.jerryio.com/
    //this website has the game and then you can set points you want to go to on the field
    //this allows you to create a path from where ever you want to start to where ever you want to go and have like extra points in the middle
    //download it as a text file
    //put the downloaded file into the static folder in the project
    //change the ASSET() under competion_initialize to the file that we have
    //actually run the pure pursuit


    //follow
    //takes in the path that it was given and you have set from the above instructions
    //how far ahead the robot will look ahead to compute the path in inches
    //time out how long is it allowed to run
    //all the extra prams same as the others
    chassis.follow(pathlearning_jerryio_txt, 10, 1000);

}

/**
 * Runs in driver control
 */
//this is where all the user control functions seem to be defined like you can set up the drive and the arm control here in like all of the templates
//its like that other file that would automaticly do it in vex code

void opcontrol() {
    // controller
    // loop to continuously update motors


    //create a toggle
    bool pistonToggle = false;

    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX); //can also be chaneged to like tank

        //where the button code will live

        if(controller.get_digital(DIGITAL_R1)){
            //move the motor takes a speed between 0 and 127
            //go back -
            //if you defined a gear set and such for the motor you can use diffrent types of move but this is just the basic one
            //MyMotor1.move(120);

            //make a piston move
            //extend the piston
            MyPiston.extend();
            //retract the piston
            MyPiston.retract();

            //work with the toggle
            if(pistonToggle == false){
                MyPiston.extend();
                pros::delay(500);//delay while the button is being pressed so it isn't just keeping on going
                pistonToggle = true;
            }
            else{
                MyPiston.retract();
                pros::delay(500);//delay while the button is being pressed so it isn't just keeping on going
                pistonToggle = false;
            }

            
        }


        // delay to save resources
        pros::delay(10);


    }

    /*
    code from the claw bot example to have arcade and have an arm be able to move 
        //this is where the components are called from their constrcters
        
        (
        //top of the file will have
        #define LEFT_WHEELS_PORT 1
        #define RIGHT_WHEELS_PORT 10
        #define ARM_PORT 8

        to define what ports the motors are on and be able to easier tell what they do reather then have magic numbers
        )

        //when defining the motors you can also along with its ports define what gear set is being used and also define if they are reversed or not
        pros::Motor left_wheels (LEFT_WHEELS_PORT);
        pros::Motor right_wheels (RIGHT_WHEELS_PORT, true);
        pros::Motor arm (ARM_PORT, MOTOR_GEARSET_36); // The arm motor has the 100rpm (red) gearset
        pros::Controller master (CONTROLLER_MASTER);

        while (true) {
            int power = master.get_analog(ANALOG_LEFT_Y);
            int turn = master.get_analog(ANALOG_RIGHT_X);
            int left = power + turn;
            int right = power - turn;
            left_wheels.move(left);
            right_wheels.move(right);

            if (master.get_digital(DIGITAL_R1)) {
            arm.move_velocity(100); // This is 100 because it's a 100rpm motor
            }
            else if (master.get_digital(DIGITAL_R2)) {
            arm.move_velocity(-100);
            }
            else {
            arm.move_velocity(0);
            }

            pros::delay(2);
        }
    
    
    */


}