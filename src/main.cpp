#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep


/**************************/
/***Chassis configuration**/
/**************************/

//Drivtrain
pros::MotorGroup LeftSide ({1,2,3}, pros::MotorGearset::blue);
pros::MotorGroup RightSide ({-4,-5,-6}, pros::MotorGearset::blue);

//Sensors
pros::Imu imu(7); //Inertial Sensor
pros::Rotation HorizontalL(8);
pros::Rotation Horizontal2(9);
pros::Rotation Vertical(10);
lemlib::TrackingWheel LeftSideTracker(&HorizontalL, lemlib::Omniwheel::NEW_275, 0/*(This is the distance from the center)*/);
lemlib::TrackingWheel RightSideTracker(&Horizontal2, lemlib::Omniwheel::NEW_275, 0/*(This is the distance from the center)*/);
lemlib::TrackingWheel VerticalTracker(&Vertical, lemlib::Omniwheel::NEW_275, 0/*(This is the distance from the center)*/);

//******All required Lemlib classes******\\

// drivetrain settings
lemlib::Drivetrain drivetrain(
	&LeftSide, // left motor group
    &RightSide, // right motor group
    10, // 10 inch track width
    lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    480, // drivetrain rpm is 480
    2 // horizontal drift is 2 (for now)
);
// odometry settings
lemlib::OdomSensors sensors(
	&VerticalTracker, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2, we are only using 1
    &LeftSideTracker, // horizontal tracking wheel 1
    &RightSideTracker, // horizontal tracking wheel 2
    &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
	10, // proportional gain (kP)
    0, // integral gain (kI)
    3, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
	2, // proportional gain (kP)
    0, // integral gain (kI)
    10, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(
	3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(
	3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(
	drivetrain, // drivetrain settings
    lateral_controller, // lateral PID settings
    angular_controller, // angular PID settings
    sensors, // odometry sensors
	&throttle_curve, 
    &steer_curve
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Sigma Panick Attack Code");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

        // delay to save resources
        pros::delay(25);
    }
}