#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup left_motors({-12, -19, 20}, pros::MotorGearset::blue);   //motor group w/ backwards ports 12 & 19 (leftFront, leftBack) forwards port 20 (leftTop)
pros::MotorGroup right_motors({9, 14, -10}, pros::MotorGearset::blue);  //motor group with forwards ports 9 & 14 (rightFront, rightBack) and backward port 10 (rightTop); all 600 rpm blue carts
pros::MotorGroup intake_motors({-4, 15}, pros::MotorGearset::blue); //motor group with left (reversed) and right (forward) intake motors
pros::adi::DigitalOut clamp('H');
pros::adi::DigitalOut intakeLift('G');
pros::adi::DigitalOut doinker('F');

//drivetrain configuration
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              13.5, // 13.5 inch track width (dist. from middle of the wheel to the other side's mid wheel.)
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

// creates an imu on port 17 -- inertial sensor (imu = inertial measurement unit)
pros::Imu imu(17);

// create a v5 rotation sensor on port 16
pros::Rotation vertical_encoder(16);

//vertical tracking wheel --the rotational sensor
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, .5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2, set to nullptr bc we don't have one
                            nullptr,	// horizontal tracking wheel 1, set to nullptr bc we don't have one
                            nullptr, // horizontal tracking wheel 2, set to nullptr bc we don't have a second one
                            &imu // inertial sensor
);


// PID will be done later
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              85, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              33, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
						&steer_curve //exp drive curve
);

// this runs at the start of the program
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    // print measurements from the rotation sensor
    chassis.calibrate(true);
	pros::lcd::print(1, "Rotation Sensor: %i", vertical_encoder.get_position());
    pros::delay(10); // delay to save resources. DO NOT REMOVE
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

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

int chosenAuton = 3;

void autonomous() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    //chassis.turnToHeading(90, 2000);
	/*chassis.waitUntilDone();
	chassis.cancelAllMotions();*/

	switch(chosenAuton){
		case 0:
			// Turn Test
			chassis.turnToHeading(90, 5000);
			chassis.turnToHeading(0, 5000);
			break;
		case 1:
			// Drive Test
			chassis.moveToPoint(0, 20, 1000);
			chassis.moveToPoint(0, 0, 1000, {.forwards = false});
			break;
		case 2:
			// Blue Negative
			// Get mogo
			clamp.set_value(true);
			chassis.moveToPoint(5, -20, 1500, {.forwards = false});
			chassis.waitUntilDone();
			clamp.set_value(false);
			pros::delay(300);
			// Get first ring (not match load) (bottom of 2 ring stack)
			intake_motors.move(270);
			chassis.moveToPose(-12, -25, 270, 1500, {.minSpeed = 30});
			// Get second ring (ring on bottom left of pile)
			chassis.moveToPose(-10, -40, 180, 1500);
			chassis.moveToPoint(-15, -25, 1500, {.forwards = false});
			// Get third ring (ring on bottom right of pile)
			chassis.moveToPose(-20, -43, 180, 1500);
			// Go back and get fourth ring
			chassis.moveToPoint(-20, 0, 1500, {.forwards = false});
			intakeLift.set_value(true);
			chassis.moveToPose(25, 0, 90, 1500);
			pros::delay(300);
			intakeLift.set_value(false);
			// Hit ladder
			chassis.moveToPose(20, -20, 0, 1500);
			break;
		case 3:
			// Red Positive
			// Mogo Rush
			clamp.set_value(true);
			chassis.moveToPoint(0, -20, 1500, {.forwards = false});
			chassis.moveToPoint(10, -37, 1500, {.forwards = false});
			chassis.waitUntilDone();
			clamp.set_value(false);
			pros::delay(500);
			// Get first ring
			intake_motors.move(270);
			chassis.moveToPose(15, -20, 0, 1500);
			// Turn to second mogo and unclamp first
			chassis.turnToHeading(270, 750);
			chassis.waitUntilDone();
			clamp.set_value(true);
			// Get second mogo
			chassis.moveToPoint(35, -20, 1500, {.forwards = false});
			chassis.waitUntilDone();
			clamp.set_value(false);
			pros::delay(500);
			// Get third ring
			intakeLift.set_value(true);
			chassis.moveToPoint(60, -5, 1500);
			chassis.waitUntilDone();
			intakeLift.set_value(false);
			// Hit ladder
			chassis.turnToHeading(0, 1500);
			chassis.moveToPoint(60, -20, 1500, {.forwards = false});
			break;
	}
}

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
    // pistons
	bool clampExtended = false;
  	bool doinkerExtended = false;
	bool intakeExtended = false;

    while (true) {
        // Exponential drive
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		double cubedLeftY = (leftY * leftY * leftY);
		double cubedRightX = (rightX * rightX * rightX);

		double expY = (cubedLeftY/10000);
		double expX = (cubedRightX/10000);

		double expL = (leftY + rightX);
		double expR = (leftY - rightX);

		left_motors.move(expL);
        right_motors.move(expR);


		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake_motors.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake_motors.move(-127);
		} else {
			intake_motors.move(0);
		}

		// Single button mogo clamp
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			clampExtended = !clampExtended;
			pros::delay(250);
			clamp.set_value(clampExtended);
		}

		// Single button doinker
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			doinkerExtended = !doinkerExtended;
			pros::delay(250);
			doinker.set_value(doinkerExtended);
		}

		// Single button doinker
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			intakeExtended = !intakeExtended;
			pros::delay(250);
			intakeLift.set_value(intakeExtended);
		}
		
		
        // delay to save resources
        pros::delay(25);
    }
}
