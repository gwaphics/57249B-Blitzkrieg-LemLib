#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.h"
#include "pros/rotation.hpp"
#include <fstream>


pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup left_motors({-6, -16, -3}, pros::MotorGearset::blue);   //motor group w/ backwards ports 12 & 19 (leftFront, leftBack) forwards port 20 (leftTop)
pros::MotorGroup right_motors({11, 12, 7}, pros::MotorGearset::blue);  //motor group with forwards ports 9 & 14 (rightFront, rightBack) and backward port 10 (rightTop); all 600 rpm blue carts
pros::MotorGroup intake_motors({20, 2}, pros::MotorGearset::blue); //motor group with left (reversed) and right (forward) intake motors
pros::adi::DigitalOut clamp('H');
pros::Motor intake(20, pros::MotorGearset::blue);
pros::Motor hooks(2, pros::MotorGearset::blue);
pros::Motor lbMotor(-18, pros::MotorGearset::green);
pros::Rotation lbRotSensor(10);
const int numStates = 3;
int states[numStates] = {0, 2000, 17000}; // these are in centi-degrees, 1 degree is 100 centi-degrees
int currentState = 0;
int target = 0;

void nextState() {
    currentState += 1;
    if (currentState == numStates) {
        currentState = 0;
    }
	if (currentState == 2) {
		hooks.move(-127);
		pros::delay(100);
		hooks.move(0);
	}

    target = states[currentState];
}

void liftControl() {
    double kp = 0.015;
    double error = target - lbRotSensor.get_position();
    double velocity = kp * error;
    lbMotor.move(velocity);
}

//drivetrain configuration
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.5, // 11.5 inch track width (dist. from middle of the wheel to the other side's mid wheel.)
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              0 // horizontal drift is 2 (for now)
);


// creates an imu on port 14 -- inertial sensor (imu = inertial measurement unit)
pros::Imu imu(15);

// create a v5 rotation sensor on port 16 (vertical) and 7 (horizontal)
pros::Rotation vertical_encoder(-9);
pros::Rotation horizontile_encoder(-13);

//vertical tracking wheel --the rotational sensor
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, .5);
lemlib::TrackingWheel horizontile_tracking_wheel(&horizontile_encoder, lemlib::Omniwheel::NEW_275, .25);

lemlib::OdomSensors sensors(&vertical_tracking_wheel,//&vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2, set to nullptr bc we don't have one
                            &horizontile_tracking_wheel,//&horizontile_tracking_wheel,	// horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr bc we don't have a second one
                            &imu // inertial sensor
);


// PID will be done later
// lateral PID controller
lemlib::ControllerSettings lateral_controller(8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              10
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              14.7, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
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
// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	lbRotSensor.set_position(0);
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
			pros::lcd::print(3, "LB Theta: %ld", lbRotSensor.get_position()); // heading

			// if (hooks.get_actual_velocity() <= hooks.get_target_velocity() && hooks.get_target_velocity() != 0) {
			// 	int targetVelocity = hooks.get_target_velocity();
			// 	hooks.move(-127);
			// 	pros::delay(500);
			// 	hooks.move(targetVelocity);
			// }

			// while (hooks.get_actual_velocity() <= hooks.get_target_velocity() && hooks.get_target_velocity() != 0) {
			// 	int targetVelocity = hooks.get_target_velocity();
			// 	hooks.move(-127);
			// 	pros::delay(500);
			// 	hooks.move(targetVelocity);
			// }
            // delay to save resources
            pros::delay(20);
        }
    });

	pros::Task liftControlTask([]{
		while (true) {
			liftControl();
			pros::delay(10);
		}
	});
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
// 2 - red rush (good)
// 5 - blue rush (need to copy from red)
// 6 - safe pos red (take from case 10)
// 7 - safe blue (take from case 10)

// Good/tunable match autos
// 0 - turn test
// 1 - drive test
// 3 - red neg sweep (good)
// 4 - blue neg sweep (good)
// 8 - auto skills (good)
// 9 - solo sig awp (perfect)
// 10 - safe pos for elims (bristol) (perfect)
// 11 - red pos solo awp && blue neg no sweep (perfect)
// 12 - blue pos solo awp && red neg no sweep (not tested)
int chosenAuton = 8;

std::ofstream xPositions;

void autonomous() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);

	switch(chosenAuton){
		case 0:
			// Turn Test
			chassis.turnToHeading(90, 1000);
			//chassis.turnToHeading(0, 5000);
			break;
		case 1:
			// Drive Test
			chassis.moveToPoint(0, -10, 1000, {.forwards = false});
			//chassis.moveToPoint(0, 24, 1000);
			//chassis.moveToPoint(0, 0, 1000, {.forwards = false});
			break;
		case 2:
			// Red Positive
			// Mogo Rush
			clamp.set_value(false);
			chassis.moveToPoint(0, -24, 750, {.forwards = false});
			chassis.moveToPoint(10, -42, 1500, {.forwards = false});
			chassis.waitUntilDone();
			pros::delay(300);
			clamp.set_value(true);
			pros::delay(200);

			// get first ring + score preload
			chassis.turnToHeading(15, 750);
			chassis.waitUntilDone();
			intake_motors.move(127);
			chassis.moveToPoint(15, -27, 1500);
			chassis.waitUntilDone();
			pros::delay(1000); 
			//intake_motors.move(0);
			clamp.set_value(false);

			// get ring on top of middle stack
			chassis.moveToPoint(55, -5, 1500, {.maxSpeed = 70});
			chassis.waitUntilDone();
			chassis.turnToHeading(90, 750);
			pros::delay(1000);
			chassis.moveToPoint(62, -5, 1500, {.maxSpeed = 50});
			chassis.waitUntilDone();
			pros::delay(200);
			chassis.moveToPoint(72, -5, 1500, {.maxSpeed = 50});
			pros::delay(400);
			intake_motors.move(0);

			// get second mogo
			// chassis.moveToPoint(30, -5, 1500, {.forwards = false});
			// chassis.waitUntilDone();
			// chassis.turnToHeading(345, 750);
			// chassis.waitUntilDone();
			// chassis.moveToPoint(34, -23, 1500, {.forwards = false});
			// chassis.waitUntilDone();
			// pros::delay(300);
			// clamp.set_value(true);
			// pros::delay(200);

			// get second mogo
			chassis.moveToPoint(40, -24, 1500, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(300);
			clamp.set_value(true);
			pros::delay(200);

			// hit ladder
			chassis.turnToHeading(160, 750);
			chassis.waitUntilDone();
			intake_motors.move(127);
			nextState();
			nextState();
			intake_motors.move(127);
			chassis.moveToPoint(43, -27, 1500, {.maxSpeed = 40});
			chassis.waitUntilDone();
			
			break;
		case 3:
		// red neg SAWP sweep
			// change pose since we start at angle and set clamp
			chassis.setPose(0, 0, -40);
			clamp.set_value(false);

			// // score allince stake (use hooks)
			// nextState();
			// intake_motors.move(127);
			// pros::delay(600);
			// target = 25000;
			// intake_motors.move(-127);
			// pros::delay(1000);
			// target = 0;

			// score preload on alliance stake (standoff)
			target = 18000;
			pros::delay(1000);
			target = 0;

			// get mogo
			chassis.moveToPoint(10, -35, 1500, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(300);
			clamp.set_value(true);
			pros::delay(200);

			// ring sweep
			chassis.turnToHeading(170, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(18, -44, 750);
			chassis.waitUntilDone();
			// chassis.turnToHeading(120, 750);
			// chassis.waitUntilDone();
			intake_motors.move(127);
			chassis.moveToPoint(25, -44, 1000, {.maxSpeed = 50});
			chassis.waitUntilDone();
			chassis.turnToHeading(90, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(32, -54, 1500, {.maxSpeed = 50});
			chassis.waitUntilDone();

			// get next ring
			chassis.turnToHeading(330, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(37, -40, 1500);
			chassis.waitUntilDone();

			// ladder touch
			chassis.turnToHeading(250, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(10, -42, 1500, {.maxSpeed = 50});
			target = 16000;

			break;
		case 4:
		// blue negative SAWP sweep
			// change pose since we start at angle and set clamp
			chassis.setPose(0, 0, 36);
			clamp.set_value(false);

			// // score allince stake (use hooks)
			// nextState();
			// intake_motors.move(127);
			// pros::delay(600);
			// target = 25000;
			// intake_motors.move(-127);
			// pros::delay(1000);
			// target = 0;

			// score preload on alliance stake (standoff)
			target = 18000;
			pros::delay(1000);
			target = 0;

			// get mogo
			chassis.moveToPoint(-12, -32, 1500, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(300);
			clamp.set_value(true);
			pros::delay(200);

			// ring sweep
			chassis.turnToHeading(-170, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-18, -39, 750);
			chassis.waitUntilDone();
			// chassis.turnToHeading(120, 750);
			// chassis.waitUntilDone();
			intake_motors.move(127);
			chassis.moveToPoint(-28, -47, 1000, {.maxSpeed = 50});
			chassis.waitUntilDone();
			chassis.turnToHeading(-90, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-40, -49, 1500, {.maxSpeed = 50});
			chassis.waitUntilDone();

			// get next ring
			chassis.turnToHeading(-330, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-42, -40, 1500);
			chassis.waitUntilDone();

			// ladder touch
			chassis.turnToHeading(-250, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-10, -37, 1500, {.maxSpeed = 50});
			target = 16000;
			break;
		case 5:
			// blue Positive
			// Mogo Rush
			clamp.set_value(true);
			chassis.moveToPoint(0, -22, 1500, {.forwards = false}); 
			chassis.moveToPoint(-10, -42, 1500, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			clamp.set_value(false);
			pros::delay(300);
			// Get first ring
			intake_motors.move(127);
			chassis.turnToHeading(0, 750);
			chassis.moveToPose(-10, -25, 0, 1500);
			// Drop fitst mogo
			chassis.turnToHeading(-180, 750);
			chassis.waitUntilDone();
			clamp.set_value(true);
			pros::delay(300);
			intake_motors.move(0);
			// Turn to second mogo and get ready to get it
			chassis.turnToHeading(-270, 750);
			chassis.waitUntilDone();
			// Get second mogo
			chassis.moveToPose(-28, -22, 90, 1500, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			clamp.set_value(false);
			pros::delay(300);
			intake_motors.move(127);

			chassis.moveToPoint(-50, -30, 1500, {.forwards = false, .maxSpeed = 50});
			// Get third ring
			// intakeLift.set_value(true);
			// chassis.moveToPoint(-55, -5, 1500, {.maxSpeed = 70});
			// chassis.waitUntilDone();
			// intakeLift.set_value(false);
			// chassis.moveToPoint(-60, -5, 1500);
			// Hit ladder
			// chassis.turnToHeading(0, 750);
			// chassis.moveToPoint(-55, -20, 1500, {.forwards = false});
			chassis.waitUntilDone();
			intake_motors.move(0); 
			break;
		case 6:
			break;
		case 7:
			break;
		case 8:
		// skills
			// set up
			chassis.setPose(0, 0, 0);
			clamp.set_value(false);

			// score alliance stake
			intake_motors.move(127);
			pros::delay(500);
			intake_motors.move(-127);

			// move off alliance stake
			chassis.moveToPoint(0, 11, 500);
			chassis.turnToHeading(270, 500);

			// get first mogo
			chassis.moveToPoint(13, 12, 1000, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			//pros::delay(300);
			clamp.set_value(true);
			pros::delay(200);

			// get first ring (1/6 on first mogo)
			chassis.turnToHeading(0, 750);
			chassis.waitUntilDone();
			intake_motors.move(127);
			chassis.moveToPoint(20, 30, 1000);
			chassis.waitUntilDone();

			// get next ring (put in lb)
			chassis.turnToHeading(45, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(40, 75, 1500, {.maxSpeed = 100});
			//pros::delay(1000);
			nextState();
			chassis.waitUntilDone();
			//pros::delay(500);

			// score ring on first wall stake and put next ring in intake
			chassis.turnToHeading(200, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(37, 60, 1000, {.maxSpeed = 100});
			chassis.waitUntilDone();
			chassis.turnToHeading(90, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(45, 56, 500, {.maxSpeed = 70});
			chassis.waitUntilDone();
			chassis.moveToPoint(55, 56, 1500, {.maxSpeed = 20});
			chassis.waitUntilDone();
			nextState();
			intake.move(127);
			pros::delay(500);

			// back off alliance stake and score (2/6 on first mogo)
			chassis.moveToPoint(36, 56.3, 1000, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			nextState();
			pros::delay(200);
			intake_motors.move(127);
			chassis.waitUntilDone();
			chassis.turnToHeading(180, 750);
			chassis.waitUntilDone();

			// get next ring (3/6 on first mogo)
			chassis.moveToPoint(39, 40, 1000);
			chassis.waitUntilDone();

			// get next rings (4/6 and 5/6 on first mogo)
			chassis.moveToPoint(39, 15, 1000, {.maxSpeed = 50});
			chassis.waitUntilDone();
			chassis.moveToPoint(39, -2, 1000, {.maxSpeed = 50});
			chassis.waitUntilDone();
			pros::delay(500);

			// get last ring (6/6 on first mogo)
			chassis.turnToHeading(55, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(45, 0, 1000);
			chassis.waitUntilDone();

			// put first full mogo in corner
			chassis.turnToHeading(330, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(51, -7, 1000, {.forwards = false});
			chassis.waitUntilDone();
			intake_motors.move(0);
			clamp.set_value(false);

			//
			// CROSS MIDDLE
			//

			// get second mogo (closest on left side)
			chassis.moveToPoint(30, -4, 1000);
			chassis.waitUntilDone();
			chassis.turnToHeading(90, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(0, -11.5, 500, {.forwards = false});
			//chassis.moveToPoint(-10, -8, 500, {.forwards = false});
			chassis.moveToPoint(-24, -11.5, 1000, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(100);
			clamp.set_value(true);
			pros::delay(200);

			// get ring (1/6 on second mogo)
			chassis.turnToHeading(5, 750);
			chassis.waitUntilDone();
			intake_motors.move(127);
			chassis.moveToPoint(-27, 5, 1000);
			chassis.waitUntilDone();

			// get next ring and load into lb
			chassis.moveToPoint(-47, 30, 500);
			chassis.moveToPoint(-45, 51, 1000);
			nextState();
			chassis.waitUntilDone();

			// score on second wall stake and put other ring in mogo (2/6 on second mogo)
			chassis.moveToPoint(-35, 36, 1000, {.forwards = false});
			chassis.waitUntilDone();
			chassis.turnToHeading(270, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-56, 40, 1500, {.maxSpeed = 30});
			chassis.waitUntilDone();
			nextState();
			pros::delay(200);
			intake_motors.move(127);
			pros::delay(500);

			// move off alliance stake
			chassis.moveToPoint(-40, 40, 1000, {.forwards = false});
			chassis.waitUntilDone();
			nextState();
			pros::delay(200);
			intake_motors.move(127);
			chassis.waitUntilDone();
			chassis.turnToHeading(180, 750);
			chassis.waitUntilDone();

			// get next ring (3/6 on second mogo)
			chassis.moveToPoint(-45, 25, 1000);
			chassis.waitUntilDone();

			// get next rings (4/6 and 5/6 on second mogo)
			chassis.moveToPoint(-47, -5, 1000, {.maxSpeed = 50});
			chassis.waitUntilDone();
			chassis.moveToPoint(-47, -15, 1000, {.maxSpeed = 50});
			chassis.waitUntilDone();

			// get last ring (6/6 on second mogo)
			chassis.turnToHeading(305, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-55, -10, 1000);
			chassis.waitUntilDone();

			// put second full mogo in corner
			chassis.turnToHeading(30, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-62, -19, 1000, {.forwards = false});
			chassis.waitUntilDone();
			clamp.set_value(false);

			break;
		case 9:
		// solo sig awp
			chassis.setPose(0, 0, 121);

			// Score wall stake w/ LB
			target = 2500;
			pros::delay(100);
			intake_motors.move(127);
			pros::delay(800);
			intake_motors.move(-70);
			pros::delay(80);
			intake_motors.move(127);
			pros::delay(200);
			intake_motors.move(-70);
			pros::delay(80);
			intake_motors.move(127);
			pros::delay(200);
			intake_motors.move(-10);
			pros::delay(50);
			target = 20500;
			pros::delay(100);
			intake_motors.move(0);
			pros::delay(400);

			// Move to first mogo
			left_motors.move(-127);
			right_motors.move(-127);
			pros::delay(200);
			left_motors.move(0);
			right_motors.move(0);
			clamp.set_value(false);
			chassis.moveToPoint(-7, 32, 1500, {.forwards = false, .maxSpeed = 70});
			target = 0;
			chassis.waitUntilDone();
			pros::delay(200);
			clamp.set_value(true);

			// Get ring
			intake_motors.move(127);
			chassis.turnToHeading(300, 750);
			chassis.moveToPoint(-27, 35, 1500); // back to x=25 for full thing
			chassis.waitUntilDone();
			pros::delay(1000); // change to 400ms but 1000ms for kirk

			chassis.turnToHeading(90, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(10, 37, 1500);

			// // Get next ring (Stacked in middle) (load it into bot)
			// chassis.turnToHeading(120, 750);
			// chassis.waitUntilDone();
			// clamp.set_value(false);
			// chassis.moveToPoint(0, 15, 1500, {.maxSpeed = 80});
			// chassis.moveToPoint(15, 15, 1500, {.maxSpeed = 80});
			// chassis.moveToPoint(30, 15, 1500, {.maxSpeed = 80});
			// chassis.waitUntilDone();
			// pros::delay(100);
			// intake_motors.move(0);

			// // Get last mogo and score loaded ring
			// chassis.turnToHeading(210, 750);
			// chassis.moveToPoint(35, 32, 1500, {.forwards = false});
			// chassis.waitUntilDone();
			// pros::delay(300);
			// clamp.set_value(true);
			// pros::delay(300); // make 200 or 300 if no touch ladder
			// intake_motors.move(127);

			// // Get last ring
			// chassis.turnToHeading(70, 750);
			// chassis.moveToPoint(55, 35, 1500);
			// chassis.waitUntilDone();
			// pros::delay(200);
			// chassis.moveToPoint(45, 35, 1500, {.forwards = false});


			// // Hit ladder
			// chassis.turnToHeading(270, 750, {.maxSpeed = 70});
			// chassis.moveToPoint(20, 37, 1500);
			break;
		case 10:
		// for wiggle
			chassis.setPose(0, 0, -121);
			clamp.set_value(false);

			// score allince stake (use hooks)
			nextState();
			intake_motors.move(127);
			pros::delay(600);
			target = 25000;
			intake_motors.move(-127);
			pros::delay(1000);
			target = 0;

			// Move to first mogo
			left_motors.move(-127);
			right_motors.move(-127);
			pros::delay(200);
			left_motors.move(0);
			right_motors.move(0);
			clamp.set_value(false);
			chassis.moveToPoint(7, 32, 1500, {.forwards = false, .maxSpeed = 70});
			target = 0;
			chassis.waitUntilDone();
			pros::delay(200);
			clamp.set_value(true);

			// Get ring
			intake_motors.move(127);
			chassis.turnToHeading(-300, 750);
			chassis.moveToPoint(29, 32, 1500);
			chassis.waitUntilDone();
			pros::delay(500);

			// Drop mogo near corner
			chassis.turnToHeading(-30, 750);
			chassis.moveToPoint(40, 5, 1500, {.forwards = false});
			chassis.waitUntilDone();
			clamp.set_value(false);
			intake_motors.move(0);
			chassis.moveToPoint(40, 15, 1500);
			chassis.waitUntilDone();

			// Almost goal rush
			chassis.turnToHeading(160, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(29, 35, 1500, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			chassis.turnToHeading(180, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(29, 47, 1500, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(200);
			clamp.set_value(true);

			break;
		case 11:
		// red pos solo awp and blue neg no sweep
			// change pose since we start at angle and set clamp
			chassis.setPose(0, 0, 36);
			clamp.set_value(false);

			// score preload on alliance stake
			target = 18000;
			pros::delay(1000);
			target = 0;

			// get ring on top of middle stack
			chassis.moveToPoint(-10, -13, 1500, {.forwards = false});
			intake_motors.move(127);
			chassis.waitUntilDone();
			chassis.turnToHeading(90, 750);
			chassis.moveToPoint(8, -13, 1000, {.maxSpeed = 50});
			chassis.waitUntilDone();
			chassis.turnToHeading(180, 750);
			pros::delay(200);
			chassis.waitUntilDone();
			intake_motors.move(-127);
			pros::delay(500);
			chassis.turnToHeading(90, 750);
			chassis.waitUntilDone();
			intake_motors.move(127);
			chassis.moveToPoint(20, -13, 1000, {.maxSpeed = 50});
			chassis.waitUntilDone();
			pros::delay(50);
			intake_motors.move(0);
			intake.move(127);

			// get mogo
			chassis.turnToHeading(40, 750);
			chassis.moveToPoint(-8, -35, 1500, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(300);
			clamp.set_value(true);
			pros::delay(200);

			// get ring
			chassis.turnToHeading(250, 750);
			chassis.waitUntilDone();
			intake_motors.move(127);
			chassis.moveToPoint(-29, -35, 1500, {.maxSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(100);
			intake_motors.move(0);

			// touch ladder
			chassis.turnToHeading(90, 750);
			chassis.waitUntilDone();
			pros::delay(200);
			intake_motors.move(127);
			chassis.moveToPoint(25, -30, 1500, {.maxSpeed = 50});

			break;
		case 12:
		// blue pos SAWP and red neg SAWP no sweep 
			// change pose since we start at angle and set clamp
			chassis.setPose(0, 0, -36);
			clamp.set_value(false);
				//i love kdis - noah b
			// score preload on alliance stake
			target = 18000;
			pros::delay(1000);
			target = 0;

			// get ring on top of middle stack
			chassis.moveToPoint(10, -10, 1500, {.forwards = false});
			intake_motors.move(127);
			chassis.waitUntilDone();
			chassis.turnToHeading(-90, 750);
			chassis.moveToPoint(-10, -10, 1000, {.maxSpeed = 50});
			chassis.waitUntilDone();
			chassis.turnToHeading(180, 750);
			pros::delay(500);
			chassis.turnToHeading(-135, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-20, -15, 1000, {.maxSpeed = 50});
			chassis.waitUntilDone();
			pros::delay(50);
			intake_motors.move(0);
			intake.move(127);

			// get mogo
			chassis.turnToHeading(-100, 750);
			chassis.moveToPoint(8, -35, 1500, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(300);
			clamp.set_value(true);
			pros::delay(200);

			// get ring
			chassis.turnToHeading(-250, 750);
			chassis.waitUntilDone();
			intake_motors.move(127);
			chassis.moveToPoint(29, -35, 1500, {.maxSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(100);
			intake_motors.move(0);

			// touch ladder
			chassis.turnToHeading(-90, 750);
			chassis.waitUntilDone();
			target = 16000;
			pros::delay(200);
			intake_motors.move(127);
			chassis.moveToPoint(-5, -45, 1500, {.maxSpeed = 50});

			break;
		case 14:
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

void opcontrol() {
	// set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);

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

		double expY = (cubedLeftY/20000);
		double expX = (cubedRightX/20000);

		double expL = (leftY + rightX);
		double expR = (leftY - rightX);

		left_motors.move(expL);
        right_motors.move(expR);


		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake.move(127);
			hooks.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake.move(-127);
			hooks.move(-127);
		} else {
			intake.move(0);
			hooks.move(0);
		}

		// Single button mogo clamp
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			clampExtended = !clampExtended;
			pros::delay(250);
			clamp.set_value(clampExtended);
		}
		

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			nextState();
		}

		// Run auton skills
		// if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
		// 	autonomous();
		// }
		
		
        // delay to save resources
        pros::delay(25);
    }

}
