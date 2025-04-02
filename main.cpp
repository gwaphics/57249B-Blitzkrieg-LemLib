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
pros::MotorGroup intake_motors({20, 1}, pros::MotorGearset::blue); //motor group with left (reversed) and right (forward) intake motors
pros::adi::DigitalOut clamp('H');
pros::adi::DigitalOut doinker('A');
pros::Motor intake(20, pros::MotorGearset::blue);
pros::Motor hooks(1, pros::MotorGearset::blue);
pros::MotorGroup lbMotors({18, -14}, pros::MotorGearset::green);
pros::Rotation lbRotSensor(10);
pros::Optical colorSortSensor(21);

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

bool colorIsCorrect = true;
bool red = true;
bool intakeSpin = false;
bool firstStageOnly = false;
bool reverse = false;
bool ringFound = false;
bool skills = false;

void colorSorting() {
	if (intakeSpin) {
		if (!skills) {
			if ((colorSortSensor.get_hue() > 300 || colorSortSensor.get_hue() < 50) && hooks.get_target_velocity() > 0 && red == false && ringFound == false) {
				ringFound = true;
				pros::delay(180);
				intake_motors.move(0);
				pros::delay(250);
				ringFound = false;
				intake_motors.move(127);
			} else if (red == false) {
				intake_motors.move(127);
			} else if (!(colorSortSensor.get_hue() > 300 || colorSortSensor.get_hue() < 80) && !(colorSortSensor.get_hue() >= 80 && colorSortSensor.get_hue() < 160) && hooks.get_target_velocity() > 0 && red == true && ringFound == false) {
				ringFound = true;
				pros::delay(180);
				intake_motors.move(0);
				pros::delay(250);
				ringFound = false;
				intake_motors.move(127);
			} else if (red == true) {
				intake_motors.move(127);
			}
		} else {
			intake_motors.move(127);
		}
	} else {
		if (firstStageOnly) {
			intake.move(127);
			hooks.move(0);
		} else if (reverse) {
			intake_motors.move(-127);
		} else {
			intake_motors.move(0);
		}
		
	}
}

const int numStates = 3;
int states[numStates] = {0, 2250, 13000}; // these are in centi-degrees, 1 degree is 100 centi-degrees
int currentState = 0;
int target = 0;

void nextState() {
    currentState += 1;
    if (currentState == numStates) {
        currentState = 0;
    }
	if (currentState == 2) {
		reverse = true;
		intakeSpin = false;
		pros::delay(100);
		reverse = false;
		intakeSpin = true;
	}

    target = states[currentState];
}

double kp = 0.015;
void liftControl() {
	if (currentState == 1 || currentState == 0) {
		kp = 0.015;
	} else {
		kp = 0.010;
	}

    double error = target - lbRotSensor.get_position();
    double velocity = kp * error;
    lbMotors.move(velocity);

	if (currentState == 0 && (error < 400 && error > -400)) {
		lbRotSensor.set_position(0);
	}
}

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

	pros::Task colorSortTask([] {
		while (true) {
			colorSorting();
			pros::delay(10);
		}
	});

	// pros::Task colorSort([] {
	// 	while (true) {
	// 		pros::lcd::print(4, "losing it bru"); // heading
	// 		if (!red) {
	// 			if ((colorSortSensor.get_hue() > 300 || colorSortSensor.get_hue() < 90) && hooks.get_target_velocity() > 0) {
	// 				float x = (hooks.get_actual_velocity()/hooks.get_target_velocity()) * 20;
	// 				pros::delay((90 - x)); // first num is 75 + value that hooks velocity ratio is being multiplied by
	// 				colorIsCorrect = false;
	// 				pros::delay(150);
	// 				colorIsCorrect = true;
	// 			}
	// 		} else {
	// 			if (!(colorSortSensor.get_hue() > 300 || colorSortSensor.get_hue() < 90) && !(colorSortSensor.get_hue() > 80 && colorSortSensor.get_hue() < 150) && hooks.get_target_velocity() > 0) {
	// 				float x = (hooks.get_actual_velocity()/hooks.get_target_velocity()) * 20;
	// 				pros::delay((95 - x)); // first num is 75 + value that hooks velocity ratio is being multiplied by
	// 				colorIsCorrect = false;
	// 				pros::delay(150);
	// 				colorIsCorrect = true;
	// 				pros::lcd::print(4, "FAAAAAAAAAAAAAA COLOR SOSRTT"); // heading
	// 			}
	// 		}
	// 	}
	// });
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

// 0 - turn test
// 1 - drive test
// 2 - red neg sig sawp
// 3 - blue neg sig sawp
// 4 - red pos goal rush
// 5 - blue pos goal rush
// 6 - red pos safe || blue neg safe
// 7 - blue pos safe || red neg safe
// 8 - red neg ring sweep
// 9 - blue neg ring sweep
// 10 - skills (brennens field)
// 11 - skills (aneeks field)
int chosenAuton = 11;

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
			// chassis.setPose(0, 0, 0);
			//clamp.set_value(false);
			red = true;
			intakeSpin = true;
			//clamp.set_value(true);
			// chassis.moveToPoint(0, 40, 10000, {.maxSpeed = 50});
			//chassis.moveToPoint(0, 24, 1000);
			//chassis.moveToPoint(0, 0, 1000, {.forwards = false});
			break;
		case 2:
		// red neg sig sawp
			// set up
			chassis.setPose(0, 0, 110);
			clamp.set_value(false);
			red = true;

			// score preload on alliance stake
			target = 19500;
			pros::delay(500);
			target = 0;

			// stack rush + grab one ring out of it
			chassis.moveToPoint(-10, 5, 500, {.forwards = false});
			chassis.turnToHeading(350, 750);
			chassis.waitUntilDone();
			doinker.set_value(true);
			firstStageOnly = true;
			chassis.turnToHeading(330, 500);
			chassis.moveToPoint(-12.5, 25, 1000);
			chassis.moveToPoint(-16, 45, 1000);
			chassis.waitUntilDone();

			// get first mogo
			chassis.moveToPoint(-3, 34, 1000, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(200);
			clamp.set_value(true);
			pros::delay(100);
			intakeSpin = true;
			firstStageOnly = false;

			// get next ring
			doinker.set_value(false);
			chassis.moveToPoint(-20, 34, 1000);

			// get ring on top of opp ring in middle
			chassis.turnToHeading(135, 500);
			chassis.moveToPoint(-3, 6, 1000);
			chassis.turnToHeading(90, 500);
			chassis.moveToPoint(25, 0, 1000, {.maxSpeed = 70});
			chassis.waitUntilDone();

			// // get second mogo
			// pros::delay(1200);
			// clamp.set_value(false);
			// intakeSpin = false;
			// firstStageOnly = true;
			// chassis.turnToHeading(215, 500);
			// chassis.waitUntilDone();
			// chassis.moveToPoint(37, 21, 1000, {.forwards = false, .maxSpeed = 70});
			// chassis.waitUntilDone();
			// pros::delay(200);
			// clamp.set_value(true);
			// pros::delay(100);

			// // get last ring
			// chassis.turnToHeading(90, 500);
			// chassis.waitUntilDone();
			// intakeSpin = true;
			// chassis.moveToPoint(57, 20, 1000);
			// chassis.waitUntilDone();
			// pros::delay(500);

			// // touch ladder
			// chassis.turnToHeading(270, 500);
			// target = 13000;
			// chassis.moveToPoint(0, 25, 10000, {.maxSpeed = 50});

			break;
		case 3:
		// blue neg sig sawp
			// set up
			chassis.setPose(0, 0, -110);
			clamp.set_value(false);
			red = false;

			// score preload on alliance stake
			target = 19500;
			pros::delay(500);
			target = 0;

			// stack rush + grab one ring out of it
			chassis.moveToPoint(14, 5, 500, {.forwards = false}); // x = 10 y = 5
			chassis.turnToHeading(-350, 750);
			chassis.waitUntilDone();
			doinker.set_value(true);
			firstStageOnly = true;
			chassis.turnToHeading(-330, 500);
			chassis.moveToPoint(15.6, 25, 1000);
			chassis.moveToPoint(21, 44, 1000);
			chassis.waitUntilDone();

			// get first mogo
			chassis.moveToPoint(7, 33, 1000, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(200);
			clamp.set_value(true);
			pros::delay(100);
			intakeSpin = true;
			firstStageOnly = false;

			// get next ring
			doinker.set_value(false);
			chassis.moveToPoint(27, 32, 1000);

			// get ring on top of opp ring in middle
			chassis.turnToHeading(-135, 500);
			chassis.moveToPoint(7, 6, 1000);
			chassis.turnToHeading(-90, 500);
			chassis.moveToPoint(-21, 0, 1000, {.maxSpeed = 70});
			chassis.waitUntilDone();

			// // get second mogo
			// pros::delay(1000);
			// intakeSpin = false;
			// firstStageOnly = true;
			// clamp.set_value(false);
			// chassis.turnToHeading(-215, 500);
			// chassis.waitUntilDone();
			// chassis.moveToPoint(-36, 21, 1000, {.forwards = false, .maxSpeed = 70});
			// chassis.waitUntilDone();
			// pros::delay(200);
			// clamp.set_value(true);
			// pros::delay(100);

			// // get last ring
			// chassis.turnToHeading(-90, 500);
			// chassis.waitUntilDone();
			// firstStageOnly = false;
			// intakeSpin = true;
			// chassis.moveToPoint(-52, 16, 1000);
			// chassis.waitUntilDone();
			// pros::delay(500);

			// // touch ladder
			// chassis.turnToHeading(-270, 500);
			// target = 16000;
			// chassis.moveToPoint(-5, 25, 10000, {.maxSpeed = 50});

			break;
		case 4:
		// red pos goal rush

			break;
		case 5:
		// blue pos goal rush

			break;
		case 6:
		// red pos safe / blue neg safe
			// set up
			chassis.setPose(0, 0, -110);
			clamp.set_value(false);
			red = true;

			// score preload on alliance stake
			target = 19500;
			pros::delay(500);
			target = 0;

			// get ring on top of opp ring and hold in first stage
			chassis.moveToPoint(5, 5, 1000, {.forwards = false});
			chassis.waitUntilDone();
			doinker.set_value(true);
			chassis.moveToPoint(-1, 7, 1000);
			chassis.waitUntilDone();
			chassis.turnToHeading(20, 750, {.maxSpeed = 50});
			chassis.waitUntilDone();
			chassis.turnToHeading(0, 750);
			firstStageOnly = true;
			chassis.moveToPoint(-2, 18, 1000);
			chassis.waitUntilDone();
			doinker.set_value(false);

			// get mogo
			chassis.turnToHeading(200, 750, {.maxSpeed = 70});
			chassis.waitUntilDone();
			chassis.moveToPoint(9, 35, 1000, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(200);
			clamp.set_value(true);
			pros::delay(100);

			// get ring
			chassis.turnToHeading(90, 750);
			firstStageOnly = false;
			intakeSpin = true;
			chassis.waitUntilDone();
			chassis.moveToPoint(28, 37, 1000);
			chassis.waitUntilDone();
			pros::delay(1000);

			clamp.set_value(false);
			chassis.turnToHeading(180, 500);
			chassis.waitUntilDone();
			chassis.moveToPoint(27, 52, 1000, {.forwards = false});
			chassis.waitUntilDone();
			clamp.set_value(true);

			// // go to corner to sweep it
			// chassis.turnToHeading(145, 750);
			// chassis.waitUntilDone();
			// chassis.moveToPoint(47, 9, 2000);
			// chassis.waitUntilDone();
			// chassis.turnToHeading(135, 500);
			// chassis.waitUntilDone();
			// intakeSpin = false;
			// doinker.set_value(true);
			// pros::delay(500);
			// chassis.turnToHeading(225, 750);
			// chassis.waitUntilDone();
			// doinker.set_value(false);
			// chassis.turnToHeading(200, 1000);
			// chassis.waitUntilDone();
			// intakeSpin = true;
			// chassis.moveToPoint(45, -5, 1000, {.maxSpeed = 70});
			// chassis.waitUntilDone();
			// chassis.moveToPoint(45, 5, 1000, {.forwards = false, .maxSpeed = 70});

			break;
		case 7:
		// blue pos safe / red neg safe

			break;
		case 8:
		// 

			break;
		case 9:
		//
		
			break;
		case 10:
		// SKILLS (brennens field)
			// set up
			chassis.setPose(0, 0, 0);
			clamp.set_value(false);
			red = true;

			// score alliance stake
			intakeSpin = true; //intake_motors.move(127);
			pros::delay(500);
			reverse = false;
			intakeSpin = false; //intake_motors.move(-127);

			// move off alliance stake
			chassis.moveToPoint(0, 10, 500);
			chassis.turnToHeading(270, 500);

			// get first mogo
			chassis.moveToPoint(15, 12, 1000, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			//pros::delay(300);
			clamp.set_value(true);
			//pros::delay(200);

			// get first ring (1/6 on first mogo)
			chassis.turnToHeading(0, 750);
			chassis.waitUntilDone();
			reverse = false;
			intakeSpin = true; //intake_motors.move(127);
			chassis.moveToPoint(20, 30, 1000);
			chassis.waitUntilDone();

			// get next ring (put in lb)
			chassis.turnToHeading(60, 500);
			chassis.moveToPoint(41, 73, 1500, {.maxSpeed = 100});
			//pros::delay(1000);
			nextState();
			chassis.waitUntilDone();
			//pros::delay(500);

			// score ring on first wall stake and put next ring in intake
			chassis.turnToHeading(200, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(42, 60.5, 1000, {.maxSpeed = 100});
			chassis.waitUntilDone();
			chassis.turnToHeading(90, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(50, 57, 500, {.maxSpeed = 70});
			chassis.waitUntilDone();
			chassis.moveToPoint(59, 57, 1500, {.maxSpeed = 30});
			chassis.waitUntilDone();
			nextState();
			intake.move(127);
			pros::delay(500);

			// back off alliance stake and score (2/6 on first mogo)
			chassis.moveToPoint(44, 57, 1000, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			nextState();
			pros::delay(200);
			intakeSpin = true; //intake_motors.move(127);
			chassis.waitUntilDone();
			chassis.turnToHeading(180, 750);
			chassis.waitUntilDone();

			// get next ring (3/6 on first mogo)
			chassis.moveToPoint(46, 40, 1000);
			chassis.waitUntilDone();

			// get next rings (4/6 and 5/6 on first mogo)
			chassis.moveToPoint(47, 15, 1000, {.maxSpeed = 70});
			chassis.waitUntilDone();
			chassis.moveToPoint(47, 0, 1500, {.maxSpeed = 50});
			chassis.waitUntilDone();
			pros::delay(500);

			// get last ring (6/6 on first mogo)
			chassis.turnToHeading(55, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(54, 0, 1500);
			chassis.waitUntilDone();

			// put first full mogo in corner
			chassis.turnToHeading(330, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(63, -7, 1000, {.forwards = false});
			chassis.waitUntilDone();
			intakeSpin = false; //intake_motors.move(0);
			clamp.set_value(false);

			//
			// CROSS MIDDLE
			//

			// get second mogo (closest on left side)
			chassis.moveToPoint(30, -3, 1000);
			chassis.waitUntilDone();
			chassis.turnToHeading(90, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(12, -6, 500, {.forwards = false});
			//chassis.moveToPoint(-10, -8, 500, {.forwards = false});
			chassis.moveToPoint(-4, -6, 1000, {.forwards = false, .maxSpeed = 70}); // -22
			chassis.waitUntilDone();
			//pros::delay(100);
			clamp.set_value(true);
			//pros::delay(200);

			// get ring (1/6 on second mogo)
			chassis.turnToHeading(5, 750);
			chassis.waitUntilDone();
			intakeSpin = true; //intake_motors.move(127);
			chassis.moveToPoint(-3, 9, 1000); // x = 25
			chassis.waitUntilDone();

			// get next ring and load into lb
			chassis.moveToPoint(-17, 25, 500); // x = 41
			chassis.moveToPoint(-22, 56, 1000);
			chassis.waitUntilDone();
			nextState();

			// score on second wall stake and put other ring in mogo (2/6 on second mogo)
			chassis.moveToPoint(-13, 42, 1000, {.forwards = false}); // x = 32
			chassis.waitUntilDone();
			chassis.turnToHeading(270, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-22, 45, 1500, {.maxSpeed = 70}); // x = 39
			chassis.waitUntilDone();
			chassis.moveToPoint(-37, 45, 1500, {.maxSpeed = 30}); // x = 48
			chassis.waitUntilDone();
			nextState();
			intakeSpin = true; //intake.move(127);
			pros::delay(500);

			// move off wall stake
			chassis.moveToPoint(-15, 45, 1000, {.forwards = false}); // x = 34
			chassis.waitUntilDone();
			nextState();
			pros::delay(200);
			intakeSpin = true; //intake_motors.move(127);
			chassis.waitUntilDone();
			chassis.turnToHeading(180, 750);
			chassis.waitUntilDone();

			// get next ring (3/6 on second mogo)
			chassis.moveToPoint(-18, 25, 1000); // x = 37
			chassis.waitUntilDone();

			// get next rings (4/6 and 5/6 on second mogo)
			chassis.moveToPoint(-17, -5, 1000, {.maxSpeed = 70}); // x = 39
			chassis.waitUntilDone();
			chassis.moveToPoint(-17, -12, 1500, {.maxSpeed = 50}); // x = 39
			chassis.waitUntilDone();
			pros::delay(500);

			// get last ring (6/6 on second mogo)
			chassis.turnToHeading(305, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-22, -10, 1500); //  x = 50
			chassis.waitUntilDone();

			// put second full mogo in corner
			chassis.turnToHeading(20, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-32, -19, 1000, {.forwards = false}); // x = 57
			chassis.waitUntilDone();
			clamp.set_value(false);

			//
			// CROSS TO FAR
			//

			// ring and put in intake before 3rd mogo
			chassis.moveToPoint(-28, 0, 500); // x = 45
			chassis.moveToPoint(-28, 20, 500); // x = 45
			chassis.moveToPoint(-24, 30, 500); // x = 41 // y = 34
			chassis.waitUntilDone();
			nextState();
			chassis.turnToHeading(50, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(4, 58, 1500, {.maxSpeed = 70}); // x = 22 // y = 56
			chassis.waitUntilDone();

			// keep ring in lb and clamp 3rd mogo
			chassis.turnToHeading(225, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(29, 87, 1000, {.forwards = false, .maxSpeed = 70}); // x = -1 // y = 78
			chassis.waitUntilDone();
			//pros::delay(100);
			clamp.set_value(true);
			//pros::delay(200);
			
			// allign with blue alliance stake
			chassis.turnToHeading(0, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(26, 96, 1000); // x = -1 // y = 90
			chassis.waitUntilDone();
			pros::delay(200);
			left_motors.move(-127);
			right_motors.move(-127);
			pros::delay(90);
			left_motors.move(0);
			right_motors.move(0);

			// score on blue alliance stake
			states[2] = 19000;
			nextState();
			pros::delay(1000);
			intakeSpin = false; //intake_motors.move(0);

			// move off blue alliance stake
			chassis.moveToPoint(26, 70, 1000, {.forwards = false}); // x = -1 // y = 75
			intakeSpin = false; //intake_motors.move(0);
			chassis.waitUntilDone();

			// get next ring (1/6 on third mogo)
			chassis.turnToHeading(130, 750);
			chassis.waitUntilDone();
			target = 6000;
			firstStageOnly = true; //intake.move(127);
			chassis.moveToPoint(44, 60, 1000); // 14 59
			chassis.waitUntilDone();

			// get next ring (2/6 on third mogo)
			chassis.turnToHeading(45, 750);
			chassis.waitUntilDone();
			firstStageOnly = false;
			intakeSpin = true; //intake_motors.move(127);
			chassis.moveToPoint(67, 73, 1000); // 37 70
			chassis.waitUntilDone();

			// get next ring (3/6 on third mogo)
			chassis.turnToHeading(0, 500);
			chassis.moveToPoint(72, 83, 1000); // 42 80
			chassis.waitUntilDone();
			chassis.turnToHeading(115, 750);
			chassis.waitUntilDone();

			// get last ring (4/6 on third mogo)
			chassis.moveToPoint(81, 80, 1000); // 50 77
			chassis.waitUntilDone();
			intakeSpin = false; //intake_motors.move(0);
			//hooks.move(127);
			intakeSpin = false; //intake.move(-127);
			pros::delay(300);

			// put third mogo in corner
			doinker.set_value(true);
			chassis.turnToHeading(0, 750);
			chassis.waitUntilDone();
			doinker.set_value(false);

			chassis.turnToHeading(225, 750);
			chassis.waitUntilDone();
			clamp.set_value(false);
			//hooks.move(0);
			chassis.moveToPoint(92, 92, 1000, {.forwards = false}); // 57 90
			chassis.waitUntilDone();
			clamp.set_value(false);

			// push last mogo in corner
			chassis.moveToPoint(59, 85, 1000); // 30 85
			//intakeSpin = false; //intake.move(0);
			chassis.moveToPoint(29, 100, 1000); // 0 100
			chassis.moveToPoint(-1, 103, 1000); // -30 105
			chassis.moveToPoint(-29, 110, 1000); // -55 110

			break;
		case 11:
		// SKILLS (aneeks field)
			// set up
			chassis.setPose(0, 0, 0);
			clamp.set_value(false);
			red = true;
			states[2] = 18000;
			skills = true;

			// score alliance stake
			intakeSpin = true; //intake_motors.move(127);
			pros::delay(500);
			reverse = false;
			intakeSpin = false; //intake_motors.move(-127);

			// move off alliance stake
			chassis.moveToPoint(0, 10, 500);
			chassis.turnToHeading(270, 500);

			// get first mogo
			chassis.moveToPoint(15, 12, 1000, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			//pros::delay(300);
			clamp.set_value(true);
			//pros::delay(200);

			// get first ring (1/6 on first mogo)
			chassis.turnToHeading(0, 750);
			chassis.waitUntilDone();
			reverse = false;
			intakeSpin = true; //intake_motors.move(127);
			chassis.moveToPoint(20, 30, 1000);
			chassis.waitUntilDone();

			// get next ring (put in lb)
			chassis.turnToHeading(60, 500);
			chassis.moveToPoint(41, 73, 1500, {.maxSpeed = 100});
			firstStageOnly = true;
			intakeSpin = false;
			//pros::delay(1000);
			nextState();
			chassis.waitUntilDone();
			//pros::delay(500);

			// score ring on first wall stake and put next ring in intake
			chassis.turnToHeading(200, 750);
			chassis.waitUntilDone();
			intakeSpin = true;
			firstStageOnly = false;
			chassis.moveToPoint(40, 60.6, 1000, {.maxSpeed = 100});
			chassis.waitUntilDone();
			chassis.turnToHeading(90, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(50, 57.1, 500, {.maxSpeed = 70});
			chassis.waitUntilDone();
			chassis.moveToPoint(59, 57.1, 1500, {.maxSpeed = 30});
			chassis.waitUntilDone();
			nextState();
			intake.move(127);
			pros::delay(500);

			// back off alliance stake and score (2/6 on first mogo)
			chassis.moveToPoint(43, 57.1, 1000, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			nextState();
			pros::delay(200);
			intakeSpin = true; //intake_motors.move(127);
			chassis.waitUntilDone();
			chassis.turnToHeading(180, 750);
			chassis.waitUntilDone();

			// get next ring (3/6 on first mogo)
			chassis.moveToPoint(45, 40, 1000);
			chassis.waitUntilDone();

			// get next rings (4/6 and 5/6 on first mogo)
			chassis.moveToPoint(46, 15, 1000, {.maxSpeed = 70});
			chassis.waitUntilDone();
			chassis.moveToPoint(46, 0, 1500, {.maxSpeed = 50});
			chassis.waitUntilDone();
			pros::delay(500);

			// get last ring (6/6 on first mogo)
			chassis.turnToHeading(55, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(54, 0, 1500);
			chassis.waitUntilDone();

			// put first full mogo in corner
			chassis.turnToHeading(330, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(63, -7, 1000, {.forwards = false});
			chassis.waitUntilDone();
			intakeSpin = false; //intake_motors.move(0);
			clamp.set_value(false);

			//
			// CROSS MIDDLE
			//

			// get second mogo (closest on left side)
			chassis.moveToPoint(30, -5, 1000);
			chassis.waitUntilDone();
			chassis.turnToHeading(90, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(12, -9, 500, {.forwards = false});
			//chassis.moveToPoint(-10, -8, 500, {.forwards = false});
			chassis.moveToPoint(-4, -9, 1000, {.forwards = false, .maxSpeed = 70}); // -6
			chassis.waitUntilDone();
			//pros::delay(100);
			clamp.set_value(true);
			//pros::delay(200);

			// get ring (1/6 on second mogo)
			chassis.turnToHeading(5, 750);
			chassis.waitUntilDone();
			intakeSpin = true; //intake_motors.move(127);
			chassis.moveToPoint(-6, 7, 1000); // x = -3
			chassis.waitUntilDone();

			// get next ring and load into lb
			chassis.moveToPoint(-19, 25, 500); // x = -17
			chassis.moveToPoint(-24, 56, 1000); // x = -22
			chassis.waitUntilDone();
			nextState();

			// score on second wall stake and put other ring in mogo (2/6 on second mogo)
			chassis.moveToPoint(-16, 41, 1000, {.forwards = false}); // x = -13
			chassis.waitUntilDone();
			chassis.turnToHeading(270, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-25, 44, 1500, {.maxSpeed = 70}); // x = -22
			chassis.waitUntilDone();
			chassis.moveToPoint(-40, 44, 1500, {.maxSpeed = 30}); // x = -37
			chassis.waitUntilDone();
			nextState();
			intakeSpin = true; //intake.move(127);
			pros::delay(500);

			// move off wall stake
			chassis.moveToPoint(-18, 43, 1000, {.forwards = false}); // x = -15
			chassis.waitUntilDone();
			nextState();
			pros::delay(200);
			intakeSpin = true; //intake_motors.move(127);
			chassis.waitUntilDone();
			chassis.turnToHeading(180, 750);
			chassis.waitUntilDone();

			// get next ring (3/6 on second mogo)
			chassis.moveToPoint(-20, 25, 1000); // x = -18
			chassis.waitUntilDone();

			// get next rings (4/6 and 5/6 on second mogo)
			chassis.moveToPoint(-19, -5, 1000, {.maxSpeed = 70}); // x = -17
			chassis.waitUntilDone();
			chassis.moveToPoint(-19, -12, 1500, {.maxSpeed = 50}); // x = -17
			chassis.waitUntilDone();
			pros::delay(500);

			// get last ring (6/6 on second mogo)
			chassis.turnToHeading(305, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-25, -10, 1500); //  x = -22
			chassis.waitUntilDone();

			// put second full mogo in corner
			chassis.turnToHeading(20, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-35, -19, 1000, {.forwards = false}); // x = -32
			chassis.waitUntilDone();
			clamp.set_value(false);

			//
			// CROSS TO FAR
			//

			// ring and put in intake before 3rd mogo
			chassis.moveToPoint(-28, 0, 500); // x = 45
			chassis.moveToPoint(-28, 20, 500); // x = 45
			chassis.moveToPoint(-24, 30, 500); // x = 41 // y = 34
			chassis.waitUntilDone();
			nextState();
			chassis.turnToHeading(50, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(4, 58, 1500, {.maxSpeed = 70}); // x = 22 // y = 56
			chassis.waitUntilDone();

			// keep ring in lb and clamp 3rd mogo
			chassis.turnToHeading(225, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(30, 90, 1000, {.forwards = false, .maxSpeed = 70}); // x = 33 // y = 85
			chassis.waitUntilDone();
			//pros::delay(100);
			clamp.set_value(true);
			//pros::delay(200);
			
			// allign with blue alliance stake
			chassis.turnToHeading(0, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(24, 96, 1000); // x = -1 // y = 90
			chassis.waitUntilDone();
			pros::delay(200);
			left_motors.move(-127);
			right_motors.move(-127);
			pros::delay(90);
			left_motors.move(0);
			right_motors.move(0);

			// score on blue alliance stake
			states[2] = 19000;
			nextState();
			pros::delay(1000);
			intakeSpin = false; //intake_motors.move(0);

			// move off blue alliance stake
			chassis.moveToPoint(24, 70, 1000, {.forwards = false}); // x = -1 // y = 75
			intakeSpin = false; //intake_motors.move(0);
			chassis.waitUntilDone();

			// get next ring (1/6 on third mogo)
			chassis.turnToHeading(130, 750);
			chassis.waitUntilDone();
			target = 6000;
			firstStageOnly = true; //intake.move(127);
			chassis.moveToPoint(40, 59, 1000); // 14 59
			chassis.waitUntilDone();

			// get next ring (2/6 on third mogo)
			chassis.turnToHeading(45, 750);
			chassis.waitUntilDone();
			firstStageOnly = false;
			intakeSpin = true; //intake_motors.move(127);
			chassis.moveToPoint(65, 71, 1000); // 37 70
			chassis.waitUntilDone();

			// get next ring (3/6 on third mogo)
			chassis.turnToHeading(0, 500);
			chassis.moveToPoint(70, 81, 1000); // 42 80
			chassis.waitUntilDone();
			chassis.turnToHeading(115, 750);
			chassis.waitUntilDone();

			// get last ring (4/6 on third mogo)
			chassis.moveToPoint(81, 80, 1000); // 50 77
			chassis.waitUntilDone();
			intakeSpin = false; //intake_motors.move(0);
			//hooks.move(127);
			intakeSpin = false; //intake.move(-127);
			pros::delay(300);

			// put third mogo in corner
			doinker.set_value(true);
			chassis.turnToHeading(0, 750);
			chassis.waitUntilDone();
			doinker.set_value(false);

			chassis.turnToHeading(225, 750);
			chassis.waitUntilDone();
			clamp.set_value(false);
			//hooks.move(0);
			chassis.moveToPoint(93, 93, 1000, {.forwards = false}); // 57 90
			chassis.waitUntilDone();
			clamp.set_value(false);

			// push last mogo in corner
			chassis.moveToPoint(59, 75, 1000); // 30 85
			//intakeSpin = false; //intake.move(0);
			chassis.moveToPoint(29, 90, 1000); // 0 100
			chassis.moveToPoint(-1, 93, 1000); // -30 105
			chassis.moveToPoint(-29, 105, 1000); // -55 110

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

	bool in360 = false;

    while (true) {
        // Exponential drive
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X))/1.2;

		double cubedLeftY = (leftY * leftY * leftY);
		double cubedRightX = (rightX * rightX * rightX);

		double expY = (cubedLeftY/20000);
		double expX = (cubedRightX/20000);

		double expL = (leftY + rightX);
		double expR = (leftY - rightX);

		left_motors.move(expL);
        right_motors.move(expR);


		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intakeSpin = true;
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			reverse = true;
			intakeSpin = false;
		} else {
			reverse = false;
			intakeSpin = false;
		}

		// Single button mogo clamp
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			clampExtended = !clampExtended;
			pros::delay(250);
			clamp.set_value(clampExtended);
		}

		// single button doinker
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			doinkerExtended = !doinkerExtended;
			pros::delay(250);
			doinker.set_value(doinkerExtended);
		}

		// 360 macro
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			intake_motors.move(-127);
			target = 25000;
			in360 = true;
		}

		// alliance stake macro
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
			left_motors.move(-127);
			right_motors.move(-127);
			pros::delay(57);
			left_motors.move(0);
			right_motors.move(0);
			reverse = true;
			intakeSpin = false;
			target = 21000;
		}

		// 0 macro
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			target = 0;
			in360 = false;
		}
		

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			if (in360 == false) {
				nextState();
			} else{
				target = states[0];
				in360 = false;
			}
		}

		// Run auton skills
		// if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
		// 	autonomous();
		// }
		
		
        // delay to save resources
        pros::delay(25);
    }

}
