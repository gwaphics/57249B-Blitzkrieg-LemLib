#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"


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

// create a v5 rotation sensor on port 16 (verticle) and 7 (horizontile)
pros::Rotation vertical_encoder(16);
pros::Rotation horizontile_encoder(7);

//vertical tracking wheel --the rotational sensor
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, .5);
lemlib::TrackingWheel horizontile_tracking_wheel(&horizontile_encoder, lemlib::Omniwheel::NEW_275, -.25);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2, set to nullptr bc we don't have one
                            &horizontile_tracking_wheel,	// horizontal tracking wheel 1
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
// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
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

// ALMOST NONE OF THESE ARE CORRECT SO FIX WHEN DOING MATCH AUTOS
// 0 - turn test
// 1 - drive test
// 7 - blue negative
// 3 - red positive (mogo rush) (no sig wp)
// 4 - red negative
// 10 - skills
int chosenAuton = 10;

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
			chassis.moveToPoint(0, 24, 1000);
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
			intake_motors.move(127);
			chassis.moveToPose(-10, -25, 270, 1500, {.minSpeed = 30}); //-12

			// Get second ring (ring on bottom left of pile)
			//chassis.moveToPose(-10, -40, 180, 1500); //40
			chassis.moveToPoint(-17, -25, 1500, {.forwards = false}); //-15

			// Get third ring (ring on bottom right of pile)
			chassis.moveToPose(-20, -43, 180, 1500); //47

			// Go back and get fourth ring
			chassis.moveToPoint(-20, 0, 1500, {.forwards = false});
			intakeLift.set_value(true);
			chassis.moveToPose(25, 0, 90, 1500); //move backwards to prep for grabbing fourth ring
			pros::delay(200);
			intakeLift.set_value(false);
			// Hit ladders
			chassis.moveToPose(20, -20, 0, 1500);
			//chassis.waitUntilDone();
			intake_motors.move(0);
            
			break;
		case 3:
			// Red Positive
			// Mogo Rush
			clamp.set_value(true);
			chassis.moveToPoint(0, -25, 1500, {.forwards = false}); 
			chassis.moveToPoint(10, -42, 1500, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			clamp.set_value(false);
			pros::delay(300);
			// Get first ring
			intake_motors.move(127);
			chassis.turnToHeading(0, 750);
			chassis.moveToPose(10, -25, 0, 1500);
			// Drop fitst mogo
			chassis.turnToHeading(180, 750);
			chassis.waitUntilDone();
			clamp.set_value(true);
			pros::delay(300);
		
			// Turn to second mogo and get ready to get it
			chassis.turnToHeading(270, 750);
			chassis.waitUntilDone();
			// Get second mogo
			chassis.moveToPoint(28, -25, 1500, {.forwards = false, .maxSpeed = 80});
			chassis.waitUntilDone();
			clamp.set_value(false);
			pros::delay(300);
			intake_motors.move(127);
			// Get third ring
			// intakeLift.set_value(true);
			// chassis.moveToPoint(60, -5, 1500);
			// chassis.waitUntilDone();
			// intakeLift.set_value(false);
			// //chassis.moveToPoint(65, -5, 1500);
			// // Hit ladder
			// chassis.turnToHeading(0, 750);
			// chassis.moveToPoint(60, -20, 1500, {.forwards = false});
			// chassis.waitUntilDone();
			// intake_motors.move(0); 
			break;
		case 4:
		// Red Negative
			// Get mogo
			clamp.set_value(true);
			chassis.moveToPoint(-5, -22, 1500, {.forwards = false, .maxSpeed = 60});
			chassis.waitUntilDone();
			clamp.set_value(false);
			pros::delay(300);
			// Get first ring (not match load) (bottom of 2 ring stack)
			intake_motors.move(127);
			chassis.turnToHeading(90, 750); //750 timeout
			chassis.moveToPoint(10, -23, 1500);
			// Get second ring (ring on bottom left of pile)
			chassis.moveToPose(20, -40, -180, 1500); //12 x -40
			chassis.moveToPoint(10, -30, 1500, {.forwards = false}); //prep back to get third ring
			// Get third ring (ring on bottom right of pile)
			chassis.moveToPose(10, -41, -180, 1500); 
			pros::delay(200);
			// Go back and get fourth ring
			// chassis.moveToPoint(20, 0, 1500);
			// intakeLift.set_value(true);
			// chassis.moveToPose(-25, 0, -90, 1500);
			// pros::delay(300);
			// intakeLift.set_value(false);
			// Hit ladder

			//chassis.moveToPose(-24, 0, 0, 1500, {.forwards = false});
			chassis.moveToPose(-25, -20, 3, 1500);
			chassis.waitUntilDone();
			intake_motors.move(0);
			break;
		case 5:
			clamp.set_value(true);
			chassis.moveToPoint(5, -20, 1500, {.forwards = false});
			chassis.waitUntilDone();
			clamp.set_value(false);
			pros::delay(300);
			// Get first ring (not match load) (bottom of 2 ring stack)
			intake_motors.move(127);
			chassis.moveToPose(0, -25, 270, 1500, {.minSpeed = 30});
			// Get second ring (ring on bottom left of pile)
			chassis.moveToPose(-20, -35, 180, 1500); //12 x -40
			chassis.moveToPoint(-10, -40, 1500, {.forwards = false}); //x15 y might need to be 43
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
			chassis.waitUntilDone();
		case 6:
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
		case 7:
			// Get mogo
			clamp.set_value(true);
			//chassis.moveToPoint(0, -22, 1500, {.forwards = false}); //y20
			//chassis.waitUntilDone();
			clamp.set_value(false);
			//pros::delay(300);
			// Get first ring (not match load) (bottom of 2 ring stack)
			intake_motors.move(200);
			chassis.moveToPose(0, -25, 270, 1500, {.minSpeed = 50}); //minsp50
			//chassis.waitUntilDone();
			//chassis.moveToPose(-20, -35, 180, 1500); //12 x -40

			// Get second ring (ring on bottom left of pile)
			//chassis.setPose(0,0,0);
			chassis.moveToPoint(-23, -43, 1500); //x15 y might need to be 35
			// Get third ring (ring on bottom right of pile

			chassis.moveToPose(-20, -25, 180, 1500); //43y 40
			chassis.moveToPose(-23, -43, 180, 1500); //43y 40

			// Go back and get fourth ring
			chassis.moveToPoint(-30, 5, 1500, {.forwards = false});


			// intakeLift.set_value(true);
			// chassis.moveToPose(25, 0, 90, 1500);
			// pros::delay(300);
			// intakeLift.set_value(false);
			// // Hit ladder
			// chassis.moveToPose(20, -20, 0, 1500);
			// chassis.waitUntilDone();
        case 8: //blue neg
            // chassis.setPose(0, 0, 0);
            //grab mogo
            clamp.set_value(true);
			chassis.moveToPoint(0, -25, 1500, {.forwards = false});
			chassis.waitUntilDone();
			clamp.set_value(false);
			pros::delay(200);
            // chassis.moveToPoint(0, -25, 1500, {.forwards = false, .maxSpeed = 60}); 
			// //pros::delay(300);
            // clamp.set_value(false);
            // //first ring
            chassis.turnToHeading(270, 1500);
            chassis.waitUntilDone();
            chassis.setPose(0, 0, 0);
            intake_motors.move(200);
            //second ring
            //chassis.turnToHeading(270, 1500);
            chassis.moveToPoint(0, 25, 1500, {.forwards = true}); //24 but we move more so that it is in line with getting the second ring
            intake_motors.move(0);

            //sophia and jake code from case 2 
            // Get third ring (ring on bottom right of pile)
        
            // chassis.setPose(0,0,0);
            // intake_motors.move(127);
            // chassis.moveToPoint(0, 20, 1500);//
            // chassis.turnToHeading(20, 1500);
            // chassis.moveToPoint(0, -15, 1500);
            // intake_motors.move(0);
            // chassis.setPose(0,0,0);
            // chassis.turnToHeading(200, 1500);
            // chassis.moveToPoint(0, 30, 1500);

            // chassis.moveToPose(-20, -43, 180, 1500);
			// // Go back and get fourth ring
			// chassis.moveToPoint(-20, 0, 1500, {.forwards = false});
			// intakeLift.set_value(true);
			// chassis.moveToPose(25, 0, 90, 1500);
			// pros::delay(300);
			// intakeLift.set_value(false);
			// // Hit ladder
			// chassis.moveToPose(20, -20, 0, 1500);
			// chassis.waitUntilDone();
			// intake_motors.move(0);
           
            // chassis.turnToHeading(270, 1500);
            // chassis.setPose(0,0,0);
            // intake_motors.move(125);
            // chassis.moveToPoint(0, 25, 1500);
            // chassis.moveToPoint(0, -30, 1500);
            // chassis.turnToHeading(30, 1500);
            // chassis.moveToPoint(0, 30, 1500, {.maxSpeed = 60});
            break;
		case 9:
		// safe positive blue
			clamp.set_value(true);
			chassis.moveToPoint(-5, -20, 1500, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			clamp.set_value(false);
			pros::delay(300);

			// Get first ring (not match load) (bottom of 2 ring stack)
			intake_motors.move(127);
			chassis.moveToPose(15, -20, -270, 1500, {.minSpeed = 30}); //-12
			chassis.waitUntilDone();
			pros::delay(300);

			// touch ladder
			chassis.moveToPose(-30, -30, 270, 1500, {.forwards = false, .maxSpeed = 70});
			chassis.waitUntilDone();
			intake_motors.move(0);
			break;
		case 10:
		// skills
			// Score preload on alliance stake (retarted version)
			// intake_motors.move(127);
			// pros::delay(250);
			// intake_motors.move(0);
			// pros::delay(100);
			// intake_motors.move(-127);
			// pros::delay(1200);
			// intake_motors.move(0);

			// Score preload on alliance stake (normal ish version)
			intake_motors.move(-89);
			pros::delay(1200);
			intake_motors.move(0);

			// Not hit wall
			chassis.moveToPoint(0, 13, 1000);
			chassis.waitUntilDone();

			// Get first mogo
			clamp.set_value(true);
			chassis.turnToHeading(270, 750);
			chassis.waitUntilDone();
			left_motors.move(-60);
			right_motors.move(-60);
			pros::delay(500);
			left_motors.move(0);
			right_motors.move(0);
			chassis.waitUntilDone();
			pros::delay(200);
			clamp.set_value(false);
			pros::delay(300);

			// Get first ring on first mogo
			intake_motors.move(127);
			chassis.moveToPose(22, 35, 0, 1500, {.minSpeed = 70});

			// Get second ring on first mogo
			chassis.moveToPose(55, 55, 60, 1500, {.minSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(750);

			// Not hit wall
			chassis.moveToPoint(45, 50, 1500, {.forwards = false});

			// Get third ring on first mogo
			chassis.moveToPose(45, 35, 180, 1500, {.minSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(300);

			// Get fourth ane fifth ring on first mogo
			chassis.moveToPose(45, 10, 180, 1500, {.maxSpeed = 50});
			chassis.moveToPose(45, -10, 180, 1500, {.maxSpeed = 50});
			chassis.waitUntilDone();
			pros::delay(300);

			// Not hit wall
			chassis.moveToPoint(40, 10, 1500, {.forwards = false, .minSpeed = 50});
			chassis.waitUntilDone();

			// Get sixth ring on first mogo
			chassis.moveToPose(65, 10, 90, 1500, {.minSpeed = 30});
			chassis.waitUntilDone();
			pros::delay(300);
			
			// Drop mogo in corner
			chassis.turnToHeading(350, 750);
			chassis.moveToPoint(60, 0, 1500, {.forwards = false, .minSpeed = 30});
			chassis.waitUntilDone();
			clamp.set_value(true);



			// Middle



			// Get second mogo
			chassis.moveToPoint(20, 10, 2000, {.maxSpeed = 70});
			chassis.turnToHeading(-275, 750);
			chassis.waitUntilDone();
			left_motors.move(-60);
			right_motors.move(-60);
			pros::delay(1100);
			left_motors.move(0);
			right_motors.move(0);
			chassis.waitUntilDone();
			pros::delay(300);
			clamp.set_value(false);
			pros::delay(300);

			// Get first ring on second mogo
			intake_motors.move(127);
			chassis.moveToPose(-25, 35, 0, 1500, {.minSpeed = 70});

			// Get second ring on second mogo
			chassis.moveToPose(-55, 55, -60, 1500, {.minSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(750);

			// Not hit wall
			chassis.moveToPoint(-45, 48, 1500, {.forwards = false}); //-48,50

			// Get third ring on second mogo
			chassis.moveToPose(-50, 35, 180, 1500, {.minSpeed = 70});
			chassis.waitUntilDone();
			pros::delay(300);

			// Get fourth ane fifth ring on second mogo
			chassis.moveToPose(-50, 10, 180, 1500, {.maxSpeed = 50});
			chassis.moveToPose(-50, -10, 180, 1500, {.maxSpeed = 50});
			chassis.waitUntilDone();
			pros::delay(300);

			// Not hit wall
			chassis.moveToPoint(-43, 10, 1500, {.forwards = false, .minSpeed = 50});
			chassis.waitUntilDone();

			// Get sixth ring on second mogo
			chassis.moveToPose(-68, 10, -90, 1500, {.minSpeed = 30});
			chassis.waitUntilDone();
			pros::delay(300);
			
			// Drop second mogo in corner
			chassis.turnToHeading(-350, 750);
			chassis.moveToPoint(-63, 0, 1500, {.forwards = false, .minSpeed = 30});
			chassis.waitUntilDone();
			clamp.set_value(true);


			// End of first half


			// Get ring to hold before 3rd mogo
			chassis.moveToPoint(-45, 25, 1500);
			chassis.moveToPoint(-45, 55, 1500);
			chassis.moveToPoint(-25, 85, 1500);
			chassis.moveToPoint(-25, 105, 1500);
			pros::delay(750);
			intake_motors.move(0);

			// Get third mogo
			chassis.turnToHeading(270, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-10, 100, 1500, {.forwards = false});
			pros::delay(300);
			clamp.set_value(false);
			pros::delay(300);
			intake_motors.move(127);

			// Get second ring on third mogo
			chassis.moveToPoint(-48, 80, 1500, {.maxSpeed = 70});

			// get third and fourth ring on third mogo
			chassis.moveToPoint(-50, 120, 1500);

			// Not hit wall
			chassis.moveToPoint(-45, 100, 1500, {.forwards = false});
			chassis.waitUntilDone();

			// get fifth ring on third mogo
			chassis.moveToPose(-70, 100, 270, 1500);

			// Drop third mogo in corner
			chassis.turnToHeading(150, 750);
			chassis.waitUntilDone();
			chassis.moveToPoint(-68, 115, 1500, {.forwards = false});
			chassis.waitUntilDone();
			clamp.set_value(true);
			pros::delay(300);


			// Move last mogo to corner
			chassis.moveToPoint(-30, 100, 1500);
			chassis.moveToPoint(-10, 100, 1500);
			chassis.moveToPose(0, 115, 270, 1500);
			chassis.moveToPoint(30, 115, 1500, {.forwards = false});
			chassis.moveToPoint(60, 115, 1500, {.forwards = false});

			// Make sure not clamped
			chassis.moveToPoint(45, 100, 1500);


			// Stop everything
			chassis.waitUntilDone();
			intake_motors.move(0);
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
