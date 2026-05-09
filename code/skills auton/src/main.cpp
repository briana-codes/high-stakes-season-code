#include "main.h"

#include <unistd.h>

#include <chrono>
#include <cmath>
#include <ctime>

// TOM EDITION
// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {11, 12, 13},     // Left Chassis Ports (negative port will reverse it!)
    {-14, -15, -16},  // Right Chassis Ports (negative port will reverse it!)

    19,    // IMU Port
    1.84,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    600);  // Wheel RPM

pros::ADIDigitalOut clema('c');
// pros::Controller master(pros::E_CONTROLLER_MASTER);

// Test motor for timer
// pros::Motor test(3, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);

// Intake motor group
pros::MotorGroup intake({17, -18}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor catapulta(1, pros::v5::MotorGears::red, pros::v5::MotorUnits::degrees);
// Motor hamster
// pros::MotorGroup hamster({4, -5}, pros::v5::MotorGears::red, pros::v5::MotorUnits::degrees);

// pros::MotorGroup centrator({9, -10}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

// Sensors
// pros::Imu imu(14);
pros::Rotation leftOdometry(12);
pros::Rotation rightOdometry(13);
pros::ADIDigitalIn bumper('b');
pros::Distance distance_sensor(2);
pros::Distance perete(6);

pros::ADIDigitalOut covrig_sus('e');

// pros::ADIDigitalOut undita('d');
pros::ADIDigitalOut undita('b');
pros::ADIDigitalOut paleta('d');

pros::ADIDigitalOut activator_pto('a');

// PID Constants
double kP = 7.1;
double kI = 2.0;
double kD = 6.05;

bool TELEOP = true;

// bool unditaState = false;
// bool upPressedLast = false;
// void updateUndita() {
//   bool upPressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
//   if (upPressed && !upPressedLast) {
//     unditaState = !unditaState;
//     undita.set_value(unditaState);
//     pros::delay(500);
//   }
//   upPressedLast = upPressed;
// }

bool paletaState = false;
bool L2PressedLast = false;
void updatePaleta() {
  bool L2Pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
  if (L2Pressed && !L2PressedLast) {
    paletaState = !paletaState;
    paleta.set_value(paletaState);
    pros::delay(20);
  }
  L2PressedLast = L2Pressed;
}

bool clemaState = false;
bool xPressedLast = false;
void updateClema() {
  bool xPressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
  if (xPressed && !xPressedLast) {
    clemaState = !clemaState;
    clema.set_value(clemaState);
    pros::delay(250);
  }
  xPressedLast = xPressed;
}

bool bLastPressed = false;
void functie_hamster() {
  bool isAPressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
  if (isAPressed) {
    // catapulta.move_absolute(130, 20);
    catapulta.move_absolute(210, 300);

    pros::delay(700);
    catapulta.move_absolute(0, 300);
  } else {
    catapulta.move_absolute(0, 300);
  }

  bool isBPressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
  bool catapultaToggled = false;
  if (isBPressed && !bLastPressed) {
    catapultaToggled = !catapultaToggled;
    if (catapultaToggled) {
      catapulta.move_absolute(300, 300);
    } else {
      catapulta.move_absolute(0, 300);
    }
  }
  bLastPressed = isBPressed;
}

bool hamsterPornit = false;
bool wasYPressedLast = false;
bool isIntakeRunningHamster = false;

void resetVariablesHamster() {
  wasYPressedLast = false;
  hamsterPornit = false;
}

const int OBJECT_THRESHOLD = 120;
int HAMSTER_SPEED = 127 * 0.7;
void fostul_hamster() {
  bool isYPressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

  if (isYPressed && !wasYPressedLast) {
    hamsterPornit = !hamsterPornit;
    intake.move(hamsterPornit ? HAMSTER_SPEED : 0);
    pros::delay(100);
  }

  if (distance_sensor.get() < OBJECT_THRESHOLD && hamsterPornit) {
    intake.move(0);
    pros::delay(100);
    resetVariablesHamster();
  }
}

// Intake control
void intake_aiurea() {
  float speedPercent = 0.8;  // Scale factor for speed
  static bool wasR1PressedLast = false;
  static bool wasR2PressedLast = false;
  static bool isIntakeForward = false;
  static bool isIntakeReverse = false;

  // Handle R1 button for forward spin
  bool isR1Pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
  if (isR1Pressed && !wasR1PressedLast) {
    isIntakeForward = !isIntakeForward;
    intake.move(isIntakeForward ? 127 * speedPercent : 0);
    resetVariablesHamster();
  }
  wasR1PressedLast = isR1Pressed;

  // Handle R2 button for reverse spin
  bool isR2Pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
  if (isR2Pressed && !wasR2PressedLast) {
    isIntakeReverse = !isIntakeReverse;
    intake.move(isIntakeReverse ? -127 * speedPercent : 0);
    resetVariablesHamster();
  }
  wasR2PressedLast = isR2Pressed;
}

// Reverse cubic scaling function
double reverseCubicScaling(double input) {
  double normalizedInput = input / 127.0;                                    // Normalize to range [-1, 1]
  double scaledInput = normalizedInput * normalizedInput * normalizedInput;  // Apply cubic transformation
  return scaledInput * 127.0;                                                // Scale back to the original range
}

void drive() {
  // Drive motors
  pros::Motor mfs3(13, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
  pros::Motor msd3(-16, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);  // Reversed
  pros::MotorGroup leftFrontMotors({11, 12}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
  pros::MotorGroup rightFrontMotors({-14, -15}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

  double speedPercent = 1.0;  // Scale factor for speed

  while (true) {
    TELEOP = true;

    // If L1 is pressed, lower the speed percent to half
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      speedPercent = 0.5;
    } else {
      speedPercent = 1.0;
    }

    // Set the drive mode to disable to prevent the motors from moving
    chassis.drive_mode_set(ez::DISABLE);

    // Get raw controller inputs
    const int rawYPower = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int rawTurnPower = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // Apply reverse cubic scaling
    double yPower = reverseCubicScaling(rawYPower);
    double turnPower = reverseCubicScaling(rawTurnPower) / 2.0;  // Reduce turn power by half for smoother turning

    // Calculate motor powers for tank drive
    double leftPower = (yPower + turnPower) * speedPercent;
    double rightPower = (yPower - turnPower) * speedPercent;

    // Move the motors
    leftFrontMotors.move(leftPower);
    mfs3.move(leftPower);
    rightFrontMotors.move(rightPower);
    msd3.move(rightPower);

    pros::delay(20);  // Add a delay to prevent overwhelming the control loop
  }
}

// Initialization and Task Setup

// Operator control function
void opcontrol() {
  if (TELEOP) {
    pros::Task driveTask(drive);
    while (true) {
      updateClema();
      updatePaleta();
      intake_aiurea();
      // updateUndita();
      functie_hamster();
      fostul_hamster();
      pros::delay(20);
    }
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();
  // pros::lcd::initialize();

  // Initialize motor to position 0
  catapulta.move_absolute(0, 100);

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(2);    // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)
  chassis.drive_rpm_set(600);
  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  ez::as::auton_selector.autons_add({
      Auton("Example Drive\n\nDrive forward and come back.", drive_example),
      Auton("Example Turn\n\nTurn 3 times.", turn_example),
      Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
      Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
      Auton("Swing Example\n\nSwing in an 'S' curve", swing_example),
      Auton("Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining),
      Auton("Combine all 3 movements", combining_movements),
      Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),

  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();

  // dj
  //  pros::Task intakeTask(controlIntakeUp);
  master.rumble(".");
}

// ...existing code...

// ...existing code...

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

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

void autonomous() {
  TELEOP = false;
  int DRIVE_SPEED = 127;
  int TURN_SPEED = 127;
  int INTAKE_SPEED = 127 * 0.75;
  int INTAKE_SPEED_HAMSTER = 127 * 0.5;
  int SLOW_DRIVE_SPEED = 40;
  int DEFAULT_WAIT = 100;
  int CLEMA_WAIT = 500;
  int HAMSTER_THRESHOLD = 50;
  int MAX_WAIT_HAMSTER = 4000;

  if (!TELEOP) {
    // Initialize the robot
    catapulta.set_zero_position(0);
    catapulta.tare_position();
    catapulta.move_absolute(0, 300);

    // Merge cu spatele
    chassis.pid_drive_set(-19, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Merge incet ca sa centreze stake-ul
    chassis.pid_drive_set(-11, SLOW_DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Clemeaza stake-ul
    clema.set_value(true);
    pros::delay(CLEMA_WAIT);

    // Intoarce la dreapta 150 de grade (cum se uita robotul cu fata)
    chassis.pid_turn_set(152, TURN_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Incepe sa suga
    intake.move(INTAKE_SPEED);
    pros::delay(DEFAULT_WAIT);

    // Merge in fata sa suga cei doi covrigi
    chassis.pid_drive_set(25, DRIVE_SPEED / 2);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT * 10);

    // Opreste suptul
    intake.move(0);
    pros::delay(DEFAULT_WAIT);

    chassis.pid_drive_set(20, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Porneste suptul
    intake.move(INTAKE_SPEED);
    pros::delay(DEFAULT_WAIT * 10);

    // Intoarce la dreapta la 305 grade
    chassis.pid_turn_set(305, TURN_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Opreste suptul
    intake.move(0);
    pros::delay(DEFAULT_WAIT);

    // Merge in fata pana la covrig
    chassis.pid_drive_set(16, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Porneste suptul
    intake.move(INTAKE_SPEED);
    pros::delay(DEFAULT_WAIT);

    // Merge in fata pana la covrig
    chassis.pid_drive_set(8, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Ridica covrigul pana la hamster
    // Only go until the distance sensor detects an object or 1 second has passed
    int startTime = pros::millis();
    while (distance_sensor.get() > HAMSTER_THRESHOLD && pros::millis() - startTime < MAX_WAIT_HAMSTER) {
      intake.move(INTAKE_SPEED_HAMSTER);
      pros::delay(20);
    }
    intake.move_velocity(0);
    pros::delay(DEFAULT_WAIT);

    // Merge cu spatele putin
    chassis.pid_drive_set(-14, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Intoarce la dreapta la 330 de grade
    chassis.pid_turn_set(330, TURN_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Merge in fata pana la linie
    chassis.pid_drive_set(16, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Intoarce la stanga pana la 240 de grade
    chassis.pid_turn_set(240, TURN_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Merge in fata pana la stake-ul de pe perete
    chassis.pid_drive_set(7, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // PUNE COVRIG SUS CU HAMSTER
    catapulta.move_absolute(210, 300);
    pros::delay(700);
    catapulta.move_absolute(0, 300);

    // Merge cu spatele putin
    chassis.pid_drive_set(-12, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Intoarce la dreapta la 380 de grade
    chassis.pid_turn_set(382, TURN_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Porneste suptul
    intake.move(INTAKE_SPEED);
    pros::delay(DEFAULT_WAIT);

    // Merge in fata pana la covrig
    chassis.pid_drive_set(32, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT * 5);

    // Intoarce inca 90 de grade la dreapta
    chassis.pid_turn_set(465, TURN_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Opreste suptul
    intake.move(0);
    pros::delay(DEFAULT_WAIT);

    // Merge in fata pana la urmatorul covrig SUB SCARA
    chassis.pid_drive_set(35, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT * 2);

    // Porneste suptul
    intake.move(INTAKE_SPEED);
    pros::delay(DEFAULT_WAIT);

    // Merge putin cu spatele
    chassis.pid_drive_set(-5, DRIVE_SPEED / 2);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT * 5);

    // Scuipa putin
    intake.move(-INTAKE_SPEED);
    pros::delay(DEFAULT_WAIT);

    // Repornește suptul
    intake.move(INTAKE_SPEED);
    pros::delay(DEFAULT_WAIT * 15);

    // Opreste suptul
    intake.move(0);
    pros::delay(DEFAULT_WAIT);

    // Intoarce la stanga 180 de grade
    chassis.pid_turn_set(285, TURN_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Da iar sa suga covrigi
    intake.move(INTAKE_SPEED);
    pros::delay(DEFAULT_WAIT);

    // Da cu fata pana la fostul covrig
    chassis.pid_drive_set(30, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Merge in fata pana la urmatorul covrig
    chassis.pid_drive_set(35, DRIVE_SPEED / 2);  // era 35
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT * 5);

    // Merge in fata pana la ultimul covrig
    chassis.pid_drive_set(30, DRIVE_SPEED / 2);  // era 30
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT * 5);

    // Da cu spatele putin
    chassis.pid_drive_set(-25, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT * 5);

    // Mai merge putin in fata
    chassis.pid_drive_set(13, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT * 5);

    // Mai da putin cu spatele
    chassis.pid_drive_set(-10, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT * 5);

    // Intoarce la dreapta 180 de grade
    chassis.pid_turn_set(465, TURN_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Declemeaza stake-ul (era dupa urm. comanda)
    clema.set_value(false);
    pros::delay(CLEMA_WAIT);

    // Da cu spatele pana la colt
    chassis.pid_drive_set(-25, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Opreste suptul
    intake.move(0);
    pros::delay(DEFAULT_WAIT);

    // PLEACA DUPA PARTEA A DOUA

    // Merge putin cu fata
    chassis.pid_drive_set(18, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Intoarce la stanga pana la 421 de grade
    chassis.pid_turn_set(421, TURN_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Merge pana la urmatorul covrig
    chassis.pid_drive_set(44, DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Suge putin
    intake.move(INTAKE_SPEED);
    pros::delay(DEFAULT_WAIT);

    // Scuipa putin
    intake.move(-INTAKE_SPEED);
    pros::delay(DEFAULT_WAIT);

    // Ridica covrigul pana la hamster
    // Only go until the distance sensor detects an object or 1 second has passed
    startTime = pros::millis();
    while (distance_sensor.get() > HAMSTER_THRESHOLD && pros::millis() - startTime < MAX_WAIT_HAMSTER) {
      intake.move(INTAKE_SPEED_HAMSTER);
      pros::delay(20);
    }
    intake.move_velocity(0);
    pros::delay(DEFAULT_WAIT);

    // Intoarce la dreapta la 510 de grade
    chassis.pid_turn_set(512, TURN_SPEED);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // // Intoarce 90 de grade la dreapta
    // chassis.pid_turn_set(45, TURN_SPEED);
    // chassis.pid_wait();
    // pros::delay(DEFAULT_WAIT);

    // Da cu spatele pana la perete
    chassis.pid_drive_set(-15, DRIVE_SPEED / 2);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Scuipa putin
    intake.move(-INTAKE_SPEED);
    pros::delay(DEFAULT_WAIT * 2);

    // Da cu fata putin
    chassis.pid_drive_set(2, DRIVE_SPEED / 2);
    chassis.pid_wait();
    pros::delay(DEFAULT_WAIT);

    // Incepe suptul iar
    intake.move(INTAKE_SPEED);
    pros::delay(DEFAULT_WAIT);

    TELEOP = true;
  }
}
