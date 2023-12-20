#include "main.h"
#include "catapult.h"
#include "okapi/api/chassis/model/threeEncoderXDriveModel.hpp"
#include "okapi/impl/device/rotarysensor/IMU.hpp"
#include "pros/rtos.hpp"
#include <memory>

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
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");

  pros::lcd::register_btn1_cb(on_center_button);
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
void opcontrol() {
  pros::Controller main_controller(pros::E_CONTROLLER_MASTER);
  std::shared_ptr<Motor> back_right_mtr(new Motor(-17));
  std::shared_ptr<Motor> back_left_mtr(new Motor(16));
  std::shared_ptr<Motor> front_right_mtr(new Motor(-15));
  std::shared_ptr<Motor> front_left_mtr(new Motor(11));

  std::shared_ptr<Motor> right_intake(new Motor(-12));
  std::shared_ptr<Motor> left_intake(new Motor(2));

  std::shared_ptr<Motor> left_lift(new Motor(0));
  std::shared_ptr<Motor> right_lift(new Motor(0));

  std::shared_ptr<MotorGroup> lift_motors(new MotorGroup({left_lift, right_lift}));

  std::shared_ptr<MotorGroup> intake(
      new MotorGroup({right_intake, left_intake}));

  // UNUSED BUT BREAKS SHIT IF YOU REMOVE
  std::shared_ptr<RotationSensor> left_rot(new RotationSensor(19));
  std::shared_ptr<RotationSensor> right_rot(new RotationSensor(2));
  std::shared_ptr<RotationSensor> back_rot(new RotationSensor(18));

  ThreeEncoderXDriveModel chasis(front_left_mtr, front_right_mtr,
                                 back_right_mtr, back_left_mtr, left_rot,
                                 right_rot, back_rot, 150.0, 12000);
  pros::IMU imu(3);

  pros::Task cataTask(catapultTask, "catapultTask");

  while (true) {
    pros::lcd::set_text(4, "Left: " + std::to_string(main_controller.get_analog(
                                          pros::E_CONTROLLER_ANALOG_LEFT_Y)));
    chasis.fieldOrientedXArcade(
        main_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
        main_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X),
        main_controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X),
        imu.get_heading() * 1_deg);
    // Intake
    if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) == 1) {
      intake->moveVoltage(12000);
    } else if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) ==
               1) {
      intake->moveVoltage(-12000);
    } else {
      intake->moveVoltage(0);
    }

  if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) == 1) {
    lift_motors->moveVoltage(12000);
  } else if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
    lift_motors->moveVoltage(-12000);
  } else {
    lift_motors->moveVoltage(0);
  }


    pros::delay(10);
  }
}
