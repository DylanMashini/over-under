#include "catapult.h"
#include "pros/llemu.hpp"

enum cataState { IDLE, WINDING, LOCKED, FIRING };

void catapultTask() {
  std::shared_ptr<pros::Controller> main_controller(
      new pros::Controller(pros::E_CONTROLLER_MASTER));

  std::shared_ptr<MotorGroup> catapult(new MotorGroup({1, -2}));

  catapult->setGearing(AbstractMotor::gearset::red);
  catapult->setBrakeMode(AbstractMotor::brakeMode::hold);
  catapult->moveAbsolute(0, 200);

  bool catapultEngaged = false;

  cataState catapultState = LOCKED;

  int absoluteNextZero = 0;

  int stuckCount = 0;

  while (true) {

    if (main_controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) ==
        1) {
      catapultEngaged = !catapultEngaged;
    }
    if (catapultEngaged) {
      if (catapultState == IDLE) {
        pros::lcd::set_text(3, "IDLE");
        catapultState = WINDING;
        stuckCount = 0;
      } else if (catapultState == WINDING) {
        stuckCount++;
        if (stuckCount > 400) {
          catapultState = IDLE;
          catapult->moveVoltage(0);
          pros::delay(2500);
        }
        if ((int)catapult->getPosition() % 1800 != 0) {
          absoluteNextZero = (int)catapult->getPosition() +
                             (1800.0 - (int)catapult->getPosition() % 1800);
        } else {
          absoluteNextZero = (int)catapult->getPosition();
        }
        pros::lcd::set_text(3, "WINDING");
        pros::lcd::set_text(4, "Next Zero (n): " +
                                   std::to_string(absoluteNextZero / 1800.0));
        catapult->moveAbsolute(absoluteNextZero - 5, 200);
        if (catapult->getPosition() > absoluteNextZero - 15) {
          catapultState = LOCKED;
        }
      } else if (catapultState == LOCKED) {
        pros::lcd::set_text(3, "LOCKED");
        stuckCount = 0;
        pros::delay(1000);
        catapultState = FIRING;
      } else if (catapultState == FIRING) {
        stuckCount++;
        pros::lcd::set_text(3, "FIRING");
        catapult->moveAbsolute(absoluteNextZero + 200, 200);
        if (catapult->getPosition() > absoluteNextZero + 190) {
          catapultState = WINDING;
        }
        if (stuckCount >= 400) {
          catapultState = IDLE;
          catapult->moveVoltage(0);
          pros::delay(2500);
        }
      }
    } else {
      catapultState = IDLE;
      catapult->moveVoltage(0);
    }

    pros::delay(10);
  }
}
