// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
public class RobotContainer {

  XboxController driverController;
  Cradle m_cradle = new Cradle(9);
  TankDrivetrain m_drivetrain = new TankDrivetrain();

  Drivetrain m_swerve = new Drivetrain();
  public RobotContainer() {
    driverController = new XboxController(0);
  }

  public void runRobot() {
    m_drivetrain.drive(
      -driverController.getRawAxis(1), 
      driverController.getRawAxis(3)); //TODO will need to fix when we get there

    if(driverController.getRawButton(0)) { //"A" button runs cradle
      m_cradle.runCradle(.2);
    } else {
      m_cradle.runCradle(0);
    }
  }

  public void runRobotSwerve() {
    m_swerve.drive(
    -Math.pow(driverController.getRawAxis(0), 3),
    -Math.pow(driverController.getRawAxis(1), 3), 
    -Math.pow(driverController.getRawAxis(4), 3),
    true, false, false);

    if(driverController.getRawButton(0)) { //"A" button runs cradle
      m_cradle.runCradle(.2);
    } else {
      m_cradle.runCradle(0);
    }
  }

}
