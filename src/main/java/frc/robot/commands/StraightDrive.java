/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class StraightDrive extends Command {
  int driveDirection;
  public StraightDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires (Robot.m_drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Determines whether to drive forward or backward
    if (Robot.m_io.getGamepadLeftStickY() >= 0) driveDirection = 1;
    else driveDirection = -1;

    //Drives the robot straight
    Robot.m_drivetrain.driveByPercent(.5 + Robot.m_drivetrain.navX_kP * Robot.m_drivetrain.navX_error, .5 - Robot.m_drivetrain.navX_kP * Robot.m_drivetrain.navX_error);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
