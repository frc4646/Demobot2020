/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class BellSpeedThroughTarget extends Command {

  private double minDist = 30;
  private double maxDist = 110;

  public BellSpeedThroughTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_flagWaver);
    requires(Robot.m_vision);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //The range of distances are 30 inches - 110 inches
    if (Robot.m_vision.IsTrackingTarget())
    {
      double dist = Robot.m_vision.getDistanceToTarget();
      if (dist < minDist) dist = minDist;
      else if (dist > maxDist) dist = maxDist;
      double speed = ((dist-minDist)*(100/(maxDist-minDist)))/100; //Cause apparently java doesn't have Math.Clamp()
      speed = 1 - speed; //Close=fast bell Far=Slow Bell
      Robot.m_flagWaver.setMotorSpeed(speed);
      System.out.println("Distance: " + dist);
      System.out.println("Flag Speed: " + speed);
    }
    else
    {
      System.out.println("Cannot find target.");
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_flagWaver.setMotorSpeed(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
