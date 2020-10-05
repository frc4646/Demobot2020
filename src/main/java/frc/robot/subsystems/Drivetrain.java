/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.GamepadDriveTeleOp;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.PIDController;


/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  final VictorSPX frontLeftDrive;
  final TalonSRX frontRightDrive;
  final TalonSRX backLeftDrive;
  final VictorSPX backRightDrive;

  final AHRS navX;
  final PIDController navX_PID;
  
  public double navX_kP;
  public double navX_kI;
  public double navX_kD;
  public double navX_tolerance;
  public double navX_error;

  public Drivetrain()
  {
    frontLeftDrive = new VictorSPX(RobotMap.frontLeftDrivePort);
    frontRightDrive = new TalonSRX(RobotMap.frontRightDrivePort);
    backLeftDrive = new TalonSRX(RobotMap.backLeftDrivePort);
    backRightDrive = new VictorSPX(RobotMap.backRightDrivePort);

    frontLeftDrive.set(ControlMode.Follower, backLeftDrive.getBaseID());
    backRightDrive.set(ControlMode.Follower, frontRightDrive.getBaseID());


    frontRightDrive.setInverted(true);
    backRightDrive.setInverted(true);
  
    navX = new AHRS();
    navX.reset();

    navX_kP = 1; navX_kI = 0; navX_kD = 0;
    navX_tolerance = 1;
    navX_error = -navX.getRate();

    navX_PID = new PIDController(navX_kP, navX_kI, navX_kD);
    navX_PID.setTolerance(navX_tolerance);
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();

    SmartDashboard.putNumber("navX_Angle", getAngle());
  }


  @Override
  public void initDefaultCommand() 
  {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new GamepadDriveTeleOp());
  }

  public void driveByPercent(double leftSpeed, double rightSpeed)
  {
      frontRightDrive.set(ControlMode.PercentOutput, rightSpeed);
      backLeftDrive.set(ControlMode.PercentOutput, leftSpeed);
  }

  public double getAngle()
  {
    return navX.getAngle();
  }

  /*public void faceAngle(double targetAngle){
    double error = targetAngle - navX.getAngle();
    frontRightDrive.set(ControlMode.PercentOutput,  kP * error);
    backLeftDrive.set(ControlMode.PercentOutput, -1 * kP * error);
  }*/

  public void faceAngle(double targetAngle){
    double calculatedPID = navX_PID.calculate(getAngle(), targetAngle);
    frontRightDrive.set(ControlMode.PercentOutput, -calculatedPID);
    backLeftDrive.set(ControlMode.PercentOutput, calculatedPID);
  }

  public boolean atTargetAngle()
  {
    return navX_PID.atSetpoint();
  }

  public void resetNavX()
  {
    navX.reset();
    navX_PID.reset();
  }

}
