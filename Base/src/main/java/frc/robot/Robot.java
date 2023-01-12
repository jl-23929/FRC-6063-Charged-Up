// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.net.ServerSocket;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalenSRX;

import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
// import com.ctri?.phoenix.motorcontrol.can.WPI_VictorSPX;

import java.lang.Math;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends TimedRobot {
  private final PWMVictorSPX m_leftFrontDrive = new PWMVictorSPX(0);
  private final PWMVictorSPX m_leftBackDrive = new PWMVictorSPX(1); 

  MotorControllerGroup leftGroup = new MotorControllerGroup(m_leftFrontDrive, m_leftBackDrive);

  private final PWMVictorSPX m_rightFrontDrive = new PWMVictorSPX(2);
  private final PWMVictorSPX m_rightBackDrive = new PWMVictorSPX(3); 

  MotorControllerGroup rightGroup = new MotorControllerGroup(m_rightFrontDrive, m_rightBackDrive);

  DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);

  private final Joystick m_controller = new Joystick(1);
  private final Timer m_timer = new Timer();

  private double speed = 0;
  private double turn = 0;
  
/** 
  private MotorAccel leftFrontMotor = new MotorAccel(m_leftFrontDrive);
  private MotorAccel leftBackMotor = new MotorAccel(m_leftBackDrive);
  private MotorAccel rightFrontMotor = new MotorAccel(m_rightFrontDrive);
  private MotorAccel rightBackMotor = new MotorAccel(m_rightBackDrive);  
*/

  private MotorAccel speedAccel = new MotorAccel();
  private MotorAccel turnAccel = new MotorAccel();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightFrontDrive.setInverted(true);
    m_rightBackDrive.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off

    } else {
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    if (!(Math.abs(m_controller.getRawAxis(1)) < 0.05) && !(Math.abs(m_controller.getRawAxis(2)) < 0.05)) {
      
      speed = -m_controller.getRawAxis(1);  // note - using xbox controller 
      turn = -m_controller.getRawAxis(2); // note - using xbox controller

      m_drive.arcadeDrive(speedAccel.accelerateSpeed(speed),turnAccel.accelerateSpeed(turn));

      // old code

      // m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

}

class MotorAccel {
  private final double accelerationIncrement = 0.1;

  private final double accelerationTime = 0.15; 

  private Timer motorTimer = new Timer();

  public double motorSpeed = 0;

  
  MotorAccel() {   
    motorTimer.reset();
    motorTimer.start();
  }
  

  public double accelerateSpeed(double desiredSpeed) {
    if (motorTimer.get() >= accelerationTime) {
      if (Math.abs(desiredSpeed-motorSpeed) < accelerationIncrement) {
       motorSpeed = desiredSpeed;
      } else {
        motorSpeed = motorSpeed + accelerationIncrement*Math.signum(desiredSpeed-motorSpeed);
      }
      
      motorTimer.reset();

    }

    return motorSpeed;

  }
  
}
