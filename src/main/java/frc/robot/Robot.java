// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalenSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */

public class Robot extends TimedRobot {
  private final WPI_TalonSRX m_leftFrontDrive = new WPI_TalonSRX(21);
  private final WPI_TalonSRX m_leftBackDrive = new WPI_TalonSRX(20);

  MotorControllerGroup leftGroup = new MotorControllerGroup(m_leftFrontDrive, m_leftBackDrive);

  private final WPI_TalonSRX m_rightFrontDrive = new WPI_TalonSRX(23);
  private final WPI_TalonSRX m_rightBackDrive = new WPI_TalonSRX(22);

  MotorControllerGroup rightGroup = new MotorControllerGroup(m_rightFrontDrive, m_rightBackDrive);

  DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);

  private final CANSparkMax turnDrive = new CANSparkMax(24, CANSparkMax.MotorType.kBrushless);
  private final SparkMaxPIDController turnPID = turnDrive.getPIDController();
  private final RelativeEncoder turnEncoder = turnDrive.getEncoder();

  private final CANSparkMax liftDrive = new CANSparkMax(25, CANSparkMax.MotorType.kBrushless);
  private final SparkMaxPIDController liftPID = turnDrive.getPIDController();
  private final RelativeEncoder liftEncoder = turnDrive.getEncoder();

  private final WPI_TalonSRX armGrab = new WPI_TalonSRX(27);
  private final WPI_TalonSRX armFlip = new WPI_TalonSRX(28);

  private final AHRS robotGyro = new AHRS();

  private final Joystick m_controller = new Joystick(0);
  private final Joystick arm_controller = new Joystick(1);
  private final Timer m_timer = new Timer();

  private double speed = 0;
  private double turn = 0;
  private double rotateTarget = 0;
  private double liftTarget = 0;

  private double speedMultiplier = 1;
  private double armSpeedMultiplier = 1;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_rightFrontDrive.setInverted(true);
    // m_rightBackDrive.setInverted(true);

    // INIT the camera
    CameraServer.startAutomaticCapture();

    turnDrive.setSmartCurrentLimit(20);

    // Set rotate PID
    turnPID.setP(5e-5);
    turnPID.setI(1e-6);
    turnPID.setD(0);
    turnPID.setFF(0.000156);
    turnPID.setOutputRange(-0.4, 0.4);
    turnDrive.setSoftLimit(SoftLimitDirection.kForward, 16);
    turnDrive.setSoftLimit(SoftLimitDirection.kReverse, 16);

    liftPID.setP(5e-5);
    liftPID.setI(1e-6);
    liftPID.setD(0);
    liftPID.setFF(0.000156);
    liftPID.setOutputRange(-1, 1);

    rotateTarget = turnEncoder.getPosition();
    liftTarget = liftEncoder.getPosition();
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
    SmartDashboard.putString("Pitch", String.valueOf(robotGyro.getPitch()));
    SmartDashboard.putString("Yaw", String.valueOf(robotGyro.getYaw()));
    SmartDashboard.putString("Roll", String.valueOf(robotGyro.getRoll()));
    SmartDashboard.putString("Rotate", String.valueOf(turnEncoder.getPosition()));
    SmartDashboard.putString("Rotate (Target)", String.valueOf(rotateTarget));
    SmartDashboard.putString("Lift", String.valueOf(liftEncoder.getPosition()));
    SmartDashboard.putString("Lift (Target)", String.valueOf(liftTarget));
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putString("Pitch", String.valueOf(robotGyro.getPitch()));
    SmartDashboard.putString("Yaw", String.valueOf(robotGyro.getYaw()));
    SmartDashboard.putString("Roll", String.valueOf(robotGyro.getRoll()));
    SmartDashboard.putString("Rotate", String.valueOf(turnEncoder.getPosition()));
    SmartDashboard.putString("Rotate (Target)", String.valueOf(rotateTarget));
    SmartDashboard.putString("Lift", String.valueOf(liftEncoder.getPosition()));
    SmartDashboard.putString("Lift (Target)", String.valueOf(liftTarget));

    speedMultiplier = 0.75 + ((1 - m_controller.getThrottle())) / 3.67;
    // controller speed

    // Base Code
    if (!(Math.abs(m_controller.getZ()) < 0.05) || !(Math.abs(m_controller.getY()) < 0.05)) {
      speed = m_controller.getY();
      turn = m_controller.getZ();
    } else {
      speed = 0;
      turn = 0;
    }
    m_drive.arcadeDrive(turn * (0.5 + speedMultiplier * 0.5) * 0.63,
        speed * (speedMultiplier) * 0.6);



    // Arm Code
    armSpeedMultiplier = 0.5 + ((1 - arm_controller.getThrottle()) / 2);

    var possible_target = rotateTarget - (arm_controller.getZ() * armSpeedMultiplier * 0.05);
    if (possible_target > -16 && possible_target < 16) {
      rotateTarget = possible_target;
    }
    if (SmartDashboard.getBoolean("Should Turn", true)) {
      turnPID.setReference(rotateTarget, ControlType.kPosition);
    } else {
      turnDrive.set(0);
    }
    SmartDashboard.putNumber("Turn PID Out", turnDrive.getAppliedOutput());

    possible_target = liftTarget - (arm_controller.getY() * armSpeedMultiplier * 0.05);
    // if (possible_target > -16 && possible_target < 16) {
      liftTarget = possible_target;
    // }
    if (SmartDashboard.getBoolean("Lift uses PID", true)) {
      liftPID.setReference(liftTarget, ControlType.kPosition);
    } else {
      liftDrive.set(-arm_controller.getY() * armSpeedMultiplier - 0.02);
    }
    SmartDashboard.putNumber("Lift PID Out", turnDrive.getAppliedOutput());

    armFlip.set((arm_controller.getPOV() == 0 ? 1 : 0) - (arm_controller.getPOV() == 180 ? 1 : 0));
    if (arm_controller.getPOV() == 180) {
      armGrab.set(0.5);
    } else {
      armGrab.set(arm_controller.getTrigger() ? 1 : arm_controller.getTop() ? -0.5 : 0);
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putString("Pitch", String.valueOf(robotGyro.getPitch()));
    SmartDashboard.putString("Yaw", String.valueOf(robotGyro.getYaw()));
    SmartDashboard.putString("Roll", String.valueOf(robotGyro.getRoll()));
  }

}