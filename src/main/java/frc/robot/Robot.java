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
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import java.lang.Math;
import java.nio.file.FileSystem;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;

import java.io.IOException;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.RobotController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends TimedRobot {

  private final WPI_TalonSRX m_leftFrontDrive = new WPI_TalonSRX(21);
  private final WPI_TalonSRX m_leftBackDrive = new WPI_TalonSRX(20);

  MotorControllerGroup leftGroup = new MotorControllerGroup(m_leftFrontDrive, m_leftBackDrive);

  private final WPI_TalonSRX m_rightFrontDrive = new WPI_TalonSRX(23);
  private final WPI_TalonSRX m_rightBackDrive = new WPI_TalonSRX(22);

  MotorControllerGroup rightGroup = new MotorControllerGroup(m_rightFrontDrive, m_rightBackDrive);

  DifferentialDrive m_robotDrive = new DifferentialDrive(leftGroup, rightGroup);

  private final Joystick m_controller1 = new Joystick(0);
  private final Joystick m_controller2 = new Joystick(1);
  private final Timer m_timer = new Timer();
  
  private CANSparkMax m_rotation = new CANSparkMax(24, MotorType.kBrushless); //CAN ID may be changed.
  private CANSparkMax m_arm = new CANSparkMax(25, MotorType.kBrushless); //CAN ID may be changed.
  private CANSparkMax m_intake = new CANSparkMax(26, MotorType.kBrushed); //CAN ID may be changed.
  private CANSparkMax m_intakeRotation = new CANSparkMax(27, MotorType.kBrushed); // CAN ID may be changed.
  private double speed = 0;
  private double turn = 0;
  private double armSpeed  = 0;
  private double rotationSpeed = 0;
  private boolean intakePower = false;
  private boolean intakePositiveRotationPower = false;
  private boolean intakeNegativeRotationPower = false;

  private double speedMultiplier = 1;

  String blueParkLocation = "pathplanner/generatedJSON/Park - Blue";
  Trajectory bluePark = new Trajectory();
  /** 
    private MotorAccel leftFrontMotor = new MotorAccel(m_leftFrontDrive);
    private MotorAccel leftBackMotor = new MotorAccel(m_leftBackDrive);
    private MotorAccel rightFrontMotor = new MotorAccel(m_rightFrontDrive);
    private MotorAccel rightBackMotor = new MotorAccel(m_rightBackDrive);  
  */

  private MotorAccel speedAccel = new MotorAccel();
  private MotorAccel turnAccel = new MotorAccel();
  private RelativeEncoder armEncoder;
  private RelativeEncoder rotationEncoder;
  private SparkMaxPIDController armPIDController;
  private SparkMaxPIDController intakeEncoder;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

     // Instantiate our RobotContainer.  This will perform all our button bindings, and put our

    // autonomous chooser on the dashboard.

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_rightFrontDrive.setInverted(true);
    // m_rightBackDrive.setInverted(true);

    // CameraServer.startAutomaticCapture();
    RamseteController controller1 = new RamseteController();
    try {
      Path blueParkPath = Filesystem.getDeployDirectory().toPath().resolve(blueParkLocation);
      bluePark = TrajectoryUtil.fromPathweaverJson(blueParkPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + blueParkLocation, ex.getStackTrace());
    }
    TrajectoryConfig config = new TrajectoryConfig(4, 3);

    
    armPIDController = m_arm.getPIDController();
    armEncoder = m_arm.getEncoder();  
    rotationEncoder = m_rotation.getEncoder();
    
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override 
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
 
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // set throttle

    speedMultiplier = 0.75 + ((1 - m_controller1.getThrottle())) / 3.67;
    // controller speed

    if (!(Math.abs(m_controller1.getZ()) < 0.05) || !(Math.abs(m_controller1.getY()) < 0.05)) {

      speed = m_controller1.getY(); // note - using xbox controller 
      turn = m_controller1.getZ(); // note - using xbox controller
      // old code

      // m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
    } else {
      speed = 0;
      turn = 0;
    }
    m_robotDrive.arcadeDrive((turnAccel.accelerateSpeed(turn)) * (0.5 + speedMultiplier * 0.5) * 0.63, (speedAccel.accelerateSpeed(speed)) * (speedMultiplier) * 0.6);
    armSpeed = m_controller2.getY();
    rotationSpeed = m_controller2.getZ();
    intakePower = m_controller2.getTrigger();
    m_arm.set(armSpeed);
    m_rotation.set(rotationSpeed);
    if (intakePower = true) {
      m_intake.set(1);
    } else {
      m_intake.set(0);
    }
    intakePositiveRotationPower = m_controller2.getRawButton(1);
    if (intakePositiveRotationPower = true) {
      m_intake.set(1);
    } else if(intakeNegativeRotationPower =true) {
      m_intake.set(-1);
    } else {
      m_intake.set(0);
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
  private double accelerationIncrement = 0.5;

  private final double accelerationTime = 0.2;

  private Timer motorTimer = new Timer();

  public double motorSpeed = 0;

  MotorAccel() {
    motorTimer.reset();
    motorTimer.start();
  }

  public double accelerateSpeed(double desiredSpeed) {
    if (motorTimer.get() >= accelerationTime) {
      accelerationIncrement = (1.5 - Math.abs(motorSpeed)) * 0.6;

      if (Math.abs(desiredSpeed - motorSpeed) <= accelerationIncrement) {
        motorSpeed = desiredSpeed;
      } else {
        motorSpeed = motorSpeed + accelerationIncrement * Math.signum(desiredSpeed - motorSpeed);
      }

      motorTimer.reset();

    }
    return motorSpeed;

  }

}