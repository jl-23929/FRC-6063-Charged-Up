// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalenSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;


// import frc.robot.subsystems.DriveSubsystem;

import java.lang.reflect.Array;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class AutoBalance extends TimedRobot {
  private final WPI_TalonSRX m_leftFrontDrive = new WPI_TalonSRX(21);
  private final WPI_TalonSRX m_leftBackDrive = new WPI_TalonSRX(20); 

  MotorControllerGroup leftGroup = new MotorControllerGroup(m_leftFrontDrive, m_leftBackDrive);

  private final WPI_TalonSRX m_rightFrontDrive = new WPI_TalonSRX(23);
  private final WPI_TalonSRX m_rightBackDrive = new WPI_TalonSRX(22); 

  MotorControllerGroup rightGroup = new MotorControllerGroup(m_rightFrontDrive, m_rightBackDrive);

  DifferentialDrive m_robotDrive = new DifferentialDrive(leftGroup, rightGroup);

  private final Joystick m_controller = new Joystick(0);
  private final Timer m_timer = new Timer();

  private double speed = 0;
  private double turn = 0;

  // auto variables

  private double phase = 0;

  private double speedMultiplier = 1; 
  
  private final Encoder m_encoder = new Encoder(0, 1);

  private double initPitch;
  private double initRoll;
  private double initYaw;

  private double zerodPitch;
  private double zerodRoll;
  private double zerodYaw;

  // pid stuff

  static double balancekP = 0;
  static double balancekI = 0;
  static double balancekD = 0;

  PIDController balancePID = new PIDController(balancekP, balancekI, balancekD);

  private double balancePIDSpeed = 0;

  static double rotatekP = 0;
  static double rotatekI = 0;
  static double rotatekD = 0;

  PIDController rotatePID = new PIDController(rotatekP, rotatekI, rotatekD);

  private double rotatePIDSpeed = 0;

  private double rotateGoal = 0;

  Thread m_visionThread;


  // gyro

  AHRS robotGyro = new AHRS();

  // public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem(); // Drivetrain subsystem 

/** 
  private MotorAccel leftFrontMotor = new MotorAccel(m_leftFrontDrive);
  private MotorAccel leftBackMotor = new MotorAccel(m_leftBackDrive); 
  private MotorAccel rightFrontMotor = new MotorAccel(m_rightFrontDrive);
  private MotorAccel rightBackMotor = new MotorAccel(m_rightBackDrive);  
*/

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override


  public void robotInit() {

    robotGyro.calibrate();

    initPitch = robotGyro.getPitch();
    initRoll = robotGyro.getRoll();
    initYaw = robotGyro.getYaw();    

    balancePID.enableContinuousInput(-180, 180);

    

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
   // m_rightFrontDrive.setInverted(true);
   // m_rightBackDrive.setInverted(true);
  }

  public void robotPeriodic() {
    zerodPitch = robotGyro.getPitch()-initPitch;
    zerodRoll = robotGyro.getRoll()-initRoll;
    zerodYaw = robotGyro.getYaw()-initYaw;

    balancePIDSpeed = balancePID.calculate(zerodPitch, 0);
    rotatePIDSpeed = rotatePID.calculate(zerodYaw, rotateGoal);
	
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();

    SmartDashboard.putString("DB/String 4", String.valueOf(robotGyro.getPitch()));
    
    

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    SmartDashboard.putString("DB/String 0",String.valueOf(robotGyro.getPitch()));
    SmartDashboard.putString("DB/String 1", String.valueOf(robotGyro.getYaw()));
    SmartDashboard.putString("DB/String 2", String.valueOf(robotGyro.getRoll()));



    SmartDashboard.putString("DB/String 5", String.valueOf(robotGyro.getPitch()-initPitch));
    SmartDashboard.putString("DB/String 6", String.valueOf(robotGyro.getRoll()-initRoll));
    SmartDashboard.putString("DB/String 7", String.valueOf(robotGyro.getYaw()-initYaw));

    rotateGoal = 0;

    m_robotDrive.arcadeDrive(0, balancePIDSpeed);

  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

}