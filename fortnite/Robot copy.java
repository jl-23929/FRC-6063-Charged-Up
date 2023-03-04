// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;

import java.util.HashSet;
import java.util.Set;

import java.net.ServerSocket;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalenSRX;

import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CAN;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagDetector.Config;

import com.kauailabs.navx.frc.AHRS;


// import frc.robot.subsystems.DriveSubsystem;

import java.lang.Math;
import java.lang.reflect.Array;

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

  private final PWMSparkMax armMotor = new PWMSparkMax(0);

  private final PWMSparkMax clawMotor = new PWMSparkMax(1);

  private final PWMSparkMax clawMotorClose = new PWMSparkMax(2);

  DifferentialDrive m_robotDrive = new DifferentialDrive(leftGroup, rightGroup);

  private final Joystick m_controller = new Joystick(0);
  private final Joystick m_controller2 = new Joystick(1);
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

  private MotorAccel speedAccel = new MotorAccel();
  private MotorAccel turnAccel = new MotorAccel();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override


  public void robotInit() {
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
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // set throttle

    speedMultiplier = 0.75 + ((1-m_controller.getThrottle()))/3.67;
    // controller speed

    if (!(Math.abs(m_controller.getZ()) < 0.05) || !(Math.abs(m_controller.getY()) < 0.05)) {
      
      speed = m_controller.getY();  // note - using xbox controller 
      turn = m_controller.getZ(); // note - using xbox controller

      // old code
      // m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
    } else {
      speed = 0;
      turn = 0;
    } 

    if (m_controller.getRawButton(2) == true) {
      if ((zerodYaw < 0.1) && (zerodYaw > -0.1)) {
        speed = 0;
        turn = rotatePIDSpeed;
      }
    } 
    
    if (m_controller.getRawButton(1) == true) {
      speed = balancePIDSpeed;
	    turn = 0;
    }
    
    // 2nd controller code
    if (!(Math.abs(m_controller2.getY()) < 0.05)) {
      armMotor.set(m_controller2.getY()/2);
    }
    if (!(Math.abs(m_controller2.getX()) < 0.05)) {
      clawMotor.set(m_controller2.getX()/2);
    }
    if (!(Math.abs(m_controller2.getZ()) < 0.05)) {
      clawMotor.set(m_controller2.getZ()/2);
    }
  
    m_robotDrive.arcadeDrive((turnAccel.accelerateSpeed(turn))*(0.5+speedMultiplier*0.5)*0.63,(speedAccel.accelerateSpeed(speed))*(speedMultiplier)*0.6);
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

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
      accelerationIncrement = (1.5-Math.abs(motorSpeed))*0.6;

      if (Math.abs(desiredSpeed-motorSpeed) <= accelerationIncrement) {
       motorSpeed = desiredSpeed; 
      } else {
        motorSpeed = motorSpeed + accelerationIncrement*Math.signum(desiredSpeed-motorSpeed);
      }
      
      motorTimer.reset();

    } 
    return motorSpeed;

  }
  
}
