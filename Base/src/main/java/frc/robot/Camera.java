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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CAN;
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

  // private String aprilTagDetections[][];

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


    m_visionThread =
    new Thread(
        () -> {
          var camera = CameraServer.startAutomaticCapture();

          var cameraWidth = 640;
          var cameraHeight = 360;

          camera.setFPS(20);


          camera.setResolution(cameraWidth, cameraHeight);

          var cvSink = CameraServer.getVideo();
          var outputStream = CameraServer.putVideo("RioApriltags", cameraWidth, cameraHeight);

          var mat = new Mat();
          var grayMat = new Mat();

          var pt0 = new Point();
          var pt1 = new Point();
          var pt2 = new Point();
          var pt3 = new Point();
          var center = new Point();
          var red = new Scalar(0, 0, 255);
          var green = new Scalar(0, 255, 0);

          var aprilTagDetector = new AprilTagDetector();

          var config = aprilTagDetector.getConfig();
          config.quadSigma = 0.8f;
          aprilTagDetector.setConfig(config);

          var quadThreshParams = aprilTagDetector.getQuadThresholdParameters();
          quadThreshParams.minClusterPixels = 250;
          quadThreshParams.criticalAngle *= 5; // default is 10
          quadThreshParams.maxLineFitMSE *= 1.5;
          aprilTagDetector.setQuadThresholdParameters(quadThreshParams);

          aprilTagDetector.addFamily("tag16h5");

          var timer = new Timer();
          timer.start();
          var count = 0;

          while (!Thread.interrupted()) {
            if (cvSink.grabFrame(mat) == 0) {
              outputStream.notifyError(cvSink.getError());
              continue;
            }

            Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

            var results = aprilTagDetector.detect(grayMat);

            var set = new HashSet<>();

            for (var result: results) {
              count += 1;
              pt0.x = result.getCornerX(0);
              pt1.x = result.getCornerX(1);
              pt2.x = result.getCornerX(2);
              pt3.x = result.getCornerX(3);

              pt0.y = result.getCornerY(0);
              pt1.y = result.getCornerY(1);
              pt2.y = result.getCornerY(2);
              pt3.y = result.getCornerY(3);

              center.x = result.getCenterX();
              center.y = result.getCenterY();

              set.add(result.getId());

              Imgproc.line(mat, pt0, pt1, red, 5);
              Imgproc.line(mat, pt1, pt2, red, 5);
              Imgproc.line(mat, pt2, pt3, red, 5);
              Imgproc.line(mat, pt3, pt0, red, 5);

              Imgproc.circle(mat, center, 4, green);
              Imgproc.putText(mat, String.valueOf(result.getId()), pt2, Imgproc.FONT_HERSHEY_SIMPLEX, 2, green, 7);

            };

            for (var id : set){
              System.out.println("Tag: " + String.valueOf(id));

              // aprilTagDetections[1] = String.valueOf(id);
            }

            if (timer.advanceIfElapsed(1.0)){
              System.out.println("detections per second: " + String.valueOf(count));
              count = 0;
            }

            outputStream.putFrame(mat);
          }
          aprilTagDetector.close();      
        });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
    
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
   // m_rightFrontDrive.setInverted(true);
   // m_rightBackDrive.setInverted(true);
  }

  public void robotPeriodic() {
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
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

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

}
