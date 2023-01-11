// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

// april tags import + open cv

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagDetector.Config;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

// OMG ^ FIRST IS SOO KIND

import java.lang.Math;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftFrontDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_leftBackDrive = new PWMSparkMax(1);  
  private final PWMSparkMax m_rightFrontDrive = new PWMSparkMax(2);
  private final PWMSparkMax m_rightBackDrive = new PWMSparkMax(3); 
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftFrontDrive, m_rightFrontDrive);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();

  private double speed = 0;
  private double turn = 0;
  

  private MotorAccel leftFrontMotor = new MotorAccel(m_leftFrontDrive);
  private MotorAccel leftBackMotor = new MotorAccel(m_leftBackDrive);
  private MotorAccel rightFrontMotor = new MotorAccel(m_rightFrontDrive);
  private MotorAccel rightBackMotor = new MotorAccel(m_rightBackDrive);  

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

    m_visionThread =
    new Thread(
            () -> {
              var camera = CameraServer.startAutomaticCapture();

              var cameraWidth = 640;
              var cameraHeight = 480;

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
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    if (!(Math.abs(m_controller.getLeftY()) < 0.05) && !(Math.abs(m_controller.getRightX()) < 0.05)) {
      
      speed = -m_controller.getLeftY();  // note - using xbox controller 
      turn = -m_controller.getRightX(); // note - using xbox controller

      leftFrontMotor.accelerateSpeed(speed + turn);
      leftBackMotor.accelerateSpeed(speed + turn);
      rightFrontMotor.accelerateSpeed(speed - turn);
      rightBackMotor.accelerateSpeed(speed - turn);
  
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

  public PWMSparkMax MotorName;


  MotorAccel(PWMSparkMax MotorTempName) {
    MotorName = MotorTempName;    
    motorTimer.reset();
    motorTimer.start();
  }

  public void accelerateSpeed(double desiredSpeed) {
    if (motorTimer.get() >= accelerationTime) {
      if (Math.abs(desiredSpeed-motorSpeed) < accelerationIncrement) {
        motorSpeed = desiredSpeed;
      } else {
        motorSpeed = motorSpeed + accelerationIncrement*Math.signum(desiredSpeed-motorSpeed);
      }
      
      MotorName.set(desiredSpeed);

      motorTimer.reset();
    }

  }
}
