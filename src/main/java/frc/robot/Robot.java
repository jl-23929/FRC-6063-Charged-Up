package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
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
  private final WPI_TalonSRX m_leftBackDrive = new WPI_TalonSRX(20);
  private final WPI_TalonSRX m_leftFrontDrive = new WPI_TalonSRX(21);
  MotorControllerGroup leftGroup = new MotorControllerGroup(m_leftFrontDrive, m_leftBackDrive);

  private final WPI_TalonSRX m_rightBackDrive = new WPI_TalonSRX(22);
  private final WPI_TalonSRX m_rightFrontDrive = new WPI_TalonSRX(23);
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
  // 0: Pitch 1: Roll 2: Yaw
  private double[] initialGyro = { 0, 0, 0 };
  private double[] gyro = { 0, 0, 0 };

  private final Joystick m_controller = new Joystick(0);
  private final Joystick arm_controller = new Joystick(1);
  private final Timer m_timer = new Timer();

  private Acceleration forwardAccel = new Acceleration("Forward Acceleration", 0.5, 0.2);
  private Acceleration turnAccel = new Acceleration("Turning Acceleration", 0.5, 0.2);

  private double rotateTarget = 0;
  private double liftTarget = 0;

  private PIDControl balanceControl = new PIDControl(0.2, 0, 0, "Balance PID");

  private Vision vision = new Vision();

  private double speedMultiplier = 1;
  private double armSpeedMultiplier = 1;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // INIT the camera
    CameraServer.startAutomaticCapture();
    robotGyro.calibrate();
    initialGyro = new double[] { robotGyro.getPitch(), robotGyro.getRoll(), robotGyro.getYaw() };

    turnDrive.setSmartCurrentLimit(20);

    // Set rotate PID
    turnPID.setP(5e-5);
    turnPID.setI(1e-6);
    turnPID.setD(0);
    turnPID.setFF(0.000156);
    turnPID.setOutputRange(-0.4, 0.4);
    turnDrive.setSoftLimit(SoftLimitDirection.kForward, 16);
    turnDrive.setSoftLimit(SoftLimitDirection.kReverse, 16);

    rotateTarget = turnEncoder.getPosition();
    SmartDashboard.putBoolean("Should Turn", true);

    liftPID.setP(5e-5);
    liftPID.setI(1e-6);
    liftPID.setD(0);
    liftPID.setFF(0.000156);
    liftPID.setOutputRange(-1, 1);

    liftTarget = liftEncoder.getPosition();
    SmartDashboard.putBoolean("Lift uses PID", true);

    balanceControl.enableContinuousInput(-180, 180);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Pitch", robotGyro.getPitch());
    SmartDashboard.putNumber("Roll", robotGyro.getRoll());
    SmartDashboard.putNumber("Yaw", robotGyro.getYaw());

    SmartDashboard.putNumber("Calibrated Pitch", gyro[0]);
    SmartDashboard.putNumber("Calibrated Roll", gyro[1]);
    SmartDashboard.putNumber("Calibrated Yaw", gyro[2]);

    SmartDashboard.putNumber("Rotate", turnEncoder.getPosition());
    SmartDashboard.putNumber("Rotate (Target)", rotateTarget);
    SmartDashboard.putNumber("Lift", liftEncoder.getPosition());
    SmartDashboard.putNumber("Lift (Target)", liftTarget);
    SmartDashboard.putNumber("Turn PID Out", turnDrive.getAppliedOutput());
    SmartDashboard.putNumber("Lift PID Out", turnDrive.getAppliedOutput());
    SmartDashboard.putNumber("Balance PID Out", balanceControl.value);

    SmartDashboard.putNumberArray("April Tags", vision.results.stream().mapToDouble(i -> (double) i).toArray());
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
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {

  }

  @Override
  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(turnDrive, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(liftDrive, DCMotor.getNEO(1));
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // Set Gyro
    balanceControl.update(gyro[0], 0);
    gyro = new double[] { robotGyro.getRoll() - initialGyro[0], robotGyro.getPitch() - initialGyro[1],
        robotGyro.getRoll() - initialGyro[2] };

    speedMultiplier = 0.75 + ((1 - m_controller.getThrottle())) / 3.67;
    SmartDashboard.putNumber("Speed Multiplier", speedMultiplier);

    // Base Code
    double speed, turn;
    if (!(Math.abs(m_controller.getZ()) < 0.05) || !(Math.abs(m_controller.getY()) < 0.05)) {
      speed = m_controller.getY();
      turn = m_controller.getZ();
    } else {
      speed = 0;
      turn = 0;
    }

    if (m_controller.getTrigger()) {
      speed += balanceControl.value;
    }

    m_drive.arcadeDrive(turnAccel.set(turn) * (0.5 + speedMultiplier * 0.5) * 0.63,
        forwardAccel.set(speed) * speedMultiplier * 0.6);

    // Arm Code
    armSpeedMultiplier = 0.5 + ((1 - arm_controller.getThrottle()) / 2);
    SmartDashboard.putNumber("Arm Speed Multiplier", armSpeedMultiplier);

    var possible_target = rotateTarget - (arm_controller.getZ() * armSpeedMultiplier * 0.05);
    if (possible_target > -16 && possible_target < 16) {
      rotateTarget = possible_target;
    }
    if (SmartDashboard.getBoolean("Should Turn", true)) {
      turnPID.setReference(rotateTarget, ControlType.kPosition);
    } else {
      turnDrive.set(0);
    }

    possible_target = liftTarget - (arm_controller.getY() * armSpeedMultiplier * 0.05);
    if (possible_target > 0 && possible_target < 90) { // TODO: Assign the top value based on testing
      liftTarget = possible_target;
    }
    if (SmartDashboard.getBoolean("Lift uses PID", true)) {
      liftPID.setReference(liftTarget, ControlType.kPosition);
    } else {
      liftDrive.set(-arm_controller.getY() * armSpeedMultiplier - 0.02);
    }

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
  }
}