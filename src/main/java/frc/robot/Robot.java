package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
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
  Encoder left_encoder = new Encoder(2, 3);

  private final WPI_TalonSRX m_rightBackDrive = new WPI_TalonSRX(22);
  private final WPI_TalonSRX m_rightFrontDrive = new WPI_TalonSRX(23);
  MotorControllerGroup rightGroup = new MotorControllerGroup(m_rightFrontDrive, m_rightBackDrive);
  Encoder right_encoder = new Encoder(0, 1);

  DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);

  private final CANSparkMax turnDrive = new CANSparkMax(24, CANSparkMax.MotorType.kBrushless);
  private final MaxPID turnPID = new MaxPID(turnDrive.getPIDController());
  private final RelativeEncoder turnEncoder = turnDrive.getEncoder();

  private final CANSparkMax liftDrive = new CANSparkMax(25, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder liftEncoder = liftDrive.getEncoder();

  private final WPI_TalonSRX armGrab = new WPI_TalonSRX(27);
  private final WPI_TalonSRX armFlip = new WPI_TalonSRX(28);

  private final AHRS robotGyro = new AHRS();
  // 0: Pitch 1: Roll 2: Yaw
  private double[] initialGyro = { 0, 0, 0 };
  private double[] gyro = { 0, 0, 0 };

  private final Joystick m_controller = new Joystick(0);
  private final Joystick arm_controller = new Joystick(1);
  private final Timer m_timer = new Timer();

  private double rotateTarget = 0;
  private double liftTarget = 0;

  private PIDController balanceControl = new PIDController(0.2, 0, 0);

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
    robotGyro.calibrate();
    initialGyro = new double[] { robotGyro.getPitch(), robotGyro.getRoll(), robotGyro.getYaw() };

    turnDrive.setSmartCurrentLimit(20);

    // Set rotate PID
    turnPID.controller.setP(0.1);
    turnPID.controller.setI(0.0005);
    turnPID.controller.setOutputRange(-0.4, 0.4);
    turnDrive.setSoftLimit(SoftLimitDirection.kForward, 9);
    turnDrive.setSoftLimit(SoftLimitDirection.kReverse, 9);

    turnDrive.setInverted(true);

    balanceControl.enableContinuousInput(-180, 180);

    SmartDashboard.putNumber("With Tag Speed", 0.8);
    SmartDashboard.putNumber("Without Tag Speed", 0.6);
  }

  @Override
  public void robotPeriodic() {
    // Set Gyro
    gyro = new double[] { robotGyro.getRoll() - initialGyro[0], robotGyro.getPitch() - initialGyro[1],
        robotGyro.getRoll() - initialGyro[2] };

    // Output Robot info
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

    SmartDashboard.putNumber("L encoder", left_encoder.get());
    SmartDashboard.putNumber("R encoder", right_encoder.get());

    SmartDashboard.putNumberArray("April Tags", vision.results.stream().mapToDouble(i -> (double) i).toArray());
  }

  public double calcBalance() {
    Double out = Math.min(Math.max(balanceControl.calculate(gyro[0], 0), -0.3), 0.3);
  
    SmartDashboard.putNumber("Balance PID Out", out);
    return out;
  }

  private boolean started_with_tags;
  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();

    started_with_tags = vision.results.contains(2) || vision.results.contains(7);
    initialGyro = new double[] { robotGyro.getPitch(), robotGyro.getRoll(), robotGyro.getYaw() };
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (started_with_tags && m_timer.get() < 2) {
      m_drive.arcadeDrive(0,-SmartDashboard.getNumber("With Tag Speed", 0.8));
    } else if (m_timer.get() < 3) {
      m_drive.arcadeDrive(0,-SmartDashboard.getNumber("Without Tag Speed", 0.6));
    } else {
      m_drive.arcadeDrive(0,calcBalance());
    }
    SmartDashboard.putNumber("Auto Time", m_timer.get());
    SmartDashboard.putNumber("Auto Speed R", leftGroup.get());
    SmartDashboard.putNumber("Auto Speed L", rightGroup.get());
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    rotateTarget = turnEncoder.getPosition();
    liftTarget = liftEncoder.getPosition();
    initialGyro = new double[] { robotGyro.getPitch(), robotGyro.getRoll(), robotGyro.getYaw() };
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
    speedMultiplier = 0.75 + ((1 - m_controller.getThrottle())) * 0.125;
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
      speed += calcBalance();
    }

    m_drive.arcadeDrive(turn * (0.5 + speedMultiplier * 0.5) * 0.63,
        speed * speedMultiplier * 0.9);

    // Arm Code
    armSpeedMultiplier = 0.5 + ((1 - arm_controller.getThrottle()) / 4);
    SmartDashboard.putNumber("Arm Speed Multiplier", armSpeedMultiplier);

    turnDrive.set(arm_controller.getZ() * armSpeedMultiplier* 0.25);

    liftDrive.set(-arm_controller.getY() * armSpeedMultiplier - 0.02);

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
    initialGyro = new double[] { robotGyro.getPitch(), robotGyro.getRoll(), robotGyro.getYaw() };
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    robotPeriodic();
  }
}