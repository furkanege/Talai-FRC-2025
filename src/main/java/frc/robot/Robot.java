package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Timer;

/**
 * =============================================================================
 *  Talai – FRC 2025
 *  FULLY COMMENTED VERSION (ENGLISH)
 *
 *  Important Note:
 *  This robot had **no encoders or limit switches** on the elevator system.
 *  The entire stability and safety of the mechanism was achieved purely through
 *  software logic and careful motor control.
 *
 *  This file explains:
 *      - Why each motor is set to certain directions
 *      - Why elevator up/down behavior is enforced in software
 *      - How the elevator "soft hold" simulates a physical brake
 *      - Why intake & drive motors are intentionally inverted
 *      - How autonomous timing replaces sensors
 *
 *  NOTHING about robot behavior was changed. Only comments and structure improved.
 * =============================================================================
 */
public class Robot extends TimedRobot {

  // ---------------------------------------------------------------------------
  // MOTOR CONTROLLERS
  // ---------------------------------------------------------------------------
  private SparkMax m_leftMotor1;
  private SparkMax m_leftMotor2;
  private SparkMax m_rightMotor1;
  private SparkMax m_rightMotor2;
  private SparkMax m_intakeLeft;
  private SparkMax m_intakeRight;
  private SparkMax m_elevatorMotor1;
  private SparkMax m_elevatorMotor2;

  // Driver controller
  private Joystick m_stick;

  // Emergency-stop button
  private static final int EMERGENCY_STOP_BUTTON = 7;
  private boolean m_emergencyLatched = false;

  // Elevator constants
  private static final double HOLD_POWER = 0.30;         // Software “brake”
  private static final double ELEVATOR_UP_MULTIPLIER = 1.0;
  private static final double ELEVATOR_DEADBAND = 0.01;  // Prevents jitter

  // NOTE ABOUT ELEVATOR:
  //   Both elevator motors are wired in opposite polarities on the real robot.
  //   Therefore one must receive +power and the other -power for synchronized lift.

  // ---------------------------------------------------------------------------
  // AUTONOMOUS STATE MACHINE
  // ---------------------------------------------------------------------------
  private enum AutoState { FORWARD, ELEVATOR, INTAKE, REVERSE, DONE }
  private AutoState autoState;

  // Autonomous timings (software replaces sensors)
  private final double forwardDuration = 4.0;
  private final double intakeDuration  = 1.5;
  private final double reverseDuration = 1.0;

  // Elevator auto raise
  private final double elevatorUpDuration = 0.15;
  private final double elevatorUpPower    = 0.7;

  private Timer autoTimer;

  // ---------------------------------------------------------------------------
  // robotInit
  // ---------------------------------------------------------------------------
  @Override
  public void robotInit() {

    // Motor IDs are based on the physical wiring order used during build season.
    m_leftMotor1     = new SparkMax(1, MotorType.kBrushed);
    m_leftMotor2     = new SparkMax(2, MotorType.kBrushed);
    m_rightMotor1    = new SparkMax(3, MotorType.kBrushed);
    m_rightMotor2    = new SparkMax(4, MotorType.kBrushed);
    m_intakeLeft     = new SparkMax(5, MotorType.kBrushed);
    m_intakeRight    = new SparkMax(6, MotorType.kBrushed);
    m_elevatorMotor1 = new SparkMax(7, MotorType.kBrushed);
    m_elevatorMotor2 = new SparkMax(8, MotorType.kBrushed);

    // Elevator motors use brake mode for mechanical stability without sensors.
    SparkBaseConfig elev1Config = new SparkMaxConfig();
    elev1Config.idleMode(IdleMode.kBrake);
    m_elevatorMotor1.configure(elev1Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkBaseConfig elev2Config = new SparkMaxConfig();
    elev2Config.idleMode(IdleMode.kBrake);
    m_elevatorMotor2.configure(elev2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_stick = new Joystick(0);
    autoTimer = new Timer();
    m_emergencyLatched = false;
  }

  // ---------------------------------------------------------------------------
  // TELEOP CONTROL
  //
  //  IMPORTANT BEHAVIOR:
  //    - Elevator DOWN is software-blocked for safety.
  //    - Elevator HOLD actively stabilizes the arm with constant low force.
  //    - Drive motors use robot-specific polarity based on physical wiring.
  // ---------------------------------------------------------------------------
  @Override
  public void teleopPeriodic() {

    // -------------------------------
    // EMERGENCY STOP
    // -------------------------------
    if (m_stick.getRawButton(EMERGENCY_STOP_BUTTON)) {
      m_emergencyLatched = true;
    }

    if (m_emergencyLatched) {
      stopAllMotors();
      SmartDashboard.putBoolean("Emergency Stop Activated", true);
      return;
    }

    SmartDashboard.putBoolean("Emergency Stop Activated", false);

    // -------------------------------
    // DRIVETRAIN (robot-specific axis mapping)
    // -------------------------------
    double forward = -m_stick.getRawAxis(4);
    double rotation = -m_stick.getRawAxis(1);

    // These formulas match the physical wiring of Talai's drive base.
    double leftPower = forward + rotation;
    double rightPower = forward - rotation;

    m_leftMotor1.set(leftPower * 0.7);
    m_leftMotor2.set(leftPower * 0.7);
    m_rightMotor1.set(rightPower * 0.7);
    m_rightMotor2.set(rightPower * 0.7);

    // -------------------------------
    // INTAKE
    // -------------------------------
    if (m_stick.getRawButton(6)) {
      // Both intake motors rotate same direction on this robot.
      m_intakeLeft.set(0.55);
      m_intakeRight.set(0.55);
    } else {
      m_intakeLeft.set(0);
      m_intakeRight.set(0);
    }

    // -------------------------------
    // ELEVATOR CONTROL (No sensors used)
    // -------------------------------
    double rt = m_stick.getRawAxis(3);  // Up
    double lt = m_stick.getRawAxis(2);  // Down (disabled)

    SmartDashboard.putNumber("Right Trigger (Up)", rt);
    SmartDashboard.putNumber("Left Trigger (Down)", lt);

    // DOWN MOVEMENT IS BLOCKED FOR SAFETY
    if (lt > 0.05) {
      stopElevatorMotors();
      SmartDashboard.putString("ElevatorState", "LT pressed => STOPPED");

    } else {
      double elevatorSpeed = (rt * ELEVATOR_UP_MULTIPLIER);

      if (Math.abs(elevatorSpeed) < ELEVATOR_DEADBAND) {
        // Software HOLD mode simulates a physical brake
        m_elevatorMotor1.set(HOLD_POWER);
        m_elevatorMotor2.set(-HOLD_POWER);
        SmartDashboard.putString("ElevatorState", "Hold");

      } else {
        // Elevator UP movement only
        m_elevatorMotor1.set(elevatorSpeed);
        m_elevatorMotor2.set(-elevatorSpeed);
        SmartDashboard.putString("ElevatorState", "Up: " + elevatorSpeed);
      }

      SmartDashboard.putNumber("ElevatorSpeedCommand", elevatorSpeed);
    }
  }

  // ---------------------------------------------------------------------------
  // AUTONOMOUS
  //
  // Timing-based movement fully replaces encoders/sensors.
  // ---------------------------------------------------------------------------
  @Override
  public void autonomousInit() {
    autoState = AutoState.FORWARD;
    autoTimer.reset();
    autoTimer.start();
    SmartDashboard.putString("Auto State", autoState.toString());
  }

  @Override
  public void autonomousPeriodic() {
    double elapsed = autoTimer.get();

    switch (autoState) {

      case FORWARD:
        if (elapsed < forwardDuration) {
          // Drive forward: motor directions match Talai's wiring layout
          driveMotors(0.35, -0.35);
        } else {
          driveMotors(0, 0);
          autoState = AutoState.ELEVATOR;
          autoTimer.reset();
        }
        break;

      case ELEVATOR:
        // Lift elevator using timing as “virtual encoder”
        if (elapsed < elevatorUpDuration) {
          m_elevatorMotor1.set(elevatorUpPower);
          m_elevatorMotor2.set(-elevatorUpPower);
        } else {
          // Switch to brake/hold
          m_elevatorMotor1.set(HOLD_POWER);
          m_elevatorMotor2.set(-HOLD_POWER);

          autoState = AutoState.INTAKE;
          autoTimer.reset();
        }
        break;

      case INTAKE:
        if (elapsed < intakeDuration) {
          m_intakeLeft.set(0.2);
          m_intakeRight.set(0.22);
        } else {
          m_intakeLeft.set(0);
          m_intakeRight.set(0);
          autoState = AutoState.REVERSE;
          autoTimer.reset();
        }
        break;

      case REVERSE:
        if (elapsed < reverseDuration) {
          driveMotors(-0.35, 0.35);
        } else {
          driveMotors(0, 0);
          autoState = AutoState.DONE;
        }
        break;

      case DONE:
        driveMotors(0, 0);
        break;
    }

    SmartDashboard.putString("Auto State", autoState.toString());
  }

  // ---------------------------------------------------------------------------
  // TEST MODE
  // ---------------------------------------------------------------------------
  @Override
  public void testInit() {
    stopAllMotors();
    SmartDashboard.putString("Robot Mode", "TEST MODE");
  }

  @Override
  public void disabledInit() {
    m_emergencyLatched = false;
    stopAllMotors();
  }

  @Override
  public void testPeriodic() {
    double rt = m_stick.getRawAxis(3);
    double lt = m_stick.getRawAxis(2);

    if (lt > 0.05) {
      stopElevatorMotors();
    } else {
      double elevatorSpeed = rt * ELEVATOR_UP_MULTIPLIER;

      if (Math.abs(elevatorSpeed) < ELEVATOR_DEADBAND) {
        m_elevatorMotor1.set(HOLD_POWER);
        m_elevatorMotor2.set(-HOLD_POWER);
      } else {
        m_elevatorMotor1.set(elevatorSpeed);
        m_elevatorMotor2.set(-elevatorSpeed);
      }
    }

    SmartDashboard.putNumber("Test Elevator Speed", rt - lt);
  }

  // ---------------------------------------------------------------------------
  // UTILITY FUNCTIONS
  // ---------------------------------------------------------------------------
  private void stopAllMotors() {
    m_leftMotor1.set(0);
    m_leftMotor2.set(0);
    m_rightMotor1.set(0);
    m_rightMotor2.set(0);
    m_intakeLeft.set(0);
    m_intakeRight.set(0);
    m_elevatorMotor1.set(0);
    m_elevatorMotor2.set(0);
  }

  private void stopElevatorMotors() {
    m_elevatorMotor1.set(0);
    m_elevatorMotor2.set(0);
  }

  private void driveMotors(double leftPower, double rightPower) {
    m_leftMotor1.set(leftPower);
    m_leftMotor2.set(leftPower);
    m_rightMotor1.set(rightPower);
    m_rightMotor2.set(rightPower);
  }
}