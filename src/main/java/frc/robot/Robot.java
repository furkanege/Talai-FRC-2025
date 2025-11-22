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

public class Robot extends TimedRobot {

  // -------------------------------------------------------
  // MOTORLAR, JOYSTICK, DEĞİŞKENLER
  // -------------------------------------------------------
  private SparkMax m_leftMotor1;
  private SparkMax m_leftMotor2;
  private SparkMax m_rightMotor1;
  private SparkMax m_rightMotor2;
  private SparkMax m_intakeLeft;
  private SparkMax m_intakeRight;
  private SparkMax m_elevatorMotor1;
  private SparkMax m_elevatorMotor2;

  // Joystick tanımlama
  private Joystick m_stick;

  // Acil durdurma butonu
  private static final int EMERGENCY_STOP_BUTTON = 7;

  // Asansörü tutan ve yavaş hareketine etki eden sabitler
  private static final double HOLD_POWER = 0.30;
  private static final double ELEVATOR_UP_MULTIPLIER   = 1.0;
  private static final double ELEVATOR_DEADBAND = 0.01;

  // Otonom aşamalar
  private enum AutoState {FORWARD, ELEVATOR, INTAKE, REVERSE, DONE}
  private AutoState autoState;

  // Otonom süreler (kolay değişebilir)
  private final double forwardDuration = 4;    // 4 sn ileri
  private final double intakeDuration  = 1.5;  // 0.7 sn intake
  private final double reverseDuration = 1;    // 1 sn geri

  // Yeni: asansörü yukarı çıkarma süresi ve gücü (otonom)
  private final double elevatorUpDuration = 0.15; // 1 sn
  private final double elevatorUpPower    = 0.7; 

  // Timer
  private Timer autoTimer;

  // -------------------------------------------------------
  // robotInit
  // -------------------------------------------------------
  @Override
  public void robotInit() {
    m_leftMotor1 = new SparkMax(1, MotorType.kBrushed);
    m_leftMotor2 = new SparkMax(2, MotorType.kBrushed);
    m_rightMotor1 = new SparkMax(3, MotorType.kBrushed);
    m_rightMotor2 = new SparkMax(4, MotorType.kBrushed);
    m_intakeLeft = new SparkMax(5, MotorType.kBrushed);
    m_intakeRight = new SparkMax(6, MotorType.kBrushed);
    m_elevatorMotor1 = new SparkMax(7, MotorType.kBrushed);
    m_elevatorMotor2 = new SparkMax(8, MotorType.kBrushed);

    SparkBaseConfig elev1Config = new SparkMaxConfig();
    elev1Config.idleMode(IdleMode.kBrake);
    m_elevatorMotor1.configure(elev1Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkBaseConfig elev2Config = new SparkMaxConfig();
    elev2Config.idleMode(IdleMode.kBrake);
    m_elevatorMotor2.configure(elev2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_stick = new Joystick(0);
    autoTimer = new Timer();
  }

  // -------------------------------------------------------
  // TELEOP
  // -------------------------------------------------------
  @Override
  public void teleopPeriodic() {
    // 7. BUTON => TÜM MOTORLARI DURDUR (ACİL STOP)
    if (m_stick.getRawButton(EMERGENCY_STOP_BUTTON)) {
      stopAllMotors();
      SmartDashboard.putBoolean("Emergency Stop Activated", true);
      return;
    } else {
      SmartDashboard.putBoolean("Emergency Stop Activated", false);
    }

    // SÜRÜŞ
    double forward = -m_stick.getRawAxis(4);
    double rotation = -m_stick.getRawAxis(1);
    double leftPower = forward + rotation;
    double rightPower = forward - rotation;
    m_leftMotor1.set(leftPower * 0.7);
    m_leftMotor2.set(leftPower * 0.7);
    m_rightMotor1.set(rightPower * 0.7);
    m_rightMotor2.set(rightPower * 0.7);

    // INTAKE
    if (m_stick.getRawButton(6)) {
      m_intakeLeft.set(0.55);
      m_intakeRight.set(0.55);
    } else {
      m_intakeLeft.set(0);
      m_intakeRight.set(0);
    }

    // ELEVATOR KONTROLÜ
    double rt = m_stick.getRawAxis(3); // Sağ tetik -> Yukarı
    double lt = m_stick.getRawAxis(2); // Sol tetik -> Aşağı
    SmartDashboard.putNumber("Right Trigger (Up)", rt);
    SmartDashboard.putNumber("Left Trigger (Down)", lt);

    // LT basılırsa => yalnızca asansör motorlarını durdur (stopElevatorMotors)
    if (lt > 0.05) {
      stopElevatorMotors();
      SmartDashboard.putString("ElevatorState", "LT pressed => Elevator Motors STOP");

    } else {
      // Aşağı yön devre dışı; sadece yukarı yön + hold
      double elevatorSpeed = (rt * ELEVATOR_UP_MULTIPLIER);

      if (Math.abs(elevatorSpeed) < ELEVATOR_DEADBAND) {
        // Ölü bölge => HOLD
        m_elevatorMotor1.set(HOLD_POWER);
        m_elevatorMotor2.set(-HOLD_POWER);
        SmartDashboard.putString("ElevatorState", "Hold");
      } else {
        // Yukarı
        m_elevatorMotor1.set(elevatorSpeed);
        m_elevatorMotor2.set(-elevatorSpeed);
        SmartDashboard.putString("ElevatorState", "Up: " + elevatorSpeed);
      }
      SmartDashboard.putNumber("ElevatorSpeedCommand", elevatorSpeed);
    }
  }

  // -------------------------------------------------------
  // OTONOM
  // -------------------------------------------------------
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
          driveMotors(0.35, -0.35);
        } else {
          driveMotors(0, 0);
          // Ardından ELEVATOR aşamasına geç
          autoState = AutoState.ELEVATOR;
          autoTimer.reset();
        }
        break;

      case ELEVATOR:
        // Asansörü yukarı => elevatorUpDuration, elevatorUpPower
        if (elapsed < elevatorUpDuration) {
          m_elevatorMotor1.set(elevatorUpPower);
          m_elevatorMotor2.set(-elevatorUpPower);
        } else {
          // Süre bitince hold
          m_elevatorMotor1.set(HOLD_POWER);
          m_elevatorMotor2.set(-HOLD_POWER);
          // Sonraki aşama => INTAKE
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

  // -------------------------------------------------------
  // TEST
  // -------------------------------------------------------
  @Override
  public void testInit() {
    stopAllMotors();
    SmartDashboard.putString("Robot Mode", "TEST MODE");
  }

  @Override
  public void testPeriodic() {
    double rt = m_stick.getRawAxis(3);
    double lt = m_stick.getRawAxis(2);

    if (lt > 0.05) {
      stopElevatorMotors();
    } else {
      double elevatorSpeed = (rt * ELEVATOR_UP_MULTIPLIER);
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

  // -------------------------------------------------------
  // YARDIMCI METOTLAR
  // -------------------------------------------------------
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

  // Sadece asansör motorlarını durdurur.
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