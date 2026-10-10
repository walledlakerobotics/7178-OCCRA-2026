// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MecanumDriveConstants;

public class IntakeSubsystem extends SubsystemBase {

  SparkMax m_IntakeSparkMax = new SparkMax(IntakeConstants.kIntakeSparkID, MotorType.kBrushless);
  SparkClosedLoopController m_IntakeClosedLoop;

  DigitalInput m_LeftSensor = new DigitalInput(IntakeConstants.kLeftSensorChannel);
  DigitalInput m_RightSensor = new DigitalInput(IntakeConstants.kRightSensorChannel);

  public IntakeSubsystem() {

    m_IntakeClosedLoop = m_IntakeSparkMax.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
      config
          .smartCurrentLimit(MecanumDriveConstants.kSmartCurrentLimit)
          .idleMode(IdleMode.kBrake);

      config.closedLoop
          .pid(IntakeConstants.kPIDp, IntakeConstants.kPIDi, IntakeConstants.kPIDd);

      config.closedLoop.maxMotion
          .cruiseVelocity(IntakeConstants.kMaxVelocity)
          .maxAcceleration(IntakeConstants.kMaxAcceleration);

      config.inverted(IntakeConstants.kIntakeMotorInverted);

      m_IntakeSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
  private void intakeVelocityControl(double velocity) {
    m_IntakeClosedLoop.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
  }

  public void startIntake() {
    double velocity = IntakeConstants.kIntakeVelocity;


    intakeVelocityControl(velocity);
  }

  public void startReverseIntake() {
    double velocity = IntakeConstants.kIntakeVelocity;

    intakeVelocityControl(-velocity);
  }



  public void stopIntake() {
    double velocity = 0.0;

    intakeVelocityControl(velocity);
  }

  public Command runIntake() {
    return startEnd(this::startIntake, this::stopIntake);
  }

  public Command reverseIntake() {
    return startEnd(this::startReverseIntake, this::stopIntake);

  }

  public Trigger noBalls() {
    return new Trigger(() -> !getLeft() && !getRight());
    
  }

  public Trigger oneLeftBall() {
    return new Trigger(() -> getLeft());
  }

   public Trigger oneRightBall() {
    return new Trigger(() -> getRight());
  }

  public Trigger twoBalls() {
    return new Trigger(() -> getLeft() && getRight());
  }

  private Trigger test = twoBalls();

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/Left", getLeft());
    SmartDashboard.putBoolean("Intake/Right", getRight());
    SmartDashboard.putBoolean("Intake/Two", twoBalls().getAsBoolean());
    SmartDashboard.putBoolean("Intake/noBalls", noBalls().getAsBoolean());
  }

  public boolean getLeft() {
    return !m_LeftSensor.get();
  }

  public boolean getRight() {
    return !m_RightSensor.get();
  }

}


