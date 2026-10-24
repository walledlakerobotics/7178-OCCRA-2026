// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.opencv.video.KalmanFilter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MecanumDriveConstants;

public class ElevatorSubsystem extends SubsystemBase {

  SparkMax m_ElevatorSparkMax = new SparkMax(ElevatorConstants.kElevatorSparkID, MotorType.kBrushless);
  SparkClosedLoopController m_ElevatorClosedLoop;
  RelativeEncoder m_ElevatorEncoder;
  
  //Unhandled exception instantiating robot edu.wpi.first.hal.AnalogJNI edu.wpi.first.hal.util.AllocationException: Code: -1030
  //AnalogInput m_StringPodAnalogInput = new AnalogInput(ElevatorConstants.kAnalogInput);
  PIDController m_ElevatorPidController = new PIDController(ElevatorConstants.kPPIDp, ElevatorConstants.kPPIDi, ElevatorConstants.kPPIDd);
  Double m_GoalHeight = 0.0;
  Double m_GoalVelocity;
  Double m_CurrentHeight;
  AnalogInput m_StringPodAnalogInput = new AnalogInput(ElevatorConstants.kAnalogInput);

  public ElevatorSubsystem() {
    m_ElevatorClosedLoop = m_ElevatorSparkMax.getClosedLoopController();
    m_ElevatorEncoder = m_ElevatorSparkMax.getEncoder();
    m_CurrentHeight = 0.0;

    SparkMaxConfig config = new SparkMaxConfig();
      config
          .smartCurrentLimit(ElevatorConstants.kElevatorSmartCurrentLimit)
          .idleMode(IdleMode.kBrake);

      config.encoder
          .positionConversionFactor(ElevatorConstants.kPosConvFactor)
          .velocityConversionFactor(ElevatorConstants.kVelConvFactor);

      config.closedLoop
          .pid(ElevatorConstants.kPIDp, ElevatorConstants.kPIDi, ElevatorConstants.kPIDd);

      config.closedLoop.maxMotion
          .cruiseVelocity(ElevatorConstants.kMaxVelocity)
          .maxAcceleration(ElevatorConstants.kMaxAcceleration);

      config.inverted(ElevatorConstants.kElevatorMotorInverted);

      m_ElevatorSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      SmartDashboard.putNumber("Elevator/Feedforward", 0.05);

  }

  public double getHeight() {
    return (m_StringPodAnalogInput.getVoltage() - 0.32) * ElevatorConstants.kStringPodPositionConversionFactor;
  }

  public Command elevatorSetPosition(double position) {
    return runOnce(() -> {
            m_GoalHeight = position;
         });
  }

  private void elevatorVelocityControl(double velocity, double feedforward) {
    m_ElevatorClosedLoop.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, feedforward);
  }

  public double clampSpeed(double speed) {
    double height = getHeight();
    if (speed > (ElevatorConstants.kMaxHeightInches - height) * ElevatorConstants.kSpeedClampCoefficient) {
      return (ElevatorConstants.kMaxHeightInches - height) * ElevatorConstants.kSpeedClampCoefficient;
    } else if (speed < -(height * ElevatorConstants.kSpeedClampCoefficient)){
      return -height * ElevatorConstants.kSpeedClampCoefficient;
    } else {
      return speed;
    }
  }

  public Command elevatorManualCommand(DoubleSupplier velocitySupplier) {
    return run(() -> {
            double velocity = MathUtil.applyDeadband(
                clampSpeed(-velocitySupplier.getAsDouble()) , ElevatorConstants.kElevatorManualControlDeadband);
  

            elevatorVelocityControl(velocity, 0.0);
            SmartDashboard.putNumber("Elevator Input", velocity);
            SmartDashboard.putNumber("Elevator Vel", m_ElevatorEncoder.getVelocity());
        });
  }

  @Override
  public void periodic() {
    if (Math.abs(m_GoalHeight - getHeight()) < 0.6 ) {
      m_GoalVelocity = 0.0;
    } else {
      m_GoalVelocity = m_ElevatorPidController.calculate(getHeight(), m_GoalHeight);
    }

    elevatorVelocityControl(clampSpeed(m_GoalVelocity), SmartDashboard.getNumber("Elevator/Feedforward", 0.05));
    SmartDashboard.putNumber("HeightVoltage", getHeight());
    SmartDashboard.putNumber("SPVoltage", m_StringPodAnalogInput.getVoltage());
  }

}


