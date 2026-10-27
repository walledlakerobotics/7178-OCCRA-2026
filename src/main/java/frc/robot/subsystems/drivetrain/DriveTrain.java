// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.MecanumDriveConstants;
import com.ctre.phoenix6.hardware.Pigeon2;
/**
 * A subsystem that controls the robot's drive train.
 */
public class DriveTrain extends SubsystemBase {
    // drive train motors
    private final SparkMax m_frontLeftMotor = new SparkMax(MecanumDriveConstants.kFrontLeftSparkID, MotorType.kBrushless);
    private final SparkMax m_rearLeftMotor = new SparkMax(MecanumDriveConstants.kBackLeftSparkID, MotorType.kBrushless);
    private final SparkMax m_frontRightMotor = new SparkMax(MecanumDriveConstants.kFrontRightSparkID, MotorType.kBrushless);
    private final SparkMax m_rearRightMotor = new SparkMax(MecanumDriveConstants.kBackRightSparkID, MotorType.kBrushless);

    // encoders
    private final RelativeEncoder m_frontLeftEncoder;
    private final RelativeEncoder m_rearLeftEncoder;
    private final RelativeEncoder m_frontRightEncoder;
    private final RelativeEncoder m_rearRightEncoder;

    // closed loop (pid) controllers
    private final SparkClosedLoopController m_frontLeftClosedLoop;
    private final SparkClosedLoopController m_rearLeftClosedLoop;
    private final SparkClosedLoopController m_frontRightClosedLoop;
    private final SparkClosedLoopController m_rearRightClosedLoop;

    //Heading Alignment PID Controller
    private final PIDController m_headingAlignController = new PIDController(
        MecanumDriveConstants.kHAPIDp, 
        MecanumDriveConstants.kHAPIDi, 
        MecanumDriveConstants.kHAPIDd);

    // TEMPORARY
    private double m_SRCF = 0.0;

    //Lateral Alignment PID Controller
    private final PIDController m_lateralAlignController = new PIDController(
        MecanumDriveConstants.kLAPIDp, 
        MecanumDriveConstants.kLAPIDi, 
        MecanumDriveConstants.kLAPIDd);

        Double m_lateralAlignSetpoint = 2.0;

    // gyro
    private final Pigeon2 m_gyro = new Pigeon2(MecanumDriveConstants.kGyroCanID);

    // calculates odometry
    private final MecanumDriveOdometry m_odometry;

    // displays robot position on field
    private final Field2d m_field = new Field2d();

    private Rotation2d m_fieldRelativeOffset = Rotation2d.kZero;

    //private Rotation2d m_rotationSetpoint = Rotation2d.kZero;
    //private PIDController m_rotationController = new PIDController(MecanumDriveConstants.kRotationP, MecanumDriveConstants.kRotationI,
        //MecanumDriveConstants.kRotationD);

    private ShuffleboardTab m_driveTab = Shuffleboard.getTab(getName());


    /**
     * Constructs a {@link DriveTrain}.
     */
    public DriveTrain() {
        

        SparkMaxConfig config = new SparkMaxConfig();
        

        // sets the idle mode, the smart current limit, and the inversion
        config
                .smartCurrentLimit(MecanumDriveConstants.kSmartCurrentLimit)
                .idleMode(IdleMode.kBrake);

        // sets the PID
        config.closedLoop
                .pid(MecanumDriveConstants.kPIDp, MecanumDriveConstants.kPIDd, MecanumDriveConstants.kPIDd);

        // sets max velocity and acceleration
        config.closedLoop.maxMotion
                .maxAcceleration(MecanumDriveConstants.kMaxAcceleration);

        // sets encoder conversion factors
        config.encoder
                .positionConversionFactor(MecanumDriveConstants.kPosConvFactor)
                .velocityConversionFactor(MecanumDriveConstants.kVelConvFactor);

        // left side motors
        config.inverted(MecanumDriveConstants.kLeftMotorsInverted);

        m_frontLeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rearLeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // right side motors
        config.inverted(MecanumDriveConstants.kRightMotorsInverted);

        m_frontRightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rearRightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // get encoders
        m_frontLeftEncoder = m_frontLeftMotor.getEncoder();
        m_rearLeftEncoder = m_rearLeftMotor.getEncoder();
        m_frontRightEncoder = m_frontRightMotor.getEncoder();
        m_rearRightEncoder = m_rearRightMotor.getEncoder();

        // get closed loop controllers
        m_frontLeftClosedLoop = m_frontLeftMotor.getClosedLoopController();
        m_rearLeftClosedLoop = m_rearLeftMotor.getClosedLoopController();
        m_frontRightClosedLoop = m_frontRightMotor.getClosedLoopController();
        m_rearRightClosedLoop = m_rearRightMotor.getClosedLoopController();

        m_odometry = new MecanumDriveOdometry(MecanumDriveConstants.kDriveKinematics, m_gyro.getRotation2d(), new MecanumDriveWheelPositions(
            m_frontLeftEncoder.getPosition(),
            m_frontRightEncoder.getPosition(),
            m_rearLeftEncoder.getPosition(),
            m_rearRightEncoder.getPosition()));

        m_driveTab.addNumber("Robot X (m)", () -> m_odometry.getPoseMeters().getX());
        m_driveTab.addNumber("Robot Y (m)", () -> m_odometry.getPoseMeters().getY());
        m_driveTab.addNumber("Robot Heading (deg)", () -> m_gyro.getRotation2d().getDegrees());
        m_driveTab.addBoolean("NavXOn", () -> m_gyro.isConnected());

        m_driveTab.add("Field", m_field);
//(Pose2d newPose2d) -> resetOdometry(newPose2d)
        AutoBuilder.configure(m_odometry::getPoseMeters, this::resetOdometry, this::getChassisSpeeds,
                this::drive, AutonConstants.kPathFollowingController, AutonConstants.kRobotConfig, () -> false, this);
        
        m_driveTab.addNumber("NavX Bearing", () -> m_gyro.getRotation3d().toRotation2d().getDegrees());
    }

    /**
     * Drives the robot using curvature drive.
     * 
     * @param xSpeed    The robot's speed along the X axis [-1, 1].
     *                  Forward is positive.
     * @param ySpeed    The robot's speed along the Y axis [-1, 1].
     *                  Left is positive.
     * @param zRotation The normalized curvature [-1, 1].
     *                  Counterclockwise is positive.
     */
    public void drive(double xSpeed, double ySpeed, double zRotation, boolean fieldRelative) {
        xSpeed *= MecanumDriveConstants.kForwardMaxSpeed;
        ySpeed *= MecanumDriveConstants.kStrafeMaxSpeed;
        zRotation *= MecanumDriveConstants.kTurnMaxSpeed;

        // m_rotationSetpoint = m_rotationSetpoint
        //         .plus(Rotation2d.fromRadians(zRotation).times(TimedRobot.kDefaultPeriod));

        // // continuously adjust for potential drift
        // zRotation = m_rotationController.calculate(m_gyro.getRotation2d().getRadians(),
        //         m_rotationSetpoint.getRadians());

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, zRotation);

        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
            m_gyro.getRotation2d().minus(m_fieldRelativeOffset));
        }

        drive(speeds);
    }

    /**
     * Drives the robot based on raw robot relative {@link ChassisSpeeds}.
     * 
     * @param speeds The speeds to drive the robot with.
     */
    public void drive(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = MecanumDriveConstants.kDriveKinematics.toWheelSpeeds(speeds);

        m_frontLeftClosedLoop.setSetpoint(wheelSpeeds.frontLeftMetersPerSecond, ControlType.kMAXMotionVelocityControl);
        m_rearLeftClosedLoop.setSetpoint(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kMAXMotionVelocityControl);
        m_frontRightClosedLoop.setSetpoint(wheelSpeeds.frontRightMetersPerSecond, ControlType.kMAXMotionVelocityControl);
        m_rearRightClosedLoop.setSetpoint(wheelSpeeds.rearRightMetersPerSecond, ControlType.kMAXMotionVelocityControl);
    }

    /**
     * Creates a {@link Command} that drives the robot based on joystick or axis
     * inputs.
     * 
     * @param xSpeedSupplier    Supplies the axis value for forward/backward
     *                          movement in the range [-1, 1]. Back is
     *                          positive. It will be transformed based on the
     *                          sensitivity,
     *                          deadband, and multiplier values.
     * @param ySpeedSupplier    Supplies the axis value for left/right
     *                          movement in the range [-1, 1]. Right is positive.
     *                          It will be transformed based on the sensitivity,
     *                          deadband, and multiplier values.
     * @param zRotationSupplier Supplies the axis value for rotation in the range
     *                          [-1, 1]. Clockwise is positive. It will be
     *                          transformed based on the sensitivity,
     *                          deadband, and multiplier values.
     * 
     * @return A Command that drives the robot based on joystick inputs.
     */

    public double sqProp(Double Ctrl) {
        if (Ctrl < 0) {
            return Ctrl * Ctrl;
        } else {
            return -1 * Ctrl * Ctrl;
        }
    }

    public Command driveJoysticks(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier zRotationSupplier, BooleanSupplier b) {
        return run(() -> {
            double xSpeed = -xSpeedSupplier.getAsDouble();
            double ySpeed = -ySpeedSupplier.getAsDouble();
            double zRotation = -zRotationSupplier.getAsDouble() * 0.8;

            xSpeed = MathUtil.applyDeadband(xSpeed, MecanumDriveConstants.kDeadband);
            ySpeed = MathUtil.applyDeadband(ySpeed, MecanumDriveConstants.kDeadband);
            zRotation = MathUtil.applyDeadband(zRotation, MecanumDriveConstants.kDeadband);

            drive(xSpeed, ySpeed, zRotation, b.getAsBoolean());
        });
    }

    public Command resetLateralAlignSetpoint() {
        return runOnce(() -> m_lateralAlignSetpoint = m_odometry.getPoseMeters().getY());
    }

    public Command driveJoysticksGoalAlign(DoubleSupplier xSpeedSupplier, BooleanSupplier b) {
        return run(() -> {
            double xSpeed = -xSpeedSupplier.getAsDouble();
            double ySpeed = m_lateralAlignController.calculate(m_odometry.getPoseMeters().getY(), m_lateralAlignSetpoint);
            double zRotation = m_headingAlignController.calculate(m_gyro.getRotation2d().getDegrees(), 180);

            xSpeed = MathUtil.applyDeadband(xSpeed, MecanumDriveConstants.kDeadband);
            // zRotation = zRotation * ySpeed * MecanumDriveConstants.kSRCF;
            // zRotation = zRotation * ySpeed * m_SRCF;

            drive(xSpeed, ySpeed, zRotation, b.getAsBoolean());
        });
    }

     public Command driveJoysticksGoalAlignHdgOnly(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier, BooleanSupplier b) {
        return run(() -> {
            double SRCF = SmartDashboard.getNumber("SCRF", 0.0);
            double xSpeed = -xSpeedSupplier.getAsDouble();
            double ySpeed = -ySpeedSupplier.getAsDouble();
            double zRotation = m_headingAlignController.calculate(m_gyro.getRotation2d().getDegrees(), 180);

            xSpeed = MathUtil.applyDeadband(xSpeed, MecanumDriveConstants.kDeadband);
            zRotation = zRotation + ySpeed * SRCF;

            drive(xSpeed, ySpeed, zRotation, b.getAsBoolean());
        });
    }

    /**
     * Stops the robot.
     */
    //public void stopDrive() {
    //    resetRotationSetpoint();
    //    drive(0, 0, 0);
    //}

    /**
     * Gets the current robot relative {@link ChassisSpeeds}.
     * 
     * @return The current robot relative chassis speeds.
     */
    public ChassisSpeeds getChassisSpeeds() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                m_frontLeftEncoder.getVelocity(),
                m_frontRightEncoder.getVelocity(),
                m_rearLeftEncoder.getVelocity(),
                m_rearRightEncoder.getVelocity());

        ChassisSpeeds speeds = MecanumDriveConstants.kDriveKinematics.toChassisSpeeds(wheelSpeeds);

        return speeds;
    }

    /**
     * Gets the current wheel positions in meters.
     * 
     * @return The {@link MecanumDriveWheelPositions} representing the current wheel
     *         positions in meters.
     */
    public MecanumDriveWheelPositions getWheelPositions() {
        return new MecanumDriveWheelPositions(
                m_frontLeftEncoder.getPosition(),
                m_frontRightEncoder.getPosition(),
                m_rearLeftEncoder.getPosition(),
                m_rearRightEncoder.getPosition());
    }

    /**
     * Resets the current tracked robot position to the specified {@link Pose2d}.
     * 
     * @param pose The Pose2d object.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(m_gyro.getRotation2d(), getWheelPositions(), pose);
    }

    /**
     * Resets the current rotation setpoint to the current rotation.
     */
    //private void resetRotationSetpoint() {
    //    m_rotationSetpoint = m_gyro.getRotation2d();
    //    m_rotationController.reset();
    //}

    /**
     * Sets the {@link IdleMode} for all drivetrain motors. This will not persist
     * through power cycles.
     * 
     * @param mode The idle mode to set.
     */
    public void setIdleMode(IdleMode mode) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(mode);

        for (SparkMax motor : new SparkMax[] { m_frontLeftMotor, m_rearLeftMotor, m_frontRightMotor,
                m_rearLeftMotor }) {
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    /**
     * Creates a {@link Command} that resets the drive train's field relative controls offset.
     * @return The command.
     */
    public Command resetFieldRelative() {
        return runOnce(() -> {
            if (DriverStation.isFMSAttached())
                return;

            m_fieldRelativeOffset = m_gyro.getRotation2d();
        }).ignoringDisable(true);
    }

    @Override
    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), getWheelPositions());

        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    @Override
    public String getName() {
        return "Drive Train";
    }
}