package frc.robot.subsystems.drivetrain;

import org.walledlakerobotics.pinpointrio.EncoderDirection;
import org.walledlakerobotics.pinpointrio.Pinpoint;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MecanumDriveConstants;
import com.ctre.phoenix6.hardware.Pigeon2;

@Logged
public class HDriveOdometry extends SubsystemBase {

    private final Pigeon2 m_gyro = new Pigeon2(MecanumDriveConstants.kGyroCanID);
    Pinpoint m_Pinpoint = new Pinpoint(Port.kUSB);
    DifferentialDriveOdometry m_OdometryBase;
    Pose2d m_Pose;
    Double dt;
    Double lastTime;
    Double dx;
    Double lastX;
    Double dy;
    Double lastY;
    DifferentialDriveOdometry m_OdometryPartial;
    Double Xoffset;
    Double Yoffset;

    public HDriveOdometry() {
        m_Pinpoint.setEncoderDirections(EncoderDirection.FORWARD, EncoderDirection.FORWARD);
        m_Pinpoint.setEncoderOffsets(0.0, 0.0);
        m_Pinpoint.setEncoderResolution(4096 / (38 * Math.PI));

        m_Pose = new Pose2d();
        lastTime = 0.0;
        lastX = 0.0;
        lastY = 0.0;
        Xoffset = 0.0;
        Yoffset = 0.0;
    }

    public boolean checkPinpoint() {
        return m_Pinpoint.getConnected();
    }

    public double getXDistMetersUnFixed() {
        return ((m_Pinpoint.getEncoderX() / (4096 / (38 * Math.PI))) / 1000.0) - Xoffset;
    }

    public double getXDistMeters() {
        return getXDistMetersUnFixed() - (0.2749 * m_gyro.getRotation2d().getRadians());
    }

    public double getRawX() {
        return m_Pinpoint.getEncoderX();
    }

    public double getYDistMeters() {
        return ((-m_Pinpoint.getEncoderY() / (4096 / (38 * Math.PI))) / 1000.0) - Yoffset;
    }

    public double getRawY() {
        return m_Pinpoint.getEncoderY();
    }

    public double getDt() {
        return dt;
    }

     public double getDx() {
        return dx;
    }

     public double getDy() {
        return dy;
    }

    public Translation2d getTranslation2d() {
        double headingSin = m_gyro.getRotation2d().getSin();
        double headingCos = m_gyro.getRotation2d().getCos();
        double dxFieldRelative = dx * headingCos - dy * headingSin;
        double dyFieldRelative = dy * headingCos + dx * headingSin;
        return new Translation2d((dxFieldRelative), (dyFieldRelative));
    }

    public ChassisSpeeds getChassisSpeeds() {
        double vx = dx / dt;
        double vy = dy / dt;
        double vr = (m_gyro.getAngularVelocityZWorld().getValueAsDouble() / 180) * Math.PI;

        return new ChassisSpeeds(vx, vy, vr);
    }

    public Pose2d getPose() {
        return m_Pose;
    }

    public void resetPose(Pose2d pose) {
        lastX = 0.0;
        lastY = 0.0;
        m_Pose = pose;
        m_gyro.setYaw(pose.getRotation().getDegrees());
        Xoffset += getXDistMetersUnFixed() - (0.2749 * pose.getRotation().getRadians());
        Yoffset += getYDistMeters();
    }
 
    @Override 
    public void periodic() {
        double st = Timer.getTimestamp();
        dt = st - lastTime;
        lastTime = st;

        double sx = getXDistMeters();
        double sy = getYDistMeters();

        dx = sx - lastX;
        dy = sy - lastY;

        lastX = sx;
        lastY = sy;

        m_Pose = new Pose2d(m_Pose.getTranslation().plus(getTranslation2d()), m_gyro.getRotation2d());
    }


}
