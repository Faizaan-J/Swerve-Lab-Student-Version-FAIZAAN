package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {
    private static final SwerveDrive instance;

    static {
        instance = new SwerveDrive(
                new SwerveModule(),
                new SwerveModule(),
                new SwerveModule(),
                new SwerveModule()
        );
    }

    public static SwerveDrive getInstance() {
        return instance;
    }

    private final SwerveModule[] modules;

    private final SwerveDriveOdometry odometry;

    private double gyroAngleRad = 0.0;

    private final Field2d field;

    public SwerveDrive(SwerveModule... modules) {
        this.modules = modules;

        odometry = new SwerveDriveOdometry(
                SwerveConstants.KINEMATICS,
                getRotation2d(),
                new SwerveModulePosition[] {
                        modules[0].getModulePosition(),
                        modules[1].getModulePosition(),
                        modules[2].getModulePosition(),
                        modules[3].getModulePosition()
                });

        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public void drive(Translation2d velocity, double rot) {
        ChassisSpeeds speeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), rot);
        
        /***
         * Hint: Look for SwerveDriveKinematics in this project. Might need to do some research.
         */
        SwerveModuleState[] states = Constants.SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        
        /***
         * What's missing here? Think about what the point of this method is?
         */
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_MPS);
        
        setDesiredStates(states);
    }

    public void setXMode() {
        SwerveModuleState[] states = {
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
        };

        setDesiredStates(states);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setDesiredStates(SwerveModuleState[] states) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    public void resetOdometry(Pose2d newPose) {
        odometry.resetPosition(
                getRotation2d(),
                new SwerveModulePosition[] {
                        modules[0].getModulePosition(),
                        modules[1].getModulePosition(),
                        modules[2].getModulePosition(),
                        modules[3].getModulePosition()
                },
                newPose);

        gyroAngleRad = newPose.getRotation().getRadians();
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(gyroAngleRad);
    }

    public void zeroHeading() {
        gyroAngleRad = 0.0;
    }

    @Override
    public void periodic() {
        odometry.update(
                getRotation2d(),
                new SwerveModulePosition[] {
                        modules[0].getModulePosition(),
                        modules[1].getModulePosition(),
                        modules[2].getModulePosition(),
                        modules[3].getModulePosition()
                });

        field.setRobotPose(getPose());

        SmartDashboard.putNumber("Robot X", getPose().getX());
        SmartDashboard.putNumber("Robot Y", getPose().getY());
        SmartDashboard.putNumber("Robot Angle", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro Angle", Math.toDegrees(gyroAngleRad));
    }
}