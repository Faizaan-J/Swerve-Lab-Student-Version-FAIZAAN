package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveDrive;

public class TeleopDrive extends Command {
    private final SwerveDrive swerveDrive;
    private final CommandXboxController controller;
    
    public TeleopDrive(SwerveDrive swerveDrive, CommandXboxController controller) {
        this.controller = new CommandXboxController(IOConstants.DRIVER_CONTROLLER_PORT);
        this.swerveDrive = swerveDrive;
        addRequirements(this.swerveDrive);
    }
    
    @Override
    public void execute() {
        /***
         * Look into controller.getRawAxis() method
         * Also look into OIConstants in java to make sure the controller syncs correctly
         * Might have to negate something 🤔🤔🤔. Do some trial and error.
         */
        double xInput = -controller.getRawAxis(IOConstants.LEFT_X_AXIS);
        double yInput = -controller.getRawAxis(IOConstants.LEFT_Y_AXIS);
        double rotInput = controller.getRawAxis(IOConstants.RIGHT_X_AXIS);

        SmartDashboard.putNumber("xInput", xInput);
        SmartDashboard.putNumber("yInput", yInput);
        SmartDashboard.putNumber("rotInput", rotInput);

        /***
         * Apply a deadband to these inputs. Look at OIConstants.
         */
        xInput = MathUtil.applyDeadband(xInput, IOConstants.DEADBAND);
        yInput = MathUtil.applyDeadband(yInput, IOConstants.DEADBAND);
        rotInput = MathUtil.applyDeadband(rotInput, IOConstants.DEADBAND);
        
        // Challenge: square the inputs while preserving the signs
        xInput *= Math.abs(xInput);
        yInput *= Math.abs(yInput);

        /***
         * Use the inputs and look at constants for the following: the max speed, the speed multipler. What do we do with these?
         */
        double xSpeed = xInput * IOConstants.SPEED_MULTIPLIER * SwerveConstants.MAX_SPEED_MPS;
        double ySpeed = yInput * IOConstants.SPEED_MULTIPLIER * SwerveConstants.MAX_SPEED_MPS;
        double rotSpeed = rotInput * IOConstants.SPEED_MULTIPLIER * SwerveConstants.MAX_ANGULAR_SPEED;
        
        /***
         * Fix the error here. What are the arguments to the method? Hint: look at the end method below...
         */
        swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rotSpeed);
    }
    
    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(new Translation2d(0, 0), 0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}