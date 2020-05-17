/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.commands;

import ca.warp7.frc2020.subsystems.DriveTrain;
import ca.warp7.frc2020.subsystems.Flywheel;
import ca.warp7.frc2020.subsystems.Limelight;
import ca.warp7.frc2020.subsystems.Turret;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.opencv.core.Mat;

import java.util.function.DoubleSupplier;

import static ca.warp7.frc2020.Constants.kFlywheelMetersPerRotation;
import static ca.warp7.frc2020.Constants.kReleaseAngle;

public class FlywheelAndTurretCommand extends CommandBase {
    private DoubleSupplier wantedRPS;
    private Flywheel flywheel = Flywheel.getInstance();
    private Limelight limelight = Limelight.getInstance();
    private DriveTrain driveTrain = DriveTrain.getInstance();
    private Turret turret = Turret.getInstance();

    public FlywheelAndTurretCommand(DoubleSupplier wantedRPS) {
        this.wantedRPS = wantedRPS;
        addRequirements(flywheel,turret);
    }

    @Override
    public void execute() {
        double unadjustedRPS = wantedRPS.getAsDouble();

        double theta = limelight.getSmoothHorizontalAngle();
        double rv = driveTrain.getRobotVelocity();
        double rvGoal = rv * Math.cos(Math.toRadians(theta));
        double rvHor = rv * Math.sin(Math.toRadians(theta));

        double x = limelight.getCameraToTarget();
        double h = 1.5303;//TODO
        double m = 2 * 9.81 * Math.cos(kReleaseAngle) *
                (x * Math.sin(kReleaseAngle) - h * Math.cos(kReleaseAngle));

        double maxAdj = 2.5;
        double adjustment;
        if (Math.abs(rvGoal) >= 0.1) {
            if (x >= h / Math.tan(kReleaseAngle)) {
                adjustment = (x * Math.sqrt(Math.pow((rvGoal * Math.sin(kReleaseAngle)), 2) + m)
                        + rvGoal * (2 * h * Math.cos(kReleaseAngle) - x * Math.sin(kReleaseAngle)))
                        / (x * Math.sqrt(m));
                if (adjustment > maxAdj && rvGoal > 0) {
                    adjustment = maxAdj;
                } else if (adjustment < 1 / maxAdj && rvGoal < 0) {
                    adjustment = 1/ maxAdj;
                }
            } else if (rvGoal > 0) {
                adjustment = maxAdj;
            } else adjustment = 1 / maxAdj;
        } else adjustment = 1;

        double adjustedRPS = unadjustedRPS * adjustment;
        double angle = Math.asin(rvHor/(adjustedRPS*kFlywheelMetersPerRotation*Math.cos(Math.toRadians(theta))+rvGoal));
        turret.setAngleRadians(angle);
        flywheel.setTargetRPS(adjustedRPS);
        flywheel.calcOutput();
        double currentRPS = flywheel.getRPS();

        SmartDashboard.putNumber("rps", currentRPS);
        SmartDashboard.putNumber("target", unadjustedRPS);
        SmartDashboard.putNumber("adjustment", adjustment);
        SmartDashboard.putNumber("adjusted", adjustedRPS);
        SmartDashboard.putNumber("err", unadjustedRPS - currentRPS);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setTargetRPS(0);
    }
}
