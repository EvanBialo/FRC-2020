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
    private DoubleSupplier rpsAdj;
    private Flywheel flywheel = Flywheel.getInstance();
    private Limelight limelight = Limelight.getInstance();
    private DriveTrain driveTrain = DriveTrain.getInstance();
    private Turret turret = Turret.getInstance();

    public FlywheelAndTurretCommand(DoubleSupplier rpsAdj) {
        this.rpsAdj = rpsAdj;
        addRequirements(flywheel, turret);
    }

    @Override
    public void execute() {

//        double rpsAdj = rpsAdj.getAsDouble();

        double g = -9.807;

        double theta = limelight.getSmoothHorizontalAngle();
        double rv = driveTrain.getRobotVelocity();
        double rvGoal = rv * Math.cos(Math.toRadians(theta));
        double rvHor = rv * Math.sin(Math.toRadians(theta));

        double x = limelight.getCameraToTarget();
        double h = 1.5303;//TODO

        double maxAdj = 4;
        double adjustedX;
        if (Math.abs(rvGoal) >= 0.1) {
            if (x > h / Math.tan(kReleaseAngle) - (rvGoal < 0 ? rvGoal * Math.tan(kReleaseAngle) / (2 * 9.81) : 0)) {
                double s = x * Math.sin(kReleaseAngle) - h * Math.cos(kReleaseAngle);
                double startVel = rvGoal * Math.sin(kReleaseAngle);
                double endVel = Math.sqrt(startVel * startVel + 2 * g * (s * Math.cos(kReleaseAngle)));
                double aveVel = (startVel + endVel) / 2;
                double vx = aveVel * (-x / s);
                adjustedX = x * (vx - rvGoal) / vx;
            } else if (rvGoal > 0) {
                adjustedX = x + maxAdj;
            } else adjustedX = x - maxAdj;
        } else adjustedX = x;

        double currentRPS = flywheel.getRPS();
        double adjustedRPS = Flywheel.calculateOptimalCloseShotRPS(adjustedX);
        double angle = Math.asin(rvHor / (adjustedRPS * kFlywheelMetersPerRotation * Math.cos(Math.toRadians(theta)) + rvGoal));
        turret.setAngleRadians(angle);
        flywheel.setTargetRPS(adjustedRPS);
        flywheel.calcOutput();

        SmartDashboard.putNumber("rps", currentRPS);
        SmartDashboard.putNumber("target", x);
        SmartDashboard.putNumber("adjustment", adjustedX - x);
        SmartDashboard.putNumber("adjusted", adjustedX);
        SmartDashboard.putNumber("err", adjustedRPS - currentRPS);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setTargetRPS(0);
    }
}
