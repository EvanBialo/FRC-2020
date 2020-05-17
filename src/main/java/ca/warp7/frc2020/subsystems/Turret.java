/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ca.warp7.frc2020.subsystems;

import ca.warp7.frc2020.Constants;
import ca.warp7.frc2020.lib.motor.MotorControlHelper;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class Turret implements Subsystem {
    private static Turret instance;

    public static Turret getInstance() {
        if (instance == null) instance = new Turret();
        return instance;
    }

    private CANSparkMax turretMiniNeo = MotorControlHelper
            .createMasterSparkMAX(Constants.kHopperID);

    private Turret() {
    }

    public void setAngleRadians(double angle) {
        this.setAngleRadians(angle, false);
    }

    public void setAngleRadians(double angle, boolean fieldRelative) {
        // dummy function
    }
}
