package com.team254.frc2023.auto.modes;

import com.team254.frc2023.AutoModeSelector;
import com.team254.frc2023.auto.AutoModeEndedException;
import com.team254.frc2023.auto.actions.*;
import com.team254.frc2023.subsystems.Drive;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

/*
* Cable Protector/Bump Side - Scores High Cone, picks up outside cube, scores it high, picks up inside cube scores mid.
* */
public class CpThree extends AutoModeBase {

    public CpThree() {
        addAction(
                new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().cpFirstScoreToOutsidePickup, false)
        );
 
    }
}