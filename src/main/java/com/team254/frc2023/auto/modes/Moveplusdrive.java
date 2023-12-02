package com.team254.frc2023.auto.modes;

import com.team254.frc2023.auto.actions.DriveTrajectoryAction;
import com.team254.frc2023.auto.actions.IntakeAction;
import com.team254.frc2023.auto.actions.SeriesAction;
import com.team254.frc2023.subsystems.Drive;

public class Moveplusdrive extends AutoModeBase{

    public Moveplusdrive(){
        addAction(        
            new SeriesAction(       
                new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().cpFirstScoreToOutsidePickup, false),
                new IntakeAction(1.0),
                new IntakeAction(0.0)
            )
         );

    }
}
