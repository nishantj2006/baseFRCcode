package com.team254.frc2023.auto.actions;

import com.team254.frc2023.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;

public class IntakeAction implements Action {

    Intake intake;
    private double mStartTime = 0.0;
    double power;


    public IntakeAction(double power){
        this.power = power;
        intake = new Intake();  
    }
    @Override
    public void start() {
        this.mStartTime = Timer.getFPGATimestamp();
        intake.runIntake(power);
        // TODO Auto-generated method stub
        
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return Timer.getFPGATimestamp() - this.mStartTime > 1.5;
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        
    }
    
}