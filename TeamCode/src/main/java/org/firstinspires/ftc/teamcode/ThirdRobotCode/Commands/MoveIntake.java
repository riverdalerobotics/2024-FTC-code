package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;

public class MoveIntake extends CommandBase {
    IntakeSubsystem intake;
    double setpoint;
    public MoveIntake(IntakeSubsystem intake, double setpoint){
        addRequirements(intake);
        this.intake = intake;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        super.initialize();
        intake.pivotIntake(setpoint);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
