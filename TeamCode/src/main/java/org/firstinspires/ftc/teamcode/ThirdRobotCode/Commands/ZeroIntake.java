package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;

public class ZeroIntake extends CommandBase {
    IntakeSubsystem intake;
    public ZeroIntake(IntakeSubsystem intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        intake.spinIntake(0);
        intake.pivotIntake(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    public void end(){
        intake.spinIntake(0);
        intake.pivotIntake(0);
    }
}
