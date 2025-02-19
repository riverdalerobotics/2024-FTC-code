package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

public class Spit extends CommandBase {
    char colour;
    IntakeSubsystem intake;
    SlideSubsystem slides;
    public Spit(IntakeSubsystem intake, char teamColour){
        this.colour = teamColour;
        this.intake = intake;
                addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.pivotIntake(Constants.IntakeConstants.SCORE_POSITION);
    }

    public void execute(){
        intake.spinIntake(Constants.IntakeConstants.SCORE_SPEED);
    }
    public boolean isFinished(){
        return intake.getColour() == 'w';
    }
    public void end() throws InterruptedException{
        Thread.sleep(Constants.IntakeConstants.WAIT_TIME);
        intake.pivotIntake(0);
        intake.spinIntake(0);

    }
}
