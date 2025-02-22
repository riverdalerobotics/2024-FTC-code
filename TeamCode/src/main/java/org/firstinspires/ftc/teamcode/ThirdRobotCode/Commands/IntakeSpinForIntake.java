package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

import java.util.concurrent.TimeUnit;

public class IntakeSpinForIntake extends CommandBase {
    char colour;
    IntakeSubsystem intake;
    SlideSubsystem slides;

    public IntakeSpinForIntake(IntakeSubsystem intake, char teamColour, SlideSubsystem slides){
        this.colour = teamColour;
        this.intake = intake;
        this.slides = slides;

        addRequirements(intake);
    }

    public void execute(){
        intake.spinIntake(0);
        if(slides.getSlidePos()>20) {
            intake.pivotIntake(Constants.IntakeConstants.INTAKE_POSITION);
        }
        intake.spinIntake(Constants.IntakeConstants.INTAKE_SPEED);

    }
    public boolean isFinished(){
        return intake.getColour() == colour || intake.getColour() == 'y';
    }
    public void end(){
        intake.pivotIntake(0);
        intake.spinIntake(0);

    }
}
