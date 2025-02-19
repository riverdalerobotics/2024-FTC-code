package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

public class MoveSlidesForIntake extends CommandBase {
    SlideSubsystem slides;
    double power = Constants.SlideConstants.INTAKE_SPEED;


    public MoveSlidesForIntake(SlideSubsystem slides){
        this.slides = slides;
        addRequirements(slides);
    }

    @Override
    public void initialize() {
        slides.runWithOutEncoder();
    }

    public void execute(){
        if(slides.getSlidePos()< Constants.SlideConstants.INTAKE_MAX){
            slides.moveSlide(power);}
        else{
            slides.moveSlide(0);
        }
        }
    public boolean isFinished(){
        return Constants.SlideConstants.INTAKE_MAX<slides.getSlidePos();
    }
    public void end(){
        slides.moveSlide(0);
    }
}
