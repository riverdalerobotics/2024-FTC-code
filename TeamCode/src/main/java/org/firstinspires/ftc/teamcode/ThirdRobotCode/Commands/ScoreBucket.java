package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

public class ScoreBucket extends SequentialCommandGroup {
    MultipleTelemetry telemetry;
    public ScoreBucket(SlideSubsystem slides, ArmSubsystem arm, IntakeSubsystem intake, MultipleTelemetry telemetry, PIDFCoefficients armpidf, PIDFCoefficients slidespidf){
        addCommands(
//                new ZeroIntake(intake),
                new MoveSlides(slides, Constants.SlideConstants.ZERO, telemetry, Constants.SlideConstants.slidesPID),
                new ArmGoToScore(arm, intake, telemetry, armpidf),
                new MoveSlides(slides, Constants.SlideConstants.SCORE_BUCKET, telemetry, slidespidf)
        );
        addRequirements(arm);
        addRequirements(slides);
//        addRequirements(intake);
        this.telemetry = telemetry;
    }

    @Override
    public void execute() {
        super.execute();
        telemetry.addLine("WE ARE SCORING");

    }
    public void end(){
        telemetry.clear();
    }
}
