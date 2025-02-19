package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

public class AutoScore extends SequentialCommandGroup {
    public AutoScore(SlideSubsystem slides, ArmSubsystem arm, IntakeSubsystem intake, MultipleTelemetry telemetry){
        addCommands(
                new ZeroIntake(intake),
                new MoveSlides(slides, Constants.SlideConstants.ZERO, telemetry, Constants.SlideConstants.slidesPID),
                new ArmGoToScore(arm, intake, telemetry, Constants.ArmConstants.armPID),
                new MoveSlides(slides, Constants.SlideConstants.SCORE_BUCKET, telemetry, Constants.SlideConstants.slidesPID),
                new Spit(intake, 'r'),
                new MoveSlides(slides, Constants.SlideConstants.ZERO, telemetry, Constants.SlideConstants.slidesPID),
                new ZeroIntake(intake),
                new ZeroArm(arm, intake, telemetry, Constants.ArmConstants.armPID)
        );
        addRequirements(intake, slides, arm);
    }
}
