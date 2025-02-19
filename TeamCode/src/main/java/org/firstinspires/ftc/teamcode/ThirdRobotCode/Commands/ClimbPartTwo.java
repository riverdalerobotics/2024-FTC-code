package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

public class ClimbPartTwo extends SequentialCommandGroup {
    IntakeSubsystem intake;
    public ClimbPartTwo(SlideSubsystem slides, ArmSubsystem arm, IntakeSubsystem intake, MultipleTelemetry telemetry, PIDFCoefficients armpidf, PIDFCoefficients slidespidf){
        addCommands(
                new MoveSlides(slides, Constants.SlideConstants.ZERO+15, telemetry, slidespidf),
                new ZeroArm(arm, intake, telemetry, armpidf),
                new MoveIntake(intake, 0.5)
        );
        addRequirements(arm);
        addRequirements(slides);
        addRequirements(intake);
        this.intake = intake;
    }

}
