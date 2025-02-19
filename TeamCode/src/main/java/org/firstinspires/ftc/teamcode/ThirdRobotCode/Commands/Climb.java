package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

public class Climb extends SequentialCommandGroup {
    public Climb(ArmSubsystem arm, SlideSubsystem slides, IntakeSubsystem intake, MultipleTelemetry telemetry, PIDFCoefficients armPid){
        addCommands(
                new MoveArm(arm, telemetry, armPid, Constants.ArmConstants.CLIMB_UP_ANGLE),
                new MoveIntake(intake, 0.6),
                new MoveSlides(slides, Constants.SlideConstants.CLIMB_UP, telemetry, Constants.SlideConstants.slidesPID)

        );
        addRequirements(arm, slides, intake);

    }
}
