package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

public class NOEND extends CommandBase {
    IntakeSubsystem intake;
    public NOEND(SlideSubsystem slides, ArmSubsystem arm, IntakeSubsystem intake, MultipleTelemetry telemetry, PIDFCoefficients armpidf, PIDFCoefficients slidespidf){
        addRequirements(arm);
        addRequirements(slides);
        addRequirements(intake);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
