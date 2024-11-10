package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Commands {
    DcMotor arm;
    CRServo intakeServo;
    Servo wrist;

    public Commands(DcMotor armMotor, Servo wrist, CRServo intakeServo){
        this.arm = armMotor;
        this.wrist = wrist;
        this.intakeServo = intakeServo;

    }
    ArmSubsystem armSubsytem = new ArmSubsystem(arm);
    IntakeSubsystem intake = new IntakeSubsystem(intakeServo, wrist);
    class teleopCommands {



        public void scoreBucket() {
            //code to score bucket
            //TODO: Instead of 90 make the desired angle a constant in Constants.java-Nicolas
            armSubsytem.moveArm(Constants.ArmConstants.BUCKET_ANGLE);

            intake.intakePowerOn(Constants.IntakeConstants.RELEASE_SAMPLE);
        }

        public void scoreBar() {
            armSubsytem.moveArm(Constants.ArmConstants.BAR_ANGLE);
            intake.intakePowerOn(Constants.IntakeConstants.RELEASE_SAMPLE);
        }


        public void intake() {
            armSubsytem.moveArm(Constants.ArmConstants.INTAKE_ANGLE);
            intake.intakePowerOn(Constants.IntakeConstants.INTAKE_SAMPLE);
        }


    }
}

