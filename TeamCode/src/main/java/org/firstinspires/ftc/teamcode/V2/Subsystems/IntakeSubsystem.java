package org.firstinspires.ftc.teamcode.V2.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
This is the subsystem for the intake of V1. It is just one motor, which can be
activated and stopped
 */

public class IntakeSubsystem extends SubsystemBase {
    public DcMotor intakeMotor;

    public IntakeSubsystem(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    // set intake motor to full power
    public void activateIntake() {
        intakeMotor.setPower(1);
    }

    // stop intake motor
    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}
