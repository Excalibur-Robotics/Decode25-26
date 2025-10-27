package org.firstinspires.ftc.teamcode.EvanTeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotor intakeMotor;
    private boolean state;

    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intake");
        state = false; // if intake is on or not
    }

    // toggle intake on/off
    public void toggleIntake() {
        if(state)
            intakeMotor.setPower(0);
        else
            intakeMotor.setPower(0.5);
        state = !state;
    }
}
