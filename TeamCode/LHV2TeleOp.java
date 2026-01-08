package org.firstinspires.ftc.teamcode.V2;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;


// This is the Spindex code so far :)
// Feel free to make changes and tell me improvements

@TeleOp(name="SpindexTester")
public class LHV2TeleOp extends OpMode {
    /* Color Sensor Variables*/
    private NormalizedColorSensor color_sensor1;

    private NormalizedColorSensor color_sensor2;
    double hue_slot1; // Will be used to store hue value for checking slots
    double hue_slot2; // Same as hue_slot1
    final static int none = 0;
    final static int green = 1;
    final static int purple = 2;
    int slot1 = none;  //Intake slot
    int slot2 = none;  //Storage slot
    int slot3 = none;  //Storage slot (The side with yellow square)
    int turret; //For turret color sensor
    int temp; //Used later to store a slot value and assign it to another slot
    public DcMotorEx bore;
    DcMotor intake;
    DcMotor frwheel;
    DcMotor flwheel;
    DcMotor brwheel;
    DcMotor blwheel;
    boolean y=true;
    boolean x=true;

    ElapsedTime timer = new ElapsedTime();


    public void init() {
        //Color sensors and Bore encoder
        color_sensor1 = hardwareMap.get(NormalizedColorSensor.class, "CS1");
        color_sensor2 = hardwareMap.get(NormalizedColorSensor.class, "CS2");
        bore= hardwareMap.get(DcMotorEx.class,"Bore");
        bore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bore.setDirection(DcMotor.Direction.REVERSE);


        // Intake and Drivetrain
        intake= hardwareMap.get(DcMotor.class, "intake");
        frwheel= hardwareMap.get(DcMotor.class, "frwheel");
        flwheel= hardwareMap.get(DcMotor.class, "flwheel");
        brwheel= hardwareMap.get(DcMotor.class, "brwheel");
        blwheel= hardwareMap.get(DcMotor.class, "blwheel");





    }

    public void loop() {
        //DriveTrain and Intake TeleOp
        double IntakePower=0;
        if (gamepad1.yWasPressed()) {
            if (y == true) {
                IntakePower=1;
                intake.setPower(IntakePower);
                y = false;
            } else {
                IntakePower=0;
                intake.setPower(IntakePower);
                y = true;
            }
        }
        if (gamepad1.xWasPressed()) {
            if (x==true) {
                IntakePower = -IntakePower;
                intake.setPower(IntakePower);
                x = false;
            }
            else {
                IntakePower=-IntakePower;
                intake.setPower(IntakePower);
                x=true;
            }
        }

        double rotate= gamepad1.right_stick_x;
        double x1= gamepad1.left_stick_x;
        double y1=gamepad1.left_stick_y;

        double frpower= x1-y1-rotate;
        double flpower= x1+y1+rotate;
        double brpower= x1+y1-rotate;
        double blpower= x1-y1+rotate;

        frwheel.setPower(frpower);
        flwheel.setPower(flpower);
        brwheel.setPower(brpower);
        blwheel.setPower(blpower);









        /// //////////////////////////////////////////

        //Color Sensor and Spindexer TeleOp

        NormalizedRGBA color_slot1 = color_sensor1.getNormalizedColors();
        NormalizedRGBA color_slot2 = color_sensor2.getNormalizedColors();
        /* JavaUtil.colorToHue() converts RGBA value into a hue number from 0 to 360
           Used later for checking the color of the balls*/
        hue_slot1 = JavaUtil.colorToHue(color_slot1.toColor());
        hue_slot2 = JavaUtil.colorToHue(color_slot2.toColor());
        telemetry.addData("hue1: ", hue_slot1); // Shows hue number
        telemetry.addData("hue2: ", hue_slot2);
        double theta= bore.getCurrentPosition();   //Position of Spindexer (Quadrature)
        telemetry.addData("Theta: ", theta);


        //Color sensor checks
        //Slot 1 (Intake Slot) Checks slot using hue value

        if (hue_slot1 <= 245 && hue_slot1 >= 210) {
            slot1 = purple;
        } else if (hue_slot1 <= 180 && hue_slot1 >= 150) {
            slot1 = green;
        } else {
            slot1 = none;
        }

        //Turret Checker (Checks color of ball about to be shot)

        if (hue_slot2 <= 245 && hue_slot2 >= 210) {
            turret = purple;
        } else if (hue_slot2 <= 180 && hue_slot2 >= 150) {
            turret = green;
        } else {
            turret = none;
        }

        //Spindexer controls  (STILL NEEDS TUNING)
        if (timer.milliseconds()>200) {
            if (gamepad1.right_bumper) { //rotates spindexer counterclockwise 1 slot
                V2PID PID = new V2PID(.00042, 0, 0); //Still needs tuning
                double CP = bore.getCurrentPosition();
                bore.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (abs((theta + 2550) - CP) > 30) {
                    if(timer.milliseconds()>15) {
                        double MotorPower = PID.Calculate(theta + 2550, CP);
                        bore.setPower(MotorPower);
                        CP = bore.getCurrentPosition();
                        timer.reset();
                    }
                }
                bore.setPower(0);
                timer.reset();
                temp = slot1;
                slot1 = slot3;
                slot3 = slot2;
                slot2 = temp;

            }
        }
        if (gamepad1.left_bumper){ //rotates spindexer clockwise 1 slot
            bore.setTargetPosition(0);
            bore.setTargetPositionTolerance(200);
            bore.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bore.setVelocity(20);
            temp=slot1;
            slot1=slot2;
            slot2=slot3;
            slot3=temp;
        }


        //Displays slot data
        ColorCheck(slot1, 1);
        ColorCheck(slot2, 2);
        ColorCheck(slot3, 3);
        ColorCheck(turret, 4);

        telemetry.update();




        /// //////////////////////////////////////////////////////////////////

    }




    public void ColorCheck(int slot_val, int slot_pos) { //slot_pos= 1 (slot1), slot_pos=4 (turret)
        switch (slot_pos) {
            case 1:
                switch (slot_val) {
                    case purple:
                        telemetry.addData("Intake slot: ", "Purple");
                    case green:
                        telemetry.addData("Intake slot: ", "Green");
                    case none:
                        telemetry.addData("Intake slot: ", "Empty");
                }
            case 2:
                switch (slot_val) {
                    case purple:
                        telemetry.addData("Slot 2: ", "Purple");
                    case green:
                        telemetry.addData("Slot 2: ", "Green");
                    case none:
                        telemetry.addData("Slot 2: ", "Empty");
                }

            case 3:
                switch (slot_val) {
                    case purple:
                        telemetry.addData("Slot 3: ", "Purple");
                    case green:
                        telemetry.addData("Slot 3: ", "Green");
                    case none:
                        telemetry.addData("Slot 3: ", "Empty");
                }
            case 4:
                switch (slot_val) {
                    case purple:
                        telemetry.addData("Turret: ", "Purple");
                    case green:
                        telemetry.addData("Turret: ", "Green");
                    case none:
                        telemetry.addData("Turret: ", "Empty");
                }
        }
    }
}