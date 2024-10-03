package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="HighFiveTest", group="TestCode")
public class IntoTheDeepArm extends LinearOpMode {
    private DcMotor extendArm;
    private DcMotor rotateArm;
/* Define motors in DriverHub */
    @Override
    public void runOpMode() {
        extendArm = hardwareMap.get(DcMotor.class,"extendArm");
        rotateArm = hardwareMap.get(DcMotor.class,"rotateArm");
        waitForStart();
        while( opModeIsActive()){
            double max;
            double extend = gamepad2.right_trigger;
            double retract = -gamepad2.left_trigger;
            boolean cwRotate = gamepad2.dpad_right;
            boolean ccRotate = gamepad2.dpad_left;
            /* cw = clockwise cc = counterclockwise */
            double extArmPower = extend + retract;
            if (cwRotate == true) {
                rotateArm.setPower(1);
            }
            else if (ccRotate == true) {
                rotateArm.setPower(-1);
            }
            extendArm.setPower(extArmPower);



            }
        }
    }

