package org.firstinspires.ftc.teamcode;


//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//import org.firstinspires.ftc.teamcode.RobotHardware;


@TeleOp(name="HomeschoolBot")
//@Disabled

public class robotIn30HoursHomeschool extends LinearOpMode {

    //actuator objects
    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;

    public double speed = .5;

    @Override
    public void runOpMode() throws InterruptedException {

        //actuator declarations
        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");

        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.dpad_up){
                speed = 1;
            }

            if (gamepad1.dpad_right){
                speed = .75;
            }

            motorRF.setPower(speed*((-gamepad1.right_stick_y - gamepad1.right_stick_x) - (gamepad1.left_stick_x)));
            motorRB.setPower(speed*(-(-gamepad1.right_stick_x + gamepad1.right_stick_y) - (gamepad1.left_stick_x)));
            motorLB.setPower(speed*((gamepad1.right_stick_y + gamepad1.right_stick_x) - (gamepad1.left_stick_x)));
            motorLF.setPower(speed*((-gamepad1.right_stick_x + gamepad1.right_stick_y)) - (gamepad1.left_stick_x));
        }
    }
}
