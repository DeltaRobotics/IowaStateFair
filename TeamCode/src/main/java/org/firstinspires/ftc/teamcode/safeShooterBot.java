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


@TeleOp(name="safeShooterBot")
//@Disabled

public class safeShooterBot extends LinearOpMode {

    //actuator objects
    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;
    public Servo flipBar = null;
    // looking from ball loading area
    public DcMotor shooterL = null;
    public DcMotor shooterR = null;
    public DcMotor intake = null;

    public double lPower = 0;
    public double rPower = 0;

    public double speed = 0.5;

    boolean intakeMoving = false;

    double servoTimeVar = 0;
    double intakeTimeVar = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime servoTime = new ElapsedTime();
        ElapsedTime intakeTime = new ElapsedTime();

        //actuator declarations
        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");
        flipBar = hardwareMap.servo.get("flipBar");
        shooterL = hardwareMap.dcMotor.get("shooterL");
        shooterR = hardwareMap.dcMotor.get("shooterR");
        intake = hardwareMap.dcMotor.get("intake");


        //set up flip bar
        flipBar.setPosition(.01);

        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive())
        {
            motorRF.setPower(speed*(-(-gamepad1.right_stick_y - gamepad1.right_stick_x) + (gamepad1.left_stick_x)));
            motorRB.setPower(speed*((-gamepad1.right_stick_x + gamepad1.right_stick_y) + (gamepad1.left_stick_x)));
            motorLB.setPower(speed*(-(gamepad1.right_stick_y + gamepad1.right_stick_x) + (gamepad1.left_stick_x)));
            motorLF.setPower(speed*(-(-gamepad1.right_stick_x + gamepad1.right_stick_y)) + (gamepad1.left_stick_x));

            if(gamepad1.right_bumper){
                lPower = .4;
                rPower = 1;
            }

            if(gamepad1.left_bumper){
                lPower = 0;
                rPower = 0;
            }

            if(gamepad1.a){
                flipBar.setPosition(.6);
                servoTimeVar = 0;
                servoTimeVar = servoTime.milliseconds();

            }

            if(servoTimeVar + 250 < servoTime.milliseconds()){
                flipBar.setPosition(.01);
            }

            if(gamepad1.b){
                intake.setPower(-0.1);
                intakeTimeVar = 0;
                intakeTimeVar = servoTime.milliseconds();
                intakeMoving = true;

            }

            if((intakeTimeVar + 250 < servoTime.milliseconds()) && intakeMoving){
                intake.setPower(0.1);
                intakeTimeVar = 0;
                intakeTimeVar = servoTime.milliseconds();
                intakeMoving = false;
            }

            if((intakeTimeVar + 250 < servoTime.milliseconds()) && !intakeMoving){
                intake.setPower(0);
            }


            shooterL.setPower(lPower);
            shooterR.setPower(rPower);
        }
    }
}
