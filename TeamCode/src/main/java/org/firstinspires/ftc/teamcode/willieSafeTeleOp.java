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


@TeleOp(name="willieSafeTeleOp")
//@Disabled

public class willieSafeTeleOp extends LinearOpMode
{
    //actuator objects
    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;
    public DcMotorEx SlideRight1 = null;
    public DcMotorEx SlideLeft1 = null;
    public DcMotorEx SlideRight2 = null;
    public DcMotorEx SlideLeft2 = null;
    public Servo Claw0 = null;
    public Servo Claw1 = null;
    public Servo odoRaise1 = null;
    public Servo odoRaise2 = null;
    public Servo odoRaise3 = null;

    //sensor objects
    public NormalizedColorSensor color = null;

    //drive variables
    double driveSpeed = 1;

    //lift variables
    public enum LiftState
    {
        RAISE_LIFT,
        FLIP_V4B,
        RESET_LIFT,
        RETRACT_V4B,
        LOWER_LIFT,
        STOP,
        RESET_ENCODERS
    }
    LiftState liftState = LiftState.RAISE_LIFT;

    int slidePose0 =0;
    int slidePose1 =0;
    int V4BPose0 =0;
    int V4BPose1 =0;
    int V4BAngle = 0;
    int junctionHeight = 0;

    //autoGrab variables
    public enum AutoGrab
    {
        DISTANCE_CHECK,
        GRAB_CONE,
        RAISE_LIFT,
        RELEASE_CONE,
        LOWER_LIFT,
        EMPTY_CLAW_CHECK,
        MANUAL
    }
    AutoGrab autoGrab = AutoGrab.DISTANCE_CHECK;
    double servoTimeVar = 0;

    //toggles
    boolean DpadUpToggle = true;
    boolean DpadDownToggle = true;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //keep track of time in auto grab for servo movement
        ElapsedTime servoTime = new ElapsedTime();

        //actuator declarations
        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");
        SlideRight1 = hardwareMap.get((DcMotorEx.class), "SlideRight1");
        SlideLeft1 = hardwareMap.get((DcMotorEx.class), "SlideLeft1");
        SlideRight2 = hardwareMap.get((DcMotorEx.class), "SlideRight2");
        SlideLeft2 = hardwareMap.get((DcMotorEx.class), "SlideLeft2");
        Claw0 = hardwareMap.servo.get("Claw0");
        Claw1 = hardwareMap.servo.get("Claw1");
        odoRaise1 = hardwareMap.servo.get("odoRaise1");
        odoRaise2 = hardwareMap.servo.get("odoRaise2");
        odoRaise3 = hardwareMap.servo.get("odoRaise3");

        //actuator run modes
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideRight1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideLeft1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);

        SlideRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        SlideLeft1.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideLeft2.setDirection(DcMotorSimple.Direction.REVERSE);

        //sensor declarations
        color = hardwareMap.get(NormalizedColorSensor.class, "color");

        odoRaise1.setPosition(.4);
        odoRaise2.setPosition(.5);
        odoRaise3.setPosition(.2);

        waitForStart();

        while (opModeIsActive())
        {
            //speed multiplier for lifted slides
            if((SlideLeft1.getCurrentPosition() + SlideRight1.getCurrentPosition())/2 > 300)
            {
                driveSpeed = .95;
            }
            else
            {
                driveSpeed = 1;
            }

            //Todo switch to mech
            //wheel powers
            //motorRF.setPower(driveSpeed * (gamepad1.right_stick_y + gamepad1.left_stick_x * 1));
            //motorRB.setPower(driveSpeed * (-(gamepad1.right_stick_y + gamepad1.left_stick_x * 1)));
            //motorLB.setPower(driveSpeed * (gamepad1.right_stick_y - gamepad1.left_stick_x * 1));
            //motorLF.setPower(driveSpeed * (-(gamepad1.right_stick_y - gamepad1.left_stick_x * 1)));

            motorRF.setPower((((-gamepad1.right_stick_y - gamepad1.right_stick_x) * 1) - (gamepad1.left_stick_x * 1))*driveSpeed);
            motorRB.setPower((((-gamepad1.right_stick_y + gamepad1.right_stick_x) * 1) - (gamepad1.left_stick_x * 1))*driveSpeed);
            motorLB.setPower((((-gamepad1.right_stick_y - gamepad1.right_stick_x) * 1) + (gamepad1.left_stick_x * 1))*driveSpeed);
            motorLF.setPower((((-gamepad1.right_stick_y + gamepad1.right_stick_x) * 1) + (gamepad1.left_stick_x * 1))*driveSpeed);



            //gamepad inputs for FSM state changes
            if (gamepad1.left_trigger > .5)
            {
                autoGrab = AutoGrab.MANUAL;
            }
            else if (gamepad1.right_trigger > .5)
            {
                autoGrab = AutoGrab.DISTANCE_CHECK;
            }

            //FSM for auto grab actions
            switch (autoGrab)
            {
                //see if a cone is in the intake
                case DISTANCE_CHECK:
                    if (((DistanceSensor) color).getDistance(DistanceUnit.CM) < 6)
                    {
                        autoGrab = AutoGrab.GRAB_CONE;
                    }
                    break;

                //close claw
                case GRAB_CONE:
                    if (servoTimeVar == 0) {
                        servoTimeVar = servoTime.milliseconds();
                    }
                    Claw0.setPosition(.5);
                    Claw1.setPosition(.5);

                    if (servoTimeVar + 250 < servoTime.milliseconds())
                    {
                        servoTimeVar = 0;
                        autoGrab = AutoGrab.RAISE_LIFT;
                    }
                    break;

                //raise lift over any stacked cone
                case RAISE_LIFT:
                    slidePose0 += 150;
                    slidePose1 += 150;
                    liftState = LiftState.RAISE_LIFT;
                    autoGrab = AutoGrab.RELEASE_CONE;
                    break;

                //driver decides when cone is dropped after manipulator puts lift at the correct height
                case RELEASE_CONE:
                    if (gamepad1.right_bumper)
                    {
                        Claw0.setPosition(.38);
                        Claw1.setPosition(.62);
                        autoGrab = AutoGrab.LOWER_LIFT;
                    }
                    break;

                //0.5s after cone is dropped, lift will come down on its own using the lift FSM
                case LOWER_LIFT:
                    if (servoTimeVar == 0) {
                        servoTimeVar = servoTime.milliseconds();
                    }

                    if (servoTimeVar + 500 < servoTime.milliseconds())
                    {
                        //Claw0.setPosition(.5);
                        //Claw1.setPosition(.5);
                        servoTimeVar = 0;
                        liftState = LiftState.LOWER_LIFT;
                        autoGrab = AutoGrab.EMPTY_CLAW_CHECK;
                    }
                    break;

                //ensures claw does not immediately close if distance sensor sees something
                //could be replaced with a button to reactivate autograb function
                case EMPTY_CLAW_CHECK:
                    if (!(((DistanceSensor) color).getDistance(DistanceUnit.CM) < 6))
                    {
                        autoGrab = AutoGrab.DISTANCE_CHECK;
                    }
                    break;

                //manual traditional control of the arm
                case MANUAL:
                    //claw grab/release
                    if (gamepad1.left_bumper)
                    {
                        Claw0.setPosition(.5);
                        Claw1.setPosition(.5);
                    }
                    else if (gamepad1.right_bumper)
                    {
                        Claw0.setPosition(.38);
                        Claw1.setPosition(.62);
                    }
                    break;
            }


            //lift preset control inputs
            if (gamepad2.back)
            {
                liftState = LiftState.STOP;
            }
            else if (gamepad2.start)
            {
                liftState = LiftState.RESET_ENCODERS;
            }
            else if (gamepad2.a)
            {
                liftState = LiftState.LOWER_LIFT;
            }
            else if (gamepad2.b)
            {
                slidePose0 = 375;
                slidePose1 = 375;
                junctionHeight = 375;
                liftState = LiftState.RAISE_LIFT;
            }
            else if (gamepad2.x)
            {
                slidePose0 = 600;
                slidePose1 = 600;
                junctionHeight = 600;
                liftState = LiftState.RAISE_LIFT;
            }
            else if (gamepad2.y)
            {
                slidePose0 = 835;
                slidePose1 = 835;
                junctionHeight = 835;
                liftState = LiftState.RAISE_LIFT;
            }

            //lift quick adjust
            if (gamepad2.dpad_up && DpadUpToggle)
            {
                slidePose0 += 30;
                slidePose1 += 30;
                liftState = LiftState.RAISE_LIFT;
                DpadUpToggle = false;
            }
            else if (!gamepad2.dpad_up && !DpadUpToggle)
            {
                DpadUpToggle = true;
            }

            if (gamepad2.dpad_down && DpadDownToggle)
            {
                slidePose0 -= 30;
                slidePose1 -= 30;
                liftState = LiftState.RAISE_LIFT;
                DpadDownToggle = false;
            }
            else if (!gamepad2.dpad_down && !DpadDownToggle)
            {
                DpadDownToggle = true;
            }

            //lift fine adjust
            if (gamepad2.right_stick_y < -.5)
            {
                slidePose0 += 6;
                slidePose1 += 6;
                liftState = LiftState.RAISE_LIFT;
            }
            else if (gamepad2.right_stick_y > .5)
            {
                slidePose0 -= 6;
                slidePose1 -= 6;
                liftState = LiftState.RAISE_LIFT;
            }

            //V4B preset control inputs
            /*
            if (gamepad2.right_bumper)
            {
                V4BPose0 = 0;
                V4BPose1 = 0;
            }

            else if (gamepad2.left_bumper)
            {
                V4BPose0 = -140;
                V4BPose1 = 140;
            }

            //V4B fine adjust
            if (gamepad2.right_trigger > .5)
            {
                V4BPose0 += 1;
                V4BPose1 -= 1;
            }
            else if (gamepad2.left_trigger > .5)
            {
                V4BPose0 -= 1;
                V4BPose1 += 1;
            }

             */

            //state machine to control lift state and order operations accordingly
            switch(liftState)
            {
                //brings lift to specified height while maintaning approximately the same V4B angle
                case RAISE_LIFT:
                    if (V4BAngle == 0)
                    {
                        V4BAngle = (SlideLeft1.getCurrentPosition() - SlideRight1.getCurrentPosition())/2;
                    }
                    SlideLeft1.setTargetPosition(slidePose0 + V4BAngle);
                    SlideRight1.setTargetPosition(slidePose1 - V4BAngle);
                    SlideLeft2.setTargetPosition(slidePose0 + V4BAngle);
                    SlideRight2.setTargetPosition(slidePose1 - V4BAngle);
                    SlideLeft1.setPower(1);
                    SlideRight1.setPower(1);
                    SlideLeft2.setPower(1);
                    SlideRight2.setPower(1);
                    SlideLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (SlideLeft1.getCurrentPosition() > ((slidePose0 + V4BAngle)-30) && SlideLeft1.getCurrentPosition() < ((slidePose0 + V4BAngle)+30)
                            &&  SlideRight1.getCurrentPosition() > ((slidePose1 - V4BAngle)-30) && SlideRight1.getCurrentPosition() < ((slidePose1 - V4BAngle)+30))
                    {
                        V4BAngle = 0;
                        liftState = LiftState.FLIP_V4B;
                    }
                    break;

                //when lift reaches desired height, V4B is controlled
                //V4B can be adjusted as long as in this state
                case FLIP_V4B:
                    SlideLeft1.setTargetPosition(slidePose0 + V4BPose0);
                    SlideRight1.setTargetPosition(slidePose1 + V4BPose1);
                    SlideLeft2.setTargetPosition(slidePose0 + V4BPose0);
                    SlideRight2.setTargetPosition(slidePose1 + V4BPose1);
                    SlideLeft1.setPower(1);
                    SlideRight1.setPower(1);
                    SlideLeft2.setPower(1);
                    SlideRight2.setPower(1);
                    SlideLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;

                case RESET_LIFT:
                    if (V4BAngle == 0)
                    {
                        V4BAngle = (SlideLeft1.getCurrentPosition() - SlideRight1.getCurrentPosition())/2;
                    }
                    SlideLeft1.setTargetPosition(junctionHeight + V4BAngle);
                    SlideRight1.setTargetPosition(junctionHeight - V4BAngle);
                    SlideLeft2.setTargetPosition(junctionHeight + V4BAngle);
                    SlideRight2.setTargetPosition(junctionHeight - V4BAngle);
                    SlideLeft1.setPower(1);
                    SlideRight1.setPower(1);
                    SlideLeft2.setPower(1);
                    SlideRight2.setPower(1);
                    SlideLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (SlideLeft1.getCurrentPosition() > ((junctionHeight + V4BAngle)-30) && SlideLeft1.getCurrentPosition() < ((junctionHeight + V4BAngle)+30)
                            &&  SlideRight1.getCurrentPosition() > ((junctionHeight - V4BAngle)-30) && SlideRight1.getCurrentPosition() < ((junctionHeight - V4BAngle)+30))
                    {
                        V4BAngle = 0;
                        liftState = LiftState.RETRACT_V4B;
                    }
                    break;

                //keeps lift at current height and brings V4B back to front
                case RETRACT_V4B:
                    V4BPose0 = -75;
                    V4BPose1 = 75;
                    SlideLeft1.setTargetPosition(junctionHeight + V4BPose0);
                    SlideRight1.setTargetPosition(junctionHeight + V4BPose1);
                    SlideLeft2.setTargetPosition(junctionHeight + V4BPose0);
                    SlideRight2.setTargetPosition(junctionHeight + V4BPose1);
                    SlideLeft1.setPower(1);
                    SlideRight1.setPower(1);
                    SlideLeft2.setPower(1);
                    SlideRight2.setPower(1);
                    SlideLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (SlideLeft1.getCurrentPosition() > ((junctionHeight + V4BPose0)-30) && SlideLeft1.getCurrentPosition() < ((junctionHeight + V4BPose0)+30)
                            &&  SlideRight1.getCurrentPosition() > ((junctionHeight + V4BPose1)-30) && SlideRight1.getCurrentPosition() < ((junctionHeight + V4BPose1)+30))
                    {
                        liftState = LiftState.LOWER_LIFT;
                    }
                    break;

                //brings lift down to height of 0
                case LOWER_LIFT:
                    slidePose0 = 0;
                    slidePose1 = 0;
                    junctionHeight = 0;
                    SlideLeft1.setTargetPosition(slidePose0 + V4BPose0);
                    SlideRight1.setTargetPosition(slidePose1 + V4BPose1);
                    SlideLeft2.setTargetPosition(slidePose0 + V4BPose0);
                    SlideRight2.setTargetPosition(slidePose1 + V4BPose1);
                    SlideLeft1.setPower(1);
                    SlideRight1.setPower(1);
                    SlideLeft2.setPower(1);
                    SlideRight2.setPower(1);
                    SlideLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (SlideLeft1.getCurrentPosition() > ((slidePose0 + V4BPose0)-30) && SlideLeft1.getCurrentPosition() < ((slidePose0 + V4BPose0)+30)
                            &&  SlideRight1.getCurrentPosition() > ((slidePose1 + V4BPose1)-30) && SlideRight1.getCurrentPosition() < ((slidePose1 + V4BPose1)+30))
                    {
                        V4BPose0 = 0;
                        V4BPose1 = 0;
                        Claw0.setPosition(.38);
                        Claw1.setPosition(.62);
                        liftState = LiftState.FLIP_V4B;
                    }
                    break;

                //emergency stop, will disable lift by turning off motors and setting all heights to 0
                case STOP:
                    SlideLeft1.setPower(0);
                    SlideRight1.setPower(0);
                    SlideLeft2.setPower(0);
                    SlideRight2.setPower(0);
                    slidePose0 = 0;
                    slidePose1 = 0;
                    V4BPose0 = 0;
                    V4BPose1 = 0;
                    junctionHeight = 0;
                    break;

                case RESET_ENCODERS:
                    SlideLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    SlideLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    SlideRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    SlideRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    break;


                //should never reach this, catch all for bugs in code
                default:
                    liftState = LiftState.RAISE_LIFT;
            }

            //telemetry
            telemetry.addData("SlideLeft1 Encoder",SlideLeft1.getCurrentPosition());
            telemetry.addData("SlideRight1 Encoder",SlideRight1.getCurrentPosition());
            telemetry.addData("SlideLeft2 Encoder",SlideLeft2.getCurrentPosition());
            telemetry.addData("SlideRight2 Encoder",SlideRight2.getCurrentPosition());
            telemetry.addData("SlideLeft1 Power",SlideLeft1.getPower());
            telemetry.addData("SlideRight1 Power",SlideRight1.getPower());
            telemetry.addData("SlideLeft2 Power",SlideLeft2.getPower());
            telemetry.addData("SlideRight2 Power",SlideRight2.getPower());
            telemetry.addData("distance",((DistanceSensor) color).getDistance(DistanceUnit.CM));


            telemetry.update();
        }
    }
}