package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * This defines the hardware that you are going to use for your robot
 */
public class HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  backLeftDrive   = null;
    public DcMotor  backRightDrive  = null;

    public Servo leftArm = null;
    public Servo rightArm= null;
    //public Servo    clawLiftRight    = null;
    //public Servo    clawLiftLeft    = null;
    public ColorSensor color;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, "front_left");//Sets the name you have to use for the phone config
        frontRightDrive = hwMap.get(DcMotor.class, "front_right");//Sets the name you have to use for the phone config
        backLeftDrive  = hwMap.get(DcMotor.class, "back_left");//Sets the name you have to use for the phone config
        backRightDrive = hwMap.get(DcMotor.class, "back_right");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Define and initialize ALL installed servos.
        leftArm  = hwMap.get(Servo.class, "left_arm");//Sets the name you have to use for the phone config

        rightArm  = hwMap.get(Servo.class, "right_arm");//Sets the name you have to use for the phone config

        //clawLiftRight  = hwMap.get(Servo.class, "claw_lift_right");//Sets the name you have to use for the phone config

        //clawLiftLeft  = hwMap.get(Servo.class, "claw_lift_left");//Sets the name you have to use for the phone config

        color = hwMap.colorSensor.get("color_sensor");
    }
}