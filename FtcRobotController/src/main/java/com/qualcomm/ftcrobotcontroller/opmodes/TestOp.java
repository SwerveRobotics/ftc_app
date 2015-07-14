package com.qualcomm.ftcrobotcontroller.opmodes;

import com.android.internal.util.Predicate;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;

/**
 * An empty op mode serving as a template for custom OpModes
 */
public class TestOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //to do: create a class to wrap the motor and its queue?
    //because: shouldn't the motor's queue know which motor it points to?
    private LinkedList<SwerveAction> motor1Queue;

    //state variables for debouncing the controller buttons
    boolean a1IsDown=false;
    boolean b1IsDown=false;
    boolean x1IsDown=false;
    boolean y1IsDown=false;

    int actioncounter = 0; //this is for creating debugging names when creating multiple actions at runtime

    /*
    * Constructor
    */
    public TestOp()
    {
        motor1Queue = new LinkedList<SwerveAction>();

        //queue autonomous commands here
        motor1Queue.add( new SwerveActionTimedMotor(Integer.toString(actioncounter++), null, DcMotor.Direction.FORWARD, 1.0, 3.0));
        motor1Queue.add( new SwerveActionDelay(Integer.toString(actioncounter++), 3.0));
        motor1Queue.add( new SwerveActionTimedMotor(Integer.toString(actioncounter++), null, DcMotor.Direction.FORWARD, 1.0, 3.0));

        //motor1Queue.add( new SwerveActionLAMotor(Integer.toString(actioncounter++), null, DcMotor.Direction.FORWARD, 0.0, 10.0, 10.0));
        //motor1Queue.add( new SwerveActionDelay(Integer.toString(actioncounter++), 3.0));
        //motor1Queue.add( new SwerveActionLAMotor(Integer.toString(actioncounter++), null, DcMotor.Direction.FORWARD, 10.0, 0.0, 10.0));

    }

    private boolean debounceA1()
    {
        if ((!a1IsDown) && (gamepad1.a))
        {
            a1IsDown = true;
            return true;
        }
        else return false;
    }
    private boolean debounceB1()
    {
        if ((!b1IsDown) && (gamepad1.b))
        {
            b1IsDown = true;
            return true;
        }
        else return false;
    }
    private boolean debounceX1()
    {
        if ((!x1IsDown) && (gamepad1.x))
        {
            x1IsDown = true;
            return true;
        }
        else return false;
    }
    private boolean debounceY1()
    {
        if ((!y1IsDown) && (gamepad1.y))
        {
            y1IsDown = true;
            return true;
        }
        else return false;
    }

    /*
    * Code to run when the op mode is first enabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
    */
    @Override
    public void start() { runtime.reset(); }


    /*
    * This method will be called repeatedly in a loop
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */
    @Override
    public void loop() {
        telemetry.addData("Timer", "Running for " + runtime.toString());

        //clear flags used in debouncing
        if (!gamepad1.a) a1IsDown = false;
        if (!gamepad1.b) b1IsDown = false;
        if (!gamepad1.x) x1IsDown = false;
        if (!gamepad1.y) y1IsDown = false;


        //handle new input because it may preempt existing items in the queue
        if (debounceA1())
        {
            motor1Queue.add( new SwerveActionTimedMotor(Integer.toString(actioncounter++), null, DcMotor.Direction.FORWARD, 1.0, 5.0));
        }
        if (debounceX1())
        {
            motor1Queue.add( new SwerveActionDelay(Integer.toString(actioncounter++), 3.0));
        }
        if (debounceY1())
        {
            motor1Queue.add( new SwerveActionTimedMotor(Integer.toString(actioncounter++), null, DcMotor.Direction.FORWARD, 1.0, 2.0));
            motor1Queue.add( new SwerveActionDelay(Integer.toString(actioncounter++), 2.0));
            motor1Queue.add( new SwerveActionTimedMotor(Integer.toString(actioncounter++), null, DcMotor.Direction.FORWARD, 1.0, 2.0));

        }
        if (gamepad1.b)
        {
            motor1Queue.clear();
        }

        //process the queue
        if (motor1Queue.size() > 0)
        {
            //get the first item
            SwerveAction a = motor1Queue.getFirst();
            if (a!=null)  //don't really need this check since we looked at the queue size already
            {
                if (!a.IsStarted()) {
                    a.Start(runtime);
                }
                a.Update(runtime);
            }

            //remove any done actions from the queue
            //in c# you can't remove an item during an iterator because that messes up the iterator
            //I'm not sure if java handles that better, but to be safer I'll remove after iterating
            LinkedList<SwerveAction> removelist = new LinkedList<SwerveAction>();
            for (SwerveAction d : motor1Queue)
            {
                if (d.IsDone()) removelist.add(d);
            }
            for (SwerveAction r : removelist)
            {
                motor1Queue.remove(r);
            }

        }

        //stop motors
        if (motor1Queue.size()==0)
        {
            //to do: stop motor 1 here?
            //instead, I think we should consider using an explicit stop action,
            //and like lego, allowing an optional param to motor moves that decides if they should brake at end.
        }

        telemetry.addData("Count", "Queue length: " + motor1Queue.size());
        String message = "";
        for (SwerveAction d : motor1Queue)
        {
            message += d.ToString() +"\n";
        }
        telemetry.addData("Message", "motor1queue:\n" + message);

    }

    /*
    * Code to run when the op mode is first disabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
    */
    @Override
    public void stop() {

    }
}
