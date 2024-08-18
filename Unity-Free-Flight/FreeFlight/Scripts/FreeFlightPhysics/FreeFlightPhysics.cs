using UnityEngine;
using System.Collections;


/// <summary>
/// 	Simulate the flow of air around an airfoil. This class applies
/// 	passive forces on a flight object, such as lift and drag. It 
/// 	only applies these forces based on the current state of the wing,
/// 	but never changes the actual wing itself (so no flapping). 
/// </summary>
public class FreeFlightPhysics : FlightObject
{

    public bool liftEnabled = true;
    public bool dragEnabled = true;
    public bool gravityEnabled = true;


    private float formDrag;
    private float liftInducedDrag;


    //These are defined here for efficiency, so they aren't re-initialized every time
    //we run FixedUpdate(). They all get new derived values every update run. You probably
    //shouldn't edit them outside of FixedUpdate()
    private float liftForce;
    private float dragForce;
    private Vector3 directionalLift;//升力
    private Vector3 directionalDrag;//反向阻力
    private float liftCoefficient;
    private float dragCoefficient;
    //angle at which flying body contacts an air mass
    //(A plane/bird has a high angle of attack when they land, nose up, into the wind)
    private float angleOfAttack;

    protected Rigidbody rigidbody;

    public FreeFlightPhysics(Rigidbody rb)
    {
        rigidbody = rb;
    }

    //Forces

    public float FormDrag
    {
        get { return convert(Units.Metric, _unit, Types.Force, formDrag); }
    }
    public float LiftInducedDrag
    {
        get { return convert(Units.Metric, _unit, Types.Force, liftInducedDrag); }
    }

    public float Drag
    {
        get { return convert(Units.Metric, _unit, Types.Force, dragForce); }
    }

    public float Lift
    {
        get { return convert(Units.Metric, _unit, Types.Force, liftForce); }
    }

    //Dimensionless numbers

    public float AngleOfAttack
    {
        get { return angleOfAttack; }
    }

    public float LiftCoefficient
    {
        get { return liftCoefficient; }
    }

    public float DragCoefficient
    {
        get { return dragCoefficient; }
    }

    //Rates of Lengthy stuff over time
    public float VerticalSpeed
    {
        get
        {
            float MPS = rigidbody.velocity.y;
            return convert(Units.Metric, _unit, Types.ShortDistance, MPS);
        }
    }

    /// <summary>
    /// 
    /// KPH：千米每小时 （kilometers per hour）
    /// </summary>
    public float Speed
    {
        get
        {
            float KPH = rigidbody.velocity.magnitude * 3.6f;
            return convert(Units.Metric, _unit, Types.LongDistance, KPH);
        }
    }

    //Vectors
    public Vector3 Velocity
    {
        get { return rigidbody.velocity; }
    }

    public Vector3 Rotation
    {
        get { return rigidbody.rotation.eulerAngles; }
    }

    //——新增——

    /// <summary>
    /// 正常速度：50~60
    /// 俯冲可能更快
    /// </summary>
    public virtual float MaxSpeed { get { return maxSpeed; } set { maxSpeed = value; } }
    private float maxSpeed;

    public virtual float LiftPower { get { return liftPower; } set { liftPower = value; } }
    private float liftPower;

    /// <summary>
    /// 	Apply the regular flight physics to the flight object.
    /// 
    /// 	Precondition: This must be called at regular intervals
    /// to have a smooth effect on the physics object (use FixedUpdate). 
    /// </summary>
    public void doStandardPhysics()
    {
        if (rigidbody.isKinematic)
            return;

        rigidbody.useGravity = gravityEnabled;

        //Apply the user rotation in a banked turn
        rigidbody.rotation = getBankedTurnRotation(rigidbody.rotation);
        //Correct our velocity for the new direction we are facing
        //		newVelocity = getDirectionalVelocity(newRotation, newVelocity);	
        rigidbody.velocity = Vector3.Lerp(rigidbody.velocity, getDirectionalVelocity(rigidbody.rotation, rigidbody.velocity), Time.deltaTime);

        //These are required for computing lift and drag	
        angleOfAttack = getAngleOfAttack(rigidbody.rotation, rigidbody.velocity);

        if (rigidbody.velocity != Vector3.zero)
        {
            if (liftEnabled)
            {
                liftCoefficient = getLiftCoefficient(angleOfAttack);
                // apply lift force
                liftForce = getLift(rigidbody.velocity.magnitude * LiftPower, 0, currentWingArea, liftCoefficient) * Time.deltaTime;

                liftForce = Mathf.Max(0, liftForce);//避免扇翅膀导致负值

                if (!float.IsNaN(liftForce))
                {
                    directionalLift = Quaternion.LookRotation(rigidbody.velocity) * Vector3.up;
                    rigidbody.AddForce(directionalLift * liftForce);
                }
                else
                {
                    //Non critical warning. These should be removed after the Tuned update v0.5.0
                    Debug.LogWarning("Lift force generated NAN error!");
                }

                //newRotation = getStallRotation (newRotation, newVelocity.magnitude);

            }

            if (dragEnabled)
            {
                dragCoefficient = getDragCoefficient(angleOfAttack);
                // get drag rotation
                dragForce = getDrag(rigidbody.velocity.magnitude, 0, currentWingArea, dragCoefficient, liftForce, AspectRatio) * Time.deltaTime;
                if (!float.IsNaN(dragForce))
                {
                    directionalDrag = Quaternion.LookRotation(rigidbody.velocity) * Vector3.back;
                    // Debug.Log(string.Format ("Drag Direction: {0}, Drag Newtons/Hour: {1}", directionalDrag, dragForce * 3600.0f));
                    rigidbody.AddForce(directionalDrag * dragForce);
                }
                else
                {
                    Debug.LogWarning("Drag force generated NAN error!");
                }
            }
        }
    }



    /// <summary>
    /// 	Get the current lift force on the object based on the paramaters. 
    /// 
    /// 	All dimensions must be in metric.
    /// </summary>
    /// <returns>The lift in Newtons</returns>
    /// <param name="velocity">Velocity Meters/Second</param>
    /// <param name="pressure">Pressure (something)</param>
    /// <param name="area">Area Meters^2</param>
    /// <param name="liftCoff">Lift coff. (dimensionless)</param>
    public float getLift(float velocity, float pressure, float area, float liftCoff)
    {
        pressure = 1.225f;
        float lift = velocity * velocity * pressure * area * liftCoff;
        return lift;
    }

    /// <summary>
    /// 	Get the current drag force on the object based on the parameters. 
    /// 
    /// 	All dimensions must be in metric.
    /// </summary>
    /// <returns>The drag in Newtons</returns>
    /// <param name="velocity">Velocity Meters/Second</param>
    /// <param name="pressure">Pressure (something)</param>
    /// <param name="area">Area Meters^2</param>
    /// <param name="dragCoff">Drag coff (dimensionless)</param>
    /// <param name="lift">Lift Newtons</param>
    /// <param name="aspectR">Aspect r. (dimensionless)</param>
    public float getDrag(float velocity, float pressure, float area, float dragCoff, float lift, float aspectR)
    {
        //wing span efficiency value
        float VSEV = .9f;
        pressure = 1.225f;

        liftInducedDrag = (lift * lift) / (.5f * pressure * velocity * velocity * area * Mathf.PI * VSEV * aspectR);
        formDrag = .5f * pressure * velocity * velocity * area * dragCoff;
        return LiftInducedDrag + FormDrag;
    }

    //Rotates the object down when velocity gets low enough to simulate "stalling"
    public Quaternion getStallRotation(Quaternion curRot, float velocity)
    {
        //This equation isn't based on any real-world physics. But it seems to work pretty well as is.
        float pitchRotationSpeed = 10.0f / (velocity * velocity);
        Quaternion pitchrot = Quaternion.LookRotation(Vector3.down);
        Quaternion newRot = Quaternion.Lerp(curRot, pitchrot, Mathf.Abs(pitchRotationSpeed) * Time.deltaTime);
        return newRot;
    }

    //When we do a turn, we don't just want to rotate our character. We want their
    //velocity to match the direction they are facing. 
    public Vector3 getDirectionalVelocity(Quaternion theCurrentRotation, Vector3 theCurrentVelocity)
    {
        Vector3 vel = theCurrentVelocity;

        vel = (theCurrentRotation * Vector3.forward).normalized * theCurrentVelocity.magnitude;
        //Debug.Log (string.Format ("velocity: {0}, New Velocity {1} mag1: {2}, mag2 {3}", theCurrentVelocity, vel, theCurrentVelocity.magnitude, vel.magnitude));
        return vel;
    }

    //Get new yaw and roll, store the value in newRotation
    public Quaternion getBankedTurnRotation(Quaternion theCurrentRotation)
    {
        //Quaternion getBankedTurnRotation(float curZRot, float curLift, float curVel, float mass) {
        // The physics of a banked turn is as follows
        //  L * Sin(0) = M * V^2 / r
        //	L is the lift acting on the aircraft
        //	θ0 is the angle of bank of the aircraft
        //	m is the mass of the aircraft
        //	v is the true airspeed of the aircraft
        //	r is the radius of the turn	
        //
        // Currently, we'll keep turn rotation simple. The following is not based on the above, but it provides
        // A pretty snappy mechanism for getting the job done.
        //Apply Yaw rotations. Yaw rotation is only applied if we have angular roll. (roll is applied directly by the 
        //player)
        Quaternion angVel = Quaternion.identity;
        //Get the current amount of Roll, it will determine how much yaw we apply.
        float zRot = Mathf.Sin(theCurrentRotation.eulerAngles.z * Mathf.Deg2Rad) * Mathf.Rad2Deg;
        //We don't want to change the pitch in turns, so we'll preserve this value.
        float prevX = theCurrentRotation.eulerAngles.x;
        //Calculate the new rotation. The constants determine how fast we will turn.
        Vector3 rot = new Vector3(0, -zRot * 0.8f, -zRot * 0.5f) * Time.deltaTime;

        //Apply the new rotation 
        angVel.eulerAngles = rot;
        angVel *= theCurrentRotation;
        angVel.eulerAngles = new Vector3(prevX, angVel.eulerAngles.y, angVel.eulerAngles.z);

        //Done!
        return angVel;
    }


    public float getLiftCoefficient(float angleDegrees)
    {
        float cof;
        //			if(angleDegrees > 40.0f)
        //				cof = 0.0f;
        //			if(angleDegrees < 0.0f)
        //				cof = angleDegrees/90.0f + 1.0f;
        //			else
        //				cof = -0.0024f * angleDegrees * angleDegrees + angleDegrees * 0.0816f + 1.0064f;
        //Formula based on theoretical thin airfoil theory. We get a very rough estimate here,
        //and this does not take into account wing aspect ratio
        cof = 2 * Mathf.PI * angleDegrees * Mathf.Deg2Rad;
        return cof;

    }

    public float getDragCoefficient(float angleDegrees)
    {
        float cof;
        //if(angleDegrees < -20.0f)
        //	cof = 0.0f;
        //else
        cof = .0039f * angleDegrees * angleDegrees + .025f;
        return cof;
    }


    //Return angle of attack based on objects current directional Velocity and rotation
    public float getAngleOfAttack(Quaternion theCurrentRotation, Vector3 theCurrentVelocity)
    {
        //Angle of attack is basically the angle air strikes a wing. Imagine a plane flying 
        //at exact level altitude into a stable air mass. The air passes over the wing very
        //efficiently, so we have an AOA of zero. When the plane pitches back, air starts to
        //strike the bottom of the wing, creating more drag and lift. The angle of pitch 
        //relative to the airmass is called angle of attack. 
        float theAngleOfAttack;
        //The direction we are going
        Vector3 dirVel;

        //We need speed in order to get directional velocity.
        if (theCurrentVelocity != Vector3.zero)
        {
            //Find the direction we are going
            dirVel = Quaternion.LookRotation(theCurrentVelocity) * Vector3.up;
        }
        else
        {
            //This has the effect of 'imagining' the craft is on a level flight
            //moving forward. Since angle of attack means nothing at zero speed,
            //this is simply a way to visualize it when we are dead stopped.
            dirVel = Vector3.up;
        }

        //		Debug.Log(string.Format ("Directional Velocity : {0}", dirVel));

        //Find the rotation directly in front of us
        Vector3 forward = theCurrentRotation * Vector3.forward;
        //The dot product returns a positive or negative float if we are 'pitched up' towards
        //our air mass, or 'pitched down into' our airmass. Remember that our airmass also has
        //a velocity coming towards us, which is somewhere between coming directly at us in
        //level flight, or if we are falling directly towards the ground, it is coming directly
        //below us. 

        //The dot product always returns between -1 to 1, so taking the ArcSin will give us
        //a reasonable angle of attack. Remember to convert to degrees from Radians. 
        theAngleOfAttack = Mathf.Asin(Vector3.Dot(forward, dirVel)) * Mathf.Rad2Deg;
        return theAngleOfAttack;
    }

}
