using UnityEngine;
using System.Collections;

/// <summary>
/// Hash IDs for animation. These numbers will be gotten from the animator on startup. The reason it isn't a 
/// MonoBehaviour is to avoid additional clutter on the editor screen. 
/// </summary>
public class FreeFlightAnimationHashIDs {

	public int speedFloat;
	public int angularSpeedFloat;
	public int flyingBool;
	public int flappingTrigger;

	public int walkingBool;
	public int flaringBool;
	public int divingBool;
    public int divingLeftTrigger;
    public int divingRightTrigger;
    public int dyingTrigger;

    //#State
    public int idleState;
    public int walkingState;
    public int flappingState;
	public int flaringState;
	public int glidingState;
	public FreeFlightAnimationHashIDs () {
		speedFloat = Animator.StringToHash("Speed");
		angularSpeedFloat = Animator.StringToHash("AngularSpeed");
		flyingBool = Animator.StringToHash ("Flying");
		flappingTrigger = Animator.StringToHash ("Flapping");
		walkingBool = Animator.StringToHash ("Walking");
		flaringBool = Animator.StringToHash ("Flaring");
		divingBool = Animator.StringToHash ("Diving");
        divingLeftTrigger = Animator.StringToHash("DivingLeft");
        divingRightTrigger = Animator.StringToHash("DivingRight");
        dyingTrigger = Animator.StringToHash ("Dying");

        idleState = Animator.StringToHash("Base Layer.Idle");
        walkingState = Animator.StringToHash("Base Layer.Walking");
        glidingState = Animator.StringToHash("Base Layer.Gliding");
		flappingState = Animator.StringToHash("Base Layer.Flapping");
		flaringState = Animator.StringToHash("Base Layer.Flaring");
	}
}
