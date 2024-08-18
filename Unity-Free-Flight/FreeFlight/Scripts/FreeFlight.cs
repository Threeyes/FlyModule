

//==================
//Organization of this class is as follows:
// * Variable Declaration
// 		* Flight input Declaration
//		* Ground input Declaration
// * Unity Events
// * Ground Functionality
// * Takeoff/landing Functionality
// * Flight Functionality
// * Audio
//==================

using UnityEngine;
using System.Collections;
using System;
using Threeyes.Core;

/// <summary>
/// 原理：
/// -Shift：张开双翼，将X、Z轴反向调最大，相当crash于刹车
/// 
/// 名词：
/// -Flare：张翼
/// -Pitch：X轴旋转
/// 
/// Todo：
/// 【参考塞尔达键位】
///     +QE改为升降
///     +WS改为加减速
///     -Boost键与方向键组合，大于一定值才触发
///         +前后是急加速/刹车
///         -左右是翻滚（使用MoveRotation）
/// -Glitch时才出现尾流
/// -贴近地面时不落下，而是贴着飞行（因为有浮力）。主动按下降键才会降落（通过射线检测，在 OnCollisionEnter 中处理）
/// -被攻击时临时切换为布娃娃物体
/// 
/// ToFix:
/// -每次重新显示物体后，Rig相关组件会被拉长（需要Rebuild）
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class FreeFlight : MonoBehaviour
{
    #region Property&Fields
    public Func<Vector3, Vector3, Vector3> GetRecoverPos;//从Ragdoll切换为正常模式时重置的位置的代码（参数分别为撞击前的位置、ragdoll的位置）

    public Action<bool> ActiveRagdollMode;//切换Ragdoll模式的方法
    public Action<bool> RagdollModeActived;//切换Ragdoll模式的回调

    public Action<bool> LockHeightModeActived;//切换锁定高度模式的回调


    /*
	 * None -- No action is taken by Free Flight
	 * Ground -- Get ground inputs from the player and apply them.
	 * Flight -- Get flight inputs from the player and apply them.
	 */
    public enum FlightState { None, Ground, Flight };
    private FlightState _state = FlightState.Ground;
    public FlightState state
    {
        get { return _state; }
        set { _state = value; }
    }
    public bool applyFlightPhysicsOnGround = false;

    private CreatureFlightPhysics _flightPhysics;
    public CreatureFlightPhysics flightPhysics
    {
        get
        {
            if (_flightPhysics == null)
            {
                _flightPhysics = new CreatureFlightPhysics(rig);
            }
            return _flightPhysics;
        }
        set { _flightPhysics = value; }
    }
    private FreeFlightAnimationHashIDs _ffhash;
    private FreeFlightAnimationHashIDs ffhash
    {
        get
        {
            if (_ffhash == null)
                _ffhash = new FreeFlightAnimationHashIDs();
            return _ffhash;
        }
        set { _ffhash = value; }
    }

    //供外部相机追踪的点
    public LayerMask layerMaskGround;
    public Transform tfOriginCenter;//正常模型的中心点
    public Transform tfOriginForward;//正常模型的前向点
    public Transform tfRagdollCenter;//Ragdoll模型的中心点
    public Transform tfRagdollForward;//Ragdoll模型的前向点
    public Transform tfDynamicCenterPoint;// { get { return isRagdollMode ? tfRagdollCenter : tfOriginCenter; } }//运行时的中心点，会根据是否Ragdoll进行改变

    public AudioClip speakSoundClip;//海鸥叫
    private AudioSource speakSoundSource;
    public Animator _anim;//直接在编译器设置
    public Camera mainCamera;//直接在编译器设置

    //Lazy instantiation allows for real time compilation 
    private Animator anim
    {
        get
        {
            if (_anim == null)
            {
                _anim = GetComponentInChildren<Animator>();
                if (_anim == null)
                {
                    //Fail if there is no animator. We want to fail instead of 
                    //simply adding an Animator because we don't know where the 
                    //model is for this object, and that decision is best left to
                    //the user. 
                    this.enabled = false;
                    throw new AnimatorNotPresentException(string.Format(
                        "Please add an Animator component to the model for " +
                        "{0}. The BasicFreeFlight Animation Controller will work" +
                        " even if your model doesn't have animations. ", gameObject.name));
                }
            }
            return _anim;
        }
        set { _anim = value; }
    }


    //=============
    //Unity Editor-Configurable Settings
    //=============

    //#新增
    public Vector3 Position { get { return tfDynamicCenterPoint.position; } }//当前坐标
    public bool enabledAccelerate = true;
    public float accelerateSensitivity = 2.0f;
    public float maxSpeed = 120.0f;//（通过加减速达到的）最大飞行速度  【海鸥正常速度：50~60。俯冲可能更快

    public bool enabledGliding = true;
    //Basic gliding input, values in degrees
    public float maxTurnBank = 45.0f;
    public float maxPitch = 20.0f;
    public float directionalSensitivity = 2.0f;

    public bool enabledFlapping = true;
    public AudioClip flapSoundClip;
    private AudioSource flapSoundSource;
    public float liftPower = 2;//Lift的缩放

    //	public float regularFlaptime = 0.5f;
    //	public float minimumFlapTime = 0.2f;
    public float flapStrength = 60.0f;
    //	public float downbeatStrength = 150.0f;

    public bool enabledFlaring = false;
    public AudioClip flareSoundClip;
    private AudioSource flareSoundSource;
    //The default pitch (x) we rotate to when we do a flare
    public float flareAngle = 70.0f;
    public float flareSpeed = 3.0f;

    public bool enabledDiving = false;
    public AudioClip divingSoundClip;
    private AudioSource divingSoundSource;

    public bool enabledTakeoff = true;
    public AudioClip takeoffSoundClip;
    private AudioSource takeoffSoundSource;

    public bool enabledLaunchIfAirborn = true;
    public float minHeightToLaunchIfAirborn = 2f;

    public bool enabledLanding = true;
    public AudioClip landingSoundClip;
    private AudioSource landingSoundSource;
    public AudioClip landingOnWaterSoundClip;
    private AudioSource landingOnWaterSoundSource;
    //Max time "standUp" will take to execute.
    public float maxStandUpTime = 2.0f;
    //Speed which "standUp" will correct rotation. 
    public float standUpSpeed = 2.0f;

    public bool enabledCrashing = false;
    public float crashSpeed = 40f;
    public AudioClip crashSoundClip;
    private AudioSource crashSoundSource;
    public Transform tfRagdoll;
    public float ragdollTime = 3;

    public bool enabledWindNoise = true;
    public AudioClip windNoiseClip;
    private AudioSource windNoiseSource;
    public float windNoiseStartSpeed = 20.0f;
    public float windNoiseMaxSpeed = 200.0f;
    public BoolEvent onWindStartStop = new BoolEvent();//风声开启/停止，可用于开启特定的特效

    //===========
    //USER INPUT FLIGHT
    //===========


    //These protected vars are meant to be directly used or modified by the 
    //child class, and generally read from by the physics model. 
    [Range(0.0f, 1.0f)]
    protected float _inputLeftWingExposure = 1.0f;
    [Range(0.0f, 1.0f)]
    protected float _inputRightWingExposure = 1.0f;
    public float LeftWingInput { get { return _inputLeftWingExposure; } }
    public float RightWingInput { get { return _inputRightWingExposure; } }
    public float LeftWingExposure { get { return flightPhysics.LeftWingExposure; } }
    public float RightWingExposure { get { return flightPhysics.RightWingExposure; } }
    protected int _inputInvertedSetting = -1;//1;
    protected bool _inputTakeoff = false;
    protected bool _inputFlaring = false;
    protected bool _inputDiving = false;
    protected bool _inputFlap = false;
    protected bool _inputFlapUp = false;
    protected float _inputAccelerate = 0.0f;
    [Range(-1.0f, 1.0f)]
    protected float _inputPitch = 0.0f;
    [Range(-1.0f, 1.0f)]
    protected float _inputBank = 0.0f;

    public bool InputTakeoff { get { return _inputTakeoff; } }
    public bool InputFlaring { get { return _inputFlaring; } }
    public bool InputDiving { get { return _inputDiving; } }
    public bool InputFlap { get { return _inputFlap; } }
    public float InputPitch { get { return _inputPitch; } }
    public float AnglePitch { get { return getPitch(_inputFlaring); } }
    public float InputBank { get { return _inputBank; } }
    public float AngleBank { get { return getBank(); } }

    public float launchTime = 0.2f;
    private float launchTimer;

    //
    /// <summary>
    /// 翻转输入
    /// 
    /// Even though Inverted as a property here is invisible to the inspector, 
    /// using the property in this way makes it convienient to access externally,
    /// in order to *toggle* the setting on and off. Expressing _invertedSetting internally
    /// an integer makes it very easy to apply to input. 
    /// </summary>
    public bool Inverted
    {
        get
        {
            if (_inputInvertedSetting == 1)
                return true;
            return false;
        }
        set
        {
            if (value == true)
                _inputInvertedSetting = -1;
            else
                _inputInvertedSetting = 1;
        }
    }

    //===========
    //USER INPUT GROUND
    //===========

    public bool enabledGround = true;//允许地面移动
    public AudioClip walkingNoiseClip;
    private AudioSource walkingNoiseSource;
    //meters/second
    public float maxGroundForwardSpeed = 40;
    //degrees/second
    public float groundDrag = 5;
    public float maxGroundTurningDegreesSecond = 40;
    //meters
    public bool enabledJumping = false;
    public float jumpHeight = .5f;
    public AudioClip jumpingNoiseClip;
    private AudioSource jumpingNoiseSource;

    [Range(-1.0f, 1.0f)]
    protected float _inputGroundForward;
    [Range(-1.0f, 1.0f)]
    protected float _inputGroundTurning;
    protected bool _inputJump;

    public float InputGroundForward { get { return _inputGroundForward; } }
    public float InputGroundTurning { get { return _inputGroundTurning; } }
    public bool InputJump { get { return _inputJump; } }

    //# Input
    public FreeFlightInput freeFlightInput;
    bool RTIsInputJump { get { return freeFlightInput.IsJumping; } }//扇翅膀上升/跳起
    float RTInputBank { get { return -freeFlightInput.HorizontalInput; } }//Z轴旋转（左右）

    float RTInputAccelerating
    {
        get
        {
            return freeFlightInput.VerticalInput;
        }
    }//加减速

    bool RTIsInputFlap { get { return freeFlightInput.IsJumping; } }//煽翅膀
    bool RTIsInputBoost { get { return freeFlightInput.IsBoosting; } }//加速操作（与方向键组合使用）
    bool RTIsInputLifting { get { return freeFlightInput.IsLifting; } }//升降操作/X轴旋转（与方向键组合使用）
    #endregion

    #region  Unity Events
    Rigidbody rig;
    Rigidbody rigRagdollCenter;
    void Awake()
    {
        rig = GetComponent<Rigidbody>();
        rigRagdollCenter = tfRagdollCenter.GetComponent<Rigidbody>();
        setupSound(speakSoundClip, ref speakSoundSource);
        setupSound(windNoiseClip, ref windNoiseSource);
        setupSound(flapSoundClip, ref flapSoundSource);
        setupSound(flareSoundClip, ref flareSoundSource);
        setupSound(divingSoundClip, ref divingSoundSource);
        setupSound(takeoffSoundClip, ref takeoffSoundSource);
        setupSound(landingSoundClip, ref landingSoundSource);
        setupSound(landingOnWaterSoundClip, ref landingOnWaterSoundSource);
        setupSound(crashSoundClip, ref crashSoundSource);
        setupSound(walkingNoiseClip, ref walkingNoiseSource);
        setupSound(jumpingNoiseClip, ref jumpingNoiseSource);
        rig.freezeRotation = true;
        rig.isKinematic = false;

        //Init FP
        flightPhysics.MaxSpeed = maxSpeed;
        flightPhysics.LiftPower = liftPower;

        InitRagdoll();
    }

    /// <summary>
    /// Get input from the player 
    /// </summary>
    void Update()
    {
        ///ToAdd:只要在特定状态（Idle、Walking，就持续检测是否为地面。避免因为鸟行走导致走到空位置）
        //IsOnGround
        if (!isInsideWater && !isRagdollMode && (IsCurAnimatorState(ffhash.idleState) || IsCurAnimatorState(ffhash.walkingState)))
        {
            if (!IsRaycastCheckOnGround)
            {
                SwitchToState(FlightState.Flight);
            }
        }

        if (isFlying())
        {
            getFlightInputs();
        }
        else if (isGrounded())
        {
            getGroundInputs();
        }

        //#Test
        if (freeFlightInput.IsBulletTimeDown) BulletTimeManager.Instance.EnterBulletTime();
        else if (freeFlightInput.IsBulletTimeUp) BulletTimeManager.Instance.ExitBulletTime();


        //#Common Settting
        CommonInputs();
        //#Visual
        UpdateVisual();
    }

    void CommonInputs()
    {
        if (!isRagdollMode)
        {
            //#按下发声 （用途：吸引人的注意力）
            if (freeFlightInput.IsYellDown)
            {
                playSound(speakSoundSource);
            }
            //else if (freeFlightInput.IsSpeakUp)抬起停止（待确认）
            //{
            //    stopSound(speakSoundSource);
            //}



            if (freeFlightInput.IsLockHeightDown)//按下锁定高度键：切换锁定
            {
                LockHeight(!isLockingHeight);
            }
            else
            {
                if (_inputDiving && isLockingHeight)//按下下潜按键：取消锁定
                {
                    LockHeight(false);
                }
            }
        }
    }

    /// <summary>
    /// In relation to Update() this is where we decide how to act on the user input, then
    /// compute the physics and animation accordingly
    /// </summary>
    void FixedUpdate()
    {
        if (!isRagdollMode)
        {
            if (isFlying())
            {
                applyFlightInputs();
            }
            else if (applyFlightPhysicsOnGround && isGrounded())
            {
                flightPhysics.doStandardPhysics();
            }

            if (isGrounded())
            {
                applyGroundInputs();
            }
        }

        if (enabledWindNoise)
            applyWindNoise();
    }

    public BoolEvent onShowHideWingTrail = new BoolEvent();//翼的尾流
    bool isShowingWingTrail = false;
    private void UpdateVisual()
    {
        bool tempShowingWingTrail = isShowingWingTrail;
        tempShowingWingTrail = !_inputFlap && IsCurAnimatorState(ffhash.glidingState) && flightPhysics.Speed >= maxSpeed * 0.8f;

        if (isShowingWingTrail != tempShowingWingTrail)
        {
            onShowHideWingTrail.Invoke(tempShowingWingTrail);
            isShowingWingTrail = tempShowingWingTrail;
        }
    }


    //Default behaviour when we hit an object (usually the ground) is to switch to a ground controller. 
    //Override in controller to change this behaviour.
    protected void OnCollisionEnter(Collision col)
    {
        //Debug.LogError("FF OnCollisionEnter: " + col.impulse.magnitude);

        //——计算出反向的作用力(改为由父类计算并传入)——
        Vector3 opposeForce = Vector3.zero;
        if (col != null && col.contactCount > 0)
        {
            // Calculate Angle Between the collision point and the player
            ContactPoint contactPoint = col.contacts[0];
            Vector3 dir = contactPoint.point - transform.position;
            // We then get the opposite (-Vector3) and normalize it
            //dir = -dir.normalized;

            dir = Vector3.Reflect(dir, contactPoint.normal);
            // And finally we add force in the direction of dir and multiply it by force. 
            // This will push back the player
            opposeForce = dir * col.relativeVelocity.magnitude;
        }

        land(opposeForce);
    }

    //#【V2】水下功能
    public bool isInsideWater = false;
    public LayerMask layerMaskWater = 16;//Water(2的N次方)
    int layerWaterNum { get { return (int)Mathf.Log(layerMaskWater.value, 2); } }//4
    private void OnTriggerEnter(Collider other)
    {
        //Debug.LogError("OnTriggerEnter" + other.gameObject.layer + "  " + layerMaskWater.value + " " + layerNumber);
        if (other.gameObject.layer == layerWaterNum)//在水面上：Land，且不让其下沉（可以是锁定Y轴，或者是增加持续的浮力）
        {
            isInsideWater = true;
            land();
        }
    }
    private void OnTriggerExit(Collider other)
    {
        if (other.gameObject.layer == layerWaterNum)
        {
            isInsideWater = false;
        }
    }
    #endregion

    #region Functionality -- Ground

    public bool isGrounded()
    {
        if (state == FlightState.Ground)
            return true;
        return false;
    }

    private void getGroundInputs()
    {

        _inputGroundForward = freeFlightInput.VerticalInput;
        _inputGroundTurning = freeFlightInput.HorizontalInput;
        _inputJump = RTIsInputJump;
        _inputTakeoff = _inputJump;

    }

    private void applyGroundInputs()
    {
        if (enabledJumping && _inputJump)
        {
            jump();
        }
        else if (enabledGround)
        {
            groundMove();
        }

        if (enabledTakeoff)
            timedLaunch(_inputTakeoff);


        if (enabledLaunchIfAirborn)//允许在空中发射（通过向上增加升力，但不扇翅膀）【Bug：可能导致从Ragdoll恢复时，意外切换到Flight状态】
            launchIfAirborn(minHeightToLaunchIfAirborn);
    }

    private void jump()
    {
        //anim.SetTrigger("Jumping");
        playSound(jumpingNoiseSource);
        rig.AddForce(0, jumpHeight, 0, ForceMode.Force);
    }

    private void groundMove()
    {
        rig.drag = groundDrag * (isInsideWater ? 2 : 1);//水中增大阻力
        //if (_inputGroundForward > 0f)//注释原因：允许退后
        {
            rig.AddRelativeForce(Vector3.forward * maxGroundForwardSpeed * _inputGroundForward * Time.fixedDeltaTime, ForceMode.VelocityChange);
        }

        float turningSpeed = maxGroundTurningDegreesSecond * _inputGroundTurning * Time.fixedDeltaTime;
        rig.rotation *= Quaternion.AngleAxis(turningSpeed, Vector3.up);

        anim.SetBool(ffhash.walkingBool, (_inputGroundForward > 0f || _inputGroundTurning != 0f));
        anim.SetFloat(ffhash.speedFloat, rig.velocity.magnitude);
        anim.SetFloat(ffhash.angularSpeedFloat, turningSpeed);
    }
    #endregion

    #region  Functionality -- Takeoff/Landing

    /// <summary>
    /// Launchs if airborn.
    /// </summary>
    /// <param name="minHeight">Minimum height.</param>
    private void launchIfAirborn(float minHeight)
    {
        if (!Physics.Raycast(transform.position, Vector3.down, minHeight))//下方无障碍物：设置为空中状态
        {
            takeoff(false);
        }
    }

    /// <summary>
    /// Calls takeoff() after "triggerSet" has been true for "launchTime". 
    /// This method needs to be called in Update or FixedUpdate to work properly. 
    /// </summary>
    /// <param name="triggerSet">If set to <c>true</c> for duration of launchTimer, triggers takeoff.</param>
    private void timedLaunch(bool triggerSet)
    {
        if (triggerSet == true)
        {
            if (launchTimer > launchTime)
            {
                takeoff(true);
                launchTimer = 0.0f;
            }
            else
            {
                launchTimer += Time.fixedDeltaTime;
            }
        }
        else
        {
            launchTimer = 0.0f;
        }
    }

    /// <summary>
    /// Set the state to flying and enable flight physics. Optionally, flapLaunch
    /// can be set to true to apply a "flap" to help get the object off the ground. 
    /// 【起跳】
    /// </summary>
    /// <param name="flapLaunch">If set to <c>true</c> flap launch.</param>
    protected void takeoff(bool flapLaunch = false)
    {
        if (!isFlying())
        {
            SwitchToState(FlightState.Flight);

            playSound(takeoffSoundSource);
            if (flapLaunch)
                flap();
        }
    }


    private void land(Vector3? ragdollForce = null)
    {
        //#Todo: 【FixedUpdate中执行】【非必须，更考验用户操作】当靠近碰撞体时，应该有一个推力，模拟流体表面的力。或者是只在特定碰撞体中降落，其他地方忽略碰撞

        if (isFlying())
        {
            SwitchToState(FlightState.Ground);
            AudioSource targetAudioSource = null;
            if (enabledCrashing && flightPhysics.Speed >= crashSpeed)//允许碰撞，且当前速度超过碰撞速度：死亡
            {
                ///ToUpdate：
                ///+切换为布娃娃物体
                ///-布娃娃状态时忽略输入，并通过协程恢复(可以改为按键唤醒)
                ///-恢复后要把当前位置重置到布娃娃当前点
                ///
                if (coroutine_IETempSwitchToRagdoll != null)
                    StopCoroutine(coroutine_IETempSwitchToRagdoll);
                coroutine_IETempSwitchToRagdoll = StartCoroutine(IETempSwitchToRagdoll(ragdollForce));

                //anim.SetTrigger(ffhash.dyingTrigger);//暂不需要死亡动画
                targetAudioSource = isInsideWater ? landingOnWaterSoundSource : crashSoundSource;
            }
            else//否则逐渐站稳
            {
                StartCoroutine(standUp());
                targetAudioSource = isInsideWater ? landingOnWaterSoundSource : landingSoundSource;
            }

            playSound(targetAudioSource, customVolume: (flightPhysics.Speed / maxSpeed) + 0.5f);//0.5f保底
        }
    }
    void SwitchToState(FlightState flightState)
    {
        state = flightState;
        //Debug.LogError("SwitchToState: " + state);
        switch (flightState)
        {
            case FlightState.Ground:
                state = FlightState.Ground;
                _inputFlaring = false;
                _inputFlap = false;
                rig.freezeRotation = true;
                rig.isKinematic = false;
                anim.SetBool(ffhash.flaringBool, false);
                anim.SetBool(ffhash.flyingBool, false);
                break;
            case FlightState.Flight:
                anim.SetBool(ffhash.flyingBool, true);
                anim.SetBool(ffhash.walkingBool, false);//避免因从悬崖中走下导致继续播放Idle动画         
                rig.freezeRotation = true;
                rig.isKinematic = false;
                break;
        }
    }


    Coroutine coroutine_IETempSwitchToRagdoll;

    [ContextMenu("TestRagdollMode")]
    public void TestRagdollMode()
    {
        if (coroutine_IETempSwitchToRagdoll != null)
        {
            StopCoroutine(coroutine_IETempSwitchToRagdoll);
        }
        coroutine_IETempSwitchToRagdoll = StartCoroutine(IETempSwitchToRagdoll());
    }

    IEnumerator IETempSwitchToRagdoll(Vector3? ragdollForce = null)
    {
        Vector3 hitPos = tfOriginCenter.position;

        Animator animatorRagdoll = tfRagdoll.GetComponent<Animator>();

        //#0 禁用当前的设置
        rig.isKinematic = true;//临时锁定
        stopSound(speakSoundSource);//停止可能正在播放的说话

        //#1 重置Ragdoll，并尝试同步动画
        SwitchToRagdoll(false);//避免第一次未锁定刚体
        ShowRagdoll(true);
        yield return null;
        animatorRagdoll.enabled = true;
        AnimatorStateInfo curstate = anim.GetCurrentAnimatorStateInfo(0);
        animatorRagdoll.CrossFade(curstate.fullPathHash, 0, 0);
        yield return null;
        SwitchToRagdoll(true);//切换为Ragdoll状态

        //为Ragdoll根节点施加反向力
        if (ragdollForce.HasValue && rigRagdollCenter)
        {
            Vector3 targetForce = ragdollForce.Value;
            //限制最大值，确保不大于5
            if (targetForce.sqrMagnitude > 25)
                targetForce = ragdollForce.Value.normalized * 5;

            //Debug.LogError($"Add opposeForce to ragdoll: {targetForce}/{targetForce.magnitude}");
            rigRagdollCenter.AddForce(targetForce, ForceMode.Impulse);
        }

        yield return new WaitForSeconds(2);//先强制等待N秒（后续每次碰撞都可以累加）
        yield return new WaitUntil(() => RTIsInputJump);//直到有输入才可恢复

        //ToAdd：如果飞出地图（Y低于某个值），则回到撞击前的点，避免偏离太远

        Vector3 ragdollPos = tfRagdollCenter.position;
        Vector3 recoverPos = ragdollPos;//恢复位置：默认为当前Ragdoll的中心位置
        if (GetRecoverPos != null)//自行实现的复原位置，适用于调出边界时重置
        {
            recoverPos = GetRecoverPos(hitPos, ragdollPos);
        }

        rig.isKinematic = false;//取消锁定
        rig.position = recoverPos;
        rig.rotation = Quaternion.Euler(0, mainCamera.transform.rotation.y, 0);  //使用相机的前方
        rig.velocity = Vector3.zero;
        rig.angularVelocity = Vector3.zero;
        SwitchToRagdoll(false);
        ShowRagdoll(false);

        //根据当前是否在空中，切换对应的动画(调用SwitchToState)
        SwitchToState(IsRaycastCheckOnGround ? FlightState.Ground : FlightState.Flight);
    }

    /// <summary>
    /// 通过Raycast检查下方是否有物体，从而判断是否站在地上
    /// </summary>
    bool IsRaycastCheckOnGround
    {
        get
        {
            bool hasHit = false;
            //string hitInfo = "";

            //使用tfCenterPoint进行检测，可以在任何状态正常工作
            if (Physics.Raycast(new Ray(tfDynamicCenterPoint.position + new Vector3(0, 0.1f, 0), Vector3.down), out RaycastHit raycastHit, 0.5f, layerMaskGround))//原点上方的点进行下投，避免因为与地面高度相近导致无法正确检测
            {
                //hitInfo = raycastHit.transform.name;
                hasHit = true;
            }
            //else
            //{
            //    Debug.LogError("Hit: " + hitInfo);
            //}
            return hasHit;
        }
    }


    bool isRagdollMode = false;
    public void InitRagdoll()
    {
        tfRagdoll.gameObject.SetActive(false);
    }

    public void SwitchToRagdoll(bool tempIsRagdollMode)
    {
        isRagdollMode = tempIsRagdollMode;

        //更新中心点的位置
        tfDynamicCenterPoint.SetParent(isRagdollMode ? tfRagdollCenter : tfOriginCenter);
        tfDynamicCenterPoint.localPosition = Vector3.zero;
        tfDynamicCenterPoint.localRotation = Quaternion.identity;

        ActiveRagdollMode.Execute(tempIsRagdollMode);//调用对应的方法
        RagdollModeActived.Execute(tempIsRagdollMode);//通知回调
    }

    public void ShowRagdoll(bool isShow)
    {
        if (tfRagdoll)
        {
            anim.gameObject.SetActive(!isShow);//临时隐藏原物体
            tfRagdoll.gameObject.SetActive(isShow);
        }
    }

    /// <summary>
    /// Straightenes the flight object on landing, by rotating the roll and pitch
    /// to zero over time. Public vars "standUpSpeed" and "maxStandUpTime" can 
    /// be used to tweak behaviour.
    /// </summary>
    /// <returns>The up.</returns>
    protected IEnumerator standUp()
    {
        //Find the direction the flight object should stand, without any pitch and roll. 
        Quaternion desiredRotation = Quaternion.identity;
        desiredRotation.eulerAngles = new Vector3(0.0f, transform.rotation.eulerAngles.y, 0.0f);
        //Grab the current time. We don't want 'standUp' to take longer than maxStandUpTime
        float time = Time.time;

        transform.rotation = desiredRotation; //Quaternion.Lerp (transform.rotation, desiredRotation, standUpSpeed * Time.deltaTime);

        //Break if the player started flying again, or if we've reached the desired rotation (within 5 degrees)
        while (!isFlying() && Quaternion.Angle(transform.rotation, desiredRotation) > 5.0f)
        {
            //Additionally break if we have gone over time
            if (time + maxStandUpTime < Time.time)
                break;
            //Correct the rotation
            transform.rotation = Quaternion.Lerp(transform.rotation, desiredRotation, standUpSpeed * Time.fixedDeltaTime);
            yield return null;
        }
        yield return null;
    }
    #endregion

    #region Functionality -- Flight

    public bool isLockingHeight = false;
    /// <summary>
    /// 锁定高度，
    /// </summary>
    /// <param name="isLock"></param>
    void LockHeight(bool isLock)
    {
        isLockingHeight = isLock;

        //flightPhysics.dragEnabled = !isLock;//取消阻力导致的减速(Bug：flap后会导致持续的加速，先不激活)
        flightPhysics.gravityEnabled = !isLock;

        LockHeightModeActived.Execute(isLock);//通知回调
    }

    public bool isFlying()
    {
        if (state == FlightState.Flight)
            return true;
        return false;
    }


    float boostInputThreshold = 0.8f;
    private void getFlightInputs()
    {
        //新增
        //if (enabledAccelerate)
        {
            _inputAccelerate = RTInputAccelerating;
        }

        _inputPitch = freeFlightInput.PitchInput;  /*RTIsInputLifting ? _inputInvertedSetting * -RTInputAccelerating : 0*/;//X轴旋转，即上下转
        _inputBank = RTInputBank;//Z轴旋转，即左右转

        _inputFlap = RTIsInputFlap;//If the user presses down the jump button, flap
        _inputFlapUp = freeFlightInput.IsJumpingUp;

        if (enabledDiving)
        {
            _inputDiving = RTIsInputBoost && RTInputAccelerating > boostInputThreshold;//下潜加速
            if (_inputDiving)
            {
                _inputLeftWingExposure = 0.0f;
                _inputRightWingExposure = 0.0f;
            }
            else
            {
                _inputLeftWingExposure = 1.0f;
                _inputRightWingExposure = 1.0f;
            }
        }

        if (enabledFlaring)//张翼减速
            _inputFlaring = RTIsInputBoost && RTInputAccelerating < -boostInputThreshold;

    }

    void applyFlightInputs()
    {
        //HACK -- currently, drag is being fully calculated in flightPhysics.cs, so we don't want the
        //rigidbody adding any more drag. This should change, it's confusing to users when they look at
        //the rigidbody drag. 
        rig.drag = 0.0f;
        //precedence is as follows: flaring, diving, regular gliding flight. This applies if the
        //player provides multiple inputs. Some mechanics can be performed at the same time, such 
        //as flapping while flaring, or turning while diving. 

        //Flaring takes precedence over everything
        if (enabledFlaring && _inputFlaring)
        {
            if (enabledFlaring)
            {
                playSound(flareSoundSource, false);//避免破音，不random pitch
                anim.SetBool(ffhash.flaringBool, true);
                //Flare is the same as directional input, except with exagerated pitch and custom speed. 【Flare与普通飞行输入不同的是，使用了自定义的偏转角度及速度】
                flightPhysics.directionalInput(/*getBank()*/0, getPitch(true), flareSpeed);//PS：Flare时禁止更改Bank（Y轴），否则会出现异常转向
            }

            if (_inputFlap)
                RuntimeTool.ExecuteOnceInCurFrameAsync(flap);//避免FixedUpdate的执行时刻不同导致Animator的Trigger被触发2次
        }

        //#新增:加速度
        if (enabledAccelerate && !(RTIsInputLifting || _inputDiving || _inputFlaring))
        {
            flightPhysics.accelerateInput(_inputAccelerate, accelerateSensitivity);
        }

        //Test :向左右快速翻转

        if (freeFlightInput.IsBoostingDown && Mathf.Abs(freeFlightInput.HorizontalInput) > boostInputThreshold && IsCurAnimatorState(ffhash.glidingState))
        {
            RuntimeTool.ExecuteOnceInCurFrameAsync(BoostLeftRight);//避免FixedUpdate导致Animator的Trigger被触发2次
        }


        //Diving takes precedence under flaring
        if (enabledDiving && _inputDiving && !_inputFlaring)
        {
            dive();
        }
        else if (!_inputDiving && !flightPhysics.wingsOpen())
        {
            //Simulates coming out of a dive
            dive();
        }


        //Regular flight takes last precedence. Do regular flight if not flaring or diving.
        if (!((enabledDiving && _inputDiving) || (enabledFlaring && _inputFlaring)))
        {
            flightPhysics.directionalInput(getBank(), getPitch(false), directionalSensitivity);
            //Allow flapping during normal flight
            if (_inputFlapUp)
                resetFlap();
            else if (_inputFlap)
            {
                RuntimeTool.ExecuteOnceInCurFrameAsync(flap);//避免FixedUpdate的执行时刻不同导致Animator的Trigger被触发2次
            }
        }
        flightPhysics.doStandardPhysics();

        //#2 更新 Animation
        if (!_inputFlaring)
            anim.SetBool(ffhash.flaringBool, false);
        if (!_inputDiving)
            anim.SetBool(ffhash.divingBool, false);

        anim.SetFloat(ffhash.speedFloat, rig.velocity.magnitude);
        anim.SetFloat(ffhash.angularSpeedFloat, getBank());
    }

    void BoostLeftRight()//向左右瞬间加速
    {
        Quaternion desiredRotation = Quaternion.identity;
        desiredRotation.eulerAngles = new Vector3(0.0f, transform.rotation.eulerAngles.y, 0.0f);

        rig.AddForce(desiredRotation * Vector3.right * freeFlightInput.HorizontalInput * flapStrength * 5);
        //播放一段翻转动画，而不是更改物体转向
        anim.SetTrigger(freeFlightInput.HorizontalInput > 0 ? ffhash.divingRightTrigger : ffhash.divingLeftTrigger);
        //Debug.LogError($"[{Time.frameCount}] Boost Down " + Vector3.right * freeFlightInput.HorizontalInput * flapStrength);
    }

    /// <summary>
    /// Calculates pitch, based on user input and configured pitch parameters.
    /// </summary>
    /// <returns>The pitch in degrees.</returns>
    /// <param name="flare">If set to <c>true</c> calculates pitch of a flare angle.</param>
    protected float getPitch(bool flare)
    {
        if (flare)
            return _inputPitch * maxPitch - flareAngle;
        else
            return _inputPitch * maxPitch;
    }

    protected float getBank()
    {
        return _inputBank * maxTurnBank;
    }

    protected void flap()
    {
        ///ToUpdate:
        ///-区分按下及按住，在动画播放完成前抬起，那就是取消Trigger

        //Debug.LogError(Time.frameCount + " Flap");
        if (!enabledFlapping)
        {
            return;
        }
        if (!IsCurAnimatorState(ffhash.flappingState))//如果当前不是Flapping，则重新执行Flap相关代码
        {
            playSound(flapSoundSource);
            rig.AddForce(rig.rotation * Vector3.up * flapStrength);
            anim.SetTrigger(ffhash.flappingTrigger);
        }
    }
    protected void resetFlap()
    {
        //Debug.LogError(Time.frameCount + "resetFlap");
        anim.ResetTrigger(ffhash.flappingTrigger);
    }

    protected void dive()
    {
        if (enabledDiving)
        {
            playSound(divingSoundSource);
            anim.SetBool(ffhash.divingBool, true);
            flightPhysics.wingFold(_inputLeftWingExposure, _inputRightWingExposure);//合翼
        }
    }

    #endregion

    #region Functionality -- Audio

    bool hasWindStart = false;
    protected void applyWindNoise()
    {
        bool windStart = isFlying() && (flightPhysics.Speed > windNoiseStartSpeed);
        if (hasWindStart != windStart)
        {
            onWindStartStop.Invoke(windStart);
            hasWindStart = windStart;
        }

        if (!windNoiseSource) return;
        if (windStart)
        {
            float volume = Mathf.Clamp(flightPhysics.Speed / (windNoiseStartSpeed + windNoiseMaxSpeed), 0.0f, 1.0f);//根据速度持续计算
            windNoiseSource.volume = volume;
            //We want pitch to pick up at about half the volume
            windNoiseSource.pitch = Mathf.Clamp(0.9f + flightPhysics.Speed / 2.0f / (windNoiseStartSpeed + windNoiseMaxSpeed), 0.9f, 1.5f);
            //Use this to see how values are applied at various speeds.
            //Debug.Log (string.Format ("Vol {0}, pitch {1}", audio.volume, audio.pitch));
            if (!windNoiseSource.isPlaying)
                windNoiseSource.Play();
        }
        else
        {
            windNoiseSource.Stop();
        }
    }

    /// <summary>
    /// Sets up the audio component for the sound source. Does nothing if the source
    /// already exists and has a clip. 
    /// </summary>
    /// <returns>A reference to the new audio source </returns>
    /// <param name="source">Source.</param>
    /// <param name="sound">Sound.</param>
    protected AudioSource setupSound(AudioClip sound, ref AudioSource source)
    {

        if (!sound && source)
            Destroy(source);

        if (!sound && !source)
            return null;

        if (sound && !source)
        {
            source = gameObject.AddComponent<AudioSource>();
            source.loop = false;
        }

        if (!source.clip)
        {
            source.clip = sound;
        }

        return source;
    }

    protected void playSound(AudioSource source, bool changePitch = true, float? customPitch = null, float? customVolume = null)
    {
        if (source)
        {
            if (!source.isPlaying)
            {
                if (changePitch)
                    source.pitch = customPitch.HasValue ? customPitch.Value : UnityEngine.Random.Range(0.8f, 1.2f);
            }

            source.volume = customVolume.HasValue ? customVolume.Value : 1;

            source.Play();
        }
    }
    protected void stopSound(AudioSource source)
    {
        if (source && source.isPlaying)
        {
            source.Stop();
        }
    }
    #endregion

    #region Utility

    /// <summary>
    /// 检查当前动画状态
    /// </summary>
    /// <param name="state"></param>
    /// <returns></returns>
    bool IsCurAnimatorState(int state)
    {
        AnimatorStateInfo curstate = anim.GetCurrentAnimatorStateInfo(0);
        return curstate.fullPathHash == state;
    }
    #endregion
}

#region Define
public class AnimatorNotPresentException : UnityException
{
    public AnimatorNotPresentException()
    {
    }

    public AnimatorNotPresentException(string message)
        : base(message)
    {
    }

    public AnimatorNotPresentException(string message, Exception inner)
        : base(message, inner)
    {
    }
    #endregion
}
