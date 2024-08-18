using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using Threeyes.Core;
/// <summary>
/// ToUpdate:
/// -将FreeFlight获取输入改为通过Action
/// </summary>
public class FreeFlightInput : InstanceBase<FreeFlightInput>
{
    public Action<bool> BirdYellStartStop;

    public PlayerInput playerInput;
    [SerializeField] InputActionReference inputActionRef_Move;
    [SerializeField] InputActionReference inputActionRef_Look;
    [SerializeField] InputActionReference inputActionRef_Pitch;//X轴旋转
    [SerializeField] InputActionReference inputActionRef_Jump;
    [SerializeField] InputActionReference inputActionRef_Yell;
    [SerializeField] InputActionReference inputActionRef_BulletTime;
    [SerializeField] InputActionReference inputActionRef_MainWeapon;
    [SerializeField] InputActionReference inputActionRef_Menu;//点击菜单
    //#Modifier
    [SerializeField] InputActionReference inputActionRef_Boost;//加速操作
    [SerializeField] InputActionReference inputActionRef_Lift;//升降操作（Gamepad的Modifier键）
    [SerializeField] InputActionReference inputActionRef_LockHeight;//锁定高度操作

    public InputAction inputAction_Move;
    public InputAction inputAction_Look;
    public InputAction inputAction_Pitch;
    public InputAction inputAction_Jump;
    public InputAction inputAction_Yell;
    public InputAction inputAction_BulletTime;
    public InputAction inputAction_MainWeapon;
    public InputAction inputAction_Menu;

    public InputAction inputAction_Boost;
    public InputAction inputAction_Lift;//组合键
    public InputAction inputAction_LockHeight;

    //Runtime
    public SchemeType curSchemeType = SchemeType.Keyboard_Mouse;

    public void SwitchSchemeMap(bool isUI)
    {
        playerInput.currentActionMap.Disable();
        playerInput.SwitchCurrentActionMap(isUI ? "UI" : "Player");
        playerInput.currentActionMap.Enable();
    }
    private void Awake()
    {
        inputAction_Move = EnableAction(inputActionRef_Move);
        inputAction_Look = EnableAction(inputActionRef_Look);
        inputAction_Pitch = EnableAction(inputActionRef_Pitch);
        inputAction_Jump = EnableAction(inputActionRef_Jump);
        inputAction_Yell = EnableAction(inputActionRef_Yell);
        inputAction_BulletTime = EnableAction(inputActionRef_BulletTime);
        inputAction_MainWeapon = EnableAction(inputActionRef_MainWeapon);
        inputAction_Menu = EnableAction(inputActionRef_Menu);

        inputAction_Boost = EnableAction(inputActionRef_Boost);
        inputAction_Lift = EnableAction(inputActionRef_Lift);
        inputAction_LockHeight = EnableAction(inputActionRef_LockHeight);


        playerInput.onControlsChanged += OnPlayerInputControlsChanged;
        inputAction_Yell.performed += OnYellPerformed;
        inputAction_Yell.canceled += OnYellCanceled;
    }

    private void OnDestroy()
    {
        playerInput.onControlsChanged -= OnPlayerInputControlsChanged;
        inputAction_Yell.performed -= OnYellPerformed;
        inputAction_Yell.canceled -= OnYellCanceled;

    }
    #region Callback
    void OnPlayerInputControlsChanged(PlayerInput input)
    {
        //Debug.LogError("OnPlayerInputControlsChanged: " + input.currentControlScheme);
        if (!Enum.TryParse(input.currentControlScheme, out curSchemeType))//更新curSchemeType
        {
            Debug.LogError("Unknown scheme: " + input.currentControlScheme);
        }
    }
    void OnYellPerformed(InputAction.CallbackContext obj)
    {
        BirdYellStartStop.Execute(true);
    }
    void OnYellCanceled(InputAction.CallbackContext obj)
    {
        BirdYellStartStop.Execute(false);
    }
    #endregion

    #region KeyState
    public float HorizontalInput { get { return GetAxis("Horizontal"); } }
    public float VerticalInput { get { return GetAxis("Vertical"); } }

    public bool IsJumping { get { return inputAction_Jump.IsPressed(); } }
    public bool IsJumpingUp { get { return inputAction_Jump.WasReleasedThisFrame(); } }
    public bool IsYellDown { get { return inputAction_Yell.WasPressedThisFrame(); } }
    public bool IsYellUp { get { return inputAction_Yell.WasReleasedThisFrame(); } }
    public bool IsBulletTimeDown { get { return inputAction_BulletTime.WasPressedThisFrame(); } }
    public bool IsBulletTimeUp { get { return inputAction_BulletTime.WasReleasedThisFrame(); } }

    public bool IsBoostingDown { get { return inputAction_Boost.WasPressedThisFrame(); } }
    public bool IsBoosting { get { return inputAction_Boost.IsPressed(); } }
    public bool IsLifting { get { return inputAction_Lift.IsPressed(); } }
    public bool IsLockHeightDown { get { return inputAction_LockHeight.WasPressedThisFrame(); } }

    public bool IsMenuDown
    {
        get
        {
            bool isDown = inputAction_Menu.WasPressedThisFrame();
            if (!Application.isEditor)//编辑模式忽略Esc，避免无法退出鼠标锁定模式
                isDown |= Input.GetKeyDown(KeyCode.Escape);
            return isDown;
        }
    }

    public float PitchInput
    {
        get
        {
            switch (curSchemeType)
            {
                case SchemeType.Gamepad:
                    //Gamepad因为键位不足，需要与组合键使用(后期考虑单独的组件，或者与前后键交换)
                    if (inputAction_Lift.IsPressed())
                    {
                        return inputAction_Move.ReadValue<Vector2>().y;//直接使用前后的值
                    }
                    return 0;
                default:
                    return inputAction_Pitch.ReadValue<float>();
            }
        }
    }

    public virtual float GetAxis(string axisName)
    {
        float result = 0;
        //float mosueSensitivity = 0.05f;//参考旧版InputManager，需要针对像素值进行缩放(Ref:https://forum.unity.com/threads/mouse-jittering-with-new-inputsystem.1173761/#post-7526288)

        if (axisName == "Horizontal")
            result = inputAction_Move.ReadValue<Vector2>().x;
        else if (axisName == "Vertical")
            result = inputAction_Move.ReadValue<Vector2>().y;
        else if (axisName == "Mouse X")
            result = inputAction_Look.ReadValue<Vector2>().x /** mosueSensitivity*/;
        else if (axisName == "Mouse Y")
            result = inputAction_Look.ReadValue<Vector2>().y /** mosueSensitivity*/;
        else
        {
            Debug.LogError($"axisName [{axisName}] Not Define!");
        }
        return result;
    }
    #endregion

    #region Utility
    static InputAction EnableAction(InputActionReference actionReference)
    {
        if (!actionReference) return null;
        InputAction action = actionReference.action;
        if (action != null && !action.enabled)
            action.Enable();
        return action;
    }
    #endregion
    #region Define
    public enum SchemeType
    {
        Keyboard_Mouse,
        Gamepad,
    }
    #endregion
}
