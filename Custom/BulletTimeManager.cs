using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using DG.Tweening;
using Threeyes.Core;
/// <summary>
/// Todo:开关子弹时间
/// </summary>
public class BulletTimeManager : InstanceBase<BulletTimeManager>
{
    public System.Action<bool> BulletTimeModeActived;
    Tween tween;
    public void EnterBulletTime(float duration = 0.5f, float targetTimeScale = 0.1f)
    {
        TryKillTween();
        tween = DOTween.To(() => Time.timeScale, SetTimeScale, targetTimeScale, duration);
        BulletTimeModeActived.Execute(true);
    }
    public void ExitBulletTime(float duration = 0.5f, float targetTimeScale = 1)
    {
        TryKillTween();
        tween = DOTween.To(() => Time.timeScale, SetTimeScale, targetTimeScale, duration);
        BulletTimeModeActived.Execute(false);
    }

    void SetTimeScale(float value)
    {
        Time.timeScale = value;
        Time.fixedDeltaTime = value * 0.02f;//确保物理效果不受影响而卡顿（https://www.youtube.com/watch?v=0VGosgaoTsw）
    }

    void TryKillTween()
    {
        if (tween == null) return;
        tween.Kill(false);//方便基于当前的时间设置
        tween = null;
    }
}
