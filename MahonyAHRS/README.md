## 算法基础
Mahony算法即所谓的互补滤波算法，通过PID反馈控制器把误差量反馈补偿修正陀螺仪的误差。

## 算法原理
根据加速度计和地磁计的数据，转换到地理坐标系后，与对应参考的重力向量和地磁向量进行求叉乘误差，这个误差用来校正陀螺仪的输出，然后用陀螺仪数据进行四元数更新，再转换到欧拉角。通俗一点的解释就是使用加速度计输出来修正陀螺仪的横滚角和俯仰角误差，使用磁力计输出来修正陀螺仪的航向角误差，从而给出一个更加平稳，发散速度更慢的姿态融合结果。


## 算法难点

我感觉难点在于怎么设置kp，ki这两个值，可能需要比较多的PID调节的经验和测试数据。

在参考【1】中提到：“关于这一块，现在研究的比较多就是如何实现自适应调参。固定的参数不能获得所有情况下的最优运动姿态角，可以设计参数可调的自适应算法在不同运动状态下进行调节参数的大小。其参数调节规则为：正常运动状态情况下，Kp和Ki值取为系统初始化值；当运动体具有较大运动加速度或姿态变化剧烈时，应选择较小的Kp值（可取其初始化值的0.1倍），而Ki值应在同一数量级内适当取大一点。具体取值需根据实际应用系统选取。”

严恭敏老师的博客最简单的航姿仪算法C程序（AHRS）中设置的kp,ki值为：
```diff
void MahonyInit(float tau)
{
    float beta = 2.146f/tau;
    Kp = 2.0f*beta, Ki = beta*beta;
    q0 = 1.0f, q1 = q2 = q3 = 0.0f;
    qua2cnb();
    exInt = eyInt = ezInt = 0.0f;
    tk = 0.0f;
}
```
Aceinna开源代码中设置的kp,ki值为：
```diff
 def __init__(self):
        '''
        vars
        '''
        # algorithm description
        self.input = ['fs', 'gyro', 'accel']#, 'mag']
        self.output = ['att_quat', 'wb', 'ab']
        self.batch = True
        self.results = None
        self.quat = None
        self.wb = None
        self.ab = None
        # algorithm vars
        # config
        self.innovationLimit = 0.1
        self.kp_acc_high = 1
        self.kp_acc_low = 0.01
        self.ki_acc_high = 0.5
        self.ki_acc_low = 0.001
        # state
        self.ini = 0                                # indicate if attitude is initialized
        self.dt = 1.0                               # sample period, sec
        self.q = np.array([1.0, 0.0, 0.0, 0.0])     # quaternion
        self.err_int = np.array([0.0, 0.0, 0.0])    # integral of error
        self.kp_acc = 1
        self.ki_acc = 0.001
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.tmp = np.array([0.0, 0.0, 0.0])

```