#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"

#include "bmi088.h"

static attitude_t *gimba_IMU_data; // ジンバルIMUデータ
static DJIMotorInstance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub;                   // ジンバルアプリケーションメッセージパブリッシャ（cmdへのジンバルフィードバック）
static Subscriber_t *gimbal_sub;                  // cmdコントロールメッセージサブスクライバ
static Gimbal_Upload_Data_s gimbal_feedback_data; // ジンバル・ステータス・メッセージのcmdへの返信
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // cmdからの制御メッセージ

void GimbalInit()
{
    gimba_IMU_data = INS_Init(); // まずIMUが初期化され、姿勢データへのポインタがヨーモーターの他のデータソースに割り当てられる。
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8, // 8
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 200, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // (toDo?)また、角速度用のフィードバックポインターを追加する必要がある。ins_task.mdには、Cボードのボディフレーム座標系についての記述がある。 
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 350, // 350
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // (toDo?)また、角速度用のフィードバックポインターを追加する必要がある。ins_task.mdには、Cボードのボディフレーム座標系についての記述がある。 
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    // モーターはtotal_angleに対してクローズドループになっており、電源投入時はゼロで、静止し、リモコンからデータを受け取ると再び動く。
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* ロボットのジンバル制御のコアタスク、その後の検討ではIMU制御のみを残し、モーターフィードバックは不要になる */
void GimbalTask()
{
    // ジンバル制御データの取得
    // @todo:非受信データの処理を追加するためのフォローアップ
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // 現在、IMUからの姿勢データは常にジンバルのフィードバックとして使用でき、ヨーモーターのオフセットはシャーシに追従するために使用されるだけだ。
    // 制御モードによるモーターフィードバックの切り替えと移行。ビジョンモードはすでにrobot_cmdモジュールで設定されており、ジンバルはyaw_refとpitch_refだけを見る。
    // @todo:この挙動について読み込み、注意が必要(2024.10.04 abe)
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        break;
    // ジャイロフィードバックを使用し、ヨーモーターのオフセットに応じて、シャーシはジンバルまたはビジョンモードに追従する。
    case GIMBAL_GYRO_MODE: // @todo:その後、このモデルのみが保持される。
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // ヨーとピッチはrobot_cmdで処理されます。
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    // ジンバルフリーモード、エンコーダーフィードバックを使用、シャーシとジンバルの分離、ジンバルの回転のみ、一般的にジンバルの姿勢を調整するために使用（ヒーロースリングショットなど）/エネルギー器官
    case GIMBAL_FREE_MODE: // @todo:後で削除するか、ジンバルが地面を追いかけるフォローモードを追加する（反応が速くなる）。
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // ヨーとピッチはrobot_cmdで処理されます。
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }

    // @todo:フィードフォワードのモーメントを補正するために、必要に応じてピッチ・グラビティを加える。(重力補償 2024/10/14 abe) 
    // @todo:IMUの姿勢/ピッチモーター角のフィードバックに基づき、現在のカウンターウェイトの下でのウェイトモーメントを計算します.(Dynamics-Conpensationでワロタ 2024/10/14 abe) 
    // ...

    // フィードバックデータ、主にimuとヨーのecdを設定する。 
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // プッシュメッセージ
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}