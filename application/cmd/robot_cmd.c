// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// エンコーダを角度値に自動変換するプライベートマクロ(中身要確認 2024.10.15 abe)
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // アライメント時の角度、0～360(多分chassisのyaw角 2024.10.15 abe)
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // ピッチが水平の時のモーター角度、0-360

/* cmdアプリケーションはモジュールインスタンスへのポインタを含み、相互作用情報を格納する */。
#ifdef GIMBAL_BOARD // デュアルボード互換性、条件付きコンパイル(多分OneBoardは不可能なのでちゃんと宣言しようね 2024.10.15 abe)
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // シャーシ制御メッセージパブリッシャー
static Subscriber_t *chassis_feed_sub; // シャーシ・フィードバック・メッセージ・サブスクライバー
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 制御メッセージや UI 描画関連など、シャーシアプリケーションに送信されるメッセージ。
static Chassis_Upload_Data_s chassis_fetch_data; // シャシー・アプリケーションから受信したフィードバック・メッセージ（シャシー・パワー、銃口熱、シャシー・モーション・ステータスなど）。

static RC_ctrl_t *rc_data;              // 初期化中に返されるリモートコントロールデータ
static Vision_Recv_s *vision_recv_data; // ビジョン・レシーバー・データ・ポインター、初期化時に返される
static Vision_Send_s vision_send_data;  // ビジョン送信データ

static Publisher_t *gimbal_cmd_pub;            // ジンバル制御メッセージ送信者
static Subscriber_t *gimbal_feed_sub;          // ジンバルフィードバックメッセージ購読者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // ジンバルに送信される制御メッセージ
static Gimbal_Upload_Data_s gimbal_fetch_data; // ジンバルからのフィードバック

static Publisher_t *shoot_cmd_pub;           // shoot制御メッセージパブリッシャー
static Subscriber_t *shoot_feed_sub;         // shootフィードバックメッセージの購読者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // shootに配信される制御メッセージ
static Shoot_Upload_Data_s shoot_fetch_data; // shootからのフィードバック

static Robot_Status_e robot_state; // ロボット全体のステータス

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);   // 対応するシリアルポートに変更する。R&Dボードのdbusプロトコルのシリアルポートであれば、インバータを追加したものを使用する必要がある。
    vision_recv_data = VisionInit(&huart1); // ビジョン通信シリアルポート

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // Oneボード対応
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_READY; // ロボットは起動時に作業モードに入り、すべてのアプリケーションが初期化された後に再度作業モードに入ります。
}

/**
 * @brief ジンバルアプリから現在のモータ角度に応じてゼロ位置の誤差を計算する。
 *        1回転の絶対角度の範囲は0~360で、ドキュメントに図示されている。
 *
 */
static void CalcOffsetAngle()
{
    // 読みやすさを向上させるためにエイリアスの角度を追加、それ以外は長すぎて良くない。
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // ジンバルから取得した現在のヨーモーターの一回転角度。
#if YAW_ECD_GREATER_THAN_4096                               // 180度より大きい場合
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 180度未満の場合
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/**
 * @brief 制御入力は、リモートコントローラのモードと制御ボリュームの設定です（試運転時）。
 *
 */
static void RemoteControlSet()
{
    // 制御シャーシとジンバルの動作モード選択、ジンバルが常にIMUデータを使用するか選択できる。
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右スイッチ状態[下]、シャーシはヘッドに従う
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右スイッチ状態[中]、シャーシとジンバルが離れ、シャーシは回転しない。
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    }

    // ジンバルパラメーター、ジンバル制御データを決定します。
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) //  左スイッチ状態[中央]、ビジョンモード
    {
        //　@todo 追加で、視覚はターゲットとの誤差を送信し、また制御のための総角度増分に変換します。
    }
    // 左スイッチの状態が[下]、またはビジョンがターゲットを認識しない、純粋にリモートコントロール制御。
    if (switch_is_down(rc_data[TEMP].rc.switch_left) || vision_recv_data->target_state == NO_TARGET)
    { // ジョイスティックの出力の大きさに応じて角度を増やし、ゲイン係数を調整する必要があります。
        gimbal_cmd_send.yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_;    //左　水平
        gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;  //左　水平
    }
    // @todo ジンバルソフトウェア制限

    // シャーシパラメータ。無限回転が考慮(実装)されていないため,係数を調整する必要がある。
    chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_r_; // 右 水平方向
    chassis_cmd_send.vy = 10.0f * (float)rc_data[TEMP].rc.rocker_r1; // 右 数值方向

    // shootパラメータ
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右側スイッチの状態を[up]にするとマガジンが開く。
        ;                                            // @todo マガジン開指令を実装する
    else
        ; // @todo マガジン閉指令を実装する

    // フリクションホイール制御。リモコンのダイヤルは上向きがマイナス、下向きはプラスです。
    if (rc_data[TEMP].rc.dial < -100) // 100以上アップ、オープンフリクションホイール
        shoot_cmd_send.friction_mode = FRICTION_ON;
    else
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    // フリクションホイール制御。モードを変更することができる。ここでは連射モードか、撃たないかのどちらか。3点バーストと単発撃ちが用意されている。
    if (rc_data[TEMP].rc.dial < -500)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else
        shoot_cmd_send.load_mode = LOAD_STOP;
    // 発射レートの設定。現在は毎秒1発になっている。(8個のloaderで秒間1/8回転するよう設定されている。)
    shoot_cmd_send.shoot_rate = 8;
}

/**
 * @brief キーボードとマウスモードの入力とボリューム設定を制御する
 *
 */
static void MouseKeySet()
{
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300; // 値の取得。係数は調整可能。
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].s * 300 - rc_data[TEMP].key[KEY_PRESS].d * 300;

    gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 値の取得。係数は調整可能。
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) //　Zキーで弾速を設定
    {
    case 0:
        shoot_cmd_send.bullet_speed = 15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = 18;
        break;
    default:
        shoot_cmd_send.bullet_speed = 30;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4) // Eキーで発射モードを設定
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    case 1:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    case 2:
        shoot_cmd_send.load_mode = LOAD_3_BULLET;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // Rキーで弾倉の開閉
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // Fキーでフリクションホイールの切り替え
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // Cキーでシャーシ速度を設定
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // @todo 保留中の追加 シフトを押してオーバーパワーを可能にする バッファエネルギーの消費(virtualCapのことだと思う 2024.10.15 abe)
    {
    case 1:

        break;

    default:

        break;
    }
}

/**
 * @brief  緊急停止、リモコンのダイヤルを下方向に入力、重要モジュールがオフライン、デュアルボード通信障害など
 *         重要モジュールに該当するものを調査する(2024.10.15 abe)
 * @todo   停止しきい値'300'を適切な値に変更するか、スイッチで制御。
 *
 */
static void EmergencyHandler()
{
    // ダイヤルを半分以上下げて緊急停止モードに入る。 ダイヤルを下側に回すとプラスになる。
    if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 重要なアプリケーションやモジュールがオフラインの判断を追加する必要がある。(書き方的に未実装? 2024.10.15 abe)
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // リモコン右側のスイッチが[上]になり、通常運転に戻ります。(マガジンの開閉と被ってて困る。なんかいい感じに実装する必要 2024.10.15 abe)
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* ロボットコア制御タスク、200Hzの周波数動作（ビジョン送信周波数より高い必要がある）。*/
void RobotCMDTask()
{
    // 他のアプリケーションからデータを取得する
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // ジンバルのフィードバックから、正方向のジンバルとシャーシの間の角度を計算する。パラメータは渡されず、これは静的なプライベート変数で行われる。
    CalcOffsetAngle();
    // リモコンの左側にあるスイッチにより、現在の制御モードがリモコンデバッグかキーボードとマウスかを判断する。
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // リモコン左側のスイッチの状態[下]の場合、リモートコントロール。
        RemoteControlSet();
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // リモコン左側のスイッチの状態が[上]の場合、キーボード制御。
        MouseKeySet();

    EmergencyHandler(); // モジュールオフラインやリモコン緊急停止などの緊急時の対応。

    // 視覚伝達データの設定、加速度・角速度データの追加。(visiondataの転送については未実装か？実装の有無の確認が必要 2024.10.15 abe)
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // プッシュメッセージ、デュアルボード通信、ビジュアル通信など
    // その他のアプリケーションに必要な制御データは、remotecontrolsetmode、mousekeysetmodeで設定します。
    // 基板間のデータのやり取り、メッセージのプッシュについて各基板での分担を確定して実装する。(toDo: 2024/10/25 abe)
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    VisionSend(&vision_send_data);
}
