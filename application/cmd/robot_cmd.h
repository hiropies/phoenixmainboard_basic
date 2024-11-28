#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H


/**
 * @brief RobotInit()によって呼び出されるコア・ロボット制御タスクの初期化。
 * 
 */
void RobotCMDInit();

/**
 * @brief ロボットのコア制御タスク、200Hzで実行（ビジョン送信周波数より高くなければならない）。
 * 
 */
void RobotCMDTask();

#endif // !ROBOT_CMD_H

// toDo記述が複数個所あるので実装し始める前に埋める必要あり
// chassis boardへの対応が完全でないので実装する。(そもそもシステム設計を再確認する必要あり)