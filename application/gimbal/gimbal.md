# gimbal

## ワークフロー

ピッチとヨーのモーターとIMUを初期化する。robot_cmdからgimbal_cmdメッセージを受信し、gimbal_feedトピックを投稿する。

1. メッセージセンターからgimbal_cmdトピックのメッセージを取得する。
2. メッセージの制御モードに従ってモード切り替えを行い、緊急停止した場合はすべてのモーターをオフにする。
3. 設定されたモードから、モーターフィードバックデータのソースを切り替え、フィードバックデータポインターを変更し、フィードフォワード制御データポインターを設定する。
4. ヨーモーターの絶対角とimuデータを含むフィードバックデータを設定する。
5. フィードバックデータをgimbal_feedトピックにプッシュする。