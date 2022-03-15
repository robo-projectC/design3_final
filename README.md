# Why did I fork?
設計製作論3のロボットアームの授業で製作したプログラムです．シミュレータをインストールしなければ使えませんが，  
OpenCV等のプログラムは知能ロボコンの機体制御に活用できると思うのでforkします(内容は脳筋なので期待は厳禁です)．  
シミュレータ等の環境構築や基本的な使い方は[こちら](https://ryuichiueda.github.io/manipulator_practice_b3/lesson1.html#/)を参照して下さい．

# design3_final
crane_x7を音声認識によって操作するパッケージです。

・unite.py   
持ってきてほしい物体の色を有線イヤホンについているマイクに話しかけると、その色のついた物体をcrane_x7が持ってきてくれます。
以下はデモ動画のリンクです。

https://youtu.be/_FOWcYkRxac

・voice.py  
音声認識のみでcrane_x7を操作することを目的としたもので、動作はunite.pyと同じです。

用意するもの  
・crane_x7   
・realsense D435i  
・マイク付き有線イヤホン  
・iPhone以外のchromeブラウザ  
・赤、青、黄のいずれかの色がはっきりしている10cm以内の大きさの物体  

使用するパッケージ  
・crane_x7_ros  
・realsense-ros  
・visualization_rwt  

実行環境  
・用意した物体をcrane_x7が設置されている机上の周囲30cm以内に重心が来るように設置して下さい  
・用意した物体の色と同じ系統のものをcrane_x7の周囲に置かないように注意してください  

実行方法  
・unite.pyの場合  

1. 端末を3つ開き、以下のコマンドを実行して下さい  
1つ目の端末  
roslaunch realsense2_camera rs_camera.launch  
2つ目の端末  
roslaunch rwt_speech_recognition rwt_speech_recognition.launch  
3つ目の端末  
sudo chmod 777 /dev/ttyUSB0  
roslaunch crane_x7_bringup demo.launch fake_execution:=false  

2. chromeブラウザを開き以下のurlにアクセスしmodeの欄からcontinousを選択し、startを押して下さい  
http://localhost:8000/rwt_speech_recognition/  

3. 4つ目の端末を開きunite.pyを実行します  
rosrun design3_final unite.py  

・voice.pyの場合  
unite.pyの1番目の手順を省略して実行して下さい  
rosrun design3_final voice.py  

操作方法  
・unite.pyの場合  
1. 指定したい色(赤色、黄色、青のいずれか)を音声認識できるまでイヤホンのマイクに話しかけて下さい 認識すると端末上に「red???,blue???,yellow???」のいずれかの音声認識した文字列が表示されます  

2. crane_x7に装着しているカメラレンズの50cm以内に物体があるとcrane_x7がその場で静止し、物体を置きます  

・voice.pyの場合  
crane_x7のアームが取ってほしい物体の真上に来た時、「つかめ」というと物体持ち上げる動作を行います 物体を置いてほしい位置にアームが移動したら、「止まれ」というとアームが持ち上げている物体を置き、停止します  

暴走プロセスの停止方法  
ctrl-z,ctrl-dとしてもアームが止まらない場合はps xを打ちpythonで実行しているPIDを確認してkill -9 PIDを入力してください
