SEED-Noidの移動台車(MecanumWheel)とリフターを「OpenRTM-aist」で用いるためのコンポーネントです．

1.開発環境
 PC:Panasonic Let's Note(CF-MX3)
 OS:Ubuntu14.04
 RTミドルウェア:OpenRTM-aist-1.1.2-RELEASE
 言語:c++
 
2.動作確認OS

 Ubuntu14.04,Ubuntu16.04

3.利用方法
 libboostを用いてシリアル通信のプログラミングを行なっているので，libboostをインストールしてください．
 
 *以下にUbuntuにlibboostをインストールするためにコマンドを示す．
 sudo apt-get install libboost-dev
 もしくは、
 sudo apt-get install libboost-all-dev
 
 OpenRTM-aistのRTコンポーネントを開発するための環境を整備し，cmake及びbuildを行ってください．
 OpenRTM-aistの動作環境構築に関しては，以下のHPにも記載していますので，そちらを参照してください．
 http://www2.meijo-u.ac.jp/~kohara/cms/technicalreport/mobilerobotnavigationframework-table-of-contents/openrtm-operation-environmental-construction
 
 *以下にbuildのための手順を示す．
 mkdir build
 cd build
 cmake ..
 make
 
4.コンポーネントの構成
 *入力ポート
 データ型:TimedVelocity2D ポート名:targetVelocity
 目標速度(Vx,Vy,ω)を入力する
 
 *出力ポート
 データ型:TimedPose2D ポート名:currentPose
 オドメトリによるロボットの現在の推定自己位置を出力する
 
 *サービスポート
 Interface:LifterPoseInterface ポート名:lifter
 SEED-Noidのリフターの動作制御を行うことができる．
 
 コンポーネントの詳細については別途資料を上げますので，少々お待ちください．
 
 ご不明な点がございましたら、以下にご連絡ください
 
##########################################################
名城大学　ロボットシステムデザイン研究室
Email : murase@rsdlab.jp
########################################################## 
