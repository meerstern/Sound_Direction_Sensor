#  音方位センサ基板
# Sound_Direction_Sensor
This board detects the direction of sound with 4-pdm mems mic using GCC-PHAT algorithm.

<img src="https://github.com/meerstern/Sound_Direction_Sensor/blob/main/IMG/sds5.gif" width="360">

## 概要
  * 音の時間差から計算して8つのLED及びI2C、UARTで結果(角度)を出力する基板です  
  * 4つのPDMデジタルマイクSPH0641LMおよびSTM32F411RCを搭載しています  
  * GCC-PHAT(Generalized Cross-Correlation. PHAse Transform)方式です  
  * 数kHz前後の音をフィルタして音の時間差から方位を検出します  
  * 誤検知を低減するために、音変化（標準偏差）が大きい場合に角度出力します  
  * 音変化（標準偏差）の閾値は基板上の可変抵抗もしくはコマンドから変更できます  
  * 人の声やリズム音等の方位検出に向いています  
  * 手やモノを叩く音等の瞬間的な音の場合、反射等の影響が大きく誤検知しやすくなります  
  
## 仕様
  * 角度分解能は約22.5度～30度程度(音源や環境、処理に依存)
  * 方位LED 8つ搭載
  * 1秒間に約5~6回測定し、結果を出力します  
  * 36kHzサンプリング
  * 電源3.3~5V、UARTもしくはI2C(5V耐圧)
  * I2CはGrove互換コネクタ搭載、UARTは基板上ピンヘッダから接続  
  * UARTボーレート9600bps、データサイズ8bit、パリティ無、ストップ1bitが既定値です  
  * 起動直後にのみ基板上の可変抵抗を読み取り、音変化（標準偏差）の閾値として設定します  
  * I2CもしくはUARTから音変化（標準偏差）閾値を上書き設定できます  
  * リセットや電源再投入で音変化（標準偏差）閾値は消去され、保持されません(可変抵抗値が再度反映)  
  * 電源投入後、自動的に方位検知を開始するため、電源供給のみで方位検知可能です  
  
<img src="https://github.com/meerstern/Sound_Direction_Sensor/blob/main/IMG/sds3.jpg" width="360">
<img src="https://github.com/meerstern/Sound_Direction_Sensor/blob/main/IMG/sds4.jpg" width="360">
  
## 外形
  * 40mm円形基板(コネクタ含まず)
  * マイク距離36mm円形
  * M2穴x3 穴間隔横および縦 24.3mm
  
## アプリケーション例
  * 音源の方向を向く小型の首振りロボット  
  * 声でカメラの向きを動かして撮影するカメラステージ
   
## 注意点
  * 厳密な角度検出が必要な用途には向いていません  
  * 音の反射やサンプリングタイミングによって誤検知する場合があります  
  * 騒音環境下、衝撃/振動環境下の用途には向いていません  
  * 5cm程度～1m程度の音の方位の検出を想定しています  
  * 数m以上の遠方からの音の場合、反射等の影響で誤検知の可能性が高くなります  
  * 頻繁に方位LEDが点灯する場合や誤検知が多発する場合は音変化閾値を上げてください  
  * 場合によって起動直後数秒間はソフトフィルタ処理の影響で方位検出が安定しない場合があります  
  * Groveケーブルは同梱されていません 別途ご用意ください   

## 閾値調整目安
  * 音源がない環境下での音変化値Actに対して閾値を1.2~1.5倍程度の値に設定してください  
  * 誤検知が多い場合は環境に応じて閾値を音変化値Actに対してより高く設定してください  
  　例　静かな状態で音変化0.8前後の場合、音変化閾値を1.2程度に設定  
   
## UART通信 
  * 起動直後はファームのバージョンおよびビルド日時、音変化の閾値が出力されます  
  * 1秒間に約5~6回、検出角度Deg、フィルタ前角度DegOrg、音変化値Actが出力されます  
  * 音変化閾値以下は未検出として-1度を出力します  
  * act=XX.XX(Enterキー)で音変換閾値を変更できます  
  * act?(Enterキー)で現状の音変換閾値を確認できます  
  * rst!(Enterキー)でソフトウェアリセットさせることができます 
  * 下記の例では音変換閾値1.0のため、1.0を超えて音変換値1.6の4つ目が206.6度として検出  
    
	Sound Direction Sensor vX.X (ビルド日時)[\r\n]  
	Activity Threshold: 1.0[\r\n]  
	Deg:-1.0,        DegOrg:201.8,   Act:0.69,[\r\n]  
	Deg:-1.0,        DegOrg:0.0,     Act:0.71,[\r\n]  
	Deg:-1.0,        DegOrg:90.0,    Act:0.73,[\r\n]  
	Deg:206.6,       DegOrg:206.6,   Act:1.60,[\r\n]  
	Deg:-1.0,        DegOrg:90.0,    Act:0.73,[\r\n]  

| コマンド名 | コマンド | 引数 | 例 | 
|:-----------|:------------|:------------|:------------|
| 音変換閾値設定 | act= | 小数設定値 | act=1.2(Enterキー) |
| 音変換閾値確認 | act? | なし | act?(Enterキー) |
| リセット |	rst! | なし | rst!(Enterキー) | 



## I2C通信
　 * I2Cデバイスアドレスは0x60(8bitアドレス)、0x30(7bitアドレス)です  
  * 角度および音変化値は10倍の整数値がレジスタに格納されています  
  * 必要に応じてマスタ側で0.1を乗算し、角度等に変換してください  
  * 書き込み時についても同様に設定したい閾値の10倍の整数値を書き込んでください  
    
  * 読み込み
  
| レジスタ名 | オフセット | 詳細 | 
|:-----------|:------------|:------------|
| 角度H(フィルタ後) |0x00 | 角度HighByte | 
| 角度L(フィルタ後) |0x01 | 角度LowByte | 
| 角度H(フィルタ前) |0x02 | 角度HighByte | 
| 角度L(フィルタ前) |0x03 | 角度LowByte | 
| 音変化値H |0x04 | 観測値HighByte | 
| 音変換値L |0x05 | 観測値LowByte | 
| 更新フラグ |0x06 | 値更新0x01 | 
| デバイスID |0x07 | 0x43固定値 | 

  
  * 書き込み  
  
| レジスタ名 | オフセット | 引数 | 
|:-----------|:------------|:------------|
| 音変換閾値H |0x04 | 設定値HighByte | 
| 音変換閾値L |0x05 | 設定値LowByte | 

※オフセット0x04で設定値のHighByteとLowByteの2バイトを一度で書き込んでください  
(スレーブアドレス)+(オフセット)+(設定値2Byte)  

<img src="https://github.com/meerstern/Sound_Direction_Sensor/blob/main/IMG/sds1.jpg" width="360">

<img src="https://github.com/meerstern/Sound_Direction_Sensor/blob/main/IMG/sds2.jpg" width="360">





https://youtu.be/5QTcDwFN5Rs

https://youtu.be/rlhwlwNsYnQ



