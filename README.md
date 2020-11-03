# Antenna_Analyzer_By_Arduino
Arduino nano,AD9580,PCB,SourceCode


  //
  //By BI3QWQ,China
  //QQ ID 657584105,download qq from http://im.qq.com/,and reg user from http://zc.qq.com
  //2018-9-20 UTC +8
  //
  --------------How to use?--------------------

  [Mode]

  1、Config Mode（Cailibration、Freq Range、Reset、Exit）
  turn off system,and always hold down the button.then,turn on system,will have tip "Config Mode".
  at the config mode,u can cailibration,change freq range,reset system rom,and exit mode to Realtime mode.

  2、RealTime Mode
  u can change any freq,rotate button left or right to change freq.
  click button <500ms,u can change freq step,is  1.0、0.1、0.01、0.001.when u roate very fast,the setp will x10 speed.
  press button <2s,system will auto scaning freq,the step is by you set step,u can roate button very fast,scan freq will be x10 speed.roate slow will normal speed.when u click button or  overflow max freq,will stop scaning,and auto enter graphic mode,u can watch the scan result by line chart.
  press button >2s,will enter graphic,the center freq is u selected,the step is u set step .at the graphic mode,program will autoscanning total freq.click button can stop scaning.

  3、Band Mode
  when u rotate button and change freq on <1.000Mhz,will enter Band Mode,roate freq >1.000Mhz will enter  RealTime Mode.
  click button <500ms,u can change freq step,is 0.001 or 0.005
  press button >2s,will auto scanning system default band(3.8-3.86,5.32-5.38,7.0-7.08,14.150-14.35,21.37-21.43,29.57-29.63).
  when u click button or overflow total band,will stop scaning,and auto enter graphic mode,u can watch the scan result by line chart.

  4、Graphic Mode
  this mode have new function,it's autoscaning total freq all the time,when u click button one times,or press button <2s.
  realtime mode or band mode  finished word,will auto enter this mode.
  click button <500ms:stop autoscaning,change roate mode max SWR or move cursor.
  press button <2s:exit mode,and return last mode,for realtime mode or band mode.
  press button >2s:will aoto scan total freq all times.

  -----------------Question?------------------------
  if startup system have tip is "Need Calibration!" and RealTime mode have "SWR!",it's need calibration res.enter Config Mode to cailibration.it's will be ok.
