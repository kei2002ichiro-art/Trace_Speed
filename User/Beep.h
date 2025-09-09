//##############################################################################
// ユニバーサルキット(Universal Kit) STM-Version
// Beepヘッダ ver.1.0  (2023.7.24-)
//##############################################################################
#include "math.h"			// pow関数使用のため

// Sound Pitch : 音階設定
#define TONE_DO		262		// ﾄﾞ
#define TONE_DS		277		// ﾄﾞ#
#define TONE_RE		294		// ﾚ
#define TONE_RS		311		// ﾚ#
#define TONE_MI		330		// ﾐ
#define TONE_FA		349		// ﾌｧ
#define TONE_FS		370		// ﾌｧ#
#define TONE_SO		392		// ｿ　※全角のソはshift-JISのダメ文字なのでコメントでの使用注意
#define TONE_SS		415		// ｿ#
#define TONE_LA		440		// ﾗ
#define TONE_LS		466		// ﾗ#
#define TONE_TI		494		// ｼ

//===============================================
// Beep関数
//		freq 	= 基本周波数（ドレミ）：ピッチ4の周波数[Hz]
// 		pitch 	= 高音　4が基本音階，5が使いやすい
// 		len 	= 音の長さ[ms]
//===============================================
void Beep(int freq, int pitch, int len)
{
  if(freq == 0)
    HAL_Delay(len);											// 0入力は休符
  else{
    int beep_freq = freq * pow( 2, pitch - 4 );     		// 周波数計算
    int beep_period = 64000 / beep_freq - 1;				// prescalerに設定する周期計算
    // 周期 = APB2(64MHz) / CounterPeriod(1000) / prescaler(beep_freq) - 1
    __HAL_TIM_SET_PRESCALER(&htim16, beep_period);			// prescalerを設定
    __HAL_TIM_SET_COMPARE( &htim16, TIM_CHANNEL_1, 500-1);	// dutyを設定
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);				// TIM16 タイマスタート
    HAL_Delay(len);
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);				// TIM16 タイマｽﾄｯﾌﾟ
  }
}
