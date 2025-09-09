//##############################################################################
// ���j�o�[�T���L�b�g(Universal Kit) STM-Version
// Beep�w�b�_ ver.1.0  (2023.7.24-)
//##############################################################################
#include "math.h"			// pow�֐��g�p�̂���

// Sound Pitch : ���K�ݒ�
#define TONE_DO		262		// ��
#define TONE_DS		277		// ��#
#define TONE_RE		294		// �
#define TONE_RS		311		// �#
#define TONE_MI		330		// �
#define TONE_FA		349		// ̧
#define TONE_FS		370		// ̧#
#define TONE_SO		392		// ��@���S�p�̃\��shift-JIS�̃_�������Ȃ̂ŃR�����g�ł̎g�p����
#define TONE_SS		415		// �#
#define TONE_LA		440		// �
#define TONE_LS		466		// �#
#define TONE_TI		494		// �

//===============================================
// Beep�֐�
//		freq 	= ��{���g���i�h���~�j�F�s�b�`4�̎��g��[Hz]
// 		pitch 	= �����@4����{���K�C5���g���₷��
// 		len 	= ���̒���[ms]
//===============================================
void Beep(int freq, int pitch, int len)
{
  if(freq == 0)
    HAL_Delay(len);											// 0���͂͋x��
  else{
    int beep_freq = freq * pow( 2, pitch - 4 );     		// ���g���v�Z
    int beep_period = 64000 / beep_freq - 1;				// prescaler�ɐݒ肷������v�Z
    // ���� = APB2(64MHz) / CounterPeriod(1000) / prescaler(beep_freq) - 1
    __HAL_TIM_SET_PRESCALER(&htim16, beep_period);			// prescaler��ݒ�
    __HAL_TIM_SET_COMPARE( &htim16, TIM_CHANNEL_1, 500-1);	// duty��ݒ�
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);				// TIM16 �^�C�}�X�^�[�g
    HAL_Delay(len);
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);				// TIM16 �^�C�}�į��
  }
}
