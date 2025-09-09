/* USER CODE BEGIN Header */
//##############################################################################
// ユニバーサルキット(Universal Kit) STM-Version
// サンプルプログラム ver.1.0 (2023.7.24-)
//
// sample12 Trace_Speed(モータ出力２：台形加速)※トレーサ用
//##############################################################################
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"	// Cの標準ライブラリ
#include "LCD_c.h"	// LCD用
#include "Beep.h"	// Beep用

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//******************************************************************************
// マクロ定義
//******************************************************************************
// スイッチ関連
#define LED_ON		GPIO_PIN_SET		// LED(LD3)は正論理ActiveHighなので1でON
#define LED_OFF		GPIO_PIN_RESET		// LED(LD3)は正論理ActiveHighなので0でOFF
#define SW_ON		GPIO_PIN_RESET		// Switchは負論理ActiveLowなので0でON
#define SW_OFF		GPIO_PIN_SET		// Switchは負論理ActiveLowなので1でOFF
#define SW_WAIT     300					// チャタリング防止用の待ち時間(ms)
// モード関連
#define MODE_MAX	9					// 動作モード数
#define DISP		0					// モード表示
#define EXEC		1					// モード実行
// センサ関連
#define SEN_WAIT    20         			// センサ発光時間調整
// モータ関連
#define MOT_ON      1         			// モータ電源 ON
#define MOT_OFF     0         			// モータ電源 OFF
#define MOT_L_FWD   1         			// 左モータ前進
#define MOT_L_BACK  0         			// 左モータ後進
#define MOT_R_FWD   0         			// 右モータ前進
#define MOT_R_BACK  1         			// 右モータ後進
#define MOT_SPD_INIT    100   			// モータ起動速度

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//******************************************************************************
// グローバル変数
//******************************************************************************
int tick_count = 0;						// 割り込みカウント用変数
int time_count = 0;						// 経過時間カウント用変数
float batt;								// 電源電圧
int mode = 0;							// 現在モード格納用
int sen_val[8];							// センサ値	7 6 5 4 3 2 1 ※右からナンバリング
int sen_ref[8];							// ライントレース用 センサ閾値
uint16_t adc1_val[4];                   // ADC1を変換した値を格納する配列
uint16_t adc2_val[4];                   // ADC2を変換した値を格納する配列
int mot_spd_r = 0;            			// 右モータ速度
int mot_spd_l = 0;            			// 左モータ速度
int target_spd_r = 0;          			// 右モータ目標速度
int target_spd_l = 0;          			// 左モータ目標速度
int target_spd = 0;
int pre_sen_val[8];

int MOT_ACC = 0;						// モータ加速度

int sen_val_r = 0;            // 右センサ値
int sen_val_f = 0;            // 前センサ値
int sen_val_l = 0;            // 左センサ値
int sen_val_ml = 0;            // 右センサ値
int sen_val_mr = 0;            // 左センサ値
int sen_val_gr = 0;
int line_val = 0;					//ライン状態格納変数
int pre_line_val;

int R_middle_SEN = 0;
int L_middle_SEN = 0;
int R_SEN = 0;
int F_SEN = 0;
int L_SEN = 0;

//片方のline_valを除く処理
int hosei_r_flag = 0;
int hosei_l_flag = 0;


//キャリブレーション用の変数(最大・最小)
int sen_val_pre_max[8];
int sen_val_pre_min[8];

/*int sen_val_pre_max[1] = 999;
int sen_val_pre_max[2] = 999;
int sen_val_pre_max[3] = 999;
int sen_val_pre_max[4] = 999;
int sen_val_pre_max[5] = 999;
int sen_val_pre_max[6] = 999;
int sen_val_pre_max[7] = 999;

int sen_val_pre_min[1] = 0;
int sen_val_pre_min[2] = 0;
int sen_val_pre_min[3] = 0;
int sen_val_pre_min[4] = 0;
int sen_val_pre_min[5] = 0;
int sen_val_pre_min[6] = 0;
int sen_val_pre_min[7] = 0;*/

//左偏差・右偏差保存用変数
int sen_diff_r = 0;
int sen_diff_l = 0;
int sen_diff_ml = 0;
int sen_diff_mr = 0;
int pre_sen_diff_r = 0;     //ひとつ前の偏差を保存
int pre_sen_diff_l = 0;     //ひとつ前の偏差を保存
int pre_sen_diff_ml = 0;
int pre_sen_diff_mr = 0;

int line_counter = 0;
int line_flag_r= 0;
int line_flag_l= 0;

//キャリブレーション計算用変数
float calibracion_r = 0;        //右センサ
float calibracion_f = 0;        //中央センサ
float calibracion_l = 0;        //左センサ


//ライントレース用制御ゲイン
float trace_gain_f    = 0;          // ライントレース用　制御ゲイン：中央
float trace_gain_fr   = 0;          // ライントレース用　制御ゲイン：中央＋右
float trace_gain_fl   = 0;          // ライントレース用　制御ゲイン：中央＋左
float trace_gain_r    = 0;          // ライントレース用　制御ゲイン：右
float trace_gain_l    = 0;          // ライントレース用　制御ゲイン：左
float trace_gain_r_ext= 0;          // ライントレース用　制御ゲイン：右センサ外
float trace_gain_l_ext= 0;          // ライントレース用　制御ゲイン：左センサ外

float control_r = 0;
float control_l = 0;


int r_out_flg;
int l_out_flg;
int ext_flag_r;			//	センサ外対応
int ext_flag_l;

int goal;                     // ゴールマーカ用　変数カウント
int c_counter;                // 真ん中のカウント
int s_handan;               //クロスかそうでない判断用変数
int cross_handan =0;          //ゴール判断時のクロス判断するための変数
int goal_extend = 0;            //ゴールしたと判断した際のまっすぐ走るための判断用変数
int goal_chase = 0;             //ゴールする際の時間数えるよう変数．
int cross_flag = 0;

int cross_count_R;				//ゴールカウント
int goal_count = 0;					//スタートからの経過時間


//現在のセンサ値保存用変数(キャリ部レーションの影響でP制御が出来ないため)
int sen_hozon_f = 0;

int sen_mokuhyou = 999;
int hensa_hozon = 0;
float hensa_Kp = 0.8;

float Kp = 0;           //比例ゲイン宣言
float Kd = 0;
float d;

int id_flag = 0;        //D制御の時間を数えるためのフラグ
int id_cnt = 0;         //D制御の時間を数えるための格納変数
int pre_id_cnt;

int i = 0;


//制御式の変数
float seigyo_p = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void finish( void );
void select_mode( int mode_com );
void mot_l_drive( void );
void mot_r_drive( void );
void sen_ref_all( void );
void calibration( void );
void set_param( void );
void second_param( void );
void third_param( void );
void fourth_param( void );
void fifth_param( void );
void countdown( void );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  LCD_init();								// LCD 初期化
  HAL_TIM_Base_Start_IT(&htim6);			// タイマスタート関数
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED); // ADC1 キャリブレーション
  HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED); // ADC2 キャリブレーション
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_val, 4);     // DMA転送開始
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_val, 4);     // DMA転送開始

  // 起動時動作
  Beep(TONE_RE,6,100);   					// 起動音:高音5のレ, 100ms
  Beep(TONE_SO,6,100);						// 起動音:高音5のソ, 100ms
  LCD_print(1, 0, "Sample12: Speed ");		// LCD1行目データセット
  LCD_print(2, 0, "       .  [ V ] ");		// LCD2行目データセット

  // 電源電圧取得
  batt = adc1_val[3] * 3.3 / 1023.0 * 10.0; // 取得値を電圧に変換：分圧比1/10を戻すために10倍
  LCD_dec_out(2, 5, (int)batt, 2);			// 電圧整数部2桁表示
  LCD_dec_out(2, 8, (int)((batt-(int)batt)*10), 1);	// 電圧小数第一位1桁表示
  HAL_Delay( 2000 );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if( HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == SW_ON ){	// SW_UPが押されたら
      HAL_Delay(SW_WAIT);											// チャタリング防止
      mode++;								// モードを1つ上げる
	}
    if( HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == SW_ON ){// SW_DOWNが押されたら
      HAL_Delay(SW_WAIT);											// チャタリング防止
      mode--;								// モードを1つ下げる
	}
    if( HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON ){	// SW_MIDが押されたら
      HAL_Delay(SW_WAIT);											// チャタリング防止
      select_mode( EXEC );					// モード実行
      mode = 0;								// 実行後は初期画面に戻す
    }
    select_mode( DISP );					// 選択中のモードを表示

    // 誤消去防止
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//==============================================================================
// 各モード
//==============================================================================
void select_mode( int mode_com ){
  //モード番号：上限／下限処理
  if( mode >= MODE_MAX )  mode = 0;		// モードが最大値を超えている場合は0に戻す
  if( mode < 0 )  mode = MODE_MAX - 1;	// モードが負の場合はモードを最大値に設定

  if( mode == 0 ){
  	//------------------------------------------------
  	// Mode0 : センサチェック
  	//------------------------------------------------
  	if( mode_com == DISP ){  			// DISP指示の場合：モードタイトル表示
  	  LCD_print( 1, 0, "0: Sensor Check " );
  	  LCD_print( 2, 0, "                " );
  	}else if( mode_com == EXEC ){		// EXEC指示の場合：モード実行
  		LCD_clear(1);
  		do{
  			LCD_dec_out( 2, 12, sen_val[1], 3 );	// 右端センサ値	×　×　×　×　×　×　〇
  			LCD_dec_out( 1, 10, sen_val[2], 3 );	// 右2つ目センサ値	×　×　×　×　×　〇　×
  			LCD_dec_out( 2,  8, sen_val[3], 3 );	// 中央右センサ値	×　×　×　×　〇　×　×
  			LCD_dec_out( 1,  6, sen_val[4], 3 );	// 中央センサ値	×　×　×　〇　×　×　×
  			LCD_dec_out( 2,  4, sen_val[5], 3 );	// 中央左センサ値	×　×　〇　×　×　×　×
  			LCD_dec_out( 1,  2, sen_val[6], 3 );	// 左2つ目センサ値	×　〇　×　×　×　×　×
  			LCD_dec_out( 2,  0, sen_val[7], 3 );	// 左端センサ値	〇　×　×　×　×　×　×
  		}while( HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_OFF );
  		// SW_MIDが押されたら
  		HAL_Delay(SW_WAIT);
  		LCD_clear(2);
  	}
    }
    else if( mode == 1 ){
  	//------------------------------------------------
  	// Mode1 : モータチェック
  	//------------------------------------------------
  	if( mode_com == DISP ){  			// DISP指示の場合：モードタイトル表示
  	  LCD_print( 1, 0, "1: Motor Check  " );
  	  LCD_print( 2, 0, "                " );
  	}else if( mode_com == EXEC ){		// EXEC指示の場合：モード実行

  		LCD_print( 2, 0, " SPD =          " );
  		HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_ON);			// モータEnable
  		HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port, MOT_R_DIR_Pin, MOT_R_FWD);	// 右モータ方向指示 : 前進
  	    HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port, MOT_L_DIR_Pin, MOT_L_FWD);	// 左モータ方向指示 : 前進

  	    while( 1 ){
  	    	LCD_dec_out( 2,  2, target_spd_r, 4 );							// 指示速度表示　※代表で右を表示
  	    	if( HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == SW_ON ){	// SW_UPが押されたら
  	        	HAL_Delay(SW_WAIT);											// チャタリング防止
  	        	target_spd_r += 100;										// 右加速
  	        	target_spd_l += 100;										// 左加速
  	    	}
  	        if( HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == SW_ON ){// SW_DOWNが押されたら
  	        	HAL_Delay(SW_WAIT);											// チャタリング防止
  	        	target_spd_r -= 100;										// 右減速
  	        	target_spd_l -= 100;										// 左減速
  	    	}
  	        if( HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON ){	// SW_MIDが押されたら
  	        	HAL_Delay(SW_WAIT);											// チャタリング防止
  	        	target_spd_r = 0;											// 右停止
  	        	target_spd_l = 0;											// 左停止
  	        	while( mot_spd_l != 0 || mot_spd_r != 0 );    				// 完全停止待ち
  	    	    HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_OFF);	// モータDisable
  	    	    break;
  	        }
  	    }
  	}
    }
    else if( mode == 2 ){
  	//------------------------------------------------
  	// Mode2 :
  	//------------------------------------------------
  	if( mode_com == DISP ){  			// DISP指示の場合：モードタイトル表示
  	  LCD_print( 1, 0, "2: sen_ref        " );
  	  LCD_print( 2, 0, "                " );
  	}else if( mode_com == EXEC ){		// EXEC指示の場合：モード実行
  		LCD_clear(1);
  		sen_ref_all();
  		do{
  			if( sen_val[1] > sen_ref[1] )	LCD_print( 2, 12, "〇" );		// 右端センサ値	×　×　×　×　×　×　〇
  			else 								LCD_print( 2, 12, "×" );
  			if( sen_val[2] > sen_ref[2] )	LCD_print( 1, 10, "〇" );		// 右2つ目センサ値	×　×　×　×　×　〇　×
  			else 								LCD_print( 1, 10, "×" );
  			if( sen_val[3] > sen_ref[3] )	LCD_print( 2, 8, "〇" );		// 中央右センサ値	×　×　×　×　〇　×　×
  			else 								LCD_print( 2, 8, "×" );
  			if( sen_val[4] > sen_ref[4] )	LCD_print( 1, 6, "〇" );		// 中央センサ値	×　×　×　〇　×　×　×
  			else 								LCD_print( 1, 6, "×" );
  			if( sen_val[5] > sen_ref[5] )	LCD_print( 2, 4, "〇" );		// 中央左センサ値	×　×　〇　×　×　×　×
  			else 								LCD_print( 2, 4, "×" );
  			if( sen_val[6] > sen_ref[6] )	LCD_print( 1, 2, "〇" );		// 左2つ目センサ値	×　〇　×　×　×　×　×
  			else 								LCD_print( 1, 2, "×" );
  			if( sen_val[7] > sen_ref[7] )	LCD_print( 2, 0, "〇" );		// 左2つ目センサ値	×　〇　×　×　×　×　×
  			else 								LCD_print( 2, 0, "×" );

  		}while( HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_OFF );
  		 // SW_MIDが押されたら
  		 HAL_Delay(SW_WAIT);
  		 LCD_clear(2);
  	  //LCD_print( 2, 0, "    OK!!!!!!    " );
  	  //HAL_Delay( 1000 );
  	}
    }
    else if( mode == 3 ){
  	//------------------------------------------------
  	// Mode 3:走行モード
  	//------------------------------------------------
  	  if( mode_com == DISP ){  			// DISP指示の場合：モードタイトル表示
  	  	  LCD_print( 1, 0, "3: tida    Trace" );
  	  	  LCD_print( 2, 0, "                " );
  	  	}else if( mode_com == EXEC ){		// EXEC指示の場合：モード実行
  	  		LCD_clear(1);
  	  		set_param();
  	  		sen_ref_all();

  	  		while(1){
  				if(HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == SW_ON ){// 右SW入力判断
  					HAL_Delay( SW_WAIT );        // チャタリング防止
  					break;
  				}
  				if( HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == SW_ON ){// 左SW入力判断
  					HAL_Delay( SW_WAIT );        // チャタリング防止
  					calibration();
  					//LCD_print( 1, 0, "3: calibration" );
  					HAL_Delay( 1000 );
  					break;
  				}
  			}

  	  		goal_count = 0;
  			s_handan = 0;
  			cross_handan = 0;
  			countdown();
  			cross_count_R = 0;

  	  	    HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_ON);			// モータEnable
  		    HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port, MOT_R_DIR_Pin, MOT_R_FWD);	// 右モータ方向指示 : 前進
  		    HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port, MOT_L_DIR_Pin, MOT_L_FWD);	// 左モータ方向指示 : 前進
  		    while(1){
  			   //do{

					   // センサ値からライン状態を保存
					   line_val = 0;                             // ライン状態リセット

					   sen_val_ml =  sen_val[6];       // 右センサのA/D生値を0-999に加工
					   sen_val_mr =  sen_val[2];       // 前センサのA/D生値を0-999に加工
					   sen_val_r =  sen_val[3];       // 右センサのA/D生値を0-999に加工
					   sen_val_f =  sen_val[4];       // 前センサのA/D生値を0-999に加工
					   sen_val_l =  sen_val[5];       // 左センサのA/D生値を0-999に加工
					   sen_val_gr = sen_val[1];       // 左センサのA/D生値を0-999に加工

					   R_SEN = (float)(sen_val_r - sen_val_pre_min[3])*1000/(sen_val_pre_max[3] - sen_val_pre_min[3]);
					   F_SEN = (float)(sen_val_f - sen_val_pre_min[4])*1000/(sen_val_pre_max[4] - sen_val_pre_min[4]);
					   L_SEN = (float)(sen_val_l - sen_val_pre_min[5])*1000/(sen_val_pre_max[5] - sen_val_pre_min[5]);
					   L_middle_SEN = (float)(sen_val_ml - sen_val_pre_min[6])*1000/(sen_val_pre_max[6] - sen_val_pre_min[6]);
					   R_middle_SEN = (float)(sen_val_mr - sen_val_pre_min[2])*1000/(sen_val_pre_max[2] - sen_val_pre_min[2]);

					   /*if( R_SEN == 1 )     line_val += 1;  // 右センサ
					   if( F_SEN == 1 )     line_val += 2;  // 中央センサ
					   if( L_SEN == 1 )     line_val += 4;  // 左センサ*/

					   //PD_control(L_SEN ,F_SEN ,R_SEN );

					   if( R_SEN > sen_ref[3] )     line_val += 2;  // 右センサ
					   if( F_SEN > sen_ref[4] )     line_val += 4;  // 中央センサ
					   if( L_SEN > sen_ref[5] )     line_val += 10;  // 左センサ
					   if( R_middle_SEN > sen_ref[2])	line_val += 1;
					   if( L_middle_SEN > sen_ref[6])	line_val += 11;


					   if( ext_flag_r == 1 ){
						   if(((( line_val == 1 || line_val == 3 )|| line_val == 6) || line_val == 7) || line_val == 4 ) {ext_flag_r = 0;}
					   }
					   else if( ext_flag_l == 1 ){
						   if(((( line_val == 11 || line_val == 21 ) || line_val == 25) || line_val == 14) || line_val == 4){ ext_flag_l = 0;}
					   }

					   if( ext_flag_r == 1 ){
						   pre_line_val = 1;
						   line_val = 0;
					   }
					   else if( ext_flag_l == 1 ){
						   pre_line_val = 11;
						   line_val = 0;
					   }
					   // センサ外対応
					   if( ( pre_line_val == 1 || pre_line_val == 30 ) && line_val == 0 )
						   line_val = 30;   // 1つ前の状態が右のみ，または右センサ外対応で，今も全センサ無反応の場合：右センサ外対応
					   if( ( pre_line_val == 11 || pre_line_val == 31 ) && line_val == 0 )
						   line_val = 31;   // 1つ前の状態が左のみ，または左センサ外対応で，今も全センサ無反応の場合：左センサ外対応

					   LCD_dec_out( 2,  1, target_spd_l, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
					   LCD_dec_out( 2,  12, target_spd_r, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
					   LCD_dec_out( 1,  7, cross_count_R, 2 );	// 左端センサ値	〇　×　×　×　×　×　×

					   // ライン状態からモータ目標速度設定
					   switch( line_val ){

						   case 1: // line_val = 1 : XXO : 右センサのみ反応
								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
								   control_r = (sen_diff_mr * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = (target_spd + control_r)-650;
								   target_spd_l = target_spd;
								   break;

						   case 2: // line_val = 2 : XOO : 中央右＋右センサ反応
								   sen_diff_r = ( sen_val_f - sen_val_r )-845;
								   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
								   control_r = (sen_diff_r * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = (target_spd + control_r)-300;
								   target_spd_l = target_spd;
								   line_flag_r = 1;
								   break;


						   case 3: // line_val = 3 : XOO : 中央右＋右センサ反応
								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
								   control_r = (sen_diff_mr * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = (target_spd + control_r)-300;
								   target_spd_l = target_spd;
								   break;

						   case 4: // line_val = 4 : OXX : 中央センサのみ反応
								   target_spd_r = target_spd;
								   target_spd_l = target_spd;
								   break;

						   case 5: // line_val = 3 : XOO : 中央右＋右センサ反応
								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
								   control_r = (sen_diff_mr * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd + control_r;
								   target_spd_l = target_spd;
								   break;

						   case 6: // line_val = 6 : OOX :中央+中央右センサ反応
								   sen_diff_r = ( sen_val_f - sen_val_r )-845;
								   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
								   control_r = (sen_diff_r * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd + control_r;
								   target_spd_l = target_spd;
								   break;

						   /*case 7: // line_val = 6 : OOX :中央+中央右センサ反応
								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
								   control_l = (sen_diff_l * Kp) + (d * Kd);
								   cross_count_R = 0;
								   line_flag_l = 1;
								   target_spd_r = target_spd;
								   target_spd_l = target_spd - control_l;
								   break;*/

						   case 14: // line_val = 11 : OOX : 中央＋中央左センサ反応
								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
								   control_l = (sen_diff_l * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd;
								   target_spd_l = target_spd - control_l;
								   break;

						   case 15:// line_val = 12: OOX :左センサ反応
								   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
								   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
								   control_l = (sen_diff_ml * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd;
								   target_spd_l = target_spd - control_l;
								   break;


						   case 16: // line_val = 16 : OXX : 中央センサのみ反応
								   target_spd_r = target_spd;
								   target_spd_l = target_spd;
								   break;

						   case 28: // line_val = 16 : OXX : 中央センサのみ反応
								   target_spd_r = target_spd;
								   target_spd_l = target_spd;
								   break;

						   case 12: // line_val = 16 : OXX : 中央センサのみ反応
								   target_spd_r = target_spd;
								   target_spd_l = target_spd;
								   break;


						   case 17:// line_val = 19: OOX : 中央左＋左センサ反応
								   sen_diff_r = ( sen_val_f - sen_val_r )-845;
								   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
								   control_r = (sen_diff_r * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd + control_r;
								   target_spd_l = target_spd;
								   break;


						   case 27:// line_val = 19: OOX : 中央左＋左センサ反応
								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
								   control_l = (sen_diff_l * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd;
								   target_spd_l = target_spd - control_l;
								   break;


						   case 10:// line_val = 19: OOX : 中央左＋左センサ反応
								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
								   control_l = (sen_diff_ml * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd;
								   target_spd_l = (target_spd - control_l)-300;
								   break;

						   case 11:// line_val = 12: OOX :左センサ反応
								   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
								   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
								   control_l = (sen_diff_ml * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd;
								   target_spd_l = (target_spd - control_l)-650;
								   break;


						   case 13: // line_val = 3 : XOO : 中央右＋右センサ反応
								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
								   control_r = (sen_diff_mr * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd + control_r;
								   target_spd_l = target_spd;
								   break;


						   case 21:// line_val = 12: OOX :左センサ反応
								   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
								   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
								   control_l = (sen_diff_ml * Kp) + (d * Kd);
								   cross_count_R = 0;
								   target_spd_r = target_spd;
								   target_spd_l = (target_spd - control_l)-300;
								   break;

						   case 30: // line_val = 8 : XXX : 右センサ外対応
								   //target_spd_r = target_spd - ((F_SENP-sen_val_f)*1.5);
								   ext_flag_r = 1;
								   cross_count_R = 0;
								   //trace_gain_r_ext= 0.28;
								   target_spd_r = 300;//target_spd_r * trace_gain_r_ext;
								   target_spd_l = target_spd;//1.1
								   break;

						   case 31: // line_val = 9 : XXX : 左センサ外対応
								   ext_flag_l = 1;
								   cross_count_R = 0;
								   //trace_gain_l_ext = 0.28;
								   target_spd_r = target_spd;//1.1
								   target_spd_l = 300;//target_spd * trace_gain_l_ext;
								   break;

						   default:// line_val = 0,5,7 : XXX, OXO, OOO : 停止
								   target_spd_r = 0;
								   target_spd_l = 0;
								   //cross_flg = 1;
								   break;
					   }
  						   LCD_dec_out( 2,  1, target_spd_l, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
  						   LCD_dec_out( 2,  12, target_spd_r, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
  						   LCD_dec_out( 1,  7, cross_count_R, 5 );	// 左端センサ値	〇　×　×　×　×　×　×

  							   // ライン状態保存
  							   pre_line_val = line_val;
  							   pre_sen_diff_r = sen_diff_r;
  							   pre_sen_diff_l = sen_diff_l;
  							   pre_sen_diff_ml = sen_diff_ml;
  							   pre_sen_diff_mr = sen_diff_mr;

  							   if(sen_val[3] > 400 && sen_val[4] > 400 &&sen_val[5] > 400) {
  								   cross_handan = 1;
  								   goal_count = 0;
  							   }

  							   if(s_handan == 0 && sen_val[1] > sen_ref[1] ){
  								   cross_handan = 1;
  								   s_handan = 1;
  							   }

  							   if( goal_count > 2000 ){
  								   cross_handan = 0;
  								   //cross_count_R = 0;
  							   }
  							   if(sen_val[1] > sen_ref[1]){
  								   if( 1 < cross_count_R && cross_count_R < 100 ){
  									   //cross_count_R = 0;
  									   if( cross_handan == 0 && s_handan == 1){
  										   target_spd_r = 1400;
  										   target_spd_l = 1400;
  										   HAL_Delay(300);
  										   target_spd_r = 1000;
  										   target_spd_l = 1000;
  										   HAL_Delay(300);
  										   target_spd_r = 500;
  										   target_spd_l = 500;
  										   HAL_Delay(100);
  										   break;
  									   }
  								   }else{
  									   //cross_count_R = 0;
  								   }
  							   }
  						   if(HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON){
  							   HAL_Delay( SW_WAIT );
  							   break;
  						   }


  					   }//while(  HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON );         // 中央SW入力判断
  				   //HAL_Delay( SW_WAIT );                          // チャタリング防止
  				   target_spd_r = 0;
  				   target_spd_l = 0;
  				   while( mot_spd_l != 0 || mot_spd_r != 0 );    // 完全停止待ち
  				   HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_OFF);                  // モータEnable OFF

  	  	}
    }
    else if( mode == 4 ){
    	//------------------------------------------------
    	// Mode 4:走行モード
    	//------------------------------------------------
    	  if( mode_com == DISP ){  			// DISP指示の場合：モードタイトル表示
    	  	  LCD_print( 1, 0, "4: Second Trace" );
    	  	  LCD_print( 2, 0, "                " );
    	  	}else if( mode_com == EXEC ){		// EXEC指示の場合：モード実行
    	  		LCD_clear(1);
    	  		second_param();
    	  		sen_ref_all();

    	  		while(1){
    				if(HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == SW_ON ){// 右SW入力判断
    					HAL_Delay( SW_WAIT );        // チャタリング防止
    					break;
    				}
    				if( HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == SW_ON ){// 左SW入力判断
    					HAL_Delay( SW_WAIT );        // チャタリング防止
    					calibration();
    					//LCD_print( 1, 0, "3: calibration" );
    					HAL_Delay( 1000 );
    					break;
    				}
    			}

    	  		goal_count = 0;
    			s_handan = 0;
    			cross_handan = 0;
    			countdown();
    			cross_count_R = 0;

    	  	    HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_ON);			// モータEnable
    		    HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port, MOT_R_DIR_Pin, MOT_R_FWD);	// 右モータ方向指示 : 前進
    		    HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port, MOT_L_DIR_Pin, MOT_L_FWD);	// 左モータ方向指示 : 前進
    		    while(1){
    		    	//do{

  					   // センサ値からライン状態を保存
  					   line_val = 0;                             // ライン状態リセット

  					   sen_val_ml =  sen_val[6];       // 右センサのA/D生値を0-999に加工
  					   sen_val_mr =  sen_val[2];       // 前センサのA/D生値を0-999に加工
  					   sen_val_r =  sen_val[3];       // 右センサのA/D生値を0-999に加工
  					   sen_val_f =  sen_val[4];       // 前センサのA/D生値を0-999に加工
  					   sen_val_l =  sen_val[5];       // 左センサのA/D生値を0-999に加工
  					   sen_val_gr = sen_val[1];       // 左センサのA/D生値を0-999に加工

  					   R_SEN = (float)(sen_val_r - sen_val_pre_min[3])*1000/(sen_val_pre_max[3] - sen_val_pre_min[3]);
  					   F_SEN = (float)(sen_val_f - sen_val_pre_min[4])*1000/(sen_val_pre_max[4] - sen_val_pre_min[4]);
  					   L_SEN = (float)(sen_val_l - sen_val_pre_min[5])*1000/(sen_val_pre_max[5] - sen_val_pre_min[5]);
  					   L_middle_SEN = (float)(sen_val_ml - sen_val_pre_min[6])*1000/(sen_val_pre_max[6] - sen_val_pre_min[6]);
  					   R_middle_SEN = (float)(sen_val_mr - sen_val_pre_min[2])*1000/(sen_val_pre_max[2] - sen_val_pre_min[2]);

  					   /*if( R_SEN == 1 )     line_val += 1;  // 右センサ
  					   if( F_SEN == 1 )     line_val += 2;  // 中央センサ
  					   if( L_SEN == 1 )     line_val += 4;  // 左センサ*/

  					   //PD_control(L_SEN ,F_SEN ,R_SEN );

  					   if( R_SEN > sen_ref[3] )     line_val += 2;  // 右センサ
  					   if( F_SEN > sen_ref[4] )     line_val += 4;  // 中央センサ
  					   if( L_SEN > sen_ref[5] )     line_val += 10;  // 左センサ
  					   if( R_middle_SEN > sen_ref[2])	line_val += 1;
  					   if( L_middle_SEN > sen_ref[6])	line_val += 11;


  					   /*if( hosei_r_flag == 1){
						   if( R_middle_SEN > sen_ref[2] ){
							   line_val =14;
							   hosei_r_flag = 0;
						   }
					   }
					   else if( hosei_l_flag == 1){
						   if( L_middle_SEN > sen_ref[6] ){
							 line_val = 6;
							 hosei_l_flag = 0;
						   }
					   }*/


  					   if( ext_flag_r == 1 ){
  						   if(((( line_val == 1 || line_val == 3 )|| line_val == 6) || line_val == 7) || line_val == 4 ) {ext_flag_r = 0;}
  					   }
  					   else if( ext_flag_l == 1 ){
  						   if(((( line_val == 11 || line_val == 21 ) || line_val == 25) || line_val == 14) || line_val == 4){ ext_flag_l = 0;}
  					   }

  					   if( ext_flag_r == 1 ){
  						   pre_line_val = 1;
  						   line_val = 0;
  					   }
  					   else if( ext_flag_l == 1 ){
  						   pre_line_val = 11;
  						   line_val = 0;
  					   }
  					   // センサ外対応
  					   if( ( pre_line_val == 1 || pre_line_val == 30 ) && line_val == 0 )
  						   line_val = 30;   // 1つ前の状態が右のみ，または右センサ外対応で，今も全センサ無反応の場合：右センサ外対応
  					   if( ( pre_line_val == 11 || pre_line_val == 31 ) && line_val == 0 )
  						   line_val = 31;   // 1つ前の状態が左のみ，または左センサ外対応で，今も全センサ無反応の場合：左センサ外対応

  					   LCD_dec_out( 2,  1, target_spd_l, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
  					   LCD_dec_out( 2,  12, target_spd_r, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
  					   LCD_dec_out( 1,  7, cross_count_R, 2 );	// 左端センサ値	〇　×　×　×　×　×　×

  					   // ライン状態からモータ目標速度設定
  					   switch( line_val ){

  						   case 1: // line_val = 1 : XXO : 右センサのみ反応
  								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  								   control_r = (sen_diff_mr * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = (target_spd + control_r)-650;
  								   target_spd_l = target_spd;
  								   break;

  						   case 2: // line_val = 2 : XOO : 中央右＋右センサ反応
  								   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  								   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  								   control_r = (sen_diff_r * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = (target_spd + control_r)-400;
  								   target_spd_l = target_spd;
  								   line_flag_r = 1;
  								   break;


  						   case 3: // line_val = 3 : XOO : 中央右＋右センサ反応
  								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  								   control_r = (sen_diff_mr * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = (target_spd + control_r)-300;
  								   target_spd_l = target_spd;
  								   break;

  						   case 4: // line_val = 4 : OXX : 中央センサのみ反応
  								   target_spd_r = target_spd;
  								   target_spd_l = target_spd;
  								   break;

  						   case 5: // line_val = 3 : XOO : 中央右＋右センサ反応
  								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  								   control_r = (sen_diff_mr * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = target_spd + control_r;
  								   target_spd_l = target_spd;
  								   break;

  						   case 6: // line_val = 6 : OOX :中央+中央右センサ反応
  								   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  								   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  								   control_r = (sen_diff_r * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = (target_spd + control_r)-200;
  								   target_spd_l = target_spd;
  								   break;

  						   /*case 7: // line_val = 6 : OOX :中央+中央右センサ反応
  								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  								   control_l = (sen_diff_l * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   line_flag_l = 1;
  								   target_spd_r = target_spd;
  								   target_spd_l = target_spd - control_l;
  								   break;*/

  						   case 14: // line_val = 11 : OOX : 中央＋中央左センサ反応
  								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  								   control_l = (sen_diff_l * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = target_spd;
  								   target_spd_l = (target_spd - control_l)-200;
  								   break;

  						   case 15:// line_val = 12: OOX :左センサ反応
  								   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  								   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  								   control_l = (sen_diff_ml * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   hosei_r_flag = 1;
  								   target_spd_r = target_spd;
  								   target_spd_l = target_spd - control_l;
  								   break;


  						   /*case 16: // line_val = 16 : OXX : 中央センサのみ反応
  								   target_spd_r = target_spd;
  								   target_spd_l = target_spd;
  								   break;

  						   case 28: // line_val = 16 : OXX : 中央センサのみ反応
  								   target_spd_r = target_spd;
  								   target_spd_l = target_spd;
  								   break;

  						   case 12: // line_val = 16 : OXX : 中央センサのみ反応
  								   target_spd_r = target_spd;
  								   target_spd_l = target_spd;
  								   break;*/


  						   case 17:// line_val = 19: OOX : 中央左＋左センサ反応
  								   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  								   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  								   control_r = (sen_diff_r * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   hosei_l_flag = 0;
  								   target_spd_r = target_spd + control_r;
  								   target_spd_l = target_spd;
  								   break;


  						   case 27:// line_val = 19: OOX : 中央左＋左センサ反応
  								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  								   control_l = (sen_diff_l * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = target_spd;
  								   target_spd_l = target_spd - control_l;
  								   break;


  						   case 10:// line_val = 19: OOX : 中央左＋左センサ反応
  								   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  								   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  								   control_l = (sen_diff_ml * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = target_spd;
  								   target_spd_l = (target_spd - control_l)-400;
  								   break;

  						   case 11:// line_val = 12: OOX :左センサ反応
  								   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  								   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  								   control_l = (sen_diff_ml * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = target_spd;
  								   target_spd_l = (target_spd - control_l)-650;
  								   break;


  						   case 13: // line_val = 3 : XOO : 中央右＋右センサ反応
  								   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  								   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  								   control_r = (sen_diff_mr * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = target_spd + control_r;
  								   target_spd_l = target_spd;
  								   break;


  						   case 21:// line_val = 12: OOX :左センサ反応
  								   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  								   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  								   control_l = (sen_diff_ml * Kp) + (d * Kd);
  								   cross_count_R = 0;
  								   target_spd_r = target_spd;
  								   target_spd_l = (target_spd - control_l)-300;
  								   break;

  						   case 30: // line_val = 8 : XXX : 右センサ外対応
  								   //target_spd_r = target_spd - ((F_SENP-sen_val_f)*1.5);
  								   ext_flag_r = 1;
  								   cross_count_R = 0;
  								   //trace_gain_r_ext= 0.28;
  								   target_spd_r = 220;//target_spd_r * trace_gain_r_ext;
  								   target_spd_l = 2200;//1.1
  								   break;

  						   case 31: // line_val = 9 : XXX : 左センサ外対応
  								   ext_flag_l = 1;
  								   cross_count_R = 0;
  								   //trace_gain_l_ext = 0.28;
  								   target_spd_r = 2200;//1.1
  								   target_spd_l = 220;//target_spd * trace_gain_l_ext;
  								   break;

  						   default:// line_val = 0,5,7 : XXX, OXO, OOO : 停止
  								   target_spd_r = 0;
  								   target_spd_l = 0;
  								   //cross_flg = 1;
  								   break;
  					   }
  					   LCD_dec_out( 2,  1, target_spd_l, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
  					   LCD_dec_out( 2,  12, target_spd_r, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
  					   LCD_dec_out( 1,  7, cross_count_R, 5 );	// 左端センサ値	〇　×　×　×　×　×　×

  						   // ライン状態保存
  						   pre_line_val = line_val;
  						   pre_sen_diff_r = sen_diff_r;
  						   pre_sen_diff_l = sen_diff_l;
  						   pre_sen_diff_ml = sen_diff_ml;
  						   pre_sen_diff_mr = sen_diff_mr;

  						   if(sen_val[3] > 400 && sen_val[4] > 400 &&sen_val[5] > 400) {
  							   cross_handan = 1;
  							   goal_count = 0;
  						   }

  						   if(s_handan == 0 && sen_val[1] > sen_ref[1] ){
  							   cross_handan = 1;
  							   s_handan = 1;
  						   }

  						   if( goal_count > 2000 ){
  							   cross_handan = 0;
  							   //cross_count_R = 0;
  						   }
  						   if(sen_val[1] > sen_ref[1]){
  							   if( 1 < cross_count_R && cross_count_R < 100 ){
  								   //cross_count_R = 0;
  								   if( cross_handan == 0 && s_handan == 1){
  									   target_spd_r = 1400;
  									   target_spd_l = 1400;
  									   HAL_Delay(300);
  									   target_spd_r = 1000;
  									   target_spd_l = 1000;
  									   HAL_Delay(300);
  									   target_spd_r = 500;
  									   target_spd_l = 500;
  									   HAL_Delay(100);
  									   break;
  								   }
  							   }else{
  								   //cross_count_R = 0;
  							   }
  						   }
  					   if(HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON){
  						   HAL_Delay( SW_WAIT );
  						   break;
  					   }


  				   }//while(  HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON );         // 中央SW入力判断
    				   //HAL_Delay( SW_WAIT );                          // チャタリング防止
    				   target_spd_r = 0;
    				   target_spd_l = 0;
    				   while( mot_spd_l != 0 || mot_spd_r != 0 );    // 完全停止待ち
    				   HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_OFF);                  // モータEnable OFF

    	  	}
      }
    	else if( mode == 5 ){
      	//------------------------------------------------
      	// Mode 5:走行モードthird
      	//------------------------------------------------
      	  if( mode_com == DISP ){  			// DISP指示の場合：モードタイトル表示
      	  	  LCD_print( 1, 0, "5: Third Trace" );
      	  	  LCD_print( 2, 0, "                " );
      	  	}else if( mode_com == EXEC ){		// EXEC指示の場合：モード実行
      	  		LCD_clear(1);
      	  		third_param();
      	  		sen_ref_all();

      	  		while(1){
      				if(HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == SW_ON ){// 右SW入力判断
      					HAL_Delay( SW_WAIT );        // チャタリング防止
      					break;
      				}
      				if( HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == SW_ON ){// 左SW入力判断
      					HAL_Delay( SW_WAIT );        // チャタリング防止
      					calibration();
      					//LCD_print( 1, 0, "3: calibration" );
      					HAL_Delay( 1000 );
      					break;
      				}
      			}

      	  		goal_count = 0;
      			s_handan = 0;
      			cross_handan = 0;
      			countdown();
      			cross_count_R = 0;

      	  	    HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_ON);			// モータEnable
      		    HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port, MOT_R_DIR_Pin, MOT_R_FWD);	// 右モータ方向指示 : 前進
      		    HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port, MOT_L_DIR_Pin, MOT_L_FWD);	// 左モータ方向指示 : 前進
      		    while(1){
      		   //do{

      					   // センサ値からライン状態を保存
      					   line_val = 0;                             // ライン状態リセット

      					   sen_val_ml =  sen_val[6];       // 右センサのA/D生値を0-999に加工
  						   sen_val_mr =  sen_val[2];       // 前センサのA/D生値を0-999に加工
      					   sen_val_r =  sen_val[3];       // 右センサのA/D生値を0-999に加工
      					   sen_val_f =  sen_val[4];       // 前センサのA/D生値を0-999に加工
      					   sen_val_l =  sen_val[5];       // 左センサのA/D生値を0-999に加工
      					   sen_val_gr = sen_val[1];       // 左センサのA/D生値を0-999に加工

      					   R_SEN = (float)(sen_val_r - sen_val_pre_min[3])*1000/(sen_val_pre_max[3] - sen_val_pre_min[3]);
      					   F_SEN = (float)(sen_val_f - sen_val_pre_min[4])*1000/(sen_val_pre_max[4] - sen_val_pre_min[4]);
      					   L_SEN = (float)(sen_val_l - sen_val_pre_min[5])*1000/(sen_val_pre_max[5] - sen_val_pre_min[5]);
      					   L_middle_SEN = (float)(sen_val_ml - sen_val_pre_min[6])*1000/(sen_val_pre_max[6] - sen_val_pre_min[6]);
      					   R_middle_SEN = (float)(sen_val_mr - sen_val_pre_min[2])*1000/(sen_val_pre_max[2] - sen_val_pre_min[2]);

      					   /*if( R_SEN == 1 )     line_val += 1;  // 右センサ
      					   if( F_SEN == 1 )     line_val += 2;  // 中央センサ
      					   if( L_SEN == 1 )     line_val += 4;  // 左センサ*/

      					   //PD_control(L_SEN ,F_SEN ,R_SEN );

      					   if( R_SEN > sen_ref[3] )     line_val += 2;  // 右センサ
      					   if( F_SEN > sen_ref[4] )     line_val += 4;  // 中央センサ
      					   if( L_SEN > sen_ref[5] )     line_val += 10;  // 左センサ
      					   if( R_middle_SEN > sen_ref[2])	line_val += 1;
      					   if( L_middle_SEN > sen_ref[6])	line_val += 11;


      					   if( ext_flag_r == 1 ){
  							   if((( line_val == 1 || line_val == 3 )|| line_val == 6) || line_val == 7) {ext_flag_r = 0;}
  						   }
  						   else if( ext_flag_l == 1 ){
  							   if((( line_val == 11 || line_val == 21 ) || line_val == 25) || line_val == 14){ ext_flag_l = 0;}
  						   }

      					   if( ext_flag_r == 1 ){
      						   pre_line_val = 1;
      						   line_val = 0;
      					   }
      					   else if( ext_flag_l == 1 ){
      						   pre_line_val = 11;
      						   line_val = 0;
      					   }
      					   // センサ外対応
      					   if( ( pre_line_val == 1 || pre_line_val == 30 ) && line_val == 0 )
      						   line_val = 30;   // 1つ前の状態が右のみ，または右センサ外対応で，今も全センサ無反応の場合：右センサ外対応
      					   if( ( pre_line_val == 11 || pre_line_val == 31 ) && line_val == 0 )
      						   line_val = 31;   // 1つ前の状態が左のみ，または左センサ外対応で，今も全センサ無反応の場合：左センサ外対応

      					   LCD_dec_out( 2,  1, target_spd_l, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
      					   LCD_dec_out( 2,  12, target_spd_r, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
      					   LCD_dec_out( 1,  7, cross_count_R, 2 );	// 左端センサ値	〇　×　×　×　×　×　×

      					   // ライン状態からモータ目標速度設定
      					   switch( line_val ){

  							   case 1: // line_val = 1 : XXO : 右センサのみ反応
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-650;
  									   target_spd_l = target_spd;
  									   break;

  							   case 2: // line_val = 2 : XOO : 中央右＋右センサ反応
  								   	   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-550;
  									   target_spd_l = target_spd;
  									   break;


  							   case 3: // line_val = 3 : XOO : 中央右＋右センサ反応
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-300;
  									   target_spd_l = target_spd;
  									   break;

  							   case 4: // line_val = 4 : OXX : 中央センサのみ反応
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 5: // line_val = 3 : XOO : 中央右＋右センサ反応
  								   	   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;

  							   case 6: // line_val = 6 : OOX :中央+中央右センサ反応
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;

  							   /*case 7: // line_val = 6 : OOX :中央+中央右センサ反応
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   line_flag_l = 1;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;*/

  							   case 14: // line_val = 11 : OOX : 中央＋中央左センサ反応
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;

  							   case 15:// line_val = 12: OOX :左センサ反応
  								   	   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;


  							   /*case 16: // line_val = 16 : OXX : 中央センサのみ反応
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 28: // line_val = 16 : OXX : 中央センサのみ反応
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 12: // line_val = 16 : OXX : 中央センサのみ反応
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;*/


  							   case 17:// line_val = 19: OOX : 中央左＋左センサ反応
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;


  							   case 27:// line_val = 19: OOX : 中央左＋左センサ反応
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;


  							   case 10:// line_val = 19: OOX : 中央左＋左センサ反応
  								   	   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-550;
  									   break;

  							   case 11:// line_val = 12: OOX :左センサ反応
  									   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-650;
  									   break;


  							   case 13: // line_val = 3 : XOO : 中央右＋右センサ反応
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;


  							   case 21:// line_val = 12: OOX :左センサ反応
  									   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-300;
  									   break;

  							   case 30: // line_val = 8 : XXX : 右センサ外対応
  									   ext_flag_r = 1;
  									   cross_count_R = 0;
  									   //trace_gain_r_ext= 0.28;
  									   target_spd_r = 225;//target_spd_r * trace_gain_r_ext;
  									   target_spd_l = target_spd;//1.1
  									   break;

  							   case 31: // line_val = 9 : XXX : 左センサ外対応
  									   ext_flag_l = 1;
  									   cross_count_R = 0;
  									   //trace_gain_l_ext = 0.28;
  									   target_spd_r = target_spd;//1.1
  									   target_spd_l = 225;//target_spd * trace_gain_l_ext;
  									   break;

  							   default:// line_val = 0,5,7 : XXX, OXO, OOO : 停止
  									   target_spd_r = 0;
  									   target_spd_l = 0;
  									   //cross_flg = 1;
  									   break;
  						   }
      					   LCD_dec_out( 2,  1, target_spd_l, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
      					   LCD_dec_out( 2,  12, target_spd_r, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
      					   LCD_dec_out( 1,  7, cross_count_R, 5 );	// 左端センサ値	〇　×　×　×　×　×　×

      						   // ライン状態保存
      						   pre_line_val = line_val;
      						   pre_sen_diff_r = sen_diff_r;
      						   pre_sen_diff_l = sen_diff_l;
      						   pre_sen_diff_ml = sen_diff_ml;
      						   pre_sen_diff_mr = sen_diff_mr;


      						   if(s_handan == 0){
  								   target_spd = 2000;
  							   }
  							   else{
  								   target_spd = 2400;
  							   }

      						   if(sen_val[3] > 400 && sen_val[4] > 400 &&sen_val[5] > 400) {
      							   cross_handan = 1;
      							   goal_count = 0;
      						   }

      						   if(s_handan == 0 && sen_val[1] > sen_ref[1] ){
      							   cross_handan = 1;
      							   s_handan = 1;
      						   }

      						   if( goal_count > 2000 ){
      							   cross_handan = 0;
      							   //cross_count_R = 0;
      						   }
      						   if(sen_val[1] > sen_ref[1]){
      							   if( 1 < cross_count_R && cross_count_R < 90 ){
      								   //cross_count_R = 0;
      								   if( cross_handan == 0 && s_handan == 1){
      									   target_spd_r = 1400;
      									   target_spd_l = 1400;
      									   HAL_Delay(300);
      									   target_spd_r = 1000;
      									   target_spd_l = 1000;
      									   HAL_Delay(300);
      									   target_spd_r = 500;
      									   target_spd_l = 500;
      									   HAL_Delay(100);
      									   break;
      								   }
      							   }else{
      								   //cross_count_R = 0;
      							   }
      						   }
      					   if(HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON){
      						   HAL_Delay( SW_WAIT );
      						   break;
      					   }


      				   }//while(  HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON );         // 中央SW入力判断
      				   //HAL_Delay( SW_WAIT );                          // チャタリング防止
      				   target_spd_r = 0;
      				   target_spd_l = 0;
      				   while( mot_spd_l != 0 || mot_spd_r != 0 );    // 完全停止待ち
      				   HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_OFF);                  // モータEnable OFF

      	  	}
        }
    else if( mode == 6 ){
        	//------------------------------------------------
        	// Mode 6:走行モードthird
        	//------------------------------------------------
        	  if( mode_com == DISP ){  			// DISP指示の場合：モードタイトル表示
        	  	  LCD_print( 1, 0, "6: Fourth Trace" );
        	  	  LCD_print( 2, 0, "                " );
        	  	}else if( mode_com == EXEC ){		// EXEC指示の場合：モード実行
        	  		LCD_clear(1);
        	  		fourth_param();
        	  		sen_ref_all();

        	  		while(1){
        				if(HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == SW_ON ){// 右SW入力判断
        					HAL_Delay( SW_WAIT );        // チャタリング防止
        					break;
        				}
        				if( HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == SW_ON ){// 左SW入力判断
        					HAL_Delay( SW_WAIT );        // チャタリング防止
        					calibration();
        					//LCD_print( 1, 0, "3: calibration" );
        					HAL_Delay( 1000 );
        					break;
        				}
        			}

        	  		goal_count = 0;
        			s_handan = 0;
        			cross_handan = 0;
        			countdown();
        			cross_count_R = 0;

        	  	    HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_ON);			// モータEnable
        		    HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port, MOT_R_DIR_Pin, MOT_R_FWD);	// 右モータ方向指示 : 前進
        		    HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port, MOT_L_DIR_Pin, MOT_L_FWD);	// 左モータ方向指示 : 前進
        		    while(1){
        		   //do{

        					   // センサ値からライン状態を保存
        					   line_val = 0;                             // ライン状態リセット

        					   sen_val_ml =  sen_val[6];       // 右センサのA/D生値を0-999に加工
    						   sen_val_mr =  sen_val[2];       // 前センサのA/D生値を0-999に加工
        					   sen_val_r =  sen_val[3];       // 右センサのA/D生値を0-999に加工
        					   sen_val_f =  sen_val[4];       // 前センサのA/D生値を0-999に加工
        					   sen_val_l =  sen_val[5];       // 左センサのA/D生値を0-999に加工
        					   sen_val_gr = sen_val[1];       // 左センサのA/D生値を0-999に加工

        					   R_SEN = (float)(sen_val_r - sen_val_pre_min[3])*1000/(sen_val_pre_max[3] - sen_val_pre_min[3]);
        					   F_SEN = (float)(sen_val_f - sen_val_pre_min[4])*1000/(sen_val_pre_max[4] - sen_val_pre_min[4]);
        					   L_SEN = (float)(sen_val_l - sen_val_pre_min[5])*1000/(sen_val_pre_max[5] - sen_val_pre_min[5]);
        					   L_middle_SEN = (float)(sen_val_ml - sen_val_pre_min[6])*1000/(sen_val_pre_max[6] - sen_val_pre_min[6]);
        					   R_middle_SEN = (float)(sen_val_mr - sen_val_pre_min[2])*1000/(sen_val_pre_max[2] - sen_val_pre_min[2]);

        					   /*if( R_SEN == 1 )     line_val += 1;  // 右センサ
        					   if( F_SEN == 1 )     line_val += 2;  // 中央センサ
        					   if( L_SEN == 1 )     line_val += 4;  // 左センサ*/

        					   //PD_control(L_SEN ,F_SEN ,R_SEN );

        					   if( R_SEN > sen_ref[3] )     line_val += 2;  // 右センサ
        					   if( F_SEN > sen_ref[4] )     line_val += 4;  // 中央センサ
        					   if( L_SEN > sen_ref[5] )     line_val += 10;  // 左センサ
        					   if( R_middle_SEN > sen_ref[2])	line_val += 1;
        					   if( L_middle_SEN > sen_ref[6])	line_val += 11;


        					   if( ext_flag_r == 1 ){
  							   if(((( line_val == 1 || line_val == 3 )|| line_val == 6) || line_val == 7) || line_val == 4 ) {ext_flag_r = 0;}
  						   }
  						   else if( ext_flag_l == 1 ){
  							   if(((( line_val == 11 || line_val == 21 ) || line_val == 25) || line_val == 14) || line_val == 4){ ext_flag_l = 0;}
  						   }

        					   if( ext_flag_r == 1 ){
        						   pre_line_val = 1;
        						   line_val = 0;
        					   }
        					   else if( ext_flag_l == 1 ){
        						   pre_line_val = 11;
        						   line_val = 0;
        					   }
        					   if( hosei_r_flag == 1){
								   if( R_middle_SEN > sen_ref[2] ){
									   line_val =14;
								   }
							   }
							   else if( hosei_l_flag == 1){
								   if( L_middle_SEN > sen_ref[6] ){
									 line_val = 6;
								   }
							   }

        					   // センサ外対応
        					   if( ( pre_line_val == 1 || pre_line_val == 30 ) && line_val == 0 )
        						   line_val = 30;   // 1つ前の状態が右のみ，または右センサ外対応で，今も全センサ無反応の場合：右センサ外対応
        					   if( ( pre_line_val == 11 || pre_line_val == 31 ) && line_val == 0 )
        						   line_val = 31;   // 1つ前の状態が左のみ，または左センサ外対応で，今も全センサ無反応の場合：左センサ外対応

        					   LCD_dec_out( 2,  1, target_spd_l, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
        					   LCD_dec_out( 2,  12, target_spd_r, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
        					   LCD_dec_out( 1,  7, cross_count_R, 2 );	// 左端センサ値	〇　×　×　×　×　×　×

        					   // ライン状態からモータ目標速度設定
        					 switch( line_val ){

  							   case 1: // line_val = 1 : XXO : 右センサのみ反応
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-750;
  									   target_spd_l = target_spd;
  									   break;

  							   case 2: // line_val = 2 : XOO : 中央右＋右センサ反応
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-500;
  									   target_spd_l = target_spd;
  									   line_flag_r = 1;
  									   break;


  							   case 3: // line_val = 3 : XOO : 中央右＋右センサ反応
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-350;
  									   target_spd_l = target_spd;
  									   break;

  							   case 4: // line_val = 4 : OXX : 中央センサのみ反応
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 5: // line_val = 3 : XOO : 中央右＋右センサ反応
  								   	   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;

  							   case 6: // line_val = 6 : OOX :中央+中央右センサ反応
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-200;
  									   target_spd_l = target_spd;
  									   break;

  							   /*case 7: // line_val = 6 : OOX :中央+中央右センサ反応
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   line_flag_l = 1;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;*/

  							   case 14: // line_val = 11 : OOX : 中央＋中央左センサ反応
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-200;
  									   break;

  							   case 15:// line_val = 12: OOX :左センサ反応
  								   	   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   hosei_r_flag = 1;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;


  							   /*case 16: // line_val = 16 : OXX : 中央センサのみ反応
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 28: // line_val = 16 : OXX : 中央センサのみ反応
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 12: // line_val = 16 : OXX : 中央センサのみ反応
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;*/


  							   case 17:// line_val = 19: OOX : 中央左＋左センサ反応
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   hosei_l_flag = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;


  							   case 27:// line_val = 19: OOX : 中央左＋左センサ反応
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;


  							   case 10:// line_val = 19: OOX : 中央左＋左センサ反応
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-500;
  									   break;

  							   case 11:// line_val = 12: OOX :左センサ反応
  									   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-750;
  									   break;


  							   case 13: // line_val = 3 : XOO : 中央右＋右センサ反応
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;


  							   case 21:// line_val = 12: OOX :左センサ反応
  									   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-350;
  									   break;

  							   case 30: // line_val = 8 : XXX : 右センサ外対応
  									   //target_spd_r = target_spd - ((F_SENP-sen_val_f)*1.5);
  									   ext_flag_r = 1;
  									   cross_count_R = 0;
  									   //trace_gain_r_ext= 0.28;
  									   target_spd_r = 100;//target_spd_r * trace_gain_r_ext;
  									   target_spd_l = target_spd;//1.1
  									   break;

  							   case 31: // line_val = 9 : XXX : 左センサ外対応
  									   ext_flag_l = 1;
  									   cross_count_R = 0;
  									   //trace_gain_l_ext = 0.28;
  									   target_spd_r = target_spd;//1.1
  									   target_spd_l = 100;//target_spd * trace_gain_l_ext;
  									   break;

  							   default:// line_val = 0,5,7 : XXX, OXO, OOO : 停止
  									   target_spd_r = 0;
  									   target_spd_l = 0;
  									   //cross_flg = 1;
  									   break;
  						   }
        					   LCD_dec_out( 2,  1, target_spd_l, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
        					   LCD_dec_out( 2,  12, target_spd_r, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
        					   LCD_dec_out( 1,  7, cross_count_R, 5 );	// 左端センサ値	〇　×　×　×　×　×　×

        						   // ライン状態保存
        						   pre_line_val = line_val;
        						   pre_sen_diff_r = sen_diff_r;
        						   pre_sen_diff_l = sen_diff_l;
        						   pre_sen_diff_ml = sen_diff_ml;
        						   pre_sen_diff_mr = sen_diff_mr;

        						   if(s_handan == 0){
  								   target_spd = 2200;
  							   }
  							   else{
  								   target_spd = 2600;
  							   }


        						   if(sen_val[3] > 400 && sen_val[4] > 400 &&sen_val[5] > 400) {
  								   cross_handan = 1;
  								   goal_count = 0;
  							   }

        						   if(s_handan == 0 && sen_val[1] > sen_ref[1] ){
        							   cross_handan = 1;
        							   s_handan = 1;
        						   }

        						   if( goal_count > 2000 ){
        							   cross_handan = 0;
        							   //cross_count_R = 0;
        						   }
        						   if(sen_val[1] > sen_ref[1]){
        							   if( 1 < cross_count_R && cross_count_R < 80 ){
        								   //cross_count_R = 0;
        								   if( cross_handan == 0 && s_handan == 1){
        									   target_spd_r = 1400;
        									   target_spd_l = 1400;
        									   HAL_Delay(300);
        									   target_spd_r = 1000;
        									   target_spd_l = 1000;
        									   HAL_Delay(300);
        									   target_spd_r = 500;
        									   target_spd_l = 500;
        									   HAL_Delay(100);
        									   break;
        								   }
        							   }else{
        								   //cross_count_R = 0;
        							   }
        						   }
        					   if(HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON){
        						   HAL_Delay( SW_WAIT );
        						   break;
        					   }


        				   }//while(  HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON );         // 中央SW入力判断
        				   //HAL_Delay( SW_WAIT );                          // チャタリング防止
        				   target_spd_r = 0;
        				   target_spd_l = 0;
        				   while( mot_spd_l != 0 || mot_spd_r != 0 );    // 完全停止待ち
        				   HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_OFF);                  // モータEnable OFF

        	  	}
        }
  	  else if( mode == 7 ){
  		//------------------------------------------------
  		// Mode7 :センサ値取得
  		//------------------------------------------------
  		if( mode_com == DISP ){  			// DISP指示の場合：モードタイトル表示
  		  LCD_print( 1, 0, "7: sen_check      " );
  		  LCD_print( 2, 0, "                " );
  		}else if( mode_com == EXEC ){		// EXEC指示の場合：モード実行
  			LCD_clear(1);
  			int syutoku_r[256],syutoku_l[256],syutoku_f[256];
  			int syutoku_mr[256],syutoku_ml[256],syutoku_goal[256],syutoku_niji[256];

  			int average_r = 0,average_f = 0,average_l = 0,average_ml =0,average_mr = 0,average_goal = 0,average_niji = 0;
  			int keka_r = 0,keka_f = 0,keka_l = 0,keka_ml =0,keka_mr = 0,keka_goal = 0,keka_niji = 0;

  			for( i=1 ; i<= 256 ; i++){

  			        syutoku_r[i-1] = sen_val[3];
  			        syutoku_l[i-1] = sen_val[5];
  			        syutoku_f[i-1] = sen_val[4];
  			        syutoku_ml[i-1] = sen_val[6];
  			        syutoku_mr[i-1] = sen_val[2];
  			        syutoku_goal[i-1] = sen_val[1];
  			        syutoku_niji[i-1] = sen_val[7];

  			        average_r += syutoku_r[i-1];
  			        average_l += syutoku_l[i-1];
  			        average_f += syutoku_f[i-1];
  			        average_ml += syutoku_ml[i-1];
  			        average_mr += syutoku_mr[i-1];
  			        average_goal += syutoku_goal[i-1];
  			        average_niji += syutoku_niji[i-1];

  			    }
  			    //if( gpio_get( SW_C ) == SW_OFF );         // 中央SW入力判断
  				HAL_Delay( 1000 );

  				keka_r = average_r/256;
  				keka_l = average_l/256;
  				keka_f = average_f/256;
  				keka_ml = average_ml/256;
  				keka_mr = average_mr/256;
  				keka_goal = average_goal/256;
  				keka_niji = average_niji/256;

  				LCD_dec_out( 2, 12, keka_goal, 3 );	// 右端センサ値	×　×　×　×　×　×　〇
  				LCD_dec_out( 1, 10, keka_mr, 3 );	// 右2つ目センサ値	×　×　×　×　×　〇　×
  				LCD_dec_out( 2,  8, keka_r, 3 );	// 中央右センサ値	×　×　×　×　〇　×　×
  				LCD_dec_out( 1,  6, keka_f, 3 );	// 中央センサ値	×　×　×　〇　×　×　×
  				LCD_dec_out( 2,  4, keka_l, 3 );	// 中央左センサ値	×　×　〇　×　×　×　×
  				LCD_dec_out( 1,  2, keka_ml, 3 );	// 左2つ目センサ値	×　〇　×　×　×　×　×
  				LCD_dec_out( 2,  0, keka_niji, 3 );	// 左端センサ値	〇　×　×　×　×　×　×

  			    HAL_Delay( 10000 );
  		}
  	}
    else if( mode == 8 ){
  			//------------------------------------------------
  			// Mode 8:走行モードthird
  			//------------------------------------------------
  			  if( mode_com == DISP ){  			// DISP指示の場合：モードタイトル表示
  				  LCD_print( 1, 0, "8: Ignore Trace" );
  				  LCD_print( 2, 0, "                " );
  				}else if( mode_com == EXEC ){		// EXEC指示の場合：モード実行
  					LCD_clear(1);
  					fifth_param();
  					sen_ref_all();

  					while(1){
  						if(HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) == SW_ON ){// 右SW入力判断
  							HAL_Delay( SW_WAIT );        // チャタリング防止
  							break;
  						}
  						if( HAL_GPIO_ReadPin(SW_DOWN_GPIO_Port, SW_DOWN_Pin) == SW_ON ){// 左SW入力判断
  							HAL_Delay( SW_WAIT );        // チャタリング防止
  							calibration();
  							//LCD_print( 1, 0, "3: calibration" );
  							HAL_Delay( 1000 );
  							break;
  						}
  					}

  					goal_count = 0;
  					s_handan = 0;
  					cross_handan = 0;
  					countdown();
  					cross_count_R = 0;

  					HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_ON);			// モータEnable
  					HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port, MOT_R_DIR_Pin, MOT_R_FWD);	// 右モータ方向指示 : 前進
  					HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port, MOT_L_DIR_Pin, MOT_L_FWD);	// 左モータ方向指示 : 前進
  					while(1){
  				   //do{

  							   // センサ値からライン状態を保存
  							   line_val = 0;                             // ライン状態リセット

  							   sen_val_ml =  sen_val[6];       // 右センサのA/D生値を0-999に加工
  							   sen_val_mr =  sen_val[2];       // 前センサのA/D生値を0-999に加工
  							   sen_val_r =  sen_val[3];       // 右センサのA/D生値を0-999に加工
  							   sen_val_f =  sen_val[4];       // 前センサのA/D生値を0-999に加工
  							   sen_val_l =  sen_val[5];       // 左センサのA/D生値を0-999に加工
  							   sen_val_gr = sen_val[1];       // 左センサのA/D生値を0-999に加工

  							   R_SEN = (float)(sen_val_r - sen_val_pre_min[3])*1000/(sen_val_pre_max[3] - sen_val_pre_min[3]);
  							   F_SEN = (float)(sen_val_f - sen_val_pre_min[4])*1000/(sen_val_pre_max[4] - sen_val_pre_min[4]);
  							   L_SEN = (float)(sen_val_l - sen_val_pre_min[5])*1000/(sen_val_pre_max[5] - sen_val_pre_min[5]);
  							   L_middle_SEN = (float)(sen_val_ml - sen_val_pre_min[6])*1000/(sen_val_pre_max[6] - sen_val_pre_min[6]);
  							   R_middle_SEN = (float)(sen_val_mr - sen_val_pre_min[2])*1000/(sen_val_pre_max[2] - sen_val_pre_min[2]);

  							   /*if( R_SEN == 1 )     line_val += 1;  // 右センサ
  							   if( F_SEN == 1 )     line_val += 2;  // 中央センサ
  							   if( L_SEN == 1 )     line_val += 4;  // 左センサ*/

  							   //PD_control(L_SEN ,F_SEN ,R_SEN );

  							   if( R_SEN > sen_ref[3] )     line_val += 2;  // 右センサ
  							   if( F_SEN > sen_ref[4] )     line_val += 4;  // 中央センサ
  							   if( L_SEN > sen_ref[5] )     line_val += 10;  // 左センサ
  							   if( R_middle_SEN > sen_ref[2])	line_val += 1;
  							   if( L_middle_SEN > sen_ref[6])	line_val += 11;


  							   if( ext_flag_r == 1 ){
  								   if(((( line_val == 1 || line_val == 3 )|| line_val == 6) || line_val == 7) || line_val == 4 ) {ext_flag_r = 0;}
  							   }
  							   else if( ext_flag_l == 1 ){
  								   if(((( line_val == 11 || line_val == 21 ) || line_val == 25) || line_val == 14) || line_val == 4){ ext_flag_l = 0;}
  							   }

  							   if( ext_flag_r == 1 ){
  								   pre_line_val = 1;
  								   line_val = 0;
  							   }
  							   else if( ext_flag_l == 1 ){
  								   pre_line_val = 11;
  								   line_val = 0;
  							   }
  							   // センサ外対応
  							   if( ( pre_line_val == 1 || pre_line_val == 30 ) && line_val == 0 )
  								   line_val = 30;   // 1つ前の状態が右のみ，または右センサ外対応で，今も全センサ無反応の場合：右センサ外対応
  							   if( ( pre_line_val == 11 || pre_line_val == 31 ) && line_val == 0 )
  								   line_val = 31;   // 1つ前の状態が左のみ，または左センサ外対応で，今も全センサ無反応の場合：左センサ外対応

  							   LCD_dec_out( 2,  1, target_spd_l, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
  							   LCD_dec_out( 2,  12, target_spd_r, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
  							   LCD_dec_out( 1,  7, cross_count_R, 2 );	// 左端センサ値	〇　×　×　×　×　×　×

  							   // ライン状態からモータ目標速度設定
  							 switch( line_val ){

  							   case 1: // line_val = 1 : XXO : 右センサのみ反応
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = (target_spd + control_r)-550;
  									   target_spd_l = target_spd;
  									   break;

  							   case 2: // line_val = 2 : XOO : 中央右＋右センサ反応
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   line_flag_r = 1;
  									   break;


  							   case 3: // line_val = 3 : XOO : 中央右＋右センサ反応
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;

  							   case 4: // line_val = 4 : OXX : 中央センサのみ反応
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 5: // line_val = 3 : XOO : 中央右＋右センサ反応
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;

  							   case 6: // line_val = 6 : OOX :中央+中央右センサ反応
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;

  							   /*case 7: // line_val = 6 : OOX :中央+中央右センサ反応
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   line_flag_l = 1;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;*/

  							   case 14: // line_val = 11 : OOX : 中央＋中央左センサ反応
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;

  							   case 15:// line_val = 12: OOX :左センサ反応
  									   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-550;
  									   break;


  							   case 16: // line_val = 16 : OXX : 中央センサのみ反応
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 28: // line_val = 16 : OXX : 中央センサのみ反応
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;

  							   case 12: // line_val = 16 : OXX : 中央センサのみ反応
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd;
  									   break;


  							   case 17:// line_val = 19: OOX : 中央左＋左センサ反応
  									   sen_diff_r = ( sen_val_f - sen_val_r )-845;
  									   d = (sen_diff_r - pre_sen_diff_r)/ pre_id_cnt;
  									   control_r = (sen_diff_r * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;


  							   case 27:// line_val = 19: OOX : 中央左＋左センサ反応
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_l * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;


  							   case 10:// line_val = 19: OOX : 中央左＋左センサ反応
  									   sen_diff_l = ( sen_val_l - sen_val_f )+845;
  									   d = (sen_diff_l - pre_sen_diff_l)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = target_spd - control_l;
  									   break;

  							   case 11:// line_val = 12: OOX :左センサ反応
  									   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l)-550;
  									   break;


  							   case 13: // line_val = 3 : XOO : 中央右＋右センサ反応
  									   sen_diff_mr = ( sen_val_r - sen_val_mr )-845;
  									   d = (sen_diff_mr - pre_sen_diff_mr)/ pre_id_cnt;
  									   control_r = (sen_diff_mr * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd + control_r;
  									   target_spd_l = target_spd;
  									   break;


  							   case 21:// line_val = 12: OOX :左センサ反応
  									   sen_diff_ml = ( sen_val_ml - sen_val_l )+845;
  									   d = (sen_diff_ml - pre_sen_diff_ml)/ pre_id_cnt;
  									   control_l = (sen_diff_ml * Kp) + (d * Kd);
  									   cross_count_R = 0;
  									   target_spd_r = target_spd;
  									   target_spd_l = (target_spd - control_l);
  									   break;

  							   case 30: // line_val = 8 : XXX : 右センサ外対応
  									   //target_spd_r = target_spd - ((F_SENP-sen_val_f)*1.5);
  									   ext_flag_r = 1;
  									   cross_count_R = 0;
  									   //trace_gain_r_ext= 0.28;
  									   target_spd_r = 100;//target_spd_r * trace_gain_r_ext;
  									   target_spd_l = target_spd;//1.1
  									   break;

  							   case 31: // line_val = 9 : XXX : 左センサ外対応
  									   ext_flag_l = 1;
  									   cross_count_R = 0;
  									   //trace_gain_l_ext = 0.28;
  									   target_spd_r = target_spd;//1.1
  									   target_spd_l = 100;//target_spd * trace_gain_l_ext;
  									   break;

  							   default:// line_val = 0,5,7 : XXX, OXO, OOO : 停止
  									   target_spd_r = 0;
  									   target_spd_l = 0;
  									   //cross_flg = 1;
  									   break;
  						   }
  							   LCD_dec_out( 2,  1, target_spd_l, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
  							   LCD_dec_out( 2,  12, target_spd_r, 4 );	// 左端センサ値	〇　×　×　×　×　×　×
  							   LCD_dec_out( 1,  7, cross_count_R, 5 );	// 左端センサ値	〇　×　×　×　×　×　×

  								   // ライン状態保存
  								   pre_line_val = line_val;
  								   pre_sen_diff_r = sen_diff_r;
  								   pre_sen_diff_l = sen_diff_l;
  								   pre_sen_diff_ml = sen_diff_ml;
  								   pre_sen_diff_mr = sen_diff_mr;

  								   if(s_handan == 0){
  									   target_spd = 2200;
  								   }
  								   else{
  									   target_spd = 2700;
  								   }


  								   if(sen_val[3] > 400 && sen_val[4] > 400 &&sen_val[5] > 400) {
  									   cross_handan = 1;
  									   goal_count = 0;
  								   }

  								   if(s_handan == 0 && sen_val[1] > sen_ref[1] ){
  									   cross_handan = 1;
  									   s_handan = 1;
  								   }

  								   if( goal_count > 2000 ){
  									   cross_handan = 0;
  									   //cross_count_R = 0;
  								   }
  								   if(sen_val[1] > sen_ref[1]){
  									   if( 1 < cross_count_R && cross_count_R < 100 ){
  										   //cross_count_R = 0;
  										   if( cross_handan == 0 && s_handan == 1){
  											   target_spd_r = 1400;
  											   target_spd_l = 1400;
  											   HAL_Delay(300);
  											   target_spd_r = 1000;
  											   target_spd_l = 1000;
  											   HAL_Delay(300);
  											   target_spd_r = 500;
  											   target_spd_l = 500;
  											   HAL_Delay(100);
  											   break;
  										   }
  									   }else{
  										   //cross_count_R = 0;
  									   }
  								   }
  							   if(HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON){
  								   HAL_Delay( SW_WAIT );
  								   break;
  							   }


  						   }//while(  HAL_GPIO_ReadPin(SW_MID_GPIO_Port, SW_MID_Pin) == SW_ON );         // 中央SW入力判断
  						   //HAL_Delay( SW_WAIT );                          // チャタリング防止
  						   target_spd_r = 0;
  						   target_spd_l = 0;
  						   while( mot_spd_l != 0 || mot_spd_r != 0 );    // 完全停止待ち
  						   HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, MOT_OFF);                  // モータEnable OFF

  				}
  		}
  }

//==============================================================================
// TIM : Timer割込み コールバック関数
//==============================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  // TIM6 : 1KHz = 1ms
  if(htim == &htim6){					// TIM6で割り込み発生
		int dev_l = 0, dev_r = 0;
		// カウンター更新
		tick_count++;

		goal_count++;
		if(tick_count == 1000){							// tick_countを1000でクリア
			tick_count = 0;
			time_count++;
		}
		if( sen_val[1] > sen_ref[1] ){
		        cross_count_R++;
		}

		switch( tick_count % 2 ){						// 割り込み毎にタスク切り替え

			// タスク0 : LCD更新
			case 0:	LCD_disp();							// LCD1文字更新
					break;

			// タスク1 : モータ速度制御
	        case 1: // 右モータ加減速処理
					dev_r = target_spd_r - mot_spd_r;	// 目標速度までの偏差
					if( dev_r > 0 )						// 偏差が正＝速度が不足＝加速
						mot_spd_r += MOT_ACC;
					else if( dev_r < 0 )				// 偏差が負＝速度が過剰＝減速
						mot_spd_r -= MOT_ACC;

	                // 左モータ加減速処理
					dev_l = target_spd_l - mot_spd_l;	// 目標速度までの偏差
					if( dev_l > 0 )						// 偏差が正＝速度が不足＝加速
						mot_spd_l += MOT_ACC;
					else if( dev_l < 0 )				// 偏差が負＝速度が過剰＝減速
						mot_spd_l -= MOT_ACC;

	                // 起動速度以下の場合の処理
	                if( mot_spd_r < MOT_SPD_INIT ){		// 右モータの現在速度が起動速度以下
	                    if( target_spd_r == 0 )			// 目標速度が0の時は現在速度を0に落とす
	                        mot_spd_r = 0;
	                    else							// 目標速度が0以外(動かそうとしている)時は起動速度に引き上げる
	                        mot_spd_r = MOT_SPD_INIT;
	                }
	                if( mot_spd_l < MOT_SPD_INIT ){		// 左モータの現在速度が起動速度以下
	                    if( target_spd_l == 0 )			// 目標速度が0の時は現在速度を0に落とす
	                        mot_spd_l = 0;
	                    else							// 目標速度が0以外(動かそうとしている)時は起動速度に引き上げる
	                        mot_spd_l = MOT_SPD_INIT;
	                }

	                // 左右モータ指示
	                mot_r_drive();						// 速度mot_spd_rで右モータ駆動
	                mot_l_drive();						// 速度mot_spd_lで左モータ駆動
	                break;

	        default:break;
		}
	}
}

//==============================================================================
// ライントレース用　センサしきい値
//==============================================================================
void sen_ref_all( void ){
	sen_ref[1] = 500;	// 右端センサ値	×　×　×　×　×　×　〇
	sen_ref[2] = 500;	// 右2つ目センサ値	×　×　×　×　×　〇　×
	sen_ref[3] = 500;	// 中央右センサ値	×　×　×　×　〇　×　×
	sen_ref[4] = 500;	// 中央センサ値	×　×　×　〇　×　×　×
	sen_ref[5] = 500;// 中央左センサ値	×　×　〇　×　×　×　×
	sen_ref[6] = 500;	// 左2つ目センサ値	×　〇　×　×　×　×　×
	sen_ref[7] = 500;	// 左端センサ値	〇　×　×　×　×　×　×
}

//==============================================================================
// ライントレース用　センサしきい値
//==============================================================================
void set_param( void ){
	    trace_gain_f    = 1.0;          // ライントレース用　制御ゲイン：中央
	    trace_gain_fr   = 0.9;          // ライントレース用　制御ゲイン：中央＋右
	    trace_gain_fl   = 0.9;          // ライントレース用　制御ゲイン：中央＋左
	    trace_gain_r    = 0.7;          // ライントレース用　制御ゲイン：右
	    trace_gain_l    = 0.7;          // ライントレース用　制御ゲイン：左
	    trace_gain_r_ext= 0.33;          // ライントレース用　制御ゲイン：右センサ外
	    trace_gain_l_ext= 0.33;          // ライントレース用　制御ゲイン：左センサ外
	    target_spd      = 1800;         // ライントレース用　直進速度

	    MOT_ACC = 50;

	    id_flag=0;
	    Kp = 0.45;
	    Kd = 0.0032;
}
//==============================================================================
// ライントレース用　センサしきい値
//==============================================================================
void second_param( void ){
	    trace_gain_f    = 1.0;          // ライントレース用　制御ゲイン：中央
	    trace_gain_fr   = 0.9;          // ライントレース用　制御ゲイン：中央＋右
	    trace_gain_fl   = 0.9;          // ライントレース用　制御ゲイン：中央＋左
	    trace_gain_r    = 0.7;          // ライントレース用　制御ゲイン：右
	    trace_gain_l    = 0.7;          // ライントレース用　制御ゲイン：左
	    trace_gain_r_ext= 0.33;          // ライントレース用　制御ゲイン：右センサ外
	    trace_gain_l_ext= 0.33;          // ライントレース用　制御ゲイン：左センサ外
	    target_spd      = 2200;         // ライントレース用　直進速度

	    MOT_ACC = 60;

	    id_flag=0;
	    Kp = 0.3;
	    Kd = 0.0032;
}
//==============================================================================
// ライントレース用　センサしきい値
//==============================================================================
void third_param( void ){
	    trace_gain_f    = 1.0;          // ライントレース用　制御ゲイン：中央
	    trace_gain_fr   = 0.9;          // ライントレース用　制御ゲイン：中央＋右
	    trace_gain_fl   = 0.9;          // ライントレース用　制御ゲイン：中央＋左
	    trace_gain_r    = 0.7;          // ライントレース用　制御ゲイン：右
	    trace_gain_l    = 0.7;          // ライントレース用　制御ゲイン：左
	    trace_gain_r_ext= 0.28;          // ライントレース用　制御ゲイン：右センサ外
	    trace_gain_l_ext= 0.28;          // ライントレース用　制御ゲイン：左センサ外
	    target_spd      = 2400;         // ライントレース用　直進速度

	    MOT_ACC = 60;

	    id_flag=0;
	    Kp = 0.6;
	    Kd = 0.0028;
}

//==============================================================================
// ライントレース用　センサしきい値
//==============================================================================
void fourth_param( void ){
	    trace_gain_f    = 1.0;          // ライントレース用　制御ゲイン：中央
	    trace_gain_fr   = 0.9;          // ライントレース用　制御ゲイン：中央＋右
	    trace_gain_fl   = 0.9;          // ライントレース用　制御ゲイン：中央＋左
	    trace_gain_r    = 0.7;          // ライントレース用　制御ゲイン：右
	    trace_gain_l    = 0.7;          // ライントレース用　制御ゲイン：左
	    trace_gain_r_ext= 0.33;          // ライントレース用　制御ゲイン：右センサ外
	    trace_gain_l_ext= 0.33;          // ライントレース用　制御ゲイン：左センサ外
	    target_spd      = 2600;         // ライントレース用　直進速度

	    MOT_ACC = 60;

	    id_flag=0;
	    Kp = 0.65;
	    Kd = 0.0032;
}

//==============================================================================
// ライントレース用　センサしきい値
//==============================================================================
void fifth_param( void ){
	    trace_gain_f    = 1.0;          // ライントレース用　制御ゲイン：中央
	    trace_gain_fr   = 0.9;          // ライントレース用　制御ゲイン：中央＋右
	    trace_gain_fl   = 0.9;          // ライントレース用　制御ゲイン：中央＋左
	    trace_gain_r    = 0.7;          // ライントレース用　制御ゲイン：右
	    trace_gain_l    = 0.7;          // ライントレース用　制御ゲイン：左
	    trace_gain_r_ext= 0.33;          // ライントレース用　制御ゲイン：右センサ外
	    trace_gain_l_ext= 0.33;          // ライントレース用　制御ゲイン：左センサ外
	    target_spd      = 2700;         // ライントレース用　直進速度

	    MOT_ACC = 50;

	    id_flag=0;
	    Kp = 0.45;
	    Kd = 0.0028;
}

//==============================================================================
// ADC1&2 : DMA-ADCコールバック関数
//==============================================================================
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)		// DMA終了後に呼び出される関数
{
	if( hadc == &hadc1 ){
		sen_val[1] = adc1_val[0];    	// 右端センサ値	×　×　×　×　×　×　〇
		sen_val[2] = adc1_val[1];   	// 右2つ目センサ値	×　×　×　×　×　〇　×
		sen_val[3] = adc1_val[2];   	// 中央右センサ値	×　×　×　×　〇　×　×
	}
	if( hadc == &hadc2 ){
		sen_val[4] = adc2_val[0];    	// 中央センサ値	×　×　×　〇　×　×　×
		sen_val[5] = adc2_val[1];   	// 中央左センサ値	×　×　〇　×　×　×　×
		sen_val[6] = adc2_val[2];   	// 左2つ目センサ値	×　〇　×　×　×　×　×
		sen_val[7] = adc2_val[3];   	// 左端センサ値	〇　×　×　×　×　×　×
	}
}


//==============================================================================
// モータ駆動関数 : 右モータ : 速度mot_spd_rで駆動
//==============================================================================
void mot_r_drive( void )
{
    if( mot_spd_r <= 0 ){
        // モータ停止指示
    	HAL_TIM_PWM_Stop_IT( &htim3, TIM_CHANNEL_4 );			// 右モータOFF
    }else{
        // モータ駆動指示
    	int pwm_period = 64000 / mot_spd_r - 1;					// prescalerに設定する周期計算
        // 周期 = APB1(64MHz) / CounterPeriod(1000) / prescaler(mot_spd) - 1
        __HAL_TIM_SET_PRESCALER(&htim3, pwm_period);			// prescalerを設定
        __HAL_TIM_SET_COMPARE( &htim3, TIM_CHANNEL_4, 500-1);	// dutyを設定
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);				// TIM3 タイマスタート
    }
}

//==============================================================================
// モータ駆動関数 : 左モータ : 速度mot_spd_lで駆動
//==============================================================================
void mot_l_drive( void )
{
    if( mot_spd_l <= 0 ){
        // モータ停止指示
    	HAL_TIM_PWM_Stop_IT( &htim2, TIM_CHANNEL_3 );			// 左モータOFF
    }else{
        // モータ駆動指示
    	int pwm_period = 64000 / mot_spd_l - 1;					// prescalerに設定する周期計算
        // 周期 = APB1(64MHz) / CounterPeriod(1000) / prescaler(mot_spd) - 1
        __HAL_TIM_SET_PRESCALER(&htim2, pwm_period);			// prescalerを設定
        __HAL_TIM_SET_COMPARE( &htim2, TIM_CHANNEL_3, 500-1);	// dutyを設定
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);				// TIM2 タイマスタート
    }
}

//==============================================================================
// ゴール音
//==============================================================================
void finish( void ){
    Beep( TONE_SO, 6, 150 );            // 起動音:音高6のソ, 150ms
    Beep(       0, 0, 150 );            // 休符, 150ms
    Beep( TONE_SO, 6, 150 );            // 起動音:音高6のソ, 150ms
    Beep(       0, 0,  50 );            // 休符, 50ms
    Beep( TONE_DO, 7, 500 );            // 起動音:音高7のド, 500ms
}

//==============================================================================
// ライントレース用　センサしきい値
//==============================================================================
void calibration( void ){
	for( i=0; i<200; i++){
		HAL_Delay(20);

	        //最小値の取得
	        if( sen_val[1] < sen_val_pre_min[1] ){        //右センサ
	        	sen_val_pre_min[1]= sen_val[1] ;
	        }
	        if( sen_val[2] < sen_val_pre_min[2] ){        //右センサ
	        	sen_val_pre_min[2]= sen_val[2] ;
			}
	        if( sen_val[3] < sen_val_pre_min[3] ){        //右センサ
	        	sen_val_pre_min[3]= sen_val[3] ;
			}
	        if( sen_val[4] < sen_val_pre_min[4] ){        //右センサ
	        	sen_val_pre_min[4]= sen_val[4] ;
			}
	        if( sen_val[5] < sen_val_pre_min[5] ){        //右センサ
	        	sen_val_pre_min[5]= sen_val[5] ;
			}
	        if( sen_val[6] < sen_val_pre_min[6] ){        //右センサ
	        	sen_val_pre_min[6]= sen_val[6] ;
			}
	        if( sen_val[7] < sen_val_pre_min[7] ){        //右センサ
	        	sen_val_pre_min[7]= sen_val[7] ;
			}
	        //最大値の取得
	        if( sen_val[1] > sen_val_pre_max[1] ){        //右センサ
	        	sen_val_pre_max[1]= sen_val[1] ;
	        }
	        if( sen_val[2] > sen_val_pre_max[2] ){        //右センサ
				sen_val_pre_max[2]= sen_val[2] ;
			}
	        if( sen_val[3] > sen_val_pre_max[3] ){        //右センサ
				sen_val_pre_max[3]= sen_val[3] ;
			}
	        if( sen_val[4] > sen_val_pre_max[4] ){        //右センサ
				sen_val_pre_max[4]= sen_val[4] ;
			}
	        if( sen_val[5] > sen_val_pre_max[5] ){        //右センサ
				sen_val_pre_max[5]= sen_val[5] ;
			}
	        if( sen_val[6] > sen_val_pre_max[6] ){        //右センサ
				sen_val_pre_max[6]= sen_val[6] ;
			}
	        if( sen_val[7] > sen_val_pre_max[7] ){        //右センサ
				sen_val_pre_max[7]= sen_val[7] ;
			}
	    }
	 Beep( TONE_DO, 7, 500 );            // 起動音:音高7のド, 500ms
}

//==============================================================================
// ゴール音
//==============================================================================
void countdown( void ){
    Beep( TONE_DO, 7, 500 );            // 起動音:音高7のド, 500ms
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
