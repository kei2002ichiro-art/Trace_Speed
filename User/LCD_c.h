//---------------------------------------------------------
//
// LCD ヘッダーファイル
//
//---------------------------------------------------------

//------------------------------------------------
// 定義
//------------------------------------------------

#define DEVICE      0x3e
#define COM_ADD     0x00
#define DAT_ADD     0x40
#define ALL_CLR     0x01
#define ADD_SET     0x80
#define L1TOP       0x00
#define L2TOP       0x40
#define LCD_BAUDRATE        (700 * 1000)

//I2C_HandleTypeDef hi2c1;

//------------------------------------------------
// 変数
//------------------------------------------------

unsigned char lcd_line[ 2 ][ 16 ];
int disp_cnt = 0;

//------------------------------------------------
// 文字表示
//------------------------------------------------
void LCD_write(char lcd_addr, char lcd_data)
{
    unsigned char lcd_send_data[2];
    lcd_send_data[ 0 ] = lcd_addr;
    lcd_send_data[ 1 ] = lcd_data;
    HAL_I2C_Master_Transmit(&hi2c1, 0x7c, lcd_send_data, 2, 1000);
}

//------------------------------------------------
// 文字表示
//------------------------------------------------

void LCD_print( int line, int LCD_pt, char *pt )
{
    if( line == 1 || line == 2 )
    	while( *pt ) lcd_line[ line - 1 ][ LCD_pt++ ] = *pt++;
}

//------------------------------------------------
// 文字更新
//------------------------------------------------

void LCD_disp( void )
{
    disp_cnt ++;
    if( disp_cnt == 35 )
        disp_cnt = 1;

    char disp_command;
    char disp_data;
    if( disp_cnt >= 1 && disp_cnt <= 16 ){
        disp_command = DAT_ADD;
        disp_data = lcd_line[ 0 ][ disp_cnt - 1 ];
    }
    else if( disp_cnt == 17 ){
        disp_command = COM_ADD;
        disp_data = ADD_SET + L2TOP;
    }

    else if( disp_cnt >= 18 && disp_cnt <= 33 ){
        disp_command = DAT_ADD;
        disp_data = lcd_line[ 1 ][ disp_cnt - 18 ];
    }

    else if( disp_cnt == 34 ){
        disp_command = COM_ADD;
        disp_data = ADD_SET + L1TOP;
    }


    LCD_write( disp_command, disp_data );
}

//------------------------------------------------
// LCD　オールクリア
//------------------------------------------------
void LCD_clear(int line)
{
    if( line == 1 || line == 2 )
    	for( int i = 0; i < 16; i++ )
	        lcd_line[ line - 1 ][ i ] = ' ';
    else
    	for( int j = 0; j < 2; j++ )
	    	for( int i = 0; i < 16; i++ )
	    	    lcd_line[ j ][ i ] = ' ';
}

//------------------------------------------------
// LCD 初期化
//------------------------------------------------

void LCD_init( void )
{
		char orders[ 9 ] = { 0x38, 0x39, 0x14, 0x73, 0x56, 0x6c, 0x38, 0x01, 0x0c };

    HAL_Delay(40);
    for( int i = 0; i < 6; i++ ){
        LCD_write( COM_ADD, orders[i] );
        HAL_Delay(1);
    }
    HAL_Delay(200);
    for( int i = 6; i < 9; i++ ){
        LCD_write( COM_ADD, orders[i] );
        HAL_Delay(1);
    }
    LCD_clear( 0 );
    }


//------------------------------------------------
// LCD 変数表示
//------------------------------------------------

void LCD_dec_out( int line, int pt, int x, int n )
{
    int y;
    if( line == 1 || line == 2 ){
    	if( n > 4 ){ y = 10000; lcd_line[ line - 1 ][ pt++ ] = '0' + x / y; x = x % y; }
	    if( n > 3 ){ y = 1000;  lcd_line[ line - 1 ][ pt++ ] = '0' + x / y; x = x % y; }
	    if( n > 2 ){ y = 100;   lcd_line[ line - 1 ][ pt++ ] = '0' + x / y; x = x % y; }
	    if( n > 1 ){ y = 10;    lcd_line[ line - 1 ][ pt++ ] = '0' + x / y; x = x % y; }
	                            lcd_line[ line - 1 ][ pt++ ] = '0' + x;
    }
}

