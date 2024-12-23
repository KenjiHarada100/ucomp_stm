/*
 * LCD.c
 *
 *  Created on: Nov 14, 2024
 *      Author: 24eo0118
 */

#include "main.h"
#include "LCD.h"

// LCDの初期化関数
void init_lcd(void) {
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);  // RSピンをリセット
    HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);  // RWピンをリセット
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);    // Eピンをリセット
    HAL_Delay(1000);  // 1秒待機

    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);  // Eピンをセット
    GPIOC->ODR |= (LCD_INIT4B & 0x00F0) << 8;  // 上位4ビットを設定
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);  // Eピンをリセット

    // LCDの基本設定コマンド送信
    write_lcd_data(LCD_FCSET4B, LCD_CMD);
    write_lcd_data(LCD_FCSET4B, LCD_CMD);
    write_lcd_data(LCD_DISP_OFF, LCD_CMD);
    write_lcd_data(LCD_CLAR, LCD_CMD);
    write_lcd_data(LCD_ENTSET, LCD_CMD);
    write_lcd_data(LCD_DISP_CUR, LCD_CMD);
}

// LCDにデータを送信する関数
void write_lcd_data(uint8_t data, uint8_t rs) {
    uint16_t upper_data;
    uint16_t lower_data;
    uint16_t temp;

    // 上位データと下位データの設定
    upper_data = data * 0x0100 & 0xF000;
    lower_data = data * 0x1000 & 0xF000;

    HAL_Delay(30);  // 短い待機

    // データモードかコマンドモードかを設定
    if (rs) {
        HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    }

    HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);  // 書き込みモードに設定

    // 上位データの送信
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
    temp = GPIOC->IDR & 0x0FFF;
    GPIOC->ODR = (upper_data & 0xF000) | temp;
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);

    // 下位データの送信
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
    temp = GPIOC->IDR & 0x0FFF;
    GPIOC->ODR = (lower_data & 0xF000) | temp;
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
}

// 文字列をLCDに表示する関数
void lcd_puts(uint8_t *str) {
    while (*str) {
        write_lcd_data(*str, LCD_DAT);
        str++;
    }
}

// LCDのカーソル位置を設定する関数
void lcd_xy(uint8_t x, uint8_t y) {
    uint8_t adr;
    adr = ((x - 1) + (y - 1) * 0x40) | 0x80;
    write_lcd_data(adr, LCD_CMD);
}

uint8_t str_count(uint8_t str[])
{
	uint8_t len = 0;
	while(str[len] != '\0'){
		len++;
	}
	return len;
}

void lcd_puts_Right(uint8_t *str) {
	uint8_t i;
	uint8_t len;

	len = str_count(str); //受け取った文字列の文字数をカウント
	for (i = 0; i < 16 - len; i++) //１６ー文字数分のスペースを表示する。
	{
		write_lcd_data(' ', LCD_DAT);
	}
    while (*str) {
        write_lcd_data(*str, LCD_DAT);
        str++;
    }
}
