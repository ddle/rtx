
extern void serial_out_lcd( uint8_t line, uint8_t *buff, uint16_t c);

extern void shift_command_line(uint8_t sel, uint8_t line);


extern void shift_command_trail(uint8_t sel);


extern uint8_t *shift_data_lcd (uint8_t *data);
extern void str2bmp ( uint8_t lp, uint8_t *str, uint8_t *buff);
