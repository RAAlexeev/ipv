/*
 * flashLoader.h
 *
 *  Created on: 9 ���. 2020 �.
 *      Author: alekseev
 *      �� �������� ������������ � �������������
 */

#ifndef INC_FLASHLOADER_H_
#define INC_FLASHLOADER_H_


void flashLoaderStartIfNeed();
void rebootToFlashLoader( void );
bool parseFlashLoaderReq(uint8_t * buf);

#endif /* INC_FLASHLOADER_H_ */
