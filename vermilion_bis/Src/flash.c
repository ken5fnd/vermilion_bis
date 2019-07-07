#include "maze.h"
#include "main.h"

void BackUpWallData(enum DataFlash _Mode) { //壁情報をデータフラッシュに保存する

	if (_Mode == Write) {			//書き込み
		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t PageError = 0;
		HAL_FLASH_Unlock();
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		EraseInitStruct.Sector = FLASH_SECTOR_1;
		EraseInitStruct.NbSectors = 1;
		HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

		for (int i = 0; i < 16; i++) {
			HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, 0x08004000 + i * 2,
					wall_ver[i]);

		}
		for (int i = 0; i < 16; i++) {
			HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, 0x08004040 + i * 2,
					wall_hor[i]);

		}
		for (int i = 0; i < 16; i++) {
			HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, 0x08004080 + i * 2,
					wall_read_ver[i]);

		}
		for (int i = 0; i < 16; i++) {
			HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, 0x080040C0 + i * 2,
					wall_read_hor[i]);

		}
		for (int i = 0; i < 16; i++) {
			for (int j = 0; j < 16; j++) {
				HAL_FLASH_Program(TYPEPROGRAM_BYTE, 0x08004100 + i * 16 + j,
						Flag_cellfound[i][j]);

			}
		}
		HAL_FLASH_Lock();
	} else if (_Mode == Read) {			//読み込み
		for (int i = 0; i < 16; i++) {
			wall_ver[i] = *(uint16_t*) (0x08004000 + i * 2);
		}
		for (int i = 0; i < 16; i++) {
			wall_hor[i] = *(uint16_t*) (0x08004040 + i * 2);
		}
		for (int i = 0; i < 16; i++) {
			wall_read_ver[i] = *(uint16_t*) (0x08004080 + i * 2);
		}
		for (int i = 0; i < 16; i++) {
			wall_read_hor[i] = *(uint16_t*) (0x080040C0 + i * 2);
		}
		for (int i = 0; i < 16; i++) {
			for (int j = 0; j < 16; j++) {
				Flag_cellfound[i][j] = *(uint8_t*) (0x08004100 + i * 16 + j);
			}
		}
	}
}

