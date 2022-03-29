/***************************************************************************
 * 	 WCH ARM mcu :CH32F10X ;CH32F20X                                       *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/
 
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* ch32x register locations */

#define FLASH_REG_BASE_B0 0x40022000
#define FLASH_REG_BASE_B1 0x40022040

#define ch32_FLASH_ACR     0x00
#define ch32_FLASH_KEYR    0x04
#define ch32_FLASH_OPTKEYR 0x08
#define ch32_FLASH_SR      0x0C
#define ch32_FLASH_CR      0x10
#define ch32_FLASH_AR      0x14
#define ch32_FLASH_OBR     0x1C
#define ch32_FLASH_WRPR    0x20
#define CH32_FLASH_MODEKEYP 0x24 //chf103�����Ĵ���

/* TODO: Check if code using these really should be hard coded to bank 0.
 * There are valid cases, on dual flash devices the protection of the
 * second bank is done on the bank0 reg's. */
#define ch32_FLASH_ACR_B0     0x40022000
#define ch32_FLASH_KEYR_B0    0x40022004
#define ch32_FLASH_OPTKEYR_B0 0x40022008
#define ch32_FLASH_SR_B0      0x4002200C
#define ch32_FLASH_CR_B0      0x40022010
#define ch32_FLASH_AR_B0      0x40022014
#define ch32_FLASH_OBR_B0     0x4002201C
#define ch32_FLASH_WRPR_B0    0x40022020

/* option byte location */

#define ch32_OB_RDP		0x1FFFF800
#define ch32_OB_USER		0x1FFFF802
#define ch32_OB_DATA0		0x1FFFF804
#define ch32_OB_DATA1		0x1FFFF806
#define ch32_OB_WRP0		0x1FFFF808
#define ch32_OB_WRP1		0x1FFFF80A
#define ch32_OB_WRP2		0x1FFFF80C
#define ch32_OB_WRP3		0x1FFFF80E

/* FLASH_CR register bits */

#define FLASH_PG			(1 << 0)
#define FLASH_PER			(1 << 1)
#define FLASH_MER			(1 << 2)
#define FLASH_OPTPG			(1 << 4)
#define FLASH_OPTER			(1 << 5)
#define FLASH_STRT			(1 << 6)
#define FLASH_LOCK			(1 << 7)
#define FLASH_OPTWRE		(1 << 9)
#define FLASH_OBL_LAUNCH	(1 << 13)	/* except ch32f1x series */


#define FLASH_PAGE_PROGRAM	  0x00010000	
#define FLASH_PAGE_ERASE		  0x00020000	
#define FLASH_STD_PAGE_ERASE  0x00000002  
#define FLASH_STD_PAGE_PRG    0x00000001  
#define FLASH_BUF_LOAD			  0x00040000	
#define FLASH_BUF_RTS				  0x00080000	



/* FLASH_SR register bits */

#define FLASH_BSY		(1 << 0)
#define FLASH_PGERR		(1 << 2)
#define FLASH_WRPRTERR	(1 << 4)
#define FLASH_EOP		(1 << 5)

/* ch32_FLASH_OBR bit definitions (reading) */

#define OPT_ERROR		0
#define OPT_READOUT		1
#define OPT_RDWDGSW		2
#define OPT_RDRSTSTOP	3
#define OPT_RDRSTSTDBY	4
#define OPT_BFB2		5	/* dual flash bank only */

/* register unlock keys */

#define KEY1			0x45670123
#define KEY2			0xCDEF89AB

/* timeout values */

#define FLASH_WRITE_TIMEOUT 10
#define FLASH_ERASE_TIMEOUT 100

struct ch32x_options {
	uint8_t rdp;
	uint8_t user;
	uint16_t data;
	uint32_t protection;
};

struct ch32x_flash_bank {
	struct ch32x_options option_bytes;
	int ppage_size;
	int probed;

	bool has_dual_banks;
	/* used to access dual flash bank ch32xl */
	bool can_load_options;
	uint32_t register_base;
	uint8_t default_rdp;
	int user_data_offset;
	int option_offset;
	uint32_t user_bank_size;
};

uint8_t armchip;

extern int wlink_armerase(void);
extern int wlink_armwrite(const uint8_t *buffer,uint32_t offset, uint32_t count);
extern int noloadflag;
static int ch32x_mass_erase(struct flash_bank *bank);
static int ch32x_get_device_id(struct flash_bank *bank, uint32_t *device_id);
static int ch32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t count);
extern int wlink_armcheckprotect(void);
extern void wlink_sendchip(void);
	


/* flash bank ch32x <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(ch32x_flash_bank_command)
{
	struct ch32x_flash_bank *ch32x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ch32x_info = malloc(sizeof(struct ch32x_flash_bank));

	bank->driver_priv = ch32x_info;
	ch32x_info->probed = 0;
	ch32x_info->has_dual_banks = false;
	ch32x_info->can_load_options = false;
	ch32x_info->register_base = FLASH_REG_BASE_B0;
	ch32x_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static inline int ch32x_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;
	return reg + ch32x_info->register_base;
}

static inline int ch32x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(target, ch32x_get_flash_reg(bank, ch32_FLASH_SR), status);
}

static int ch32x_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = ch32x_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & FLASH_BSY) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (status & FLASH_WRPRTERR) {
		LOG_ERROR("ch32x device protected");
		retval = ERROR_FAIL;
	}

	if (status & FLASH_PGERR) {
		LOG_ERROR("ch32x device programming failed");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & (FLASH_WRPRTERR | FLASH_PGERR)) {
		/* If this operation fails, we ignore it and report the original
		 * retval
		 */
		target_write_u32(target, ch32x_get_flash_reg(bank, ch32_FLASH_SR),
				FLASH_WRPRTERR | FLASH_PGERR);
	}
	return retval;
}

static int ch32x_check_operation_supported(struct flash_bank *bank)
{
	
	
	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;

	/* if we have a dual flash bank device then
	 * we need to perform option byte stuff on bank0 only */
	if (ch32x_info->register_base != FLASH_REG_BASE_B0) {
		LOG_ERROR("Option Byte Operation's must use bank0");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int ch32x_read_options(struct flash_bank *bank)
{
	
	
	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t option_bytes;
	int retval;

	/* read user and read protection option bytes */
	retval = target_read_u32(target, ch32_OB_RDP, &option_bytes);
	if (retval != ERROR_OK)
		return retval;

	ch32x_info->option_bytes.rdp = option_bytes & 0xFF;
	ch32x_info->option_bytes.user = (option_bytes >> 16) & 0xFF;

	/* read user data option bytes */
	retval = target_read_u32(target, ch32_OB_DATA0, &option_bytes);
	if (retval != ERROR_OK)
		return retval;

	ch32x_info->option_bytes.data = ((option_bytes >> 8) & 0xFF00) | (option_bytes & 0xFF);

	/* read write protection option bytes */
	retval = target_read_u32(target, ch32_OB_WRP0, &option_bytes);
	if (retval != ERROR_OK)
		return retval;

	ch32x_info->option_bytes.protection = ((option_bytes >> 8) & 0xFF00) | (option_bytes & 0xFF);

	retval = target_read_u32(target, ch32_OB_WRP2, &option_bytes);
	if (retval != ERROR_OK)
		return retval;

	ch32x_info->option_bytes.protection |= (((option_bytes >> 8) & 0xFF00) | (option_bytes & 0xFF)) << 16;

	return ERROR_OK;
}

static int ch32x_erase_options(struct flash_bank *bank)
{
	
	
	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;
	struct target *target = bank->target;

	/* read current options */
	ch32x_read_options(bank);

	/* unlock flash registers */
	int retval = target_write_u32(target, ch32_FLASH_KEYR_B0, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, ch32_FLASH_KEYR_B0, KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* unlock option flash registers */
	retval = target_write_u32(target, ch32_FLASH_OPTKEYR_B0, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, ch32_FLASH_OPTKEYR_B0, KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* erase option bytes */
	retval = target_write_u32(target, ch32_FLASH_CR_B0, FLASH_OPTER | FLASH_OPTWRE);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, ch32_FLASH_CR_B0, FLASH_OPTER | FLASH_STRT | FLASH_OPTWRE);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* clear read protection option byte
	 * this will also force a device unlock if set */
	ch32x_info->option_bytes.rdp = ch32x_info->default_rdp;

	return ERROR_OK;
}

static int ch32x_write_options(struct flash_bank *bank)
{
	
	
	struct ch32x_flash_bank *ch32x_info = NULL;
	struct target *target = bank->target;

	ch32x_info = bank->driver_priv;

	/* unlock flash registers */
	int retval = target_write_u32(target, ch32_FLASH_KEYR_B0, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, ch32_FLASH_KEYR_B0, KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* unlock option flash registers */
	retval = target_write_u32(target, ch32_FLASH_OPTKEYR_B0, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, ch32_FLASH_OPTKEYR_B0, KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* program option bytes */
	retval = target_write_u32(target, ch32_FLASH_CR_B0, FLASH_OPTPG | FLASH_OPTWRE);
	if (retval != ERROR_OK)
		return retval;

	uint8_t opt_bytes[16];

	target_buffer_set_u16(target, opt_bytes, ch32x_info->option_bytes.rdp);
	target_buffer_set_u16(target, opt_bytes + 2, ch32x_info->option_bytes.user);
	target_buffer_set_u16(target, opt_bytes + 4, ch32x_info->option_bytes.data & 0xff);
	target_buffer_set_u16(target, opt_bytes + 6, (ch32x_info->option_bytes.data >> 8) & 0xff);
	target_buffer_set_u16(target, opt_bytes + 8, ch32x_info->option_bytes.protection & 0xff);
	target_buffer_set_u16(target, opt_bytes + 10, (ch32x_info->option_bytes.protection >> 8) & 0xff);
	target_buffer_set_u16(target, opt_bytes + 12, (ch32x_info->option_bytes.protection >> 16) & 0xff);
	target_buffer_set_u16(target, opt_bytes + 14, (ch32x_info->option_bytes.protection >> 24) & 0xff);

	retval = ch32x_write_block(bank, opt_bytes, ch32_OB_RDP, sizeof(opt_bytes) / 2);
	if (retval != ERROR_OK) {
		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			LOG_ERROR("working area required to erase options bytes");
		return retval;
	}

	retval = target_write_u32(target, ch32_FLASH_CR_B0, FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int ch32x_protect_check(struct flash_bank *bank)
{
	

	struct target *target = bank->target;
	uint32_t protection;

	int retval = ch32x_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	/* medium density - each bit refers to a 4 sector protection block
	 * high density - each bit refers to a 2 sector protection block
	 * bit 31 refers to all remaining sectors in a bank */
	retval = target_read_u32(target, ch32_FLASH_WRPR_B0, &protection);
	if (retval != ERROR_OK)
		return retval;

	for (int i = 0; i < bank->num_prot_blocks; i++)
		bank->prot_blocks[i].is_protected = (protection & (1 << i)) ? 0 : 1;

	return ERROR_OK;
}

static int ch32x_erase(struct flash_bank *bank, int first, int last)
{	
 if(armchip)
 {
	if(noloadflag)
		return ERROR_OK;
				
	    int ret=wlink_armerase();
		
		return ret;
			
 }	
}

static int ch32x_protect(struct flash_bank *bank, int set, int first, int last)
{
	
	struct target *target = bank->target;
	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = ch32x_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32x_erase_options(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("ch32x failed to erase options");
		return retval;
	}

	for (int i = first; i <= last; i++) {
		if (set)
			ch32x_info->option_bytes.protection &= ~(1 << i);
		else
			ch32x_info->option_bytes.protection |= (1 << i);
	}

	return ch32x_write_options(bank);
}

static int ch32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t count)
{
	return ERROR_OK;
}

static int ch32x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
 if(armchip)
 {
		if(noloadflag)
				return ERROR_OK;			
		int ret=wlink_armwrite(buffer,bank->base + offset,count);				
		return ret;				
 }

}

static int ch32x_get_device_id(struct flash_bank *bank, uint32_t *device_id)
{
	/* This check the device CPUID core register to detect
	 * the M0 from the M3 devices. */
 
	struct target *target = bank->target;
	uint32_t cpuid, device_id_register = 0;
    uint32_t testid=0;
    uint32_t tmp,tmp1,tmp2=0;

	target_read_u32(target, 0x1ffff884, &testid);			
  	if(((testid>>16)==0x2000)||((testid>>16)==0x1000)||((testid>>16)==0x3000)){
  		target_read_u32(target, 0x1ffff7e8, &tmp);
  		target_read_u32(target, 0x1ffff8a0, &tmp1);
  		if(tmp==tmp1){
  				target_read_u32(target, 0x1ffff880, &tmp2);
  			  if(tmp2==0xdc78fe34)
  			    armchip=1;
  				*device_id=0x20000410;
  				wlink_sendchip();
  				return ERROR_OK;
  		}
  	
  	}
  	target_read_u32(target, 0xe000edfc, &testid);
 
  	target_read_u32(target, 0xe000edf0, &testid);
  	
  	target_read_u32(target, 0x1ffff704, &testid);
  	
  	if(((testid>>20)==0x203)||((testid>>20)==0x205)||((testid>>20)==0x207)||((testid>>20)==0x208)){	
  		 	armchip=2;
  			*device_id=0x20000410;
  			wlink_sendchip();
  			return ERROR_OK;
		}

	return ERROR_FAIL;
}

static int ch32x_get_flash_size(struct flash_bank *bank, uint16_t *flash_size_in_kb)
{	
	struct target *target = bank->target;
	uint32_t cpuid, flash_size_reg;
    uint32_t temp;
	int retval = target_read_u32(target, 0x1ffff7e0, flash_size_in_kb);	
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int ch32x_probe(struct flash_bank *bank)
{	
	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;
	uint16_t flash_size_in_kb;
	uint16_t max_flash_size_in_kb;
	uint32_t device_id;
	int page_size;
	uint32_t base_address = 0x08000000;
    uint32_t rid=0;
	ch32x_info->probed = 0;
	ch32x_info->register_base = FLASH_REG_BASE_B0;
	ch32x_info->user_data_offset = 10;
	ch32x_info->option_offset = 0;

	/* default factory read protection level 0 */
	ch32x_info->default_rdp = 0xA5;	
	
  int retval = ch32x_get_device_id(bank, &device_id);
	if (retval != ERROR_OK)
		return retval;
	
	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);
	rid=device_id & 0xfff ;
	/* set page size, protection granularity and max flash size depending on family */
	switch (device_id & 0xfff) {
	case 0x410: /* medium density */
		page_size = 1024;
		ch32x_info->ppage_size = 4;
		max_flash_size_in_kb = 512;
		break;
	
	default:
		LOG_WARNING("Cannot identify target as a ch32 family.");
		return ERROR_FAIL;
	}

	/* get flash size from target. */
	retval = ch32x_get_flash_size(bank, &flash_size_in_kb);

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		//LOG_WARNING("ch32 flash size failed, probe inaccurate - assuming %dk flash",
			//max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}

	if (ch32x_info->has_dual_banks) {
		/* split reported size into matching bank */
		if (bank->base != 0x08080000) {
			/* bank 0 will be fixed 512k */
			flash_size_in_kb = 512;
		} else {
			flash_size_in_kb -= 512;
			/* bank1 also uses a register offset */
			ch32x_info->register_base = FLASH_REG_BASE_B1;
			base_address = 0x08080000;
		}
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */


	LOG_INFO("flash size = %dkbytes", flash_size_in_kb);

	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* calculate numbers of pages */
	int num_pages = flash_size_in_kb * 1024 / page_size;

	/* check that calculation result makes sense */
	assert(num_pages > 0);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	if (bank->prot_blocks) {
		free(bank->prot_blocks);
		bank->prot_blocks = NULL;
	}

	bank->base = base_address;
	bank->size = (num_pages * page_size);

	bank->num_sectors = num_pages;
	bank->sectors = alloc_block_array(0, page_size, num_pages);
	if (!bank->sectors)
		return ERROR_FAIL;

	/* calculate number of write protection blocks */
	int num_prot_blocks = num_pages / ch32x_info->ppage_size;
	if (num_prot_blocks > 32)
		num_prot_blocks = 32;

	bank->num_prot_blocks = num_prot_blocks;
	bank->prot_blocks = alloc_block_array(0, ch32x_info->ppage_size * page_size, num_prot_blocks);
	if (!bank->prot_blocks)
		return ERROR_FAIL;

	if (num_prot_blocks == 32)
		bank->prot_blocks[31].size = (num_pages - (31 * ch32x_info->ppage_size)) * page_size;

	ch32x_info->probed = 1;

	return ERROR_OK;
}

static int ch32x_auto_probe(struct flash_bank *bank)
{
	
	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;
	if (ch32x_info->probed)
		return ERROR_OK;
	return ch32x_probe(bank);
}

#if 0
COMMAND_HANDLER(ch32x_handle_part_id_command)
{
	return ERROR_OK;
}
#endif

static int get_ch32x_info(struct flash_bank *bank, char *buf, int buf_size)
{


	return ERROR_OK;
}

COMMAND_HANDLER(ch32x_handle_lock_command)
{
	struct target *target = NULL;
	struct ch32x_flash_bank *ch32x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	ch32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = ch32x_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	if (ch32x_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "ch32x failed to erase options");
		return ERROR_OK;
	}

	/* set readout protection */
	ch32x_info->option_bytes.rdp = 0;

	if (ch32x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "ch32x failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD, "ch32x locked");

	return ERROR_OK;
}

COMMAND_HANDLER(ch32x_handle_unlock_command)
{
	struct target *target = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = ch32x_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	if (ch32x_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "ch32x failed to erase options");
		return ERROR_OK;
	}

	if (ch32x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "ch32x failed to unlock device");
		return ERROR_OK;
	}

	command_print(CMD, "ch32x unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
}

COMMAND_HANDLER(ch32x_handle_options_read_command)
{
	uint32_t optionbyte, protection;
	struct target *target = NULL;
	struct ch32x_flash_bank *ch32x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	ch32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = ch32x_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = target_read_u32(target, ch32_FLASH_OBR_B0, &optionbyte);
	if (retval != ERROR_OK)
		return retval;

	uint16_t user_data = optionbyte >> ch32x_info->user_data_offset;

	retval = target_read_u32(target, ch32_FLASH_WRPR_B0, &protection);
	if (retval != ERROR_OK)
		return retval;

	if (optionbyte & (1 << OPT_ERROR))
		command_print(CMD, "option byte complement error");

	command_print(CMD, "option byte register = 0x%" PRIx32 "", optionbyte);
	command_print(CMD, "write protection register = 0x%" PRIx32 "", protection);

	command_print(CMD, "read protection: %s",
				(optionbyte & (1 << OPT_READOUT)) ? "on" : "off");

	/* user option bytes are offset depending on variant */
	optionbyte >>= ch32x_info->option_offset;

	command_print(CMD, "watchdog: %sware",
				(optionbyte & (1 << OPT_RDWDGSW)) ? "soft" : "hard");

	command_print(CMD, "stop mode: %sreset generated upon entry",
				(optionbyte & (1 << OPT_RDRSTSTOP)) ? "no " : "");

	command_print(CMD, "standby mode: %sreset generated upon entry",
				(optionbyte & (1 << OPT_RDRSTSTDBY)) ? "no " : "");

	if (ch32x_info->has_dual_banks)
		command_print(CMD, "boot: bank %d", (optionbyte & (1 << OPT_BFB2)) ? 0 : 1);

	command_print(CMD, "user data = 0x%02" PRIx16 "", user_data);

	return ERROR_OK;
}

COMMAND_HANDLER(ch32x_handle_options_write_command)
{
	struct target *target = NULL;
	struct ch32x_flash_bank *ch32x_info = NULL;
	uint8_t optionbyte;
	uint16_t useropt;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	ch32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = ch32x_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = ch32x_read_options(bank);
	if (ERROR_OK != retval)
		return retval;

	/* start with current options */
	optionbyte = ch32x_info->option_bytes.user;
	useropt = ch32x_info->option_bytes.data;

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	while (CMD_ARGC) {
		if (strcmp("SWWDG", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 0);
		else if (strcmp("HWWDG", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 0);
		else if (strcmp("NORSTSTOP", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 1);
		else if (strcmp("RSTSTOP", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 1);
		else if (strcmp("NORSTSTNDBY", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 2);
		else if (strcmp("RSTSTNDBY", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 2);
		else if (strcmp("USEROPT", CMD_ARGV[0]) == 0) {
			if (CMD_ARGC < 2)
				return ERROR_COMMAND_SYNTAX_ERROR;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], useropt);
			CMD_ARGC--;
			CMD_ARGV++;
		}
		else if (ch32x_info->has_dual_banks) {
			if (strcmp("BOOT0", CMD_ARGV[0]) == 0)
				optionbyte |= (1 << 3);
			else if (strcmp("BOOT1", CMD_ARGV[0]) == 0)
				optionbyte &= ~(1 << 3);
			else
				return ERROR_COMMAND_SYNTAX_ERROR;
		} else
			return ERROR_COMMAND_SYNTAX_ERROR;
		CMD_ARGC--;
		CMD_ARGV++;
	}

	if (ch32x_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "ch32x failed to erase options");
		return ERROR_OK;
	}

	ch32x_info->option_bytes.user = optionbyte;
	ch32x_info->option_bytes.data = useropt;

	if (ch32x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "ch32x failed to write options");
		return ERROR_OK;
	}

	command_print(CMD, "ch32x write options complete.\n"
				"INFO: %spower cycle is required "
				"for the new settings to take effect.",
				ch32x_info->can_load_options
					? "'ch32f1x options_load' command or " : "");

	return ERROR_OK;
}

COMMAND_HANDLER(ch32x_handle_options_load_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct ch32x_flash_bank *ch32x_info = bank->driver_priv;

	if (!ch32x_info->can_load_options) {
		LOG_ERROR("Command not applicable to ch32f1x devices - power cycle is "
			"required instead.");
		return ERROR_FAIL;
	}

	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = ch32x_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	/* unlock option flash registers */
	retval = target_write_u32(target, ch32x_get_flash_reg(bank, ch32_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, ch32x_get_flash_reg(bank, ch32_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* force re-load of option bytes - generates software reset */
	retval = target_write_u32(target, ch32x_get_flash_reg(bank, ch32_FLASH_CR), FLASH_OBL_LAUNCH);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int ch32x_mass_erase(struct flash_bank *bank)
{
	
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* unlock option flash registers */
	int retval = target_write_u32(target, ch32x_get_flash_reg(bank, ch32_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, ch32x_get_flash_reg(bank, ch32_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* mass erase flash memory */
	retval = target_write_u32(target, ch32x_get_flash_reg(bank, ch32_FLASH_CR), FLASH_MER);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, ch32x_get_flash_reg(bank, ch32_FLASH_CR),
			FLASH_MER | FLASH_STRT);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, ch32x_get_flash_reg(bank, ch32_FLASH_CR), FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(ch32x_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = ch32x_mass_erase(bank);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "ch32x mass erase complete");
	} else
		command_print(CMD, "ch32x mass erase failed");

	return retval;
}

static const struct command_registration ch32x_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = ch32x_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = ch32x_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = ch32x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "options_read",
		.handler = ch32x_handle_options_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Read and display device option bytes.",
	},
	{
		.name = "options_write",
		.handler = ch32x_handle_options_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('SWWDG'|'HWWDG') "
			"('RSTSTNDBY'|'NORSTSTNDBY') "
			"('RSTSTOP'|'NORSTSTOP') ('USEROPT' user_data)",
		.help = "Replace bits in device option bytes.",
	},
	{
		.name = "options_load",
		.handler = ch32x_handle_options_load_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Force re-load of device option bytes.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration ch32x_command_handlers[] = {
	{
		.name = "wch_arm",
		.mode = COMMAND_ANY,
		.help = "wch_arm flash command group",
		.usage = "",
		.chain = ch32x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver wch_arm_flash = {
	.name = "wch_arm",
	.commands = ch32x_command_handlers,
	.flash_bank_command = ch32x_flash_bank_command,
	.erase = ch32x_erase,
	.protect = ch32x_protect,
	.write = ch32x_write,
	.read = default_flash_read,
	.probe = ch32x_probe,
	.auto_probe = ch32x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = ch32x_protect_check,
	.info = get_ch32x_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
