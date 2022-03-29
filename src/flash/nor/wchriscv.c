/***************************************************************************
 *   WCH RISC-V mcu :CH32V103X CH32V20X CH32V30X CH56X CH57X CH58X         *
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
extern int wlink_erase(void);
extern unsigned char riscvchip;
extern int wlink_reset();
extern int wlink_write(const uint8_t *buffer, uint32_t offset, uint32_t count);
extern int noloadflag;
struct ch32vx_options
{
	uint8_t rdp;
	uint8_t user;
	uint16_t data;
	uint32_t protection;
};

struct ch32vx_flash_bank
{
	struct ch32vx_options option_bytes;
	int ppage_size;
	int probed;

	bool has_dual_banks;
	bool can_load_options;
	uint32_t register_base;
	uint8_t default_rdp;
	int user_data_offset;
	int option_offset;
	uint32_t user_bank_size;
};

FLASH_BANK_COMMAND_HANDLER(ch32vx_flash_bank_command)
{
	struct ch32vx_flash_bank *ch32vx_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ch32vx_info = malloc(sizeof(struct ch32vx_flash_bank));

	bank->driver_priv = ch32vx_info;
	ch32vx_info->probed = 0;
	ch32vx_info->has_dual_banks = false;
	ch32vx_info->can_load_options = false;
	ch32vx_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static int ch32vx_erase(struct flash_bank *bank, int first, int last)
{	
	 
	if(noloadflag)
		return ERROR_OK;
	wlink_reset();
	int ret = wlink_erase();
	target_halt(bank->target);
	if (ret)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int ch32vx_write(struct flash_bank *bank, const uint8_t *buffer,
						uint32_t offset, uint32_t count)
{
	if(noloadflag)
		return ERROR_OK;
	int ret = wlink_write(buffer, offset, count);
	if(riscvchip==0x03)
		 wlink_reset(); 
	return ret;
}

static int ch32vx_get_device_id(struct flash_bank *bank, uint32_t *device_id)
{
	struct target *target = bank->target;
	int retval = target_read_u32(target, 0x1ffff7e8, device_id);
	if (retval != ERROR_OK)
		return retval;
	return ERROR_OK;
}

static int ch32vx_get_flash_size(struct flash_bank *bank, uint16_t *flash_size_in_kb)
{

	struct target *target = bank->target;
	int retval = target_read_u16(target, 0x1ffff7e0, flash_size_in_kb);
	if (retval != ERROR_OK)
		return retval;
	return ERROR_OK;
}

static int ch32vx_probe(struct flash_bank *bank)
{
	struct ch32vx_flash_bank *ch32vx_info = bank->driver_priv;
	uint16_t delfault_max_flash_size=512;
	uint16_t flash_size_in_kb;
	uint32_t device_id;
	int page_size;
	uint32_t base_address = 0x00000000;
	uint32_t rid = 0;
	ch32vx_info->probed = 0;
	/* read ch32 device id register */
	int retval = ch32vx_get_device_id(bank, &device_id);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);
	page_size = 1024;
	ch32vx_info->ppage_size = 4;
	/* get flash size from target. */
	retval = ch32vx_get_flash_size(bank, &flash_size_in_kb);
	if(flash_size_in_kb)
		LOG_INFO("flash size = %dkbytes", flash_size_in_kb);
	else
		flash_size_in_kb=delfault_max_flash_size;
	// /* calculate numbers of pages */
	int num_pages = flash_size_in_kb * 1024 / page_size;
	bank->base = base_address;
	bank->size = (num_pages * page_size);
	bank->num_sectors = num_pages;
	bank->sectors = alloc_block_array(0, page_size, num_pages);
	ch32vx_info->probed = 1;

	return ERROR_OK;
}

static int ch32vx_auto_probe(struct flash_bank *bank)
{

	struct ch32vx_flash_bank *ch32vx_info = bank->driver_priv;
	if (ch32vx_info->probed)
		return ERROR_OK;
	return ch32vx_probe(bank);
}

static const struct command_registration ch32vx_command_handlers[] = {
	{
		.name = "wch_riscv",
		.mode = COMMAND_ANY,
		.help = "wch_riscv flash command group",
		.usage = "",

	},
	COMMAND_REGISTRATION_DONE};

const struct flash_driver wch_riscv_flash = {
	.name = "wch_riscv",
	.commands = ch32vx_command_handlers,
	.flash_bank_command = ch32vx_flash_bank_command,
	.erase = ch32vx_erase,
	.write = ch32vx_write,
	.read = default_flash_read,
	.probe = ch32vx_probe,
	.auto_probe = ch32vx_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
