#ifndef _ARM_LIBFDT_ENV_H
#define _ARM_LIBFDT_ENV_H

#include <linux/types.h>
#include <linux/string.h>
#include <asm/byteorder.h>

#define fdt16_to_cpu(x)		le16_to_cpu(x)
#define cpu_to_fdt16(x)		cpu_to_le16(x)
#define fdt32_to_cpu(x)		le32_to_cpu(x)
#define cpu_to_fdt32(x)		cpu_to_le32(x)
#define fdt64_to_cpu(x)		le64_to_cpu(x)
#define cpu_to_fdt64(x)		cpu_to_le64(x)

#endif
