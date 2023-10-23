// SPDX-License-Identifier: GPL-2.0
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/setup.h>

#if defined(CONFIG_INITRAMFS_IGNORE_SKIP_FLAG) || \
    defined(CONFIG_PROC_CMDLINE_APPEND_ANDROID_FORCE_NORMAL_BOOT)
#define INITRAMFS_STR_FIND "skip_initramf"
#endif

static char new_command_line[COMMAND_LINE_SIZE];

#ifdef CONFIG_INITRAMFS_IGNORE_SKIP_FLAG
#define INITRAMFS_STR_REPLACE "want_initramf"
#define INITRAMFS_STR_LEN (sizeof(INITRAMFS_STR_FIND) - 1)
#endif

#ifdef CONFIG_PROC_CMDLINE_APPEND_ANDROID_FORCE_NORMAL_BOOT
#define ANDROID_FORCE_NORMAL_BOOT_STR "androidboot.force_normal_boot=1"
#endif

static char proc_command_line[COMMAND_LINE_SIZE];

static void proc_command_line_init(void) {
	char *offset_addr;
	char *proc_command_line_tail;

	strcpy(proc_command_line, saved_command_line);

#ifdef CONFIG_INITRAMFS_IGNORE_SKIP_FLAG
	offset_addr = strstr(proc_command_line, INITRAMFS_STR_FIND);
	if (offset_addr)
		memcpy(offset_addr, INITRAMFS_STR_REPLACE, INITRAMFS_STR_LEN);
#endif


#ifdef CONFIG_PROC_CMDLINE_APPEND_ANDROID_FORCE_NORMAL_BOOT
	if (strstr(saved_command_line, INITRAMFS_STR_FIND)) {
		// point proc_command_line_tail to the null terminator of the cmdline
		proc_command_line_tail = proc_command_line + strlen(proc_command_line);
		memcpy(proc_command_line_tail, " ", 1);
		memcpy(proc_command_line_tail + 1, ANDROID_FORCE_NORMAL_BOOT_STR,
                        sizeof(ANDROID_FORCE_NORMAL_BOOT_STR));
	}
#endif
}

static int cmdline_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", proc_command_line);
	return 0;
}

static int cmdline_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdline_proc_show, NULL);
}

static const struct file_operations cmdline_proc_fops = {
	.open		= cmdline_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void patch_flag(char *cmd, const char *flag, const char *val)
{
	size_t flag_len, val_len;
	char *start, *end;

	start = strstr(cmd, flag);
	if (!start)
		return;

	flag_len = strlen(flag);
	val_len = strlen(val);
	end = start + flag_len + strcspn(start + flag_len, " ");
	memmove(start + flag_len + val_len, end, strlen(end) + 1);
	memcpy(start + flag_len, val, val_len);
}

static void patch_safetynet_flags(char *cmd)
{
	patch_flag(cmd, "androidboot.flash.locked=", "1");
	if (strstr(saved_command_line, "project_name=18857") ||
		strstr(saved_command_line, "project_name=18821")) {
		patch_flag(cmd, "androidboot.verifiedbootstate=", "green");
	}
	patch_flag(cmd, "androidboot.veritymode=", "enforcing");
	patch_flag(cmd, "androidboot.vbmeta.device_state=", "locked");
}

static int __init proc_cmdline_init(void)
{
	strcpy(new_command_line, saved_command_line);

	/*
	 * Patch various flags from command line seen by userspace in order to
	 * pass SafetyNet checks.
	 */
	patch_safetynet_flags(new_command_line);
	proc_command_line_init();

	proc_create("cmdline", 0, NULL, &cmdline_proc_fops);
	return 0;
}
fs_initcall(proc_cmdline_init);
