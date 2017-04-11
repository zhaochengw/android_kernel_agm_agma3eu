
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/ctype.h>
#include <linux/highmem.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/version.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irqnr.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/kernel_stat.h>
#include <linux/hardirq.h>
#include <asm/hw_irq.h>
#include "rs_common.h"

extern unsigned long irq_err_count;
static u32 s_size = 0;

#ifdef CONFIG_ARM64
enum ipi_msg_type {
	IPI_RESCHEDULE,
	IPI_CALL_FUNC,
	IPI_CALL_FUNC_SINGLE,
	IPI_CPU_STOP,
	IPI_TIMER,
	IPI_IRQ_WORK,
	IPI_WAKEUP,
	IPI_CPU_BACKTRACE,
};

static const char *ipi_types[NR_IPI] = {
#define S(x, s)[x - IPI_RESCHEDULE] = s
	S(IPI_RESCHEDULE, "Rescheduling interrupts"),
	S(IPI_CALL_FUNC, "Function call interrupts"),
	S(IPI_CALL_FUNC_SINGLE, "Single function call interrupts"),
	S(IPI_CPU_STOP, "CPU stop interrupts"),
	S(IPI_TIMER, "Timer broadcast interrupts"),
	S(IPI_IRQ_WORK, "IRQ work interrupts"),
	S(IPI_WAKEUP, "CPU wakeup interrupts"),
	S(IPI_CPU_BACKTRACE, "CPU backtrace"),
};

#else

enum ipi_msg_type {
	IPI_WAKEUP,
	IPI_TIMER,
	IPI_RESCHEDULE,
	IPI_CALL_FUNC,
	IPI_CALL_FUNC_SINGLE,
	IPI_CPU_STOP,
	IPI_CPU_BACKTRACE,
};
static const char *ipi_types[NR_IPI] = {
#define S(x, s)[x] = s
	S(IPI_WAKEUP, "CPU wakeup interrupts"),
	S(IPI_TIMER, "Timer broadcast interrupts"),
	S(IPI_RESCHEDULE, "Rescheduling interrupts"),
	S(IPI_CALL_FUNC, "Function call interrupts"),
	S(IPI_CALL_FUNC_SINGLE, "Single function call interrupts"),
	S(IPI_CPU_STOP, "CPU stop interrupts"),
	S(IPI_CPU_BACKTRACE, "CPU backtrace"),
};
#endif

int rs_logbuff_print(struct rs_recorder_log *log, int sz, const char *fmt, ...)
{
	int len = 0;
	va_list args;
	char *logbuff = log->log_buf+log->w_pos + sz;

	va_start(args, fmt);
	len = vsnprintf(logbuff, log->left, fmt, args);
	va_end(args);

	return len;
}
EXPORT_SYMBOL(rs_logbuff_print);

static void rs_logbuff_putc(struct rs_recorder_log *log, char c)
{
	char * buff;

	buff = log->log_buf + log->w_pos + s_size;
	*buff = c;
	s_size += 1;
}

static void rs_show_ipi_list(struct rs_recorder_log *log, int prec)
{
	unsigned int cpu, i;

	for (i = 0; i < NR_IPI; i++) {
		s_size += rs_logbuff_print(log, s_size, "%*s%u:%s",
			prec - 1, "IPI", i + IPI_RESCHEDULE,
			prec >= 4 ? " " : "");

		for_each_online_cpu(cpu)
			s_size += rs_logbuff_print(log, s_size, "%10u ",
				   __get_irq_stat(cpu, ipi_irqs[i]));

		s_size += rs_logbuff_print(log, s_size, "      %s\n", ipi_types[i]);
	}
}

static void rs_recorder_get_irqs(struct rs_recorder_log *log)
{
	static int prec;
	unsigned long flags, any_count;
	int i, j;
	struct irqaction *action;
	struct irq_desc *desc;

	for (i = 0; i <= nr_irqs; i++) {
		/* print header and calculate the width of the first column */
		any_count = 0;

		if (i == nr_irqs) {
#ifdef CONFIG_SMP
			rs_show_ipi_list(log, prec);
#endif
			s_size += rs_logbuff_print(log, s_size, "%*s: %10lu\n",
				prec, "Err", irq_err_count);
		}

		if (i == 0) {
			for (prec = 3, j = 1000;
					prec < 10 && j <= nr_irqs; ++prec)
				j *= 10;

			s_size += rs_logbuff_print(log, s_size, "%*s", prec + 8, "");
			for_each_online_cpu(j)
				s_size += rs_logbuff_print(log, s_size, "CPU%-8d", j);

			rs_logbuff_putc(log, '\n');
		}

		desc = irq_to_desc(i);
		if (!desc)
			continue;

		raw_spin_lock_irqsave(&desc->lock, flags);
		for_each_online_cpu(j)
			any_count |= kstat_irqs_cpu(i, j);
		action = desc->action;
		if (!action && !any_count)
			goto out;

		s_size += rs_logbuff_print(log, s_size, "%*d: ", prec, i);
		for_each_online_cpu(j)
			s_size += rs_logbuff_print(log, s_size, "%10u ", kstat_irqs_cpu(i, j));

		if (desc->irq_data.chip) {
			if (desc->irq_data.chip->name)
				s_size += rs_logbuff_print(log, s_size, " %8s",
					desc->irq_data.chip->name);
			else
				s_size += rs_logbuff_print(log, s_size, " %8s", "-");
		} else {
			s_size += rs_logbuff_print(log, s_size, " %8s", "None");
		}

#ifdef CONFIG_GENERIC_IRQ_SHOW_LEVEL
		s_size += rs_logbuff_print(log, s_size, " %-8s",
			irqd_is_level_type(&desc->irq_data) ? "Level" : "Edge");
#endif
		if (desc->name)
			s_size += rs_logbuff_print(log, s_size, "-%-8s", desc->name);

		if (action) {
			s_size += rs_logbuff_print(log, s_size, "  %s", action->name);
			while ((action = action->next) != NULL)
				s_size += rs_logbuff_print(log, s_size, ", %s", action->name);
		}

		rs_logbuff_putc(log, '\n');
out:
		raw_spin_unlock_irqrestore(&desc->lock, flags);
	}
}

int rs_dump_interrupts(struct rs_recorder_log *log, char *fname)
{
	char * buff;

	if (NULL == log) {
		pr_err("[%s]: line [%d] invalid param!\n", __FILE__, __LINE__);
		return -EINVAL;
	}

	s_size = 0;
	buff = log->log_buf + log->w_pos;

	rs_recorder_get_irqs(log);

	if (!log->is_panic)
		rs_save_file(log->path, fname, buff, s_size);

	rs_update_buf_header(log, fname, s_size);

	return 0;
}

