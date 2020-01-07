#include <zephyr.h>
#include <misc/util.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(sched_demo, 4);


#define LOW_PRIO  ( 1)
#define MID_PRIO  (-1)
#define HIGH_PRIO (-2)


//#define LOW_PRIO  (2)
//#define MID_PRIO  (1)
//#define HIGH_PRIO (0)

#define STACK_SIZE 4096


struct thread_config {
	struct k_thread  thread;
	k_thread_entry_t entry_fn;
	int              prio;
	u32_t            exec_time_us;
	s32_t            sleep_time_ms;
};


static void thread_fn(void *p1, void *p2, void *p3);

static struct thread_config thread[] = {
	{ .prio = LOW_PRIO, .entry_fn = (k_thread_entry_t)thread_fn, .exec_time_us = K_MSEC(100) * 1000, .sleep_time_ms = K_MSEC(100) },
	{ .prio = MID_PRIO, .entry_fn = (k_thread_entry_t)thread_fn, .exec_time_us = K_MSEC(100) * 1000, .sleep_time_ms = K_MSEC(100) },
	{ .prio = HIGH_PRIO, .entry_fn = (k_thread_entry_t)thread_fn, .exec_time_us = K_MSEC(100) * 1000, .sleep_time_ms = K_MSEC(100) },
};

static K_THREAD_STACK_ARRAY_DEFINE(stack, ARRAY_SIZE(thread), STACK_SIZE);


static void thread_fn(void *p1, void *p2, void *p3)
{
	size_t id                    = (size_t)p1;
	struct thread_config *thread = p2;

	int    prio          = thread->prio;
	u32_t  exec_time_us  = thread->exec_time_us;
	s32_t  sleep_time_ms = thread->sleep_time_ms;

	size_t i = 0;

	while (true) {
		LOG_INF("thread %p: id:%zu prio:%d ... %zu exec",
			k_current_get(), id, prio, i);
		k_busy_wait(exec_time_us);
		LOG_INF("thread %p: id:%zu prio:%d ... %zu exec done",
			k_current_get(), id, prio, i);


		if (sleep_time_ms > 0) {
			LOG_INF("thread %p: id:%zu prio:%d ... %zu sleep",
				k_current_get(), id, prio, i);
			k_sleep(sleep_time_ms);
			LOG_INF("thread %p: id:%zu prio:%d ... %zu sleep done",
				k_current_get(), id, prio, i);
		}

		i++;
	}
}

void main(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(thread); i++) {
		k_thread_create(&thread[i].thread,
				stack[i], STACK_SIZE,
				thread[i].entry_fn,
				(void *)i, (void *)&thread[i], NULL,
				thread[i].prio, 0, 0);
		LOG_INF("Thread %zu created", i);
	}
	LOG_INF("Start");
}
