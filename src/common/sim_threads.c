/*
 * Thread pool for parallel node execution within simulation time steps.
 * See sim_threads.h for design overview.
 */
#include "sim_threads.h"
#include <stdlib.h>
#include <string.h>

/* --- Atomic spin barrier --- */

void sim_barrier_init(sim_barrier_t *b, int count) {
    atomic_store(&b->arrived, 0);
    atomic_store(&b->phase, 0);
    b->count = count;
}

void sim_barrier_destroy(sim_barrier_t *b) {
    (void)b;
}

void sim_barrier_wait(sim_barrier_t *b) {
    int my_phase = atomic_load_explicit(&b->phase, memory_order_relaxed);

    if (atomic_fetch_add_explicit(&b->arrived, 1, memory_order_acq_rel) == b->count - 1) {
        /* Last thread to arrive: reset counter and flip phase */
        atomic_store_explicit(&b->arrived, 0, memory_order_relaxed);
        atomic_store_explicit(&b->phase, 1 - my_phase, memory_order_release);
    } else {
        /* Spin until phase flips */
        while (atomic_load_explicit(&b->phase, memory_order_acquire) == my_phase)
            ;  /* spin */
    }
}

/* --- Worker thread --- */

typedef struct {
    sim_thread_pool_t *pool;
    int thread_id;
} worker_arg_t;

static void *worker_func(void *arg) {
    worker_arg_t *wa = (worker_arg_t *)arg;
    sim_thread_pool_t *pool = wa->pool;
    int tid = wa->thread_id;

    while (1) {
        /* Spin-wait for main thread to signal work is ready */
        sim_barrier_wait(&pool->start_barrier);
        if (pool->shutdown)
            break;

        /* Step assigned nodes: tid, tid+num_threads, ... */
        for (int i = tid; i < pool->num_nodes; i += pool->num_threads)
            pool->step_fn(i, pool->user_data);

        /* Signal completion */
        sim_barrier_wait(&pool->end_barrier);
    }
    return NULL;
}

/* --- Public API --- */

int sim_thread_pool_init(sim_thread_pool_t *pool, int num_threads, int num_nodes) {
    memset(pool, 0, sizeof(*pool));
    pool->num_threads = num_threads;
    pool->num_nodes = num_nodes;
    pool->shutdown = false;

    sim_barrier_init(&pool->start_barrier, num_threads);
    sim_barrier_init(&pool->end_barrier, num_threads);

    int workers = num_threads - 1;  /* main thread is worker 0 */
    if (workers > 0) {
        pool->threads = calloc((size_t)workers, sizeof(pthread_t));
        if (!pool->threads)
            return -1;

        for (int i = 0; i < workers; i++) {
            worker_arg_t *wa = malloc(sizeof(worker_arg_t));
            if (!wa) return -1;
            wa->pool = pool;
            wa->thread_id = i + 1;  /* thread 0 is main */
            if (pthread_create(&pool->threads[i], NULL, worker_func, wa) != 0) {
                free(wa);
                return -1;
            }
        }
    }
    return 0;
}

void sim_thread_pool_run(sim_thread_pool_t *pool,
                         sim_thread_step_fn step_fn, void *user_data) {
    pool->step_fn = step_fn;
    pool->user_data = user_data;

    /* Release all workers (including main as worker 0) */
    sim_barrier_wait(&pool->start_barrier);

    /* Main thread does its share: nodes 0, num_threads, 2*num_threads, ... */
    for (int i = 0; i < pool->num_nodes; i += pool->num_threads)
        step_fn(i, user_data);

    /* Wait for all workers to finish */
    sim_barrier_wait(&pool->end_barrier);
}

void sim_thread_pool_destroy(sim_thread_pool_t *pool) {
    if (!pool->threads)
        return;

    pool->shutdown = true;

    /* Wake workers so they see shutdown flag */
    pool->step_fn = NULL;
    pool->user_data = NULL;
    sim_barrier_wait(&pool->start_barrier);

    int workers = pool->num_threads - 1;
    for (int i = 0; i < workers; i++)
        pthread_join(pool->threads[i], NULL);

    free(pool->threads);
    pool->threads = NULL;

    sim_barrier_destroy(&pool->start_barrier);
    sim_barrier_destroy(&pool->end_barrier);
}
