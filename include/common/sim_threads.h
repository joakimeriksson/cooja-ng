/*
 * Thread pool for parallel node execution within simulation time steps.
 *
 * Design: barrier-based pool where the main thread participates as worker 0.
 * Static node assignment: thread t handles nodes t, t+num_threads, t+2*num_threads, ...
 *
 * Uses atomic spin barrier — no mutex/cond/syscalls. Workers spin briefly
 * between time steps, which is fine since they always have work coming.
 */
#ifndef SIM_THREADS_H
#define SIM_THREADS_H

#include <pthread.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Spin barrier using C11 atomics — no kernel transitions */
typedef struct {
    atomic_int arrived;     /* threads that have reached the barrier */
    atomic_int phase;       /* toggles 0/1 each time barrier completes */
    int        count;       /* total threads expected */
} sim_barrier_t;

void sim_barrier_init(sim_barrier_t *b, int count);
void sim_barrier_destroy(sim_barrier_t *b);
void sim_barrier_wait(sim_barrier_t *b);

/* Worker callback: called for each node assigned to the thread */
typedef void (*sim_thread_step_fn)(int node_idx, void *user_data);

typedef struct {
    pthread_t       *threads;       /* worker threads (not including main) */
    int              num_threads;   /* total threads including main */
    int              num_nodes;
    sim_barrier_t    start_barrier;
    sim_barrier_t    end_barrier;
    volatile bool    shutdown;

    /* Per-step parameters set by main thread before start_barrier */
    sim_thread_step_fn step_fn;
    void              *user_data;
} sim_thread_pool_t;

/*
 * Initialize thread pool with num_threads total workers (including main).
 * num_threads must be >= 1. Spawns (num_threads - 1) worker threads.
 */
int sim_thread_pool_init(sim_thread_pool_t *pool, int num_threads, int num_nodes);

/*
 * Run one parallel step: calls step_fn(node_idx, user_data) for each node,
 * distributed across threads. Main thread participates as worker 0.
 * Returns after all threads complete.
 */
void sim_thread_pool_run(sim_thread_pool_t *pool,
                         sim_thread_step_fn step_fn, void *user_data);

/* Shut down worker threads and free resources */
void sim_thread_pool_destroy(sim_thread_pool_t *pool);

#ifdef __cplusplus
}
#endif

#endif /* SIM_THREADS_H */
