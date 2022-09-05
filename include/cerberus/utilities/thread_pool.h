#ifndef __CERBERUS_UTILITIES_THREAD_POOL_H_
#define __CERBERUS_UTILITIES_THREAD_POOL_H_

#include <atomic>
#include <cmath>
#include <stdexcept>
#include <thread>

#include <absl/strings/str_format.h>

#include <boost/asio/thread_pool.hpp>

#include <oneapi/tbb.h>

namespace cerberus::utilities::threads {

    class ThreadPool {

      public:
        struct PoolRatios {
            const float tasks;
            const float asio;
        };

        const PoolRatios internal_pool_ratios;

      public: //funcs
        ThreadPool(const ThreadPool&) = delete;
        ThreadPool& operator=(const ThreadPool&) = delete;

        explicit ThreadPool(const PoolRatios& ratios, size_t num_program_pools = 1)
            : internal_pool_ratios(ratios), _asio_pool(_init_asio_pool(num_program_pools, ratios)) {
            class_count += 1;
            if (std::thread::hardware_concurrency() == 1 && num_program_pools > 1) {
                if (class_count.load() == 1) {
                    //TODO: log (absl::StrFormat("Number of Threads exceeds the number of reasonable pools, treating this Pool as the only pool"))
                } else {
                    throw std::runtime_error(absl::StrFormat(
                        "Thread Pool with ID {%i} exceeds the expected number of pools for the hardware's single thread\n", class_count
                    ));
                }
            }
        }

        void enqueue_asio() {}

        void enqueue() {}

      private: //vars
        inline static std::atomic_size_t class_count{0};
        boost::asio::thread_pool _asio_pool;

        oneapi::tbb::task_group_context _context[2];
        oneapi::tbb::task_arena _arenas[2];
        oneapi::tbb::task_group _groups[2];

      private: //funcs
        static boost::asio::thread_pool _init_asio_pool(size_t num_pools, const PoolRatios& ratios) {
            auto asio_count = static_cast<size_t>(std::round(static_cast<float>(std::thread::hardware_concurrency()) * ratios.asio));
            return boost::asio::thread_pool(asio_count / num_pools);
        }
        static oneapi::tbb::task_arena _init_tbb_pool(size_t num_pools, const PoolRatios& ratios) {
            auto task_count = static_cast<int>(std::round(static_cast<float>(std::thread::hardware_concurrency()) * ratios.tasks));
            return oneapi::tbb::task_arena(task_count / static_cast<int>(num_pools));
        }
    };
} // namespace cerberus::utilities::threads

#endif //__CERBERUS_UTILITIES_THREAD_POOL_H_