#include "logger.h"

#ifndef DISABLE_SPDLOG
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <mutex>
#include <string>

namespace coacd
{
    namespace logger
    {
        namespace
        {
            /// Forwards each message's payload to a C callback (used by the
            /// C API so a host application can capture CoACD's log instead of
            /// writing it to a console it may not have).
            class callback_sink : public spdlog::sinks::base_sink<std::mutex>
            {
            public:
                callback_sink(callback_t callback, void *user) : callback(callback), user(user) {}

            protected:
                void sink_it_(const spdlog::details::log_msg &msg) override
                {
                    std::string text(msg.payload.data(), msg.payload.size());
                    callback(static_cast<int>(msg.level), text.c_str(), user);
                }
                void flush_() override {}

            private:
                callback_t callback;
                void *user;
            };
        }

        std::shared_ptr<spdlog::logger> get()
        {
            static std::shared_ptr<spdlog::logger> logger;
            if (!logger)
            {
                logger = spdlog::stdout_color_mt("CoACD");
                logger->set_level(spdlog::level::info);
            }
            return logger;
        }

        void set_callback(callback_t callback, void *user)
        {
            auto logger = get();
            auto &sinks = logger->sinks();
            sinks.clear();
            if (callback)
            {
                sinks.push_back(std::make_shared<callback_sink>(callback, user));
            }
            else
            {
                sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
            }
        }
    } // namespace logger
}
#else
namespace coacd
{
    namespace logger
    {
        void set_callback(callback_t, void *) {}
    }
}
#endif
