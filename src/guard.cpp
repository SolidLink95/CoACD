#include "guard.h"

#include <chrono>
#include <cstdio>

#ifdef _MSC_VER
#include <windows.h>
#include <eh.h>
#include <malloc.h>
#endif

namespace coacd
{
    namespace
    {
        // Steady-clock deadline in milliseconds; 0 means no limit. Per
        // thread: the calling thread arms it, workers inherit a copy.
        thread_local long long t_deadline_ms = 0;
        thread_local double t_limit_seconds = 0.0;

        long long now_ms()
        {
            using namespace std::chrono;
            return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
        }
    }

    deadline_scope::deadline_scope(double seconds)
    {
        if (seconds > 0.0)
        {
            t_limit_seconds = seconds;
            t_deadline_ms = now_ms() + static_cast<long long>(seconds * 1000.0);
        }
        else
        {
            t_limit_seconds = 0.0;
            t_deadline_ms = 0;
        }
    }

    deadline_scope::~deadline_scope()
    {
        t_deadline_ms = 0;
        t_limit_seconds = 0.0;
    }

    void check_deadline()
    {
        if (t_deadline_ms != 0 && now_ms() > t_deadline_ms)
        {
            char text[96];
            std::snprintf(text, sizeof text, "time limit of %.0f s exceeded", t_limit_seconds);
            throw timeout_error(text);
        }
    }

    deadline_state current_deadline()
    {
        return deadline_state{t_deadline_ms, t_limit_seconds};
    }

    void enter_worker_thread(deadline_state deadline)
    {
        install_fault_translator();
        t_deadline_ms = deadline.deadline_ms;
        t_limit_seconds = deadline.limit_seconds;
    }

    const char *fault_name(unsigned long code)
    {
        switch (code)
        {
#ifdef _MSC_VER
        case EXCEPTION_ACCESS_VIOLATION:
            return "access violation";
        case EXCEPTION_STACK_OVERFLOW:
            return "stack overflow";
        case EXCEPTION_ARRAY_BOUNDS_EXCEEDED:
            return "array bounds exceeded";
        case EXCEPTION_INT_DIVIDE_BY_ZERO:
            return "integer division by zero";
        case EXCEPTION_INT_OVERFLOW:
            return "integer overflow";
        case EXCEPTION_ILLEGAL_INSTRUCTION:
            return "illegal instruction";
        case EXCEPTION_PRIV_INSTRUCTION:
            return "privileged instruction";
        case EXCEPTION_IN_PAGE_ERROR:
            return "in-page error";
        case EXCEPTION_DATATYPE_MISALIGNMENT:
            return "datatype misalignment";
        case EXCEPTION_FLT_DIVIDE_BY_ZERO:
            return "floating point division by zero";
        case EXCEPTION_FLT_INVALID_OPERATION:
            return "invalid floating point operation";
        case EXCEPTION_FLT_OVERFLOW:
            return "floating point overflow";
        case EXCEPTION_FLT_STACK_CHECK:
            return "floating point stack check";
        case EXCEPTION_INVALID_HANDLE:
            return "invalid handle";
        case STATUS_HEAP_CORRUPTION:
            return "heap corruption";
#endif
        default:
            return "structured exception";
        }
    }

#ifdef _MSC_VER
    namespace
    {
        void translate(unsigned int code, EXCEPTION_POINTERS *info)
        {
            char text[160];
            const char *name = fault_name(code);
            if (code == EXCEPTION_ACCESS_VIOLATION && info && info->ExceptionRecord &&
                info->ExceptionRecord->NumberParameters >= 2)
            {
                const char *kind = info->ExceptionRecord->ExceptionInformation[0] == 1 ? "writing" : "reading";
                std::snprintf(text, sizeof text, "%s (0x%08lX) %s address 0x%llX", name, (unsigned long)code,
                              kind, (unsigned long long)info->ExceptionRecord->ExceptionInformation[1]);
            }
            else
            {
                std::snprintf(text, sizeof text, "%s (0x%08lX)", name, (unsigned long)code);
            }
            throw hardware_fault(code, text);
        }
    }

    void install_fault_translator()
    {
        _set_se_translator(translate);
    }

    fault_translator_scope::fault_translator_scope()
        : previous(reinterpret_cast<void *>(_set_se_translator(translate))) {}

    fault_translator_scope::~fault_translator_scope()
    {
        _set_se_translator(reinterpret_cast<_se_translator_function>(previous));
    }

    void after_fault(unsigned long code)
    {
        if (code == EXCEPTION_STACK_OVERFLOW)
        {
            _resetstkoflw();
        }
    }
#else
    void install_fault_translator() {}
    fault_translator_scope::fault_translator_scope() : previous(nullptr) {}
    fault_translator_scope::~fault_translator_scope() {}
    void after_fault(unsigned long) {}
#endif
}
