#pragma once
#include <stdexcept>
#include <string>

namespace coacd
{
    /// A CPU/OS-level fault (access violation, stack overflow, illegal
    /// instruction, ...) that `install_fault_translator()` turned into a C++
    /// exception so the C API can report it instead of taking the host
    /// process down.
    struct hardware_fault : std::runtime_error
    {
        unsigned long code;
        hardware_fault(unsigned long code, const std::string &what)
            : std::runtime_error(what), code(code) {}
    };

    /// Makes structured exceptions on the calling thread throw
    /// `hardware_fault` (MSVC, needs `/EHa`). A no-op on other toolchains.
    /// Per thread: worker threads install it themselves.
    void install_fault_translator();

    /// Restores whatever `install_fault_translator()` replaced.
    struct fault_translator_scope
    {
        fault_translator_scope();
        ~fault_translator_scope();
        fault_translator_scope(const fault_translator_scope &) = delete;
        fault_translator_scope &operator=(const fault_translator_scope &) = delete;

    private:
        void *previous;
    };

    /// Housekeeping after a fault was caught (re-arms the stack guard page
    /// after a stack overflow on MSVC).
    void after_fault(unsigned long code);

    const char *fault_name(unsigned long code);

    /// Thrown by `check_deadline()` once the time limit of the current run
    /// has passed.
    struct timeout_error : std::runtime_error
    {
        explicit timeout_error(const std::string &what) : std::runtime_error(what) {}
    };

    /// Arms a deadline `seconds` from now (0 disables it) on the calling
    /// thread and disarms it again when destroyed. Worker threads inherit it
    /// through `enter_worker_thread`.
    struct deadline_scope
    {
        explicit deadline_scope(double seconds);
        ~deadline_scope();
        deadline_scope(const deadline_scope &) = delete;
        deadline_scope &operator=(const deadline_scope &) = delete;
    };

    /// Throws `timeout_error` when the armed deadline has passed. Called
    /// from the decomposition's loops (any thread).
    void check_deadline();

    /// The calling thread's deadline (steady-clock milliseconds, 0 for
    /// none) and configured limit, to hand to a worker thread.
    struct deadline_state
    {
        long long deadline_ms;
        double limit_seconds;
    };
    deadline_state current_deadline();

    /// First call on a worker thread: installs the fault translator and the
    /// spawning thread's deadline.
    void enter_worker_thread(deadline_state deadline);
}
