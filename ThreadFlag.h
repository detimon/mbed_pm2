#ifndef THREAD_FLAG_H
#define THREAD_FLAG_H

#include "mbed.h"

class ThreadFlag {
public:
    ThreadFlag() : _flags() {}
    void set(uint32_t flag) { _flags.set(flag); }
    uint32_t wait_any(uint32_t flags) { return _flags.wait_any(flags); }
    // Füge hier weitere Methoden hinzu, falls dein Code sie braucht
private:
    rtos::EventFlags _flags;
};

#endif