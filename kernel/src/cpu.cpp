/* Copyright (c) 2026
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/* kernel/src/cpu.cpp
 * CPU-local kernel state for the uniprocessor scheduler.
 */

#include <assert.h>
#include <errno.h>
#include <sched.h>
#include <lunix/kernel/cpu.h>
#include <lunix/kernel/gdt.h>
#include <lunix/kernel/interrupts.h>
#include <lunix/kernel/registers.h>
#include <lunix/kernel/thread.h>

struct CpuState {
    bool lockHeld;
    unsigned int kernelEntryDepth;
    Thread* currentThread;
    Thread* idleThread;
};

static CpuState cpuState;
static int bootErrno;
static kthread_spinlock_t kernelLock = KTHREAD_SPINLOCK_INITIALIZER;
extern "C" { extern int* __errno_location; }

extern "C" void cpuEnterKernel(void) {
    Cpu::enterKernel();
}

extern "C" void cpuLeaveKernel(void) {
    Cpu::leaveKernel(true);
}

extern "C" void cpuLeaveKernelNoUnlock(void) {
    Cpu::leaveKernel(false);
}

extern "C" bool cpuShouldReleaseKernelLockOnReturn(void) {
    Thread* thread = Cpu::currentThread();
    return !thread || thread->isIdleThread() || !thread->isKernelThread();
}

extern "C" bool cpuShouldLeaveKernelNoUnlockOnReturn(void) {
    Thread* thread = Cpu::currentThread();
    if (!thread || thread->isIdleThread() || !thread->isKernelThread()) {
        return false;
    }
    return cpuState.kernelEntryDepth > 1;
}

Thread* Cpu::currentThread() {
    return cpuState.currentThread;
}

void Cpu::enterKernel() {
    if (!cpuState.lockHeld) {
        kthread_spinlock_lock(&kernelLock);
        cpuState.lockHeld = true;
    }
    cpuState.kernelEntryDepth++;
    __errno_location = getErrnoLocation();
}

int* Cpu::getErrnoLocation() {
    Thread* thread = currentThread();
    if (!thread) return &bootErrno;
    return thread->errnoLocation();
}

void Cpu::initializeCurrentCpu() {
    Gdt::initializeCurrentCpu();
    Interrupts::initializeCurrentCpu();
    Registers::initializeCurrentCpu();
    __errno_location = getErrnoLocation();
}

void Cpu::leaveKernel(bool releaseLock) {
    assert(cpuState.kernelEntryDepth > 0);

    cpuState.kernelEntryDepth--;
    if (cpuState.kernelEntryDepth == 0 && cpuState.lockHeld && releaseLock) {
        cpuState.lockHeld = false;
        kthread_spinlock_unlock(&kernelLock);
    }
}

void Cpu::prepareForScheduleReturn(Thread* nextThread) {
    // Thread::schedule() always runs from an interrupt/trap frame. If we switch
    // away from the interrupted kernel context, keep only the active frame in
    // kernelEntryDepth so that returning to userspace actually releases the
    // big kernel lock again.
    if (cpuState.kernelEntryDepth > 1) {
        cpuState.kernelEntryDepth = 1;
    }

    if (nextThread && nextThread->isKernelThread() && !cpuState.lockHeld) {
        kthread_spinlock_lock(&kernelLock);
        cpuState.lockHeld = true;
    }
}

void Cpu::setCurrentThread(Thread* thread) {
    cpuState.currentThread = thread;
}

void Cpu::setIdleThread(Thread* thread) {
    cpuState.idleThread = thread;
    if (!cpuState.currentThread) {
        cpuState.currentThread = thread;
    }
}

Thread* Cpu::idleThread() {
    return cpuState.idleThread;
}

void Cpu::yield() {
    bool releasedKernelLock = cpuState.lockHeld &&
            cpuState.kernelEntryDepth == 1;
    if (releasedKernelLock) {
        Cpu::leaveKernel(true);
    }

    sched_yield();

    if (releasedKernelLock && !cpuState.lockHeld) {
        Cpu::enterKernel();
    }
}
