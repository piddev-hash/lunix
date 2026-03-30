/* Copyright (c) 2018, 2019, 2020, 2021, 2022, 2023 Dennis Wölfing
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

/* kernel/src/thread.cpp
 * Thread class.
 */

#include <assert.h>
#include <errno.h>
#include <sched.h>
#include <string.h>
#include <lunix/kernel/panic.h>
#include <lunix/kernel/process.h>
#include <lunix/kernel/registers.h>
#include <lunix/kernel/worker.h>

Thread* Thread::_current;
Thread* Thread::idleThread;
static ThreadList threadList;
static bool deferKernelThreadStart = true;
static Thread* firstDeferredKernelThread;

__fpu_t initFpu;

static int bootErrno;
extern "C" { int* __errno_location = &bootErrno; }

Thread::Thread(Process* process) {
    contextChanged = false;
    forceKill = false;
    interruptContext = nullptr;
    kernelStack = 0;
    next = nullptr;
    pendingSignals = nullptr;
    prev = nullptr;
    this->process = process;
    returnSignalMask = 0;
    signalMask = 0;
    signalMutex = KTHREAD_MUTEX_INITIALIZER;
    signalCond = KTHREAD_COND_INITIALIZER;
    tid = -1;
    tlsBase = 0;
}

Thread::~Thread() {
    if (kernelStack != 0) {
        kernelSpace->unmapMemory(kernelStack, PAGESIZE);
    }
}

void Thread::initializeIdleThread() {
    Process* idleProcess = xnew Process();
    idleProcess->addressSpace = kernelSpace;
    Process::addProcess(idleProcess);
    assert(idleProcess->pid == 0);
    idleThread = xnew Thread(idleProcess);
    idleThread->tid = idleProcess->threads.add(idleThread);
    assert(idleThread->tid == 0);
    _current = idleThread;
}

void Thread::addThread(Thread* thread) {
    Interrupts::disable();
    threadList.addFront(*thread);
    Interrupts::enable();
}

Thread* Thread::createKernelThread(void (*entry)(void*), void* argument) {
    Thread* thread = xnew Thread(Thread::idleThread->process);
    vaddr_t stack = kernelSpace->mapMemory(PAGESIZE, PROT_READ | PROT_WRITE);
    if (!stack) PANIC("Failed to allocate kernel thread stack");

    InterruptContext* context = (InterruptContext*)
            (stack + PAGESIZE - sizeof(InterruptContext));
    *context = {};

#ifdef __i386__
    uintptr_t* initialStack = (uintptr_t*) (stack + PAGESIZE -
            2 * sizeof(void*));
    initialStack[0] = 0;
    initialStack[1] = (uintptr_t) argument;
    context->eip = (vaddr_t) entry;
    context->cs = 0x8;
    context->eflags = 0x200;
    context->esp = (vaddr_t) initialStack;
    context->ss = 0x10;
#elif defined(__x86_64__)
    context->rip = (vaddr_t) entry;
    context->cs = 0x8;
    context->rdi = (uintptr_t) argument;
    context->rflags = 0x200;
    context->rsp = stack + PAGESIZE - sizeof(void*);
    context->ss = 0x10;
#else
#  error "InterruptContext for kernel thread is uninitialized."
#endif

    thread->updateContext(stack, context, &initFpu);
    if (deferKernelThreadStart) {
        Interrupts::disable();
        thread->next = firstDeferredKernelThread;
        firstDeferredKernelThread = thread;
        Interrupts::enable();
    } else {
        Thread::addThread(thread);
    }
    return thread;
}

void Thread::removeThread(Thread* thread) {
    threadList.remove(*thread);
}

InterruptContext* Thread::schedule(InterruptContext* context) {
    if (likely(!_current->contextChanged)) {
        _current->interruptContext = context;
        Registers::saveFpu(&_current->fpuEnv);
        _current->tlsBase = getTlsBase();
    } else {
        _current->contextChanged = false;
    }

    if (_current->next) {
        _current = _current->next;
    } else {
        if (!threadList.empty()) {
            _current = &threadList.front();
        } else {
            _current = idleThread;
        }
    }

    setKernelStack(_current->kernelStack + PAGESIZE);
    Registers::restoreFpu(&_current->fpuEnv);
    setTlsBase(_current->tlsBase);
    __errno_location = &_current->errorNumber;

    _current->process->addressSpace->activate();
    _current->checkSigalarm(true);
    _current->updatePendingSignals();
    return _current->interruptContext;
}

void Thread::startDeferredKernelThreads() {
    deferKernelThreadStart = false;

    Interrupts::disable();
    Thread* thread = firstDeferredKernelThread;
    firstDeferredKernelThread = nullptr;
    Interrupts::enable();

    while (thread) {
        Thread* next = thread->next;
        thread->next = nullptr;
        thread->prev = nullptr;
        Thread::addThread(thread);
        thread = next;
    }
}

static void deleteThread(void* thread) {
    delete (Thread*) thread;
}

NORETURN void Thread::terminate(bool alsoTerminateProcess) {
    assert(this == Thread::current());

    kthread_mutex_lock(&process->threadsMutex);
    process->threads[tid] = nullptr;
    kthread_mutex_unlock(&process->threadsMutex);

    WorkerJob job;
    job.func = deleteThread;
    job.context = this;

    Interrupts::disable();
    Thread::removeThread(this);
    WorkerThread::addJob(&job);
    if (alsoTerminateProcess) {
        WorkerThread::addJob(&process->terminationJob);
    }
    Interrupts::enable();

    sched_yield();
    __builtin_unreachable();
}

static void deallocateStack(void* address) {
    kernelSpace->unmapMemory((vaddr_t) address, PAGESIZE);
}

void Thread::updateContext(vaddr_t newKernelStack, InterruptContext* newContext,
            const __fpu_t* newFpuEnv) {
    Interrupts::disable();
    if (this == _current) {
        contextChanged = true;
    }

    vaddr_t oldKernelStack = kernelStack;
    kernelStack = newKernelStack;
    interruptContext = newContext;
    memcpy(fpuEnv, newFpuEnv, sizeof(__fpu_t));

    if (this == _current) {
        WorkerJob job;
        if (oldKernelStack) {
            job.func = deallocateStack;
            job.context = (void*) oldKernelStack;
            WorkerThread::addJob(&job);
        }

        sched_yield();
        __builtin_unreachable();
    } else if (oldKernelStack) {
        kernelSpace->unmapMemory(oldKernelStack, PAGESIZE);
    }

    Interrupts::enable();
}
