//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/Scheduler/scheduler.h"

Scheduler::Scheduler()
{
    constexpr uint8_t MAX_TASKS = static_cast<uint8_t>(TaskID::COUNT) - 1;

    m_tasks = new Task[MAX_TASKS];
    m_tasks = {};
}

Scheduler::~Scheduler()
{
    delete[] m_tasks;
}

void Scheduler::AddTask(TaskID taskId, const uint32_t periodUs)
{
    if (taskId == TaskID::COUNT || taskId == TaskID::INVALID)
    {
        //INVALID TASK
        return;
    }

    const uint8_t taskPosition = static_cast<uint8_t>(taskId);

    if (m_tasks[taskPosition].taskId == TaskID::INVALID)
    {
        //TASK ALREADY ADDED
        return;
    }

    m_tasks[taskPosition].taskId = taskId;
    m_tasks[taskPosition].periodUs = periodUs;
    m_tasks[taskPosition].lastRunUs = 0;
    m_tasks[taskPosition].enable = true;
}

void Scheduler::Update(const uint32_t timeTick)
{
    m_nowUs = timeTick;
}

bool Scheduler::ConsumeTask(TaskID taskId)
{
    Task task;

    if (!GetTask(taskId, task))
    {
        return false;
    }

    const uint32_t elapsedUs = m_nowUs - task.lastRunUs;

    if (elapsedUs < task.periodUs)
    {
        return false;
    }

    constexpr uint8_t scalerTimeToVeryLateUpdate = 4U;

    if (elapsedUs > task.periodUs * scalerTimeToVeryLateUpdate)
    {
        task.lastRunUs = m_nowUs;
    }
    else
    {
        task.lastRunUs += task.periodUs;
    }

    return true;
}

bool Scheduler::ShouldRun(TaskID taskId)
{
    Task task;

    if (!GetTask(taskId, task))
    {
        return false;
    }

    if (m_nowUs - task.lastRunUs >= task.periodUs)
    {
        return true;
    }

    return false;
}

void Scheduler::MarkRun(TaskID taskId)
{
    Task task;

    if (GetTask(taskId, task))
    {
        task.lastRunUs = m_nowUs;
    }
}

void Scheduler::EnableTask(TaskID taskId)
{
    Task task;

    if (GetTask(taskId, task))
    {
        task.enable = true;
    }
}

void Scheduler::DiableTask(TaskID taskId)
{
    Task task;

    if (GetTask(taskId, task))
    {
        task.enable = false;
    }
}

bool Scheduler::GetTask(TaskID taskId, Task& outTask)
{
    if (taskId == TaskID::COUNT || taskId == TaskID::INVALID)
    {
        //INVALID TASK
        return false;
    }

    const uint8_t taskPosition = static_cast<uint8_t>(taskId);
    Task task = m_tasks[taskPosition];

    if (task.taskId == TaskID::INVALID)
    {
        //TASK NOT INICIALIZED
        return false;;
    }

    outTask = task;
    return true;
}
