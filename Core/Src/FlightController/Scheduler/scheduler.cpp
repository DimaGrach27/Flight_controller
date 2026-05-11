//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/Scheduler/scheduler.h"

Scheduler::Scheduler()
{

}

Scheduler::~Scheduler()
{
}

void Scheduler::AddTask(TaskID taskId, const uint32_t periodUs)
{
    if (taskId == TaskID::COUNT || taskId == TaskID::INVALID)
    {
        //INVALID TASK
        return;
    }

    const uint8_t taskPosition = static_cast<uint8_t>(taskId);

    if (m_tasks[taskPosition].taskId != TaskID::INVALID)
    {
        //TASK ALREADY ADDED
        return;
    }

    m_tasks[taskPosition].taskId = taskId;
    m_tasks[taskPosition].periodUs = periodUs;
    m_tasks[taskPosition].lastRunUs = 0;
    m_tasks[taskPosition].enabled = true;
}

void Scheduler::Update(const uint32_t timeTick)
{
    m_nowUs = timeTick;
}

bool Scheduler::ConsumeTask(TaskID taskId)
{
    if (taskId == TaskID::INVALID || taskId == TaskID::COUNT)
    {
        return false;
    }

    Task& task = GetTaskRef(taskId);

    if (!task.enabled)
    {
        return false;
    }

    const uint32_t elapsedUs = m_nowUs - task.lastRunUs;

    if (elapsedUs < task.periodUs)
    {
        return false;
    }

    if (elapsedUs > task.periodUs * 4U)
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
    const Task& task = GetTaskRef(taskId);
    if (m_nowUs - task.lastRunUs >= task.periodUs)
    {
        return true;
    }

    return false;
}

void Scheduler::MarkRun(TaskID taskId)
{
    Task& task = GetTaskRef(taskId);
    task.lastRunUs = m_nowUs;
}

void Scheduler::EnableTask(TaskID taskId)
{
    Task& task = GetTaskRef(taskId);
    task.enabled = true;
}

void Scheduler::DiableTask(TaskID taskId)
{
    Task& task = GetTaskRef(taskId);
    task.enabled = false;
}

Scheduler::Task& Scheduler::GetTaskRef(TaskID taskId)
{
    return m_tasks[static_cast<uint8_t>(taskId)];
}

const Scheduler::Task& Scheduler::GetTaskRef(TaskID taskId) const
{
    return m_tasks[static_cast<uint8_t>(taskId)];
}
