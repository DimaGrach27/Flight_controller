//
// Created by Dmytro Hrachov on 11.05.2026.
//

#pragma once
#include "taskid.h"

class Scheduler
{
public:
    Scheduler();
    ~Scheduler();

    void AddTask(TaskID taskId, const uint32_t periodUs);

    void Update(const uint32_t timeTick);
    bool ConsumeTask(TaskID taskId);

    void EnableTask(TaskID taskId);
    void DiableTask(TaskID taskId);

private:
    struct Task
    {
        TaskID taskId = TaskID::INVALID;
        uint32_t periodUs = 0;
        uint32_t lastRunUs = 0;
        bool enable = true;
    };

private:
    bool ShouldRun(TaskID taskId);
    void MarkRun(TaskID taskId);
    bool GetTask(TaskID taskId, Task& outTask);

private:
    uint32_t m_nowUs = 0;

    Task* m_tasks = nullptr;
};
