//
// Created by Dmytro Hrachov on 11.05.2026.
//

#pragma once
#include "taskid.h"
#include <array>

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
        bool enabled = false;
    };

private:
    bool ShouldRun(TaskID taskId);
    void MarkRun(TaskID taskId);

    Task& GetTaskRef(TaskID taskId);
    const Task& GetTaskRef(TaskID taskId) const;

private:
    uint32_t m_nowUs = 0;

    static constexpr uint8_t ArraySize = static_cast<uint8_t>(TaskID::COUNT);
    std::array<Task, ArraySize> m_tasks;
};
