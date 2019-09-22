#ifndef __SCHEDULE_H_
#define __SCHEDULE_H_

#include <avr/sleep.h>

static constexpr uint16_t TickPrecision = 15;
class Task
{
  protected:
    enum class State {
      Waiting,
      Running
    };
    State current_state;
  public:
    virtual unsigned long step() {
      return 0;
    }
    void play() {
      current_state = State::Running;
    }
    unsigned long pause() {
      current_state = State::Waiting;
      return pause_wait;
    }
    Task(State start_state, unsigned long sleep_delay = 1000)
      : current_state(start_state),
      pause_wait(sleep_delay)
    {
    }
    virtual unsigned long run() {
      if(current_state == State::Running) {
        return step();
      }
      return pause_wait;
    }
    virtual void setup() {}

    virtual void stop() {}
    bool is_running() {
      return current_state == State::Running;
    }

    virtual bool can_sleep() const {
      return true;
    }
  private:
    unsigned long pause_wait;
};

class Scheduler
{
  private:
    struct TaskNode
    {
      Task* task;
      unsigned long delay;
      TaskNode* next;
      TaskNode(Task* t, unsigned long dl, TaskNode* n)
        : task(t), delay(dl), next(n) {}
    };
    void update_delays() {
      auto current_millis = millis();
      if(current_millis == prev_millis) {
        return;
      }
      auto diff = current_millis - prev_millis;
      prev_millis = current_millis;
      TaskNode* n = tasks;
      while(n) {
        if(n->delay > diff) {
          n->delay -= diff;
        } else {
          n->delay = 0;
        }
        n = n->next;
      }
    }
    TaskNode* get_next_task() {
      update_delays();
      if(tasks && tasks->delay == 0) {
        auto n = tasks;
        tasks = tasks->next;
        n->next = nullptr;
        return n;
      }
      return nullptr;
    }
    void add_task_sorted(TaskNode* tsk) {
      if(!tasks || tasks->delay > tsk->delay) {
        tsk->next = tasks;
        tasks = tsk;
      } else {
        auto p = tasks;
        auto n = tasks->next;
        while(n && n->delay <= tsk->delay) {
          p = n;
          n = n->next;
        }
        tsk->next = n;
        p->next = tsk;
      }
    }

    void stop_tasks() {
      TaskNode *p = tasks;
      while(p) {
        p->task->stop();
        p = p->next;
      }
    }

    void sleep_now()         // here we put the arduino to sleep
    {
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
        sleep_enable();          // enables the sleep bit in the mcucr register
                                 // so sleep is possible. just a safety pin
        sleep_mode();            // here the device is actually put to sleep!!
                                 // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
        sleep_disable();         // first thing after waking from sleep:
                                 // disable sleep...
    }
    TaskNode* tasks;
    TaskNode* unscheduled;
    unsigned long prev_millis;
  public:
    Scheduler() : tasks(nullptr), unscheduled(nullptr), prev_millis(millis()) {
    }

    void add_task(Task& task) {
      tasks = new TaskNode { &task, 0, tasks};
    }

    void setup_tasks() {
      TaskNode *p = tasks;
      while(p) {
        p->task->setup();
        p = p->next;
      }
    }
    void run_next() {
      TaskNode* tsk = get_next_task();
      while(tsk)
      {
        auto next_delay = tsk->task->run();
        if(next_delay == static_cast<unsigned long>(-1)) {
          tsk->next = unscheduled;
          unscheduled = tsk;
        }
        else {
          tsk->delay = next_delay;
          add_task_sorted(tsk);
        }
        tsk = get_next_task();
      }
      if(tasks->delay > 0) {
        delay(tasks->delay);
      }
    }
    void sleep() {
      stop_tasks();
      sleep_now();
    }
};

#endif
