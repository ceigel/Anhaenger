// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library

// Output port connections:
// X | X | X | X | X | X | X
// G  D6  5V  D3   G   D2  G
// X | X | X | X | X | X | X | X
// G  D5  5V   G   D9  5V  G  A4
//

#include <Adafruit_NeoPixel.h>
#include "schedule.h"

Scheduler sched;

class ButtonPins {
  public:
    static void setup() {
      EICRA = 0x00; // Low level both pins
      EIMSK = 0x11; // Enable both interrupts
    }

    static void int0_interrupt() {
      static unsigned long last_interrupt = 0UL;
      on_interrupt(last_interrupt, [](){ tick0Count++; });
    }

    static void int1_interrupt() {
      static unsigned long last_interrupt = 0UL;
      on_interrupt(last_interrupt, [](){ tick1Count++; });
    }

    static unsigned long getInt0TickCount() {
      return tick0Count;
    }
    static unsigned long getInt1TickCount() {
      return tick1Count;
    }

  private:
    static volatile unsigned long tick0Count;
    static volatile unsigned long tick1Count;
    static void on_interrupt(unsigned long& last_interrupt, void(*handle)(void)) {
      auto ct = millis();
      if(ct - last_interrupt >= 50UL) {
        handle();
      }
      last_interrupt = ct;
    }
};

volatile unsigned long ButtonPins::tick0Count = 0;
volatile unsigned long ButtonPins::tick1Count = 0;

ISR(INT0_vect) {
  ButtonPins::int0_interrupt();
}

ISR(INT1_vect) {
  ButtonPins::int1_interrupt();
}

class SpeedMeasure: public Task
{
  public:
    SpeedMeasure() : Task(Task::State::Running), lastTick(0), lastMillis(0), computed_acceleration(0) {
      for(size_t i = 0; i < sizeof(times) / sizeof(times[0]); ++i) {
        times[i] = millis();
        speeds[i] = 0;
      }
    }
    void setup() override {
      pinMode(interruptPin, INPUT_PULLUP);
      PCICR |= 0b00000010;    // turn on port c
      PCMSK1 |= 0b00010000;    // turn on pin PC4, which is PCINT12, physical pin 27
      // attachInterrupt(digitalPinToInterrupt(interruptPin), tickFunc, CHANGE);
    }

    void wakeup() override {
      setup();
    }

    void stop() override {
      // detachInterrupt(digitalPinToInterrupt(interruptPin));
      PCICR ^= 0b00000010;    // turn off port c
      PCMSK1 ^= 0b00010000;    // turn off pin PC4, which is PCINT12, physical pin 27
    }

    unsigned long step() {
      shiftArray(speeds);
      shiftArray(times);
      times[0] = millis();
      unsigned long tick = tickCount;
      if(tick != lastTick) {
        lastMoved = times[0];
      }
      auto tickDiff = tick - lastTick;
      auto timeDiff = times[0] - times[1]; // ms
      speeds[0] = (tickDist * tickDiff * 1000.0) / timeDiff;
      lastTick = tick;
      compute_acceleration();
      return refreshRate;
    }

    float acceleration() const {
      return computed_acceleration;
    }

    float speed() const {
      return speeds[0] + 10;
    }

    unsigned long getTickCount() const {
      return tickCount;
    }

    unsigned long stoppedTime() const {
      return millis() - lastMoved;
    }
    bool can_sleep() const override {
      auto st = stoppedTime();
      if(st >= stoppedTimeToSleep) {
        return true;
      }
      return false;
    }

    static void tickFunc() {
      if(digitalRead(interruptPin) == HIGH) { // Interrupt is on change. Take only the low state
        return;
      }
      static unsigned long last_interrupt = 0UL;
      auto ct = millis();
      if(ct - last_interrupt >= 30UL) {
        tickCount += 1;
      }
      last_interrupt = ct;
    }
  private:
    void compute_acceleration() {
      auto timeDiff = times[0] - times[1]; // ms
      computed_acceleration = (speeds[0] - speeds[1]) * 1000.0 / timeDiff;
    }

    template<typename T, unsigned int size>
    void shiftArray(T(&arr)[size]) {
      for(size_t i = sizeof(arr)/sizeof(arr[0]) - 1; i > 0 ; --i) {
        arr[i] = arr[i - 1];
      }
    }
    static volatile unsigned long tickCount;
    static const int interruptPin = A4;
    static constexpr unsigned long stoppedTimeToSleep = 300000;
    static constexpr unsigned long refreshRate = 1000;
    static constexpr float tickDist = 1.2f;
    unsigned long lastTick;
    unsigned long lastMillis;
    unsigned long lastMoved;
    float speeds[2];
    float computed_acceleration;
    unsigned long times[2];
};

ISR(PCINT1_vect) {
  SpeedMeasure::tickFunc();
}

class ShowSpeed: public Task {
    public:
    ShowSpeed(const SpeedMeasure& speed, int pin)
    : Task(Task::State::Running)
    , ledPin(pin)
    , speedMeasure(speed)
    , lastTickCount(speed.getTickCount() - 1)
    , led_state(false)
    {
    }
    void setup() override {
      pinMode(ledPin, OUTPUT);
    }
    void stop() override {
      digitalWrite(ledPin, LOW);
    }
    unsigned long step() {
      auto tick = speedMeasure.getTickCount();
      if(tick != lastTickCount)
      {
        lastTickCount = tick;
        led_state = !led_state;
        digitalWrite(ledPin, led_state ? HIGH : LOW);
      }
      return refreshRate;
    }
    private:
      int ledPin;
      static constexpr unsigned long refreshRate = 500;
      const SpeedMeasure &speedMeasure;
      unsigned long lastTickCount;
      bool led_state;
};

class DebugLedTask : public Task
{
  public:
    DebugLedTask(int pin) : Task(Task::State::Running), ledPin(pin) {}
    void setup() override {
      pinMode(ledPin, OUTPUT);
    }
    unsigned long step() override {
      led_state = !led_state;
      digitalWrite(ledPin, led_state ? HIGH : LOW);
      if(led_state)
        return on_wait;
      return off_wait;
    }
    void stop() override {
      digitalWrite(ledPin, LOW);
    }
  private:
    int ledPin;     // the number of the debug led pin
    bool led_state = false;
    static constexpr unsigned long on_wait = 50;
    static constexpr unsigned long off_wait = 2500;
};


class SwingingLights: public Task
{
  public:
    SwingingLights(Adafruit_NeoPixel& usingStrip, const SpeedMeasure& speedMeasure)
    : Task(State::Running)
    , stopped(false)
    , current_pixel(trail_length)
    , direction(1)
    , strip(usingStrip)
    , speed(speedMeasure)
    {
    }

    unsigned long step() override {
      auto acc = speed.acceleration();
      if(acc <= -3.5) {
        return showStop();
      }
      else if(speed.stoppedTime() > stoppedShutDown) {
        return turnOffLights();
      }
      return showTrail();
    }

    void stop() override {
      for(uint16_t i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 0));
      }
    }

  private:
    static constexpr uint8_t trail_length = 5;
    static constexpr unsigned long trail_wait = 40;
    static constexpr unsigned long stop_wait = 2000;
    static constexpr unsigned long stoppedShutDown = 60000;
    bool stopped;
    uint16_t current_pixel;
    int8_t direction;
    Adafruit_NeoPixel& strip;
    const SpeedMeasure& speed;

    unsigned long showTrail() {
      if(stopped) {
        stopped = false;
        turnOffLights();
      }
      auto i = current_pixel;
      strip.setPixelColor(max(0, i - trail_length - 1), 0);
      strip.setPixelColor(min(i + trail_length + 1, strip.numPixels()), 0);
      for(uint16_t j = i - trail_length; j < i + trail_length; j++) {
        strip.setPixelColor(j, strip.Color(0, 255, 64));
      }
      strip.show();
      current_pixel = current_pixel + direction;
      if( (direction > 0 && current_pixel == strip.numPixels() - trail_length) ||
          (direction < 0 && current_pixel == trail_length))
      {
        direction = -direction;
      }
      return trail_wait;
    }

    unsigned long turnOffLights() {
      for(uint16_t i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 0));
      }
      strip.show();
      return stop_wait;
    }
    unsigned long showStop() {
      for(uint16_t i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(255, 0, 0));
      }
      strip.show();
      stopped = true;
      return stop_wait;
    }
};

class LedStripOffTask : public Task
{
  public:
    LedStripOffTask(Adafruit_NeoPixel& usingStrip)
    : Task(State::Waiting)
    , strip(usingStrip)
    , current_pixel(0)
    {
    }
    void setup() override {
    }
    unsigned long step() override {
      auto i = current_pixel;
      uint16_t trail_start = i >= trail_length ? i - trail_length : 0;
      for(uint16_t j = trail_start ; j <= min(strip.numPixels(), i); ++j) {
        uint8_t blue = max(127, (trail_start > 0 ? (j - trail_start) : (j + trail_length)) * 3);
        strip.setPixelColor(j, strip.Color(0, 0, blue));
      }
      if( i >= trail_length) {
        strip.setPixelColor(i - trail_length, strip.Color(0,0,0));
      }
      strip.show();
      current_pixel++;
      if(current_pixel >= strip.numPixels() + trail_length) {
        current_pixel = 0;
        stop();
      }
      return max(5, 20 - int16_t(i/2));
    }
    void stop() override {
      for(uint16_t i = 0; i < strip.numPixels(); ++i)
      {
        strip.setPixelColor(i, 0);
      }
      strip.show();
    }
  private:
    static constexpr uint8_t trail_length = 5;
    Adafruit_NeoPixel& strip;
    uint16_t current_pixel;
};

class TaskSleep : public Task
{
  public:
    TaskSleep(const Task** tasks, size_t count) :
      Task(Task::State::Running)
      , _tasks(tasks)
      , _task_size(count)
    {
    }

    void setup() override {
      pinMode(railPin, OUTPUT);
      digitalWrite(railPin, HIGH);
    }

    void wakeup() override {
      digitalWrite(railPin, HIGH);
    }

    void stop() override {
      digitalWrite(railPin, LOW);
    }

    unsigned long step() override {
      if(can_sleep()) {
        sched.sleep();
      }
      return refreshRate;
    }

    bool can_sleep() const override {
      bool cs = true;
      for(size_t i = 0; i < _task_size; ++i) {
        cs &= _tasks[i]->can_sleep();
      }
      return cs;
    }
  private:
    const Task** _tasks;
    size_t _task_size;
    static constexpr int railPin = 8;
    static constexpr unsigned long refreshRate = 5000;
};

class SubTask {
  public:
    enum class State {
      Stopped,
      Running,
    };

    SubTask(): _state(State::Stopped){}

    State getState() {
      return _state;
    }

    unsigned long run() {
      if(_state != State::Stopped) {
        return step();
      }
      return 0;
    }

    virtual unsigned long step() = 0;
    virtual void setup() {
    }
    virtual void stop() {
      _state = State::Stopped;
    }

    virtual void start() {
      _state = State::Running;
    }

    ~SubTask() {}

  protected:
    State _state;
};


class RandomLights : public SubTask
{
  public:
    RandomLights(Adafruit_NeoPixel& usingStrip) : SubTask(), strip(usingStrip)
    {
    }

    unsigned long step() override {
      Serial.println("RandomLights::step()");
      fade_strip();
      for(size_t i = 0; i < randomCount; ++i) {
        auto new_led = random(0, strip.numPixels());
        strip.setPixelColor(new_led, strip.Color(0, 0, 32));
      }
      strip.show();
      return refresh_rate;
    }
  private:

    void fade_strip() {
      for(uint16_t i = 0; i < strip.numPixels(); i++) {
        auto pixelColor = strip.getPixelColor(i);
        uint8_t b = pixelColor & 0xFF;
        pixelColor >>= 8;
        uint8_t g = pixelColor & 0xFF;
        pixelColor >>= 8;
        uint8_t r = pixelColor & 0xFF;
        r = decrease(r);
        g = decrease(g);
        b = decrease(b);

        strip.setPixelColor(i, strip.Color(r, g, b));
      }
    }
    void stop() override {
      for(uint16_t i = 0; i < strip.numPixels(); ++i)
      {
        strip.setPixelColor(i, 0);
      }
      strip.show();
    }
    uint8_t decrease(uint8_t val) const {
      return val < fade_step ? 0 : val - fade_step;
    }

    Adafruit_NeoPixel& strip;
    static const size_t randomCount = 1;
    static const uint8_t fade_step = 4;
    unsigned long refresh_rate = 100;
};

class RailgunFireTask : public SubTask
{
  public:
    RailgunFireTask(Adafruit_NeoPixel& usingStrip)
    : SubTask()
    , strip(usingStrip)
    , current_pixel(0)
    {
    }
    unsigned long step() override {
      auto i = current_pixel;
      uint16_t trail_start = i >= trail_length ? i - trail_length : 0;
      for(uint16_t j = trail_start ; j <= min(strip.numPixels(), i); ++j) {
        uint8_t blue = max(127, (trail_start > 0 ? (j - trail_start) : (j + trail_length)) * 3);
        strip.setPixelColor(j, strip.Color(0, 0, blue));
      }
      if( i >= trail_length) {
        strip.setPixelColor(i - trail_length, strip.Color(0,0,0));
      }
      strip.show();
      current_pixel++;
      if(current_pixel >= strip.numPixels() + trail_length) {
        current_pixel = 0;
        _state = State::Stopped;
        return 0;
      }
      return max(5, 20 - int16_t(i/2));
    }
    void stop() override {
      SubTask::stop();
      for(uint16_t i = 0; i < strip.numPixels(); ++i)
      {
        strip.setPixelColor(i, 0);
      }
      strip.show();
    }
  private:
    static constexpr uint8_t trail_length = 5;
    Adafruit_NeoPixel& strip;
    uint16_t current_pixel;
};

class LaserFireTask: public SubTask
{
  private:
    enum class WeaponState {
      Firing,
      Cooling
    };

    struct Color {
      uint8_t r,g,b;
    };

    WeaponState weaponState;
    uint16_t current_pixel;
    Adafruit_NeoPixel& strip;
    static const unsigned long cooling_wait = 30;
    const Color fireColor = Color{127, 0, 0};
    Color currentColor;

    unsigned long continue_firing() {
      // Serial.print("continue firing: "); Serial.print(current_pixel); Serial.print(" "); Serial.print(fireColor.r);Serial.print(" "); Serial.print(fireColor.g);Serial.print(" "); Serial.println(fireColor.b);
      if(current_pixel < strip.numPixels()) {
        strip.setPixelColor(current_pixel, currentColor.r, currentColor.g, currentColor.b);
        current_pixel++;
        return max((strip.numPixels() - current_pixel) >> 1, 1);
      } else {
        current_pixel = 0;
        weaponState = WeaponState::Cooling;
        return cooling_wait;
      }
    }

    void decreaseColor(uint8_t step) {
      uint8_t r = currentColor.r <= step ? static_cast<uint8_t>(0) : currentColor.r - step;
      uint8_t g = currentColor.g <= step ? static_cast<uint8_t>(0) : currentColor.g - step;
      uint8_t b = currentColor.b <= step ? static_cast<uint8_t>(0) : currentColor.b - step;
      currentColor = Color{r,g,b};
    }

    unsigned long cool_down() {
      decreaseColor(3);
      for(uint16_t i = 0; i < strip.numPixels(); ++i) {
        strip.setPixelColor(i, currentColor.r, currentColor.g, currentColor.b);
      }
      if(currentColor.r == 0 && currentColor.g == 0 && currentColor.b == 0) {
        _state = State::Stopped;
        return 0;
      }
      return cooling_wait;
    }
  public:
    LaserFireTask(Adafruit_NeoPixel& usingStrip)
      : SubTask()
      , current_pixel(0)
      , strip(usingStrip)
      , currentColor(fireColor)
    {
    }

    void setup() override {
    }

    virtual unsigned long step() {
      unsigned long wait = 0;
      if(weaponState == WeaponState::Firing) {
        wait = continue_firing();
      } else if(weaponState == WeaponState::Cooling) {
         wait = cool_down();
      }
      strip.show();
      return wait;
    }

    void stop() override {
      for(uint16_t i = 0; i < strip.numPixels(); ++i)
      {
        strip.setPixelColor(i, 0);
      }
      strip.show();
    }

    void start() override {
      weaponState = WeaponState::Firing;
      current_pixel = 0;
      currentColor = fireColor;
      SubTask::start();
    }
};

class ButtonSensingTask: public Task
{
  public:
    enum class ButtonState {
      Released = 0,
      Pressed = 1,
      DoubleClick = 2
    };

    ButtonSensingTask(unsigned long (*tick_getter)(), int pin)
      : Task(Task::State::Running),
      state(ButtonState::Released),
      tick_func(tick_getter),
      buttonPin(pin),
      lastTick(0),
      lastPressed(0)
    {
    }

    ButtonState currentButtonState() const {
      return state;
    }

    void setup() override {
      pinMode(buttonPin, INPUT_PULLUP);
      lastPressed = millis();
    }

    unsigned long step() {
      auto currentTick = tick_func();
      if(currentTick == lastTick) {
        state = ButtonState::Released;
      }
      else {
        state = currentTick - lastTick > 1 ? ButtonState::DoubleClick : ButtonState::Pressed;
        lastPressed = millis();
        lastTick = currentTick;
      }
      return refreshRate;
    }

    bool can_sleep() const override {
      if(millis() - lastPressed >= stoppedTimeToSleep) {
        return true;
      }
      return false;
    }

    ButtonState getState() const {
      return state;
    }

    unsigned long getLastPressed() const {
      return lastPressed;
    }
    static constexpr unsigned long refreshRate = 400;
  private:
    ButtonState state;
    unsigned long (*tick_func)();
    int buttonPin;
    unsigned long lastTick;
    unsigned long lastPressed;
    static constexpr unsigned long stoppedTimeToSleep = 300000;

};

class WeaponStateTask: public Task {
  public:
    WeaponStateTask(ButtonSensingTask& button, const SpeedMeasure& speed, Adafruit_NeoPixel& ledStrip)
      :Task(Task::State::Running),
      laserFireTask(ledStrip),
      railgunFireTask(ledStrip),
      randomLights(ledStrip),
      subTasks{&randomLights, &laserFireTask, &railgunFireTask},
      currentSubTask(0),
      buttonSensingTask(button),
      speedMeasure(speed),
      refreshRate(button.refreshRate)
    {
    }
    void setup() override {
      buttonSensingTask.setup();
      subTasks[currentSubTask]->start();
    }

    void stop() override {
      buttonSensingTask.stop();
      subTasks[currentSubTask]->stop();
    }

    bool can_sleep() const override {
      return buttonSensingTask.can_sleep();
    }

    unsigned long step() override {
      buttonSensingTask.run();
      auto buttonState = buttonSensingTask.currentButtonState();
      if(currentSubTask == 0) {
        if(buttonState == ButtonSensingTask::ButtonState::DoubleClick ||
          buttonState == ButtonSensingTask::ButtonState::Pressed ) {
          switch_task(last_switched_task);
        }
        else {
          Serial.print("Call run on task "); Serial.println(currentSubTask);
          subTasks[currentSubTask]->run();
          return refreshRate;
        }
      } else if(buttonState == ButtonSensingTask::ButtonState::DoubleClick) {
          auto newSubtask = (currentSubTask + 1 == sizeof(subTasks)/sizeof(subTasks[0])) ? 1 : currentSubTask + 1;
          switch_task(newSubtask);
      }
      else if(subTasks[currentSubTask]->getState() == SubTask::State::Stopped) {
        if(buttonSensingTask.currentButtonState() == ButtonSensingTask::ButtonState::Released) {
          if(millis() - buttonSensingTask.getLastPressed() > weapon_pause) {
            last_switched_task = currentSubTask;
            switch_task(0);
          }
        }
        else if(buttonSensingTask.currentButtonState() == ButtonSensingTask::ButtonState::Pressed) {
          Serial.print("Start subtask "); Serial.println(currentSubTask);
          subTasks[currentSubTask]->start();
        }
      }
      auto wait = subTasks[currentSubTask]->run();
      return wait == 0 ? refreshRate:wait;
    }
  private:

    void switch_task(size_t newTask) {
        Serial.print("Switch subtask from"); Serial.print(currentSubTask); Serial.print(" to "); Serial.println(newTask);
        subTasks[currentSubTask]->stop();
        currentSubTask = newTask;
        subTasks[currentSubTask]->start();
    }

    LaserFireTask laserFireTask;
    RailgunFireTask railgunFireTask;
    RandomLights randomLights;
    SubTask* subTasks[3];
    size_t currentSubTask;
    size_t last_switched_task = 1;
    ButtonSensingTask& buttonSensingTask;
    const SpeedMeasure& speedMeasure;
    static constexpr unsigned long weapon_pause = 60000;
    unsigned long refreshRate;
};

class ButtonLed: public Task {
    public:
      ButtonLed(const ButtonSensingTask& buttonSensingTask, int pin):
        Task(Task::State::Running),
        ledPin(pin),
        button(buttonSensingTask)
      {
      }

    unsigned long step() override {
      auto buttonState = button.getState();
      if(buttonState == ButtonSensingTask::ButtonState::Released) {
        digitalWrite(ledPin,  LOW);
      }
      else if(buttonState == ButtonSensingTask::ButtonState::Pressed) {
        digitalWrite(ledPin,  HIGH);
      } else {
        ledState = ledState == HIGH ? LOW: HIGH;
        digitalWrite(ledPin,  ledState);
        return blinkingRate;
      }
      return refreshRate;
    }

    void stop() override {
      digitalWrite(ledPin, LOW);
    }
    private:
      int ledPin;
      static constexpr unsigned long refreshRate = 100;
      static constexpr unsigned long blinkingRate = 100;
      bool ledState  = LOW;

      const ButtonSensingTask &button;
};

Adafruit_NeoPixel strip_under = Adafruit_NeoPixel(28, 6, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_left = Adafruit_NeoPixel(28, 5, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_right = Adafruit_NeoPixel(28, 9, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_unused = Adafruit_NeoPixel(0, 10, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel* strips[] = {&strip_under, &strip_left, &strip_right, &strip_unused};

volatile unsigned long SpeedMeasure::tickCount = 0;
DebugLedTask debugLedTask(13);
SpeedMeasure speedMeasureTask;
ButtonSensingTask buttonLeft(ButtonPins::getInt0TickCount, 2);
ButtonSensingTask buttonRight(ButtonPins::getInt1TickCount, 3);
ButtonLed blLeft(buttonLeft, 13);
ButtonLed blRight(buttonRight, 4);
SwingingLights swingingLights(strip_under, speedMeasureTask);
ShowSpeed show_speed(speedMeasureTask, 4);
WeaponStateTask weaponLeft(buttonLeft, speedMeasureTask, strip_left);
WeaponStateTask weaponRight(buttonRight, speedMeasureTask, strip_right);

const Task* sleep_control[] = {&speedMeasureTask, &buttonLeft, &buttonRight};
TaskSleep task_sleep(sleep_control, sizeof(sleep_control)/sizeof(sleep_control[0]));

void setup() {
  Serial.begin(9600);
  ButtonPins::setup();
  for(auto strip: strips) {
    strip->begin();// This initializes the NeoPixel library.
    strip->show();
  }
  sched.add_task(debugLedTask);
  sched.add_task(speedMeasureTask);
  sched.add_task(swingingLights);
  sched.add_task(show_speed);
  sched.add_task(task_sleep);
  sched.add_task(weaponLeft);
  sched.add_task(weaponRight);
//  sched.add_task(blLeft);
//  sched.add_task(blRight);
  sched.setup_tasks();
}

void loop() {
  sched.run_next();
}
