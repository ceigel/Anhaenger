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

    void sleep() override {
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
    static constexpr unsigned long stoppedTimeToSleep = 120000;
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
    , led_state(true)
    {
    }
    void setup() override {
      pinMode(ledPin, OUTPUT);
    }

    void sleep() override {
      digitalWrite(ledPin, LOW);
    }

    void wakeup() override {
      digitalWrite(ledPin, led_state ? HIGH : LOW);
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
    DebugLedTask(int pin) : Task(Task::State::Running), ledPin(pin), on_count(0), start_millis(0) {}

    void setup() override {
      pinMode(ledPin, OUTPUT);
      record_start();
    }

    unsigned long step() override {
      if(millis() - start_millis > debug_led_duration) {
        digitalWrite(ledPin, LOW);
        Serial.println("Debug led off");
        return stopped_duration;
      }

      led_state = !led_state;
      digitalWrite(ledPin, led_state ? HIGH : LOW);
      if(led_state) {
        auto wait = on_count >= 3 ? long_wait : short_wait;
        on_count = on_count == 5 ? 0 : on_count + 1;
        return wait;
      }
      return off_wait;
    }

    void sleep() override {
      digitalWrite(ledPin, LOW);
    }

    void wakeup() override {
      digitalWrite(ledPin, led_state ? HIGH : LOW);
      record_start();
    }

    void record_start() {
      start_millis = millis();
    }

  private:
    int ledPin;     // the number of the debug led pin
    bool led_state = false;
    size_t on_count;
    static constexpr unsigned long long_wait = 300;
    static constexpr unsigned long short_wait = 100;
    static constexpr unsigned long off_wait = 1000;
    static constexpr unsigned long debug_led_duration = 30000;
    static constexpr unsigned long stopped_duration = 0xFFFFFFFF;
    unsigned long start_millis;
};


class SwingingLights: public Task
{
  public:
    SwingingLights(Adafruit_NeoPixel& usingStrip, const SpeedMeasure& speedMeasure, int powerPin)
    : Task(State::Running)
    , stopped(false)
    , current_pixel(trail_length)
    , direction(1)
    , strip(usingStrip)
    , speed(speedMeasure)
    , _powerPin(powerPin)
    {
    }

    void setup() override {
      pinMode(_powerPin, OUTPUT);
      digitalWrite(_powerPin, LOW);
    }

    unsigned long step() override {
      auto acc = speed.acceleration();
      if(speed.stoppedTime() > stoppedShutDown) {
        return turnOffLights();
      }
      turnOnLights();
      if(acc <= -2.0) {
        return showStop();
      }
      return showTrail();
    }

    void sleep() override {
      digitalWrite(_powerPin, HIGH);
    }

    void wakeup() override {
    }

  private:
    static constexpr uint8_t trail_length = 5;
    static constexpr unsigned long trail_wait = 40;
    static constexpr unsigned long stop_wait = 2000;
    static constexpr unsigned long stoppedShutDown = 30000;
    bool stopped;
    uint16_t current_pixel;
    int8_t direction;
    Adafruit_NeoPixel& strip;
    const SpeedMeasure& speed;
    int _powerPin;

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

    void turnOnLights() {
      digitalWrite(_powerPin, LOW);
    }

    unsigned long turnOffLights() {
      for(uint16_t i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 0));
      }
      strip.show();
      digitalWrite(_powerPin, HIGH);
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

    virtual bool can_sleep() const {
      return true;
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

    void start() override {
      current_pixel = 0;
      SubTask::start();
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

    ButtonSensingTask(int pin)
      : Task(Task::State::Running),
      readings_count(sizeof(sensor_readings) / sizeof(sensor_readings[0])),
      reading_index(0),
      last_reading(HIGH),
      buttonPin(pin),
      lastPressed(millis()),
      state(ButtonState::Released)
    {
    }

    void setup() override {
      pinMode(buttonPin, INPUT_PULLUP);
      lastPressed = millis();
    }

    unsigned long step() {
      if(reading_index != readings_count) {
        read_button();
        return refreshRate;
      }
      reading_index = 0;
      if(!readings_stable()) {
        return refreshRate;
      }

      auto currentReading = sensor_readings[0];
      auto now = millis();
      if(currentReading != last_reading) {
        if(currentReading == LOW) {
          state = (now - lastPressed < doubleClickDelay) ? ButtonState::DoubleClick : ButtonState::Pressed;
          Serial.println(state == ButtonState::Pressed ? "Pressed" : "DoubleClick");
          lastPressed = now;
        }
        else {
          state = ButtonState::Released;
        }
        last_reading = currentReading;
      }

      return refreshRate;
    }

    bool can_sleep() const override {
      if(millis() - lastPressed >= stoppedTimeToSleep) {
        return true;
      }
      return false;
    }

    void wakeup() {
      detachInterrupt(digitalPinToInterrupt(buttonPin));
    }

    void sleep() override {
      attachInterrupt(digitalPinToInterrupt(buttonPin), ButtonSensingTask::wakeButtonPressed, FALLING);
    }

    ButtonState buttonState() const {
      return state;
    }

    unsigned long getLastPressed() const {
      return lastPressed;
    }
  private:
    static void wakeButtonPressed(){
    }
    void read_button() {
        sensor_readings[reading_index] = digitalRead(buttonPin);
        reading_index++;
    }

    bool readings_stable() {
      auto first_reading = sensor_readings[0];
      for(size_t i = 1; i < readings_count; ++i) {
        if(sensor_readings[i] != first_reading) {
          return false;
        }
      }
      return true;
    }

    int sensor_readings[3];
    size_t readings_count;
    size_t reading_index;
    int last_reading;
    unsigned int buttonPin;
    unsigned long lastPressed;
    ButtonState state;
    static constexpr unsigned long refreshRate = 10;
    static constexpr unsigned long doubleClickDelay = 700;
    static constexpr unsigned long stoppedTimeToSleep = 120000;

};

class WeaponStateTask: public Task {
  public:
    WeaponStateTask(ButtonSensingTask& button, const SpeedMeasure& speed, Adafruit_NeoPixel& ledStrip, int powerPin)
      :Task(Task::State::Running),
      laserFireTask(ledStrip),
      railgunFireTask(ledStrip),
      randomLights(ledStrip),
      subTasks{&randomLights, &railgunFireTask, &laserFireTask},
      subTasksSize(sizeof(subTasks) / sizeof(subTasks[0])),
      currentSubTask(0),
      buttonSensingTask(button),
      speedMeasure(speed),
      _powerPin(powerPin)
    {
    }

    void setup() override {
      buttonSensingTask.setup();
      subTasks[currentSubTask]->start();
      pinMode(_powerPin, OUTPUT);
      digitalWrite(_powerPin, LOW);
    }

    void sleep() override {
      subTasks[currentSubTask]->stop();
      digitalWrite(_powerPin, HIGH);
    }

    void wakeup() override {
      digitalWrite(_powerPin, LOW);
    }


    bool can_sleep() const override {
      return buttonSensingTask.can_sleep();
    }

    unsigned long step() override {
      auto buttonState = buttonSensingTask.buttonState();
      if(currentSubTask == 0) {
        if(buttonState == ButtonSensingTask::ButtonState::DoubleClick ||
          buttonState == ButtonSensingTask::ButtonState::Pressed ) {
          switch_task(last_switched_task);
          return refreshRate;
        }
        else {
          subTasks[currentSubTask]->run();
          return refreshRate;
        }
      } else if(buttonState == ButtonSensingTask::ButtonState::DoubleClick) {
          auto newSubtask = (currentSubTask + 1 == subTasksSize) ? 1 : currentSubTask + 1;
          switch_task(newSubtask);
          return refreshRate;
      }
      else if(subTasks[currentSubTask]->getState() == SubTask::State::Stopped) {
        if(buttonSensingTask.buttonState() == ButtonSensingTask::ButtonState::Released) {
          if(millis() - buttonSensingTask.getLastPressed() > weapon_pause) {
            last_switched_task = currentSubTask;
            switch_task(0);
            return refreshRate;
          }
        }
        else if(buttonSensingTask.buttonState() == ButtonSensingTask::ButtonState::Pressed) {
          Serial.print("Start subtask "); Serial.println(currentSubTask);
          subTasks[currentSubTask]->start();
        }
      }
      auto wait = subTasks[currentSubTask]->run();
      return wait == 0 ? refreshRate : wait;
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
    size_t subTasksSize;
    size_t currentSubTask;
    size_t last_switched_task = 1;
    ButtonSensingTask& buttonSensingTask;
    const SpeedMeasure& speedMeasure;
    int _powerPin;
    static constexpr unsigned long weapon_pause = 20000;
    static constexpr unsigned long refreshRate = 300;
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

    void sleep() override {
      Serial.println("Sleep TaskSleep");
    }
    void wakeup() override {
      Serial.println("Wakup TaskSleep");
    }

    unsigned long step() override {
      if(can_sleep()) {
        sched.start_sleep();
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
    static constexpr unsigned long refreshRate = 5000;
};

Adafruit_NeoPixel strip_under = Adafruit_NeoPixel(30, 6, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_left = Adafruit_NeoPixel(41, 5, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_right = Adafruit_NeoPixel(41, 9, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_unused = Adafruit_NeoPixel(0, 10, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel* strips[] = {&strip_under, &strip_left, &strip_right, &strip_unused};

volatile unsigned long SpeedMeasure::tickCount = 0;
DebugLedTask debugLedTask(4);
SpeedMeasure speedMeasureTask;
ButtonSensingTask buttonLeft(2);
ButtonSensingTask buttonRight(3);
SwingingLights swingingLights(strip_under, speedMeasureTask, 7);
ShowSpeed show_speed(speedMeasureTask, 13);
WeaponStateTask weaponLeft(buttonLeft, speedMeasureTask, strip_left, 8);
WeaponStateTask weaponRight(buttonRight, speedMeasureTask, strip_right, 8);

const Task* sleep_control[] = {&speedMeasureTask, &buttonLeft, &buttonRight};
TaskSleep task_sleep(sleep_control, sizeof(sleep_control)/sizeof(sleep_control[0]));

void setup() {
  Serial.begin(9600);
  for(auto strip: strips) {
    strip->begin();// This initializes the NeoPixel library.
    strip->show();
  }
  sched.add_task(debugLedTask);
  sched.add_task(speedMeasureTask);
  sched.add_task(swingingLights);
  sched.add_task(show_speed);
  sched.add_task(weaponLeft);
  sched.add_task(weaponRight);
  sched.add_task(buttonLeft);
  sched.add_task(buttonRight);
  sched.add_task(task_sleep);
  sched.setup_tasks();
}

void loop() {
  sched.run_next();
}
