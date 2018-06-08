// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#include "schedule.h"

Adafruit_NeoPixel strip_under = Adafruit_NeoPixel(30, 6, NEO_GRB + NEO_KHZ800);
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
      attachInterrupt(digitalPinToInterrupt(interruptPin), tickFunc, FALLING);
    }
    void stop() override {
      detachInterrupt(digitalPinToInterrupt(interruptPin));
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
      Serial.println(tickCount);
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
  private:
    void compute_acceleration() {
      auto timeDiff = times[0] - times[1]; // ms
      computed_acceleration = (speeds[0] - speeds[1]) * 1000.0 / timeDiff;
    }

    static void tickFunc() {
      static unsigned long last_interrupt = 0UL;
      auto ct = millis();
      if(ct - last_interrupt >= 30UL) {
        tickCount += 1;
      }
      last_interrupt = ct;
    }
    template<typename T, unsigned int size>
    void shiftArray(T(&arr)[size]) {
      for(size_t i = sizeof(arr)/sizeof(arr[0]) - 1; i > 0 ; --i) {
        arr[i] = arr[i - 1];
      }
    }
    static volatile unsigned long tickCount;
    static constexpr int interruptPin = 2;
    static constexpr unsigned long refreshRate = 1000;
    static constexpr float tickDist = 1.2f;
    unsigned long lastTick;
    unsigned long lastMillis;
    unsigned long lastMoved;
    float speeds[2];
    float computed_acceleration;
    unsigned long times[2];
};

class ShowSpeed: public Task {
    public:
    ShowSpeed(const SpeedMeasure& speed)
    : Task(Task::State::Running)
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
      static constexpr int ledPin = 4;
      static constexpr unsigned long refreshRate = 500;
      const SpeedMeasure &speedMeasure;
      unsigned long lastTickCount;
      bool led_state;
};

class DebugLedTask : public Task
{
  public:
    DebugLedTask() : Task(Task::State::Running) {}
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
    static constexpr int ledPin = 13;     // the number of the pushbutton pin
    bool led_state = false;
    static constexpr unsigned long on_wait = 75;
    static constexpr unsigned long off_wait = 425;
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

class WeaponFireTask : public Task
{
  public:
    WeaponFireTask(Adafruit_NeoPixel& usingStrip)
    : Task(State::Waiting)
    , strip(usingStrip)
    , current_pixel(0)
    {
    }
    void setup() override {
      pinMode(buttonPin, INPUT_PULLUP);
    }
    bool should_run() override {
      return digitalRead(buttonPin) == LOW;
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
        pause();
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
    static constexpr int buttonPin = 9;
    Adafruit_NeoPixel& strip;
    uint16_t current_pixel;
};

class LaserFireTask: public Task
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
    static const unsigned long cooling_wait = 3;
    static const unsigned long fire_wait = 3000;
    const Color fireColor = Color{0, 0, 127};
    Color currentColor;

    unsigned long continue_firing() {
      strip.setPixelColor(current_pixel, fireColor.r, fireColor.g, fireColor.b);
      if(current_pixel < strip.numPixels()) {
        current_pixel++;
      } else {
        current_pixel = 0;
        weaponState = WeaponState::Cooling;
      }
      return (strip.numPixels() - current_pixel) / 10;
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
        weaponState = WeaponState::Firing;
        currentColor = fireColor;
        return fire_wait;
      }
      return cooling_wait;
    }
  public:
    LaserFireTask(Adafruit_NeoPixel& usingStrip)
      : Task(Task::State::Running)
      , current_pixel(0)
      , strip(usingStrip)
      , currentColor(fireColor)
    {
    }

    void setup() override {
    }
    virtual unsigned long run() {
      unsigned long wait = 0;
      if(weaponState == WeaponState::Firing) {
        wait = continue_firing();
      } else if(weaponState == WeaponState::Cooling) {
        wait = cool_down();
      }
      strip.show();
      return wait;
    }
};

class TaskSleep : public Task
{
  public:
    TaskSleep(const SpeedMeasure& speed) :
      Task(Task::State::Running)
      , speedMeasure(speed)
    {
    }

    void setup() override {
      pinMode(railPin, OUTPUT);
      digitalWrite(railPin, HIGH);
    }
    unsigned long step() override {
      auto stoppedTime = speedMeasure.stoppedTime();
      if(stoppedTime >= stoppedTimeToSleep) {
        digitalWrite(railPin, LOW);
        sched.sleep();
      }
      return refreshRate;
    }
  private:
    const SpeedMeasure &speedMeasure;
    static constexpr int railPin = 8;
    static constexpr unsigned long refreshRate = 5000;
    static constexpr unsigned long stoppedTimeToSleep = 300000;
};

volatile unsigned long SpeedMeasure::tickCount = 0;
DebugLedTask debugLedTask;
SpeedMeasure speedMeasureTask;
TaskSleep task_sleep(speedMeasureTask);
SwingingLights swingingLights(strip_under, speedMeasureTask);
ShowSpeed show_speed(speedMeasureTask);

void setup() {
  Serial.begin(9600);
  strip_under.begin(); // This initializes the NeoPixel library.
  strip_under.show();
  sched.add_task(debugLedTask);
  sched.add_task(speedMeasureTask);
  sched.add_task(swingingLights);
  sched.add_task(show_speed);
  sched.add_task(task_sleep);
  sched.setup_tasks();
}

void loop() {
  sched.run_next();
}
