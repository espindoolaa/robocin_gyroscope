#include "utils.h"

namespace utils {
  double param_angular_pos = 6.222;
  double param_angular_neg = 6.222;
  double param_linear_pos = 57.71;
  double param_linear_neg = -60.51;

  // To control M5
  // 9.4 - 22 PWM == -13.44 ~ -314.159 RAD/S
  double param_linear_r1 = 210.9059206;
  double param_angular_r1 = -23.8665873;
  // 22 - 38 PWM == -314.159 ~ -688.158 RAD/S
  double param_linear_r2 = 200.089625;
  double param_angular_r2 = -23.3749375;
  // 38 - 53 PWM == -688.158 ~ -1002.318 RAD/S
  double param_linear_r3 = 107.714;
  double param_angular_r3 = -20.944;
  // 53 - 73 PWM == -1002.318 ~ -1436.150 RAD/S
  double param_linear_r4 = 127.3368;
  double param_angular_r4 = -20.600;
  // 73 PWM == -1436.150 RAD/S
  double param_linear_r5 = 140.3368;
  double param_angular_r5 = -20.320;

  DigitalIn PBT1(PIN_PB1);
  DigitalIn PBT2(PIN_PB2);
  DigitalIn SW1(PIN_SELECTOR_1);
  DigitalIn SW2(PIN_SELECTOR_2);
  DigitalIn SW3(PIN_SELECTOR_3);
  DigitalIn SW4(PIN_SELECTOR_4);
  PwmOut Buzzer(PIN_BUZZER);

  AnalogIn BATT(PIN_BATT);

  void initRobot() {
    printf("Booting Robot....id %d\n", getRobotId());

#ifdef USING_NEW_MOTOR
    Buzzer.period_us(DEFAULT_BUZZER_CYCLE);
    Status::init(PIN_RGB);
    Status::send(Color::BLUE, DRIBBLER_LED);
    Buzzer.write(DEFAULT_BUZZER_FREQUENCY);
    Status::sendAllColor(Color::GREEN, 80);
    Buzzer.write(0);
    ThisThread::sleep_for(80ms);
    Buzzer.write(DEFAULT_BUZZER_FREQUENCY);
    Status::sendAllColor(Color::GREEN, 80);
    Buzzer.write(0);
    ThisThread::sleep_for(80ms);
    Buzzer.write(DEFAULT_BUZZER_FREQUENCY);
    Status::sendAllColor(Color::GREEN, 80);
    Buzzer.write(0);
    Status::clearColors();
#else
    printf("! Old motor control loaded !\n");
    Buzzer.period_us(390);
    Status::init(PIN_RGB);
    Status::send(Color::BLUE, DRIBBLER_LED);
    Buzzer.write(DEFAULT_BUZZER_FREQUENCY);
    Status::sendAllColor(Color::GREEN, 80);
    Buzzer.write(0);
    ThisThread::sleep_for(80ms);
    Buzzer.write(DEFAULT_BUZZER_FREQUENCY);
    Status::sendAllColor(Color::GREEN, 80);
    Buzzer.write(0);
    ThisThread::sleep_for(80ms);
    Buzzer.write(DEFAULT_BUZZER_FREQUENCY);
    Status::sendAllColor(Color::GREEN, 80);
    Buzzer.write(0);
    Status::clearColors();
#endif
  }

  bool timerMillisExpired(const Timer& timer, double time) {
    return timerRead<chrono::milliseconds>(timer) > time;
  }

  double timerMillisRead(const Timer& timer) {
    return timerRead<chrono::milliseconds>(timer);
  }

  ButtonState pressedFor(DigitalIn button, double time) {
    if (button)
      return ButtonState::UNPRESSED;

    Timer timer;
    timer.start();

    while (!button) {
      if (timerMillisExpired(timer, time)) {
        utils::beep(180);
        return ButtonState::HELD;
      }
    }
    return ButtonState::RELEASED;
  }

  // New line equation to control M1, M2, M3, M4
  double rad_s_to_pwm_pi(double rad_s) {
    double param_angular = param_angular_pos;
    double param_linear = param_linear_pos;

    if (fabs(rad_s) < MOTOR_MIN_SPEED_RAD_S) {
      return 0;
    }
    if (rad_s < 0) {
      param_angular = param_angular_neg;
      param_linear = param_linear_neg;
    }
    return (param_angular * rad_s) + param_linear;
  }

  double last_battery = ((BATT.read() * 3.3) * 54) / 10.0;
  double getBattery() {
    last_battery = (last_battery + (((BATT.read() * 3.3) * 54) / 10.0)) / 2.0;
    return last_battery;
  }
  void checkBattery() {
    while (utils::getBattery() < 15.5) {
      Buzzer.write(DEFAULT_BUZZER_FREQUENCY);
      Status::sendRoundColor(Color::RED, 100);
      Buzzer.write(0);
      ThisThread::sleep_for(500ms);
    }
  }
  int getRobotId() {
    unsigned int _robotId = 0;
    _robotId += (SW1.read());
    _robotId += (SW2.read() * 2);
    _robotId += (SW3.read() * 4);
    _robotId += (SW4.read() * 8);
    if (_robotId <= 15)
      return _robotId;
    else
      return 16;
  }
  void beep(int time, int cycleUs, double frequency) {
    bool cycleUpdated = false;
    if (cycleUs != DEFAULT_BUZZER_CYCLE) {
      Buzzer.period_us(cycleUs);
      cycleUpdated = true;
    }

    Buzzer.write(frequency);
    ThisThread::sleep_for(chrono::milliseconds(time));
    Buzzer.write(0);

    if (cycleUpdated) {
      Buzzer.period_us(DEFAULT_BUZZER_CYCLE);
    }
  }
} // namespace utils
