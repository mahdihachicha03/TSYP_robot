#!/usr/bin/env python3
"""
Raspberry Pi 4 + L298N-style dual motor driver + GA25-370 encoders
--------------------------------------------------------------------
Pin mapping (BCM numbering) matches your physical-pin wiring:

  Driver:
    ENA -> physical 33 -> GPIO13   (Motor A speed, PWM)
    ENB -> physical 32 -> GPIO12   (Motor B speed, PWM)
    IN1 -> physical 36 -> GPIO16   (Motor A direction)
    IN2 -> physical 38 -> GPIO20   (Motor A direction)
    IN3 -> physical 40 -> GPIO21   (Motor B direction)
    IN4 -> physical 37 -> GPIO26   (Motor B direction)

  Encoders (per the motor's color legend: green=signal A, yellow=signal B):
    Motor A  green(A) -> physical 16 -> GPIO23
    Motor A  yellow(B) -> physical 18 -> GPIO24
    Motor B  green(A) -> physical 13 -> GPIO27
    Motor B  yellow(B) -> physical 11 -> GPIO17

  Reminder on motor power wiring (NOT covered by this script, just physical wiring):
    Red   -> Motor power +   -> driver OUT terminal
    White -> Motor power -   -> driver OUT terminal
    Black -> Encoder GND     -> Pi GND
    Blue  -> Encoder VCC     -> Pi 3.3V

Requires: gpiozero (preinstalled on Raspberry Pi OS)
    sudo apt install python3-gpiozero      # if missing
Optional but recommended for smoother PWM / lower-latency encoder reads:
    sudo apt install pigpio
    sudo systemctl enable --now pigpiod
    then set: from gpiozero.pins.pigpio import PiGPIOFactory
              Device.pin_factory = PiGPIOFactory()
"""

import time
from gpiozero import Motor, RotaryEncoder
from signal import pause

# ---------------------------------------------------------------------------
# Motor setup
# ---------------------------------------------------------------------------
# gpiozero's Motor(forward, backward, enable) drives `forward`/`backward` as
# plain direction pins and PWMs the `enable` pin for speed (0.0 - 1.0).
motor_a = Motor(forward=16, backward=20, enable=13, pwm=True)   # Right motor
motor_b = Motor(forward=21, backward=26, enable=12, pwm=True)   # Left motor

# ---------------------------------------------------------------------------
# Encoder setup
# ---------------------------------------------------------------------------
# RotaryEncoder counts quadrature edges on the A/B channel pair.
# max_steps=0 disables clamping so it counts freely in both directions.
encoder_a = RotaryEncoder(a=23, b=24, max_steps=0)  # Motor A: green=23, yellow=24
encoder_b = RotaryEncoder(a=27, b=17, max_steps=0)  # Motor B: green=27, yellow=17

# ---------------------------------------------------------------------------
# Encoder resolution math
# ---------------------------------------------------------------------------
# The encoder disk gives 11 pulses per channel for every MOTOR-SHAFT
# revolution (this is fixed by the encoder hardware, confirmed from the
# motor's datasheet - it does not change with gear ratio).
# RotaryEncoder counts BOTH edges of BOTH channels (quadrature x4), so:
PULSES_PER_CHANNEL = 11
QUADRATURE_MULTIPLIER = 4
COUNTS_PER_MOTOR_REV = PULSES_PER_CHANNEL * QUADRATURE_MULTIPLIER  # = 44

# Your motor is rated 620 RPM at 12V - but that's the OUTPUT SHAFT speed,
# i.e. AFTER the internal gearbox. The encoder sits on the motor shaft,
# BEFORE the gearbox, so raw counts need to be divided by the gear ratio
# to get output-shaft RPM. Sellers for this motor family don't always
# print the exact ratio, so rather than guess, set GEAR_RATIO = None and
# run calibrate_gear_ratio() once — it uses your known 620 RPM rating to
# work out the ratio for you from real encoder counts.
# Your gearbox datasheet (19mm length, 4-stage) lists two possible ratios
# for this configuration: 15.5:1 or 20.4:1. Which one your unit actually
# has depends on the internal gear stack fitted at the factory - use
# manual_calibrate_gear_ratio() below to measure it and match against
# whichever of these two it lands closest to, then hardcode the EXACT
# datasheet value (not the raw measured one) into GEAR_RATIO.
DATASHEET_RATIO_CANDIDATES = (15.5, 20.4)
GEAR_RATIO = 15.5          # confirmed via hand-turn calibration (measured ~16, datasheet: 15.5:1)
RATED_OUTPUT_RPM = 620     # your motor's printed no-load rating at 12V

# Counts per OUTPUT shaft revolution - only valid once GEAR_RATIO is set.
def counts_per_output_rev():
    if GEAR_RATIO is None:
        raise ValueError(
            "GEAR_RATIO is not set yet. Run calibrate_gear_ratio() first, "
            "then hardcode the printed value into GEAR_RATIO above."
        )
    return COUNTS_PER_MOTOR_REV * GEAR_RATIO


def set_motor(motor, speed):
    """speed: -1.0 (full reverse) to 1.0 (full forward), 0 = stop."""
    speed = max(-1.0, min(1.0, speed))
    if speed > 0:
        motor.forward(speed)
    elif speed < 0:
        motor.backward(-speed)
    else:
        motor.stop()


def stop_all():
    motor_a.stop()
    motor_b.stop()


# ---------------------------------------------------------------------------
# Calibration: derive GEAR_RATIO by turning the OUTPUT shaft by hand
# ---------------------------------------------------------------------------
# Running the motor to calibrate is unreliable on cheap brushed DC motors:
# brush/commutator sparking injects electrical noise onto the encoder
# signal wires, which gets misread as extra pulses and inflates the count
# by orders of magnitude (this is almost certainly what produced the
# nonsense -1769 ratio - real counts should be in the tens/hundreds for a
# couple of manual turns, not hundreds of thousands).
#
# Turning the shaft BY HAND avoids this completely: the motor is unpowered,
# so there's no electrical noise, and you control exactly how many output
# revolutions happen.
def manual_calibrate_gear_ratio(encoder=None, turns=5):
    encoder = encoder or encoder_a
    encoder.steps = 0
    input(f"Motor OFF now. Slowly turn the OUTPUT shaft exactly "
          f"{turns} full turns by hand, then press Enter...")
    counts = abs(encoder.steps)

    if counts == 0:
        print("No counts detected - check encoder wiring (VCC/GND/A/B) "
              "before calibrating.")
        return None

    ratio = counts / (COUNTS_PER_MOTOR_REV * turns)
    closest = min(DATASHEET_RATIO_CANDIDATES, key=lambda r: abs(r - ratio))
    print(f"Raw counts for {turns} output turns: {counts}")
    print(f"=> Measured GEAR_RATIO ≈ {ratio:.2f}")
    print(f"=> Closest datasheet value: {closest}:1 "
          f"(candidates were {DATASHEET_RATIO_CANDIDATES})")

    if not (2 <= ratio <= 300):
        print("Warning: this is outside the normal range for this motor "
              "family (roughly 2:1 to 300:1). Re-check wiring/noise before "
              "trusting this number.")
    else:
        print(f"Set GEAR_RATIO = {closest} at the top of the script.")
    return closest


# ---------------------------------------------------------------------------
# Simple test routine: drive both motors forward, print encoder counts + RPM
# ---------------------------------------------------------------------------
def test_run(duration=5, speed=0.5):
    print(f"Driving both motors forward at {speed*100:.0f}% for {duration}s...")
    set_motor(motor_a, speed)
    set_motor(motor_b, speed)

    cpr = counts_per_output_rev()  # raises if GEAR_RATIO isn't set yet
    last_a, last_b = encoder_a.steps, encoder_b.steps
    t_start = time.time()
    try:
        while time.time() - t_start < duration:
            time.sleep(0.5)
            now_a, now_b = encoder_a.steps, encoder_b.steps
            da, db = now_a - last_a, now_b - last_b
            rpm_a = (da / cpr) / (0.5 / 60)
            rpm_b = (db / cpr) / (0.5 / 60)
            print(f"A: total={now_a:6d}  d={da:4d}  ~{rpm_a:6.1f} RPM   "
                  f"B: total={now_b:6d}  d={db:4d}  ~{rpm_b:6.1f} RPM")
            last_a, last_b = now_a, now_b
    finally:
        stop_all()
        print("Stopped.")


if __name__ == "__main__":
    try:
        if GEAR_RATIO is None:
            # First run: figure out the gear ratio by hand, then stop.
            manual_calibrate_gear_ratio(encoder_a, turns=5)
            print("\nRe-run this script with GEAR_RATIO set to see live RPM.")
        else:
            test_run(duration=5, speed=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop_all()
        print(f"Final counts -> A: {encoder_a.steps}, B: {encoder_b.steps}")
