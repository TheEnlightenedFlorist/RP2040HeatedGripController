"""Heating controller for RC-controlled grip warmers."""

import time


class HeatingController:
    """
    Controls relay heating element based on RC channel input.
    - Below 1000µs: Always off
    - 1000µs: 25% duty cycle (2.5s on, 7.5s off)
    - 2000µs: 100% duty cycle (always on)
    - Above 2000µs: Always on
    - Varies smoothly between 1000-2000µs
    """
    
    def __init__(self, relay_pin, cycle_period=10.0):
        """
        Initialize the heating controller.
        
        Args:
            relay_pin: GPIO pin for the relay
            cycle_period: Total cycle period in seconds (default 10s)
        """
        self.relay_pin = relay_pin
        self.cycle_period = cycle_period
        self.relay_on = False
        self.next_switch_time = time.monotonic()
        self.mode = None  # 'fixed_on', 'fixed_off', or 'proportional'
        
    def update(self, channel_value):
        """
        Update the relay state based on channel value.
        Call this regularly in your main loop.
        
        Args:
            channel_value: PWM value in microseconds (typically 988-2012)
        """
        current_time = time.monotonic()
        
        # Determine if we should switch states
        if current_time >= self.next_switch_time:
            self._switch_state(channel_value)
    
    def _switch_state(self, channel_value):
        """Switch relay on/off based on current channel value."""
        if channel_value < 1000:
            # Always off
            self.relay_pin.value = False
            self.relay_on = False
            self.mode = 'fixed_off'
            self.next_switch_time = time.monotonic() + self.cycle_period
            print("Heating OFF due to low channel value")
        elif channel_value > 2000:
            # Always on
            self.relay_pin.value = True
            self.relay_on = True
            self.mode = 'fixed_on'
            self.next_switch_time = time.monotonic() + self.cycle_period
            print("Heating ON due to high channel value")
        else:
            # Proportional control between 1000-2000µs
            # Calculate duty cycle: 25% at 1000µs, 100% at 2000µs
            duty_cycle = 0.25 + (channel_value - 1000) / 1000.0 * 0.75
            self.mode = 'proportional'
            
            if self.relay_on:
                # Currently on, switch to off
                on_time = duty_cycle * self.cycle_period
                off_time = self.cycle_period - on_time
                self.next_switch_time = time.monotonic() + off_time
                self.relay_pin.value = False
                self.relay_on = False
                print(f"Heating OFF for {off_time:.2f}s")
            else:
                # Currently off, switch to on
                on_time = duty_cycle * self.cycle_period
                self.next_switch_time = time.monotonic() + on_time
                self.relay_pin.value = True
                self.relay_on = True
                print(f"Heating ON for {on_time:.2f}s")
