import board
import busio
import digitalio
import neopixel
import time
from sbus_decoder import SBUSDecoder
from heating_controller import HeatingController

# Try to initialize NeoPixel if available
pixel = None
has_pixel = False
try:
    if hasattr(board, 'NEOPIXEL'):
        pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
        pixel.brightness = 0.1
        pixel.fill((0, 255, 0))  # Green for startup
        has_pixel = True
except Exception:
    pass

time.sleep(5)  # Wait for system to stabilize

relayPin = digitalio.DigitalInOut(board.D13) # Pin connected to the relay (pull high to connect)
relayPin.direction = digitalio.Direction.OUTPUT

sBUSPin = board.A3 # Pin connected to SBUS signal

channel = 15  # Channel to monitor (0-15)


# Initialize UART for SBUS
uart = busio.UART(board.TX, sBUSPin, baudrate=100000, parity=busio.UART.Parity.EVEN, receiver_buffer_size=128)
sbus_decoder = SBUSDecoder(uart)

# Initialize heating controller
heating = HeatingController(relayPin)


def update_pixel_status(heating_controller):
    """Update NeoPixel color based on heating controller status."""
    if not has_pixel:
        return
    
    mode = heating_controller.mode
    is_on = heating_controller.relay_on
    
    if mode == 'fixed_on':
        pixel.fill((255, 0, 0))  # Red: heater on due to high channel
    elif mode == 'fixed_off':
        pixel.fill((0, 0, 255))  # Blue: heater off due to low channel
    elif mode == 'proportional':
        if is_on:
            pixel.fill((255, 100, 0))  # Orange: heater on for proportional control
        else:
            pixel.fill((128, 0, 128))  # Purple: heater off for proportional control


# Main loop
while True:
    sbus_decoder.update()

    if sbus_decoder.has_signal:
        channel_value = sbus_decoder.get_channel(channel)
        heating.update(channel_value)
        update_pixel_status(heating)
    else:
        # No valid SBUS signal
        relayPin.value = False  # Ensure relay is off
        if has_pixel:
            pixel.fill((255, 255, 255))  # White indicates no signal

    time.sleep(0.01)  # Update every 10ms



