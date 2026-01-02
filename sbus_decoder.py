"""SBUS (Serial Bus) decoder for RC controllers."""

import time


class SBUSDecoder:
    """
    Decodes SBUS (Serial Bus) signals from an RC controller.
    
    Configuration:
    - 100k baud, 8 data bits, 1 stop bit, even parity
    - Supports up to 16 channels
    - Packet format: [0x0F] [11 bytes data] [1 flags byte] [0x00]
    """
    
    def __init__(self, uart, signal_timeout=1.0):
        """
        Initialize the SBUS decoder.
        
        Args:
            uart: A busio.UART object configured for SBUS (100k baud)
            signal_timeout: Timeout in seconds to detect signal loss (default 1.0s)
        """
        self.uart = uart
        self.channels = [1500] * 16  # 16 channels, default neutral (1500µs equivalent)
        self._buffer = bytearray()
        self._prev_channels = [1500] * 16  # For smoothing
        self._max_jump = 500  # Reject changes larger than this (in µs)
        self.has_signal = False  # True when valid signal is received
        self._last_packet_time = time.monotonic()  # Track last valid packet
        self.signal_timeout = signal_timeout
        self.update(debug=False)
        
    def update(self, debug=False):
        """
        Call this in your main loop to decode SBUS packets.
        
        Args:
            debug: If True, print received packets
        """
        # Check for signal timeout
        current_time = time.monotonic()
        if self.has_signal and (current_time - self._last_packet_time) > self.signal_timeout:
            self.has_signal = False
            if debug:
                print("SBUS: Signal lost (timeout)")
        
        if self.uart.in_waiting > 0:
            data = self.uart.read(self.uart.in_waiting)
            if data:
                self._buffer.extend(data)
                self._parse_buffer(debug)
    
    def _parse_buffer(self, debug=False):
        """Parse SBUS packets from the buffer."""
        while len(self._buffer) >= 25:  # SBUS packet is 25 bytes
            # Find the start byte (0x0F)
            if self._buffer[0] != 0x0F:
                self._buffer[:] = self._buffer[1:]
                continue
            
            # Check if we have a complete packet
            if len(self._buffer) < 25:
                break
            
            # Verify end byte (0x00)
            if self._buffer[24] != 0x00:
                self._buffer[:] = self._buffer[1:]
                continue
            
            # Extract and decode packet
            self._decode_channels(bytes(self._buffer[:25]), debug)
            self._buffer[:] = self._buffer[25:]
    
    def _decode_channels(self, packet, debug=False):
        """Decode 16 channels from SBUS packet."""
        # Channel data is in bytes 1-22 (11 bytes per 8 channels, overlapping bits)
        channels_raw = [
            ((packet[1] | (packet[2] << 8)) & 0x7FF),
            ((packet[2] >> 3 | (packet[3] << 5)) & 0x7FF),
            ((packet[3] >> 6 | (packet[4] << 2) | (packet[5] << 10)) & 0x7FF),
            ((packet[5] >> 1 | (packet[6] << 7)) & 0x7FF),
            ((packet[6] >> 4 | (packet[7] << 4)) & 0x7FF),
            ((packet[7] >> 7 | (packet[8] << 1) | (packet[9] << 9)) & 0x7FF),
            ((packet[9] >> 2 | (packet[10] << 6)) & 0x7FF),
            ((packet[10] >> 5 | (packet[11] << 3)) & 0x7FF),
            ((packet[12] | (packet[13] << 8)) & 0x7FF),
            ((packet[13] >> 3 | (packet[14] << 5)) & 0x7FF),
            ((packet[14] >> 6 | (packet[15] << 2) | (packet[16] << 10)) & 0x7FF),
            ((packet[16] >> 1 | (packet[17] << 7)) & 0x7FF),
            ((packet[17] >> 4 | (packet[18] << 4)) & 0x7FF),
            ((packet[18] >> 7 | (packet[19] << 1) | (packet[20] << 9)) & 0x7FF),
            ((packet[20] >> 2 | (packet[21] << 6)) & 0x7FF),
            ((packet[21] >> 5 | (packet[22] << 3)) & 0x7FF),
        ]
        
        # Convert from 11-bit to PWM microseconds
        # Calibrated: raw 173 = 988µs, raw 1811 = 2012µs
        new_channels = []
        for i, raw in enumerate(channels_raw):
            pwm = 988 + (raw - 173) * 1024 // 1638
            new_channels.append(pwm)
        
        if debug and False:  # Set to True to see raw 11-bit values
            print(f"Raw: {[channels_raw[i] for i in range(8)]}")
        
        # Validate decoded values:
        # 1. Check range (800-2200 µs)
        # 2. Check against previous values with smoothing
        valid = True
        if not all(800 < ch < 2200 for ch in new_channels):
            valid = False
        else:
            # Use moving average: accept if close to previous value
            smoothed = []
            for i, new_val in enumerate(new_channels):
                jump = abs(new_val - self._prev_channels[i])
                if jump > self._max_jump:
                    # Instead of rejecting, average with previous
                    smoothed_val = (self._prev_channels[i] + new_val) // 2
                    smoothed.append(smoothed_val)
                else:
                    smoothed.append(new_val)
            new_channels = smoothed
        
        if valid:
            self._prev_channels = new_channels
            self.channels = new_channels
            self._last_packet_time = time.monotonic()
            if not self.has_signal:
                self.has_signal = True
                if debug:
                    print("SBUS: Signal acquired")
            if debug:
                print(f"SBUS: {[self.channels[i] for i in range(8)]}")
        elif debug:
            print(f"SBUS: Rejected corrupted packet")
    
    def get_channel(self, channel):
        """
        Get the current value of a specific channel.
        
        Args:
            channel: Channel number (0-15)
            
        Returns:
            PWM pulse width in microseconds (1000-2000)
        """
        if 0 <= channel < 16:
            return self.channels[channel]
        raise ValueError(f"Channel {channel} out of range (0-15)")
    
    def get_all_channels(self):
        """
        Get all channel values at once.
        
        Returns:
            List of PWM values for all 16 channels
        """
        return self.channels.copy()
